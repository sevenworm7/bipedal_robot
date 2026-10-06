"""Straight-walking environment used by the rule-based controller, data collection, BC and PPO.

Ported from ``00_test/shared/env`` (experiment 1). The ``leg_*`` buffers keep their experiment 1 names but
cover every joint of the robot's ``ACTION_JOINT_NAMES`` (for ``custom_biped``: 8 leg joints + 2 tail joints)."""

from __future__ import annotations

import math

import torch

from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sensors import ContactSensor

from .. import interface as itf
from .. import interface_v2 as itf2
from ..metrics import WalkTracker
from ..robots import get_robot
from .biped_env_cfg import BipedWalkEnvCfg


def _activate_nested_contact_reporters(robot_path: str) -> None:
    """Apply the PhysX contact-report API to every rigid body under ``robot_path``.

    ``activate_contact_sensors=True`` only reaches the outermost rigid body because Isaac Lab stops
    descending at the first rigid body, while the Isaac Sim 6 URDF importer nests every link
    under its parent link. Must run on the source env before cloning.
    """
    import isaaclab.sim as sim_utils
    from pxr import Usd, UsdPhysics

    stage = sim_utils.get_current_stage()
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_path)):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            sim_utils.schemas.activate_contact_sensors(str(prim.GetPath()), 0.0, stage)


def _configure_startup_randomisation(cfg: BipedWalkEnvCfg, rb) -> None:
    """Translate ``cfg.dr`` startup items (friction, mass, CoM) into Isaac Lab event terms. No-op when all are off."""
    dr = cfg.dr
    if dr.rough_terrain is not None:
        _configure_rough_terrain(cfg, dr.rough_terrain)
    if dr.foot_friction is None and dr.mass_scale is None and dr.base_com_offset is None:
        return
    from isaaclab.envs import mdp
    from isaaclab.managers import EventTermCfg, SceneEntityCfg

    events = dict(cfg.events) if isinstance(cfg.events, dict) else {}
    if dr.foot_friction is not None:
        lo, hi = dr.foot_friction
        events["dr_foot_friction"] = EventTermCfg(
            func=mdp.randomize_rigid_body_material, mode="startup",
            params={"asset_cfg": SceneEntityCfg("robot", body_names=list(rb.FOOT_BODY_NAMES)),
                    "static_friction_range": (lo, hi), "dynamic_friction_range": (lo, hi),
                    "restitution_range": (0.0, 0.0), "num_buckets": 64, "make_consistent": True})
        # the effective friction is the product with the ground's 1.0 (multiply has the highest combine priority)
        cfg.terrain.physics_material.friction_combine_mode = "multiply"
    if dr.mass_scale is not None:
        events["dr_mass"] = EventTermCfg(
            func=mdp.randomize_rigid_body_mass, mode="startup",
            params={"asset_cfg": SceneEntityCfg("robot", body_names=".*"), "mass_distribution_params": tuple(dr.mass_scale),
                    "operation": "scale", "recompute_inertia": True})
    if dr.base_com_offset is not None:
        r = dr.base_com_offset
        events["dr_base_com"] = EventTermCfg(
            func=mdp.randomize_rigid_body_com, mode="startup",
            params={"asset_cfg": SceneEntityCfg("robot", body_names=rb.BASE_BODY_NAME),
                    "com_range": {"x": (-r, r), "y": (-r, r), "z": (-r, r)}})
    cfg.events = events


def _configure_rough_terrain(cfg: BipedWalkEnvCfg, rt: dict) -> None:
    """Generated uneven ground (``shared/env/terrain.py``): ``levels`` rows of increasing bump height x ``cols``."""
    from isaaclab.terrains import TerrainGeneratorCfg, TerrainImporterCfg

    from .terrain import GradedBumpsCfg

    gen = TerrainGeneratorCfg(
        size=tuple(rt["tile_size"]), border_width=1.0, num_rows=int(rt["levels"]), num_cols=int(rt["cols"]),
        horizontal_scale=rt["horizontal_scale"], vertical_scale=0.001, slope_threshold=None, curriculum=True,
        use_cache=False,
        sub_terrains={"bumps": GradedBumpsCfg(proportion=1.0, amplitude_range=(0.0, float(rt["amplitude"])),
                                              horizontal_scale=rt["horizontal_scale"], vertical_scale=0.001)},
    )
    cfg.terrain = TerrainImporterCfg(
        prim_path="/World/ground", terrain_type="generator", terrain_generator=gen,
        max_init_terrain_level=int(rt.get("init_level_max", 0)), collision_group=-1,
        physics_material=cfg.terrain.physics_material, debug_vis=False,
    )


class BipedWalkEnv(DirectRLEnv):
    cfg: BipedWalkEnvCfg

    def __init__(self, cfg: BipedWalkEnvCfg, render_mode: str | None = None, **kwargs):
        self.rb = get_robot(cfg.robot_name)
        _configure_startup_randomisation(cfg, self.rb)
        if cfg.obs_version == 2:
            cfg.observation_space = itf2.LayoutV2(len(self.rb.ACTION_JOINT_NAMES)).teacher_dim
        super().__init__(cfg, render_mode, **kwargs)
        dev = self.device
        n = self.num_envs

        # Robots stand side by side in rows (world Y), all walking toward +X; rows are far apart so
        # nobody walks into the next row. Replaces the cloner's square grid for placement/metrics.
        idx = torch.arange(n, device=dev)
        self.spawn_origins = torch.zeros(n, 3, device=dev)
        rt = self.cfg.dr.rough_terrain
        if rt is None:
            per_row = min(n, self.cfg.robots_per_row)
            self.spawn_origins[:, 0] = (idx // per_row).float() * self.cfg.row_spacing
            self.spawn_origins[:, 1] = ((idx % per_row).float() - (per_row - 1) / 2) * self.cfg.lateral_spacing
            self.terrain_level = None
        else:
            # tiles: one column per group of robots_per_tile robots; level = tile row (bump height)
            per = int(rt["robots_per_tile"])
            origins = self.terrain.terrain_origins  # [levels, cols, 3]
            self.terrain_col = (idx // per) % origins.shape[1]
            self.terrain_slot = idx % per
            if "fixed_level" in rt:  # evaluation: every robot on one difficulty level, no curriculum
                self.terrain_level = torch.full((n,), int(rt["fixed_level"]), dtype=torch.long, device=dev)
            else:
                self.terrain_level = torch.randint(0, int(rt.get("init_level_max", 0)) + 1, (n,), device=dev)
            self._update_terrain_spawn(idx)

        self.leg_ids, _ = self.robot.find_joints(list(self.rb.ACTION_JOINT_NAMES), preserve_order=True)
        self.leg_ids_t = torch.tensor(self.leg_ids, device=dev)
        self.num_legs = len(self.leg_ids)
        self.base_body_id = self.robot.find_bodies(self.rb.BASE_BODY_NAME)[0][0]
        self.layout = itf.ObsLayout(self.num_legs)

        self.default_leg_pos = self.robot.data.default_joint_pos.torch[:, self.leg_ids].clone()
        limits = self.robot.data.soft_joint_pos_limits.torch[:, self.leg_ids]
        self.soft_lower, self.soft_upper = limits[..., 0], limits[..., 1]

        self.actions = torch.zeros(n, self.num_legs, device=dev)
        self.prev_actions = torch.zeros_like(self.actions)
        self.q_target = self.default_leg_pos.clone()
        self.cmd_vx = torch.zeros(n, device=dev)
        self.gait_freq = torch.ones(n, device=dev)
        self.phase0 = torch.zeros(n, device=dev)
        self.target_heading = torch.zeros(n, device=dev)
        self.fell = torch.zeros(n, dtype=torch.bool, device=dev)
        self.tracker = WalkTracker(n, self.num_legs, dev)
        n_feet = len(self.foot_sensors)
        self.first_contact = torch.zeros(n, n_feet, dtype=torch.bool, device=dev)
        self._prev_contact = torch.ones(n, n_feet, dtype=torch.bool, device=dev)
        self._air_time = torch.zeros(n, n_feet, device=dev)
        self._last_air_time = torch.zeros(n, n_feet, device=dev)
        self._next_push = torch.full((n,), float("inf"), device=dev)
        self.completed_episodes: list[dict[str, torch.Tensor]] = []
        self._init_gait_variation()
        self._init_runtime_randomisation()
        self._episode_sums = {k: torch.zeros(n, device=dev) for k in self.cfg.rewards.to_dict()}
        self._compute_state()

    # ------------------------------------------------------------------ scene
    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot)
        _activate_nested_contact_reporters("/World/envs/env_0/Robot")
        self.foot_sensors = [ContactSensor(c) for c in self.cfg.foot_sensors]
        self.cfg.terrain.num_envs = self.scene.cfg.num_envs
        self.cfg.terrain.env_spacing = self.scene.cfg.env_spacing
        self.terrain = self.cfg.terrain.class_type(self.cfg.terrain)
        self.scene.clone_environments(copy_from_source=False)
        if self.cfg.dr.rough_terrain is not None:
            # several robots share a tile: disable robot-robot collisions across envs (kept off on flat_v1 for
            # bit-identical 03 results; there the robots stand 1 m apart and never touch)
            self.scene.filter_collisions(global_prim_paths=[self.cfg.terrain.prim_path])
        self.scene.articulations["robot"] = self.robot
        for i, sensor in enumerate(self.foot_sensors):
            self.scene.sensors[f"foot_contact_{i}"] = sensor
        import isaaclab.sim as sim_utils

        light = sim_utils.DomeLightCfg(intensity=2500.0, color=(0.8, 0.8, 0.8))
        light.func("/World/Light", light)

        self.eval_cam = None
        if self.cfg.eval_camera:
            from isaaclab.sensors.camera import Camera, CameraCfg

            w, h = self.cfg.viewer.resolution
            self.eval_cam = Camera(CameraCfg(
                prim_path="/World/EvalCamera", update_period=0, width=w, height=h, data_types=["rgb"],
                spawn=sim_utils.PinholeCameraCfg(focal_length=14.0, clipping_range=(0.05, 100.0)),
            ))

    # ------------------------------------------------------------------ helpers
    @property
    def gait_phase(self) -> torch.Tensor:
        """Gait clock phase in [0, 1) (stateless: derived from the episode time)."""
        t = self.episode_length_buf.float() * self.step_dt
        if self._phase_acc is not None:  # gait_var.resample_interval_s: integrated clock (frequency may change)
            return torch.remainder(self.phase0 + self._phase_acc, 1.0)
        return torch.remainder(self.phase0 + self.gait_freq * t, 1.0)

    @property
    def step_length_cmd(self) -> torch.Tensor:
        """Nominal step length implied by the command [m] (two steps per gait cycle)."""
        return self.cmd_vx / (2.0 * self.gait_freq.clamp_min(1e-3))  # standing: freq 0 -> step length 0

    def _update_touchdowns(self) -> torch.Tensor:
        """Detect foot touchdowns during the last control step, shape [N, F] (bool). Call once per step.

        A touchdown is an air -> contact transition of the foot after at least ``cfg.min_step_air_time``
        in the air (filters contact chatter when the sole rocks on an edge). Tracked here instead of
        ``ContactSensor.compute_first_contact``: the sensor samples contact lazily (once per control
        step) and its ``contact_time < dt`` test sits exactly on the float32 boundary, so it misses
        most touchdowns.
        """
        contact = self.foot_in_contact()
        landing = contact & ~self._prev_contact
        touchdown = landing & (self._air_time >= self.cfg.min_step_air_time - 1e-6)
        self._last_air_time = torch.where(landing, self._air_time, self._last_air_time)
        self._air_time = torch.where(contact, torch.zeros_like(self._air_time), self._air_time + self.step_dt)
        self._prev_contact = contact
        return touchdown

    def foot_last_air_time(self) -> torch.Tensor:
        """Duration of the last swing of each foot [s], shape [N, F] (same contact tracking as the step count,
        sampled once per control step)."""
        return self._last_air_time

    def foot_in_contact(self) -> torch.Tensor:
        """Current contact state of each foot, shape [N, F] (bool)."""
        return torch.cat([s.data.current_contact_time.torch[:, :1] > 0.0 for s in self.foot_sensors], dim=-1)

    def foot_forces(self) -> torch.Tensor:
        """Net contact force on each foot, world frame [N], shape [N, F, 3]."""
        return torch.cat([s.data.net_forces_w.torch[:, :1] for s in self.foot_sensors], dim=1)

    def capture_frame(self, env_id: int = 0):
        """Render robot ``env_id`` from the follow camera; returns an RGB uint8 array (H, W, 3).

        Requires ``cfg.eval_camera = True`` and ``--enable_cameras``.
        """
        if self.eval_cam is None:
            raise RuntimeError("capture_frame() needs cfg.eval_camera=True and --enable_cameras")
        h = float(self.target_heading[env_id])
        ox, oy, oz = self.cfg.follow_cam_offset
        com = self.com_pos[env_id] + self.spawn_origins[env_id]
        eye = torch.tensor([[
            float(com[0]) + ox * math.cos(h) - oy * math.sin(h),
            float(com[1]) + ox * math.sin(h) + oy * math.cos(h),
            float(com[2]) + oz,
        ]], device=self.device)
        target = torch.tensor([[float(com[0]), float(com[1]), float(com[2]) - self.cfg.follow_cam_target_drop]],
                              device=self.device)
        self.eval_cam.set_world_poses_from_view(eye, target)
        self.sim.render()
        self.eval_cam.update(dt=0.0, force_recompute=True)
        return self.eval_cam.data.output["rgb"][0, ..., :3].cpu().numpy()

    def _compute_state(self):
        d = self.robot.data
        yaw0 = self.rb.INIT_YAW
        self.base_lin_vel = itf.root_to_canonical(d.root_com_lin_vel_b.torch, yaw0)
        self.base_ang_vel = itf.root_to_canonical(d.root_ang_vel_b.torch, yaw0)
        self.projected_gravity = itf.root_to_canonical(d.projected_gravity_b.torch, yaw0)
        self.heading = itf.wrap_angle(d.heading_w.torch - yaw0)
        self.heading_error = itf.wrap_angle(self.target_heading - self.heading)
        self.com_pos = d.root_com_pos_w.torch - self.spawn_origins
        self.leg_pos = d.joint_pos.torch[:, self.leg_ids]
        self.leg_vel = d.joint_vel.torch[:, self.leg_ids]
        self.leg_torque = d.applied_torque.torch[:, self.leg_ids]
        # the actuator model clipped the PD torque (DC motor torque-speed curve or continuous limit)
        self.leg_saturated = (d.computed_torque.torch[:, self.leg_ids] - self.leg_torque).abs() > 1e-4

    # ------------------------------------------------------------------ step
    def _apply_pushes(self):
        """Random horizontal velocity kicks every ``cfg.push_interval_s`` seconds (training only)."""
        if self.cfg.push_interval_s is None:
            return
        t = self.episode_length_buf.float() * self.step_dt
        due = t >= self._next_push
        if not bool(due.any()):
            return
        ids = due.nonzero().flatten()
        vel = self.robot.data.root_vel_w.torch[ids].clone()
        ang = torch.rand(len(ids), device=self.device) * 2 * math.pi
        vel[:, 0] += self.cfg.push_speed * torch.cos(ang)
        vel[:, 1] += self.cfg.push_speed * torch.sin(ang)
        self.robot.write_root_velocity_to_sim_index(root_velocity=vel, env_ids=ids)
        self._next_push[ids] = t[ids] + torch.empty(len(ids), device=self.device).uniform_(*self.cfg.push_interval_s)

    def _pre_physics_step(self, actions: torch.Tensor):
        self._apply_pushes()
        self.prev_actions[:] = self.actions
        self.actions[:] = actions.clamp(-itf.ACTION_CLIP, itf.ACTION_CLIP)
        self.q_target[:] = itf.action_to_joint_targets(self.actions, self.default_leg_pos)

    def _apply_action(self):
        target = self.q_target
        if self._delay_hist is not None:  # dr.action_delay_phys_steps: called once per physics step
            self._delay_hist = torch.roll(self._delay_hist, 1, dims=0)
            self._delay_hist[0] = self.q_target
            target = self._delay_hist[self._delay, torch.arange(self.num_envs, device=self.device)]
        self.robot.set_joint_position_target_index(target=target, joint_ids=self.leg_ids)

    def _get_observations(self) -> dict:
        self._compute_state()
        obs = itf.build_observation(
            base_lin_vel=self.base_lin_vel,
            base_ang_vel=self.base_ang_vel,
            projected_gravity=self.projected_gravity,
            cmd_vx=self.cmd_vx,
            gait_freq=self.gait_freq,
            heading_error=self.heading_error,
            joint_pos_rel=self.leg_pos - self.default_leg_pos,
            joint_vel=self.leg_vel,
            last_action=self.actions,
            gait_phase=self.gait_phase,
        )
        if self.cfg.obs_version == 2:
            return self._observations_v2(obs)
        if self._obs_noise is not None:
            obs = obs + self._obs_noise * torch.randn_like(obs)
        if self._heading_bias is not None:
            obs[:, self.layout.slices()["heading_error"]] += self._heading_bias.unsqueeze(-1)
        return {"policy": obs}

    def _observations_v2(self, clean: torch.Tensor) -> dict:
        """``shared/interface_v2.py``: privileged teacher (true state + PRIV_FIELDS) and the noisy student history."""
        noisy = clean if self._obs_noise is None else clean + self._obs_noise * torch.randn_like(clean)
        if self._heading_bias is not None:
            noisy = noisy.clone()
            noisy[:, self.layout.slices()["heading_error"]] += self._heading_bias.unsqueeze(-1)
        frame = itf2.student_frame(noisy, self.clearance_cmd)
        fresh = self.episode_length_buf == 0
        self._hist = torch.roll(self._hist, 1, dims=1)
        self._hist[:, 0] = frame
        self._hist[fresh] = frame[fresh].unsqueeze(1)
        n = self.num_envs
        amp = torch.zeros(n, device=self.device)
        if self.terrain_level is not None:
            rt = self.cfg.dr.rough_terrain
            amp = float(rt["amplitude"]) * self.terrain_level.float() / max(1, int(rt["levels"]) - 1)
        delay = (self._delay.float() * self.physics_dt) if self._delay is not None else torch.zeros(n, device=self.device)
        priv = torch.cat((self._foot_friction.unsqueeze(-1), self._mass_scale.unsqueeze(-1), self._gain_scale,
                          delay.unsqueeze(-1), amp.unsqueeze(-1), self.com_pos[:, 2:3],
                          self.foot_in_contact().float(), self.clearance_cmd.unsqueeze(-1)), dim=-1)
        teacher = torch.cat((clean, priv), dim=-1)
        self.obs_v2 = {"policy": teacher, "critic": teacher, "student": self._hist.reshape(n, -1)}
        return self.obs_v2  # kept for agents that act on the student group (reset() does not refresh obs_buf)

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        self._compute_state()
        tilt_limit = -math.cos(math.radians(self.cfg.fall_tilt_deg))
        self.fell = (self.com_pos[:, 2] < self.cfg.fall_com_height) | (self.projected_gravity[:, 2] > tilt_limit)
        time_out = self.episode_length_buf >= self.max_episode_length - 1

        self.first_contact = self._update_touchdowns()
        # the initial landing after spawn (soles start a few mm above ground) is not a step
        settled = (self.episode_length_buf.float() * self.step_dt > self.cfg.settle_time).float()
        torque_abs = self.leg_torque.abs()
        self.tracker.update(
            com_xy=self.com_pos[:, :2],
            touchdowns=self.first_contact.float().sum(-1) * settled,
            heading_error=self.heading_error,
            torque_abs=torque_abs,
            torque_saturated=self.leg_saturated.any(-1),
        )
        if self._gait_var_on:
            self._step_gait_variation()
        return self.fell, time_out

    def _gait_contact_match(self) -> torch.Tensor:
        """Fraction of feet whose contact state matches the gait-clock schedule, shape [N]."""
        phase = self.gait_phase.unsqueeze(1) + torch.tensor([0.0, 0.5], device=self.device)
        in_swing = torch.remainder(phase - self.cfg.gait_swing_start, 1.0) < self.cfg.gait_swing_frac
        if self._gait_var_on:
            in_swing = in_swing & (self.gait_freq > 0).unsqueeze(1)
        return (in_swing != self.foot_in_contact()).float().mean(-1)

    def _get_rewards(self) -> torch.Tensor:
        c, w = self.cfg, self.cfg.rewards
        v = self.base_lin_vel
        air = self.foot_last_air_time()
        forces = self.foot_forces().norm(dim=-1)
        terms = {
            "track_vx": torch.exp(-torch.square((v[:, 0] - self.cmd_vx) / c.sigma_vx)),
            "track_heading": torch.exp(-torch.square(self.heading_error / c.sigma_heading)),
            "lateral_vel": torch.square(v[:, 1]),
            "lateral_deviation": torch.square(self.tracker.lateral),
            "orientation": torch.square(self.projected_gravity[:, :2]).sum(-1),
            "base_height": torch.square(self.com_pos[:, 2] - self.rb.NOMINAL_COM_HEIGHT),
            "alive": torch.ones_like(self.cmd_vx),
            "termination": self.fell.float(),
            "torque": torch.square(self.leg_torque).sum(-1),
            "action_rate": torch.square(self.actions - self.prev_actions).sum(-1),
            "joint_limits": ((self.soft_lower - self.leg_pos).clamp_min(0) + (self.leg_pos - self.soft_upper).clamp_min(0)).sum(-1),
            "feet_air_time": ((air - c.target_air_time) * self.first_contact.float()).sum(-1),
            "contact_force": (forces - c.contact_force_limit).clamp_min(0).sum(-1),
            "gait_contact": self._gait_contact_match(),
        }
        if self._clearance_on:
            terms["clearance"] = self._clearance_reward()
        reward = torch.zeros_like(self.cmd_vx)
        for k, val in terms.items():
            r = getattr(w, k) * val * self.step_dt
            self._episode_sums[k] += r
            reward += r
        return reward

    # ------------------------------------------------------------------ gait diversity (04_sim2real phase 3)
    def _init_gait_variation(self):
        gv, n, dev = self.cfg.gait_var, self.num_envs, self.device
        self._gait_var_on = (gv.resample_interval_s is not None or gv.zero_speed_prob > 0 or gv.stand_prob > 0
                             or gv.clearance_range is not None)
        self._clearance_on = gv.clearance_range is not None
        self._phase_acc = torch.zeros(n, device=dev) if gv.resample_interval_s is not None else None
        self._next_resample = torch.full((n,), float("inf"), device=dev)
        self.clearance_cmd = torch.zeros(n, device=dev)
        self._swing_apex = torch.zeros(n, 2, device=dev)
        self._foot_body_ids = [self.robot.find_bodies(nm)[0][0] for nm in self.rb.FOOT_BODY_NAMES]

    def _sample_commands(self, env_ids: torch.Tensor):
        """Speed, gait frequency (and clearance) for ``env_ids``: uniform in the env ranges, then stepping in place
        (speed 0) with ``zero_speed_prob`` and standing (speed 0, frequency 0) with ``stand_prob``."""
        gv, k, dev = self.cfg.gait_var, len(env_ids), self.device
        cmd = torch.empty(k, device=dev).uniform_(*self.cfg.cmd_vx_range)
        freq = torch.empty(k, device=dev).uniform_(*self.cfg.gait_freq_range)
        u = torch.rand(k, device=dev)
        stand = u < gv.stand_prob
        zero = (u >= gv.stand_prob) & (u < gv.stand_prob + gv.zero_speed_prob)
        cmd = torch.where(stand | zero, torch.zeros_like(cmd), cmd)
        freq = torch.where(stand, torch.zeros_like(freq), freq)
        self.cmd_vx[env_ids] = cmd
        self.gait_freq[env_ids] = freq
        if gv.clearance_range is not None:
            self.clearance_cmd[env_ids] = torch.empty(k, device=dev).uniform_(*gv.clearance_range)
        if gv.resample_interval_s is not None:
            t = self.episode_length_buf[env_ids].float() * self.step_dt
            self._next_resample[env_ids] = t + torch.empty(k, device=dev).uniform_(*gv.resample_interval_s)

    def _step_gait_variation(self):
        """Once per control step (from ``_get_dones``): advance the clock, resample due commands, track swing apexes."""
        if self._phase_acc is not None:
            self._phase_acc += self.gait_freq * self.step_dt
            t = self.episode_length_buf.float() * self.step_dt
            due = t >= self._next_resample
            if bool(due.any()):
                self._sample_commands(due.nonzero().flatten())
        if self._clearance_on:
            z = self.robot.data.body_pos_w.torch[:, self._foot_body_ids, 2]
            rel = z - z.flip(-1)  # each foot relative to the other one
            air = ~self.foot_in_contact()
            self._swing_apex = torch.where(air, torch.maximum(self._swing_apex, rel), self._swing_apex)

    def _clearance_reward(self) -> torch.Tensor:
        """Sum over touchdowns of exp(-((apex - cmd) / sigma)^2); the apex buffer restarts after each touchdown."""
        err = (self._swing_apex - self.clearance_cmd.unsqueeze(-1)) / self.cfg.gait_var.sigma_clearance
        r = (torch.exp(-err * err) * self.first_contact.float()).sum(-1)
        self._swing_apex = torch.where(self.first_contact, torch.zeros_like(self._swing_apex), self._swing_apex)
        return r

    # ------------------------------------------------------------------ sim-to-real randomisation (04_sim2real)
    def _init_runtime_randomisation(self):
        """Buffers for the per-episode items of ``cfg.dr``; everything stays None when they are off."""
        dr, n, dev = self.cfg.dr, self.num_envs, self.device
        self._delay_hist = self._delay = None
        if dr.action_delay_phys_steps is not None:
            d_max = int(dr.action_delay_phys_steps[1])
            self._delay_hist = self.q_target.unsqueeze(0).repeat(d_max + 1, 1, 1)
            self._delay = torch.zeros(n, dtype=torch.long, device=dev)
        self._actuators = list(self.robot.actuators.values())
        self._act_defaults = [(a.stiffness.clone(), a.damping.clone(), a.effort_limit.clone()) for a in self._actuators]
        self._obs_noise = None
        noise = {"base_ang_vel": dr.obs_noise_ang_vel, "projected_gravity": dr.obs_noise_gravity,
                 "joint_pos_rel": dr.obs_noise_joint_pos, "joint_vel": dr.obs_noise_joint_vel}
        if any(v is not None for v in noise.values()):
            self._obs_noise = torch.zeros(self.layout.dim, device=dev)
            for k, v in noise.items():
                if v is not None:
                    self._obs_noise[self.layout.slices()[k]] = v
        self._heading_bias = torch.zeros(n, device=dev) if dr.heading_bias is not None else None
        # privileged values (interface v2); scales are 1 unless randomised
        self._gain_scale = torch.ones(n, 3, device=dev)  # kp, kd, effort
        if self.cfg.obs_version == 2:
            self.layout2 = itf2.LayoutV2(self.num_legs)
            self._hist = torch.zeros(n, self.layout2.history, self.layout2.frame_dim, device=dev)
            self._mass_scale = self.robot.data.body_mass.torch.sum(-1) / float(self.rb.TOTAL_MASS)
            self._foot_friction = self._read_foot_friction()

    def _read_foot_friction(self) -> torch.Tensor:
        """Mean static friction of the foot shapes per env (after the startup material randomisation)."""
        import warp as wp

        view = self.robot.root_view
        shapes = [self.robot._physics_sim_view.create_rigid_body_view(lp).max_shapes for lp in view.link_paths[0]]
        mats = wp.to_torch(view.get_material_properties())  # [N, shapes, 3] (static, dynamic, restitution), CPU
        foot_ids = [self.robot.find_bodies(nm)[0][0] for nm in self.rb.FOOT_BODY_NAMES]
        vals = [mats[:, sum(shapes[:b]):sum(shapes[:b]) + shapes[b], 0] for b in foot_ids]
        return torch.cat(vals, dim=-1).mean(-1).to(self.device)

    def _update_terrain_spawn(self, env_ids: torch.Tensor):
        """Spawn point of each robot on its tile: back of the tile, side by side across it, above the bumps."""
        rt = self.cfg.dr.rough_terrain
        sx, sy = rt["tile_size"]
        per = int(rt["robots_per_tile"])
        o = self.terrain.terrain_origins[self.terrain_level[env_ids], self.terrain_col[env_ids]]
        self.spawn_origins[env_ids, 0] = o[:, 0] - sx / 2 + 0.5
        self.spawn_origins[env_ids, 1] = o[:, 1] + (self.terrain_slot[env_ids].float() - (per - 1) / 2) * (sy / per)
        self.spawn_origins[env_ids, 2] = o[:, 2] + float(rt["amplitude"])

    def _reset_runtime_randomisation(self, env_ids: torch.Tensor):
        dr, k, dev = self.cfg.dr, len(env_ids), self.device

        def uniform(lo_hi):
            return torch.empty(k, device=dev).uniform_(*lo_hi)

        if self._delay is not None:
            lo, hi = dr.action_delay_phys_steps
            self._delay[env_ids] = torch.randint(int(lo), int(hi) + 1, (k,), device=dev)
            self._delay_hist[:, env_ids] = self.q_target[env_ids].unsqueeze(0)
        for name, scale in (("stiffness", dr.kp_scale), ("damping", dr.kd_scale), ("effort_limit", dr.effort_scale)):
            if scale is None:
                continue
            s = uniform(scale).unsqueeze(-1)
            idx = ("stiffness", "damping", "effort_limit").index(name)
            for act, defaults in zip(self._actuators, self._act_defaults):
                getattr(act, name)[env_ids] = defaults[idx][env_ids] * s
            self._gain_scale[env_ids, idx] = s.squeeze(-1)
        if self._heading_bias is not None:
            self._heading_bias[env_ids] = uniform((-dr.heading_bias, dr.heading_bias))

    # ------------------------------------------------------------------ reset
    def _reset_idx(self, env_ids: torch.Tensor | None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        env_ids = torch.as_tensor(env_ids, device=self.device)

        ran = self.episode_length_buf[env_ids] > 0
        if ran.any():
            done_ids = env_ids[ran]
            summary = self.tracker.summary(
                done_ids,
                time_s=self.episode_length_buf[done_ids].float() * self.step_dt,
                cmd_vx=self.cmd_vx[done_ids],
                step_length_cmd=self.step_length_cmd[done_ids],
                fell=self.fell[done_ids],
            )
            if self.cfg.record_episodes:
                summary["env_ids"] = done_ids.clone()
                summary["gait_freq"] = self.gait_freq[done_ids].clone()
                for k, v in self._episode_sums.items():  # per-term reward sums of the episode
                    summary[f"rew_{k}"] = v[done_ids].clone()
                self.completed_episodes.append(summary)
            log = {f"Episode_Reward/{k}": (v[done_ids].mean() / self.max_episode_length_s).item()
                   for k, v in self._episode_sums.items()}
            log.update({
                "Metrics/fall_rate": summary["fell"].mean().item(),
                "Metrics/steps": summary["steps"].mean().item(),
                "Metrics/distance": summary["distance"].mean().item(),
                "Metrics/max_abs_lateral": summary["max_abs_lateral"].mean().item(),
                "Metrics/torque_sat_frac": summary["torque_sat_frac"].mean().item(),
            })
            self.extras["log"] = log
        for v in self._episode_sums.values():
            v[env_ids] = 0.0

        if self.terrain_level is not None and bool(ran.any()) and self.cfg.dr.rough_terrain.get("curriculum", True)                 and "fixed_level" not in self.cfg.dr.rough_terrain:
            done_ids = env_ids[ran]
            t = self.episode_length_buf[done_ids].float() * self.step_dt
            walked = self.tracker.distance[done_ids] >= 0.8 * self.cmd_vx[done_ids] * t
            full = t >= self.max_episode_length_s - 2 * self.step_dt
            up = walked & full & ~self.fell[done_ids]
            down = self.fell[done_ids]
            top = self.terrain.terrain_origins.shape[0] - 1
            self.terrain_level[done_ids] = (self.terrain_level[done_ids] + up.long() - down.long()).clamp(0, top)
            self._update_terrain_spawn(env_ids)
        self.robot.reset(env_ids)
        super()._reset_idx(env_ids)
        n = len(env_ids)
        dev = self.device

        def uniform(lo_hi):
            return torch.empty(n, device=dev).uniform_(*lo_hi)

        if self._gait_var_on:
            self._sample_commands(env_ids)
            self._phase_acc[env_ids] = 0.0
            self._swing_apex[env_ids] = 0.0
        else:
            self.cmd_vx[env_ids] = uniform(self.cfg.cmd_vx_range)
            self.gait_freq[env_ids] = uniform(self.cfg.gait_freq_range)
        self.phase0[env_ids] = torch.rand(n, device=dev) if self.cfg.init_phase_random else 0.0
        yaw = uniform(self.cfg.init_yaw_range)
        self.target_heading[env_ids] = itf.wrap_angle(yaw + uniform(self.cfg.heading_offset_range))
        self.actions[env_ids] = 0.0
        self.prev_actions[env_ids] = 0.0
        self.q_target[env_ids] = self.default_leg_pos[env_ids]
        self.fell[env_ids] = False
        self._prev_contact[env_ids] = False  # the initial landing is masked by cfg.settle_time
        self._air_time[env_ids] = 0.0
        self._last_air_time[env_ids] = 0.0
        if self.cfg.push_interval_s is not None:
            self._next_push[env_ids] = torch.empty(n, device=dev).uniform_(*self.cfg.push_interval_s)

        root_pose = self.robot.data.default_root_pose.torch[env_ids].clone()
        root_pose[:, :3] += self.spawn_origins[env_ids]
        full_yaw = yaw + self.rb.INIT_YAW  # both rotations are about z
        root_pose[:, 3:7] = torch.stack(
            (torch.zeros_like(yaw), torch.zeros_like(yaw), torch.sin(full_yaw / 2), torch.cos(full_yaw / 2)), dim=-1
        )  # XYZW
        root_vel = torch.zeros_like(self.robot.data.default_root_vel.torch[env_ids])
        joint_pos = self.robot.data.default_joint_pos.torch[env_ids].clone()
        noise = torch.empty(n, self.num_legs, device=dev).uniform_(-self.cfg.init_joint_noise, self.cfg.init_joint_noise)
        joint_pos[:, self.leg_ids] += noise
        joint_vel = torch.zeros_like(joint_pos)
        self.robot.write_root_pose_to_sim_index(root_pose=root_pose, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=root_vel, env_ids=env_ids)
        self.robot.write_joint_position_to_sim_index(position=joint_pos, env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=joint_vel, env_ids=env_ids)

        start_xy = root_pose[:, :2] - self.spawn_origins[env_ids, :2]
        self.tracker.reset(env_ids, start_xy, self.target_heading[env_ids])
        self._reset_runtime_randomisation(env_ids)
