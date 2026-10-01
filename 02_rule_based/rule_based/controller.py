"""Rule-based walking controller (teacher policy) for the shared straight-walking interface.

Only the observation vector of ``shared/interface.py`` and the robot module's geometry are used, so a
learned policy sees exactly the same information. Internal state (all per environment): the time since
reset (ramp-up) and an integrated lateral velocity (drift estimate).

Method (batched over environments, canonical frame x forward / y left / z up):

1. Gait timing from the clock phase and gait frequency in the observation; each leg swings once per cycle
   (left leg phase = clock phase, right = clock phase + 0.5).
2. Sagittal foot trajectory relative to the hip pitch axis: the stance foot moves back linearly at
   the body speed, over D = (cmd_vx / f) * (1 - swing_frac); the swing foot moves forward to the
   Raibert landing point on a cosine profile (after an optional lift-only delay) with a sine lift.
   The hip height follows the legs as inverted pendulums (lowest sqrt(r^2 - x^2) of both legs).
   Step length = cmd_vx / (2 f).
3. Two-link IK gives hip pitch and knee; the ankle keeps the sole parallel to the ground.
4. Lateral sway: both hip rolls follow the same sinusoid (parallelogram), shifting the pelvis over
   the stance foot; the swing leg adds abduction and lateral foot-placement feedback.
5. Balance feedback: stance hip pitch/roll posture servo, swing leg tilt compensation, sagittal and
   lateral Raibert foot placement, lateral drift correction, differential step length for heading.
6. Tail (experiment 2, only if the robot has ``tail_yaw`` / ``tail_pitch``): tail yaw follows the sway
   waveform toward the stance side (moves the whole-body CoM over the stance foot) plus a roll posture
   term; tail pitch holds an offset plus an optional pitch posture term.

Ported from ``00_test/rule_based/controller.py``; the joint vector is the robot's ``ACTION_JOINT_NAMES``.
"""

from __future__ import annotations

import math

import torch

from shared import interface as itf

from .config import GaitParams, RuleBasedCfg
from .leg_kinematics import SagittalLegIK

SIDES = ("left", "right")
ROLES = ("hip_roll", "hip_pitch", "knee", "ankle_pitch")


class RuleBasedBipedController:
    """Batched rule-based controller: ``reset(env_ids)`` and ``compute_action(obs) -> action``."""

    def __init__(self, rb, num_envs: int, device: str | torch.device, cfg: RuleBasedCfg | None = None):
        self.rb = rb
        self.cfg = (cfg or RuleBasedCfg()).resolve(rb)
        self.num_envs = num_envs
        self.device = torch.device(device)
        self.dt = itf.CONTROL_DT
        self.layout = itf.ObsLayout(len(rb.ACTION_JOINT_NAMES))
        self.ik = SagittalLegIK(rb, self.cfg.knee_margin, self.cfg.min_knee)

        # joint bookkeeping: index[side][role] into the action vector; sign/limits/default per joint
        names = list(rb.ACTION_JOINT_NAMES)
        leg_names, layout = list(rb.LEG_JOINT_NAMES), list(rb.LEG_JOINT_LAYOUT)
        self.index = torch.tensor([[names.index(leg_names[layout.index((s, r))]) for r in ROLES] for s in SIDES],
                                  device=self.device)
        tail_limits = getattr(rb, "TAIL_JOINT_LIMITS", {})
        role = {leg_names[i]: r for i, (_, r) in enumerate(layout)}
        lim = [rb.LEG_JOINT_LIMITS[role[n]] if n in role else tail_limits[n] for n in names]
        self.sign = torch.tensor([rb.JOINT_AXIS_SIGN[role[n]] if n in role else 1.0 for n in names], device=self.device)
        margin = 0.98
        self.q_lo = torch.tensor([lo * margin for lo, _ in lim], device=self.device)
        self.q_hi = torch.tensor([hi * margin for _, hi in lim], device=self.device)
        self.q_default = torch.tensor([rb.DEFAULT_JOINT_POS[n] for n in names], device=self.device)
        # tail joints (None if the robot has none); tail_yaw + = tail tip to the right, tail_pitch + = tip up
        self.tail_yaw = names.index("tail_yaw") if "tail_yaw" in names else None
        self.tail_pitch = names.index("tail_pitch") if "tail_pitch" in names else None
        self.side_sign = torch.tensor([1.0, -1.0], device=self.device)  # +y is outward for the left leg
        self.leg_phase_offset = torch.tensor([0.0, 0.5], device=self.device)
        # lever arm hip roll axis -> sole, used to turn lateral foot offsets into hip roll angles
        self.roll_lever = float(-rb.HIP_ROLL_TO_PITCH[2] + self.cfg.gait.stance_leg_length + rb.ANKLE_HEIGHT)

        z = lambda: torch.zeros(num_envs, device=self.device)  # noqa: E731
        self.p: dict[str, torch.Tensor] = {k: z() for k in GaitParams.__dataclass_fields__}
        self._overrides: dict[str, torch.Tensor] = {}
        self.time = z()
        self.lat_est = z()
        self.debug: dict[str, torch.Tensor] = {}
        self.reset()

    # ------------------------------------------------------------------ parameters
    def override(self, name: str, values: torch.Tensor | float) -> None:
        """Pin a gait parameter per environment (kept across resets). Used by parameter sweeps."""
        v = torch.as_tensor(values, dtype=torch.float32, device=self.device).expand(self.num_envs).clone()
        self._overrides[name] = v
        self.p[name][:] = v

    def reset(self, env_ids: torch.Tensor | None = None) -> None:
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        env_ids = torch.as_tensor(env_ids, device=self.device).long().flatten()
        if env_ids.numel() == 0:
            return
        n = env_ids.numel()
        for name, buf in self.p.items():
            if name in self._overrides:
                buf[env_ids] = self._overrides[name][env_ids]
            elif name in self.cfg.param_ranges:
                lo, hi = self.cfg.param_ranges[name]
                buf[env_ids] = torch.empty(n, device=self.device).uniform_(lo, hi)
            else:
                buf[env_ids] = float(getattr(self.cfg.gait, name))
        self.time[env_ids] = 0.0
        self.lat_est[env_ids] = 0.0

    # ------------------------------------------------------------------ control
    @torch.no_grad()
    def compute_action(self, obs: torch.Tensor) -> torch.Tensor:
        """Observation ``[N, 14+3J]`` -> normalised action ``[N, J]`` (see ``shared/interface.py``)."""
        o = self.layout.split(obs)
        p = self.p
        c = self.cfg

        # --- clock, gait frequency (both from the observation), ramp
        sc = o["gait_phase"]
        phase = torch.remainder(torch.atan2(sc[:, 0], sc[:, 1]) / (2 * math.pi), 1.0)
        f = o["gait_freq"][:, 0].clamp(0.2, 5.0)
        ramp = (self.time * f / p["ramp_cycles"].clamp_min(1e-3)).clamp(0.0, 1.0)

        # --- base state from the observation
        v = o["base_lin_vel"]
        g = o["projected_gravity"]
        cmd = o["cmd_vx"][:, 0]
        herr = o["heading_error"][:, 0]
        pitch = torch.asin(g[:, 0].clamp(-1, 1))  # + = leaning forward (nose down)
        roll = -torch.asin(g[:, 1].clamp(-1, 1))  # + = right-handed about +x (leaning right)
        self.lat_est = self.lat_est + v[:, 1] * self.dt

        # --- per-leg timing, shape [N, 2] (left, right)
        leg_phase = torch.remainder(phase.unsqueeze(1) + self.leg_phase_offset, 1.0)
        swing_frac = p["swing_frac"].clamp(0.05, 0.5).unsqueeze(1)
        u = torch.remainder(leg_phase - p["swing_start"].unsqueeze(1), 1.0)
        in_swing = u < swing_frac
        s_sw = (u / swing_frac).clamp(0.0, 1.0)
        s_st = ((u - swing_frac) / (1.0 - swing_frac)).clamp(0.0, 1.0)

        # --- stride: during stance the foot moves back relative to the hip at the body speed, so its
        # excursion is D = v * T_stance = (cmd / f) * (1 - swing_frac); step length = cmd / (2 f).
        # Heading: turning left (herr > 0) -> the right (outer) leg sweeps further.
        sweep = ramp * p["step_scale"] * cmd / f * (1.0 - p["swing_frac"].clamp(0.05, 0.5))
        diff = (p["k_heading"] * herr).clamp(-0.5, 0.5)
        sweep_leg = sweep.unsqueeze(1) * (1.0 - self.side_sign * diff.unsqueeze(1))

        # --- sagittal foot positions relative to the hip pitch axis (gravity-aligned frame)
        x_st = sweep_leg * (0.5 - s_st)
        x_land = (sweep_leg / 2 + (p["k_vx_place"] * (v[:, 0] - cmd)).unsqueeze(1)).clamp(-c.max_step_x, c.max_step_x)
        delay = p["swing_x_delay"].clamp(0.0, 0.8).unsqueeze(1)
        s_x = ((s_sw - delay) / (1.0 - delay)).clamp(0.0, 1.0)
        x_sw = -sweep_leg / 2 + (x_land + sweep_leg / 2) * (1 - torch.cos(math.pi * s_x)) / 2
        x = torch.where(in_swing, x_sw, x_st)
        x = x + p["foot_x_offset"].unsqueeze(1)
        # one hip height per env (inverted-pendulum vaulting): the lowest of sqrt(r^2 - x^2) over both
        # legs. Including the swing leg lowers the hip before touchdown, so the landing foot meets the
        # ground at the post-touchdown height (continuous in time). A trailing leg straightens toward
        # the IK reach limit (the ankle range cannot keep a bent-knee trailing foot flat).
        r_st = p["stance_leg_length"].unsqueeze(1)
        trail = (-x / c.max_step_x).clamp(0.0, 1.0) * p["stance_extend"].unsqueeze(1)
        r_leg = r_st + (self.ik.r_max - r_st).clamp_min(0.0) * trail
        hip_h = torch.sqrt((r_leg * r_leg - x * x).clamp_min(1e-6)).min(1, keepdim=True).values
        lift = (ramp * p["lift_height"]).unsqueeze(1) * torch.sin(math.pi * s_sw) * in_swing
        z = -(hip_h - lift)

        # swing legs place the foot in the gravity frame: rotate by the measured pitch into the pelvis frame
        th = (pitch.unsqueeze(1) * p["swing_tilt_comp"].unsqueeze(1)) * in_swing
        cth, sth = torch.cos(th), torch.sin(th)
        xb, zb = cth * x - sth * z, sth * x + cth * z
        hip, knee, reach_err = self.ik.ik(xb, zb)
        ankle = -(hip + knee) - th
        stance = ~in_swing
        hip = hip + (p["k_pitch_hip"] * pitch).unsqueeze(1) * stance
        ankle = ankle + (p["k_pitch_ankle"] * pitch).unsqueeze(1) * stance

        # --- frontal plane: canonical hip roll (+ = foot moves left)
        phi_c = 0.5 + p["swing_start"] + p["swing_frac"] / 2  # middle of the right leg's swing
        sway = torch.cos(2 * math.pi * (phase - phi_c + p["sway_lead"]))  # +1: shift the pelvis left
        sway = torch.sign(sway) * sway.abs().pow(p["sway_shape"].clamp(0.1, 2.0))
        roll_q = (-(ramp * p["sway_amp"] * sway)).unsqueeze(1).expand(-1, 2).clone()
        roll_q = roll_q + self.side_sign * (ramp * p["swing_abduct"]).unsqueeze(1) * torch.sin(math.pi * s_sw) * in_swing
        vy_des = -p["k_lat_pos"] * self.lat_est
        dy = (p["k_vy_place"] * (v[:, 1] - vy_des)).clamp(-c.max_place_y, c.max_place_y)
        roll_q = roll_q + (dy / self.roll_lever).unsqueeze(1) * in_swing
        roll_q = roll_q - (roll * p["swing_tilt_comp"]).unsqueeze(1) * in_swing
        roll_q = roll_q + (p["k_roll_hip"] * roll).unsqueeze(1) * stance

        # --- assemble canonical angles [N, 2, 4] -> joint targets in ACTION_JOINT_NAMES order
        canon = torch.stack((roll_q, hip, knee, ankle), dim=-1)
        q = self.q_default.expand(obs.shape[0], -1).clone()
        q.scatter_(1, self.index.flatten().expand(obs.shape[0], -1), canon.reshape(obs.shape[0], -1))

        # --- tail: swing toward the stance side (sway +1 = pelvis left -> tail tip left = negative yaw)
        if self.tail_yaw is not None:
            tail_sway = torch.cos(2 * math.pi * (phase - phi_c + p["sway_lead"] + p["tail_yaw_lead"]))
            tail_sway = torch.sign(tail_sway) * tail_sway.abs().pow(p["sway_shape"].clamp(0.1, 2.0))
            q[:, self.tail_yaw] += -ramp * p["tail_yaw_amp"] * tail_sway - p["k_tail_roll"] * roll
        if self.tail_pitch is not None:
            q[:, self.tail_pitch] += p["tail_pitch_offset"] + p["k_tail_pitch"] * pitch
        q = (q * self.sign).clamp(self.q_lo, self.q_hi)
        self.time = self.time + self.dt

        self.debug = {"phase": phase, "freq": f, "ramp": ramp, "in_swing": in_swing, "reach_err": reach_err,
                      "x": x, "z": z, "hip_h": hip_h[:, 0],
                      "pitch": pitch, "roll": roll, "lat_est": self.lat_est, "sway": sway}
        return itf.joint_targets_to_action(q, self.q_default)
