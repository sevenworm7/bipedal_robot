"""Configuration of the straight-walking environment (robot-agnostic, robot chosen by name).

Ported from ``archive/exp1_reference/shared/env`` (experiment 1). Fields whose value depends on the robot's size default to
None and are filled from the robot module's ``ENV_DEFAULTS`` in ``__post_init__``; an explicit value set by a
script is kept."""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg, ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim import SimulationCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils.configclass import configclass

from core import interface as itf
from core.robots import get_robot


@configclass
class RewardWeightsCfg:
    """Reward weights (initial values; tuned in the PPO phase). Each term is multiplied by ``step_dt``."""

    track_vx: float = 1.5
    """exp(-((v_fwd - cmd_vx) / sigma_vx)^2)"""
    track_heading: float = 0.5
    """exp(-(heading_error / sigma_heading)^2)"""
    lateral_vel: float = -1.0
    """v_lateral^2 [m^2/s^2]"""
    lateral_deviation: float = -40.0
    """lateral offset from the straight line^2 [m^2]; -10 -> -40 on 2026-09-23 (user decision): with -10 the
    iteration-2 PPO let slow commands drift sideways (median max offset 0.30 m, only ~2.5 reward lost per episode)."""
    orientation: float = -2.0
    """|projected_gravity_xy|^2"""
    base_height: float = -20.0
    """(com_z - nominal)^2 [m^2]"""
    alive: float = 0.5
    termination: float = -20.0
    """applied once per control step while terminated by a fall (i.e. once per fall)"""
    torque: float = -2.0e-4
    """sum tau^2 [N^2 m^2]"""
    action_rate: float = -0.01
    """sum (a_t - a_{t-1})^2"""
    joint_limits: float = -2.0
    """sum of soft joint limit violations [rad]"""
    feet_air_time: float = 1.0
    """sum over touchdowns of (air_time - target_air_time) [s]"""
    contact_force: float = -1.0e-3
    """sum max(|F_foot| - contact_force_limit, 0) [N]"""
    lateral_vel_avg: float = 0.0
    """(lateral velocity low-pass filtered with ``lateral_vel_tau``)^2 [m^2/s^2]: penalises a steady sideways drift but not
    the step-to-step sway; observable by the privileged teacher (control/sim2real round 5, replaces ``lateral_deviation``,
    which penalises an unobservable, accumulating position). 0 keeps the 03 reward."""
    clearance: float = 0.0
    """per touchdown exp(-((swing apex - commanded clearance) / sigma)^2); used only with ``gait_var.clearance_range``
    (control/sim2real). 0 keeps the 03 reward."""
    gait_contact: float = 2.0
    """mean over feet of [contact state == clock schedule] (in the air inside the foot's swing window of the
    gait clock, on the ground outside it); added 2026-09-22 (user decision) because PPO drifted away from
    the clock (0.5-1.8 touchdowns per clock step). Teacher 19.0 / 20 per 20 s episode, old PPO 17.2 (it gained
    +11.5 on track_vx), hence weight 2."""


@configclass
class DomainRandCfg:
    """Sim-to-real randomisation (control/sim2real). **Every field defaults to None = off**; with all fields off the env
    is bit-identical to the 03 environment (checked by the flat_v1 regression run). Values are set through the named
    scenarios in ``simulation/env/scenarios.py``, never edited in place once a scenario has been used."""

    action_delay_phys_steps: tuple[int, int] | None = None
    """Per-episode delay of the joint targets in physics steps (5 ms each), sampled uniformly (inclusive)."""
    kp_scale: tuple[float, float] | None = None
    """Per-episode scale of the actuator stiffness."""
    kd_scale: tuple[float, float] | None = None
    """Per-episode scale of the actuator damping."""
    effort_scale: tuple[float, float] | None = None
    """Per-episode scale of the actuator continuous effort limit."""
    foot_friction: tuple[float, float] | None = None
    """Static friction of the foot soles, sampled once per env at startup (64 buckets). When set, the ground uses
    the ``multiply`` combine mode with friction 1.0, so the effective friction equals the foot value."""
    mass_scale: tuple[float, float] | None = None
    """Mass scale of every robot link, once per env at startup (printed-part masses are estimates)."""
    base_com_offset: float | None = None
    """Uniform offset of the base CoM along x / y / z [m], once per env at startup."""
    obs_noise_ang_vel: float | None = None
    """Std of Gaussian noise on the base angular velocity observation [rad/s]."""
    obs_noise_gravity: float | None = None
    """Std of Gaussian noise on the projected gravity observation [-]."""
    obs_noise_joint_pos: float | None = None
    """Std of Gaussian noise on the joint position observation [rad]."""
    obs_noise_joint_vel: float | None = None
    """Std of Gaussian noise on the joint velocity observation [rad/s]."""
    heading_bias: float | None = None
    """Per-episode constant bias of the heading-error observation, uniform in +-value [rad] (IMU yaw offset)."""
    rough_terrain: dict | None = None
    """Uneven ground with a curriculum (``simulation/env/terrain.py``). Keys: ``amplitude`` (max bump height [m] at the
    hardest level), ``levels``, ``cols``, ``tile_size`` ((x, y) [m]), ``robots_per_tile``, ``horizontal_scale`` [m],
    ``init_level_max``, ``curriculum`` (bool: promote / demote robots between levels at reset). Replaces the flat plane
    and the row layout: robots start at the back of their tile (side by side, 1 tile column = ``robots_per_tile``)."""


@configclass
class GaitVariationCfg:
    """Gait diversity (control/sim2real phase 3, user decision 2026-10-06). **All off by default** (03 behaviour)."""

    resample_interval_s: tuple[float, float] | None = None
    """Resample speed and gait frequency mid-episode after this many seconds (start / stop / speed changes).
    When set, the gait clock is integrated (phase += f dt) so that frequency changes do not make the phase jump."""
    zero_speed_prob: float = 0.0
    """Probability that a sampled command is 0 m/s with the clock running (stepping in place)."""
    stand_prob: float = 0.0
    """Probability that a sampled command is "stand": 0 m/s and gait frequency 0 (clock frozen, both feet down)."""
    clearance_range: tuple[float, float] | None = None
    """Commanded swing-foot apex height above the stance foot [m], sampled with each command (interface v2 only).
    Rewarded at touchdown by the ``clearance`` reward term."""
    sigma_clearance: float = 0.008
    """Width of the clearance reward exp(-((apex - cmd) / sigma)^2) [m]."""


@configclass
class BipedWalkEnvCfg(DirectRLEnvCfg):
    """Straight walking on flat ground with a forward-speed command and a fixed target heading."""

    robot_name: str = "custom_biped"

    # -- timing / spaces (see core/interface.py) --
    decimation: int = itf.DECIMATION
    episode_length_s: float = 20.0
    action_space: int = 10
    observation_space: int = 44
    state_space: int = 0

    sim: SimulationCfg = SimulationCfg(dt=itf.SIM_DT, render_interval=itf.DECIMATION)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=1024, env_spacing=2.0, replicate_physics=True)
    terrain: TerrainImporterCfg = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="average",
            restitution_combine_mode="average",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )
    viewer: ViewerCfg = ViewerCfg(eye=(1.5, -2.5, 1.0), lookat=(0.0, 0.0, 0.2), resolution=(1280, 720))
    """eye / lookat are replaced by the robot's ``viewer_eye`` / ``viewer_lookat``."""
    eval_camera: bool = False
    """Spawn a follow camera for evaluation videos (needs ``--enable_cameras``); see
    :meth:`BipedWalkEnv.capture_frame`."""
    follow_cam_offset: tuple[float, float, float] | None = None
    """Camera position relative to the tracked robot's base CoM, in its target-heading frame
    (x forward, y left, z up) [m]."""
    follow_cam_target_drop: float | None = None
    """The follow camera looks at the base CoM lowered by this much [m]."""

    # filled in __post_init__ from the robot module
    robot: ArticulationCfg | None = None
    foot_sensors: list[ContactSensorCfg] | None = None
    """One contact sensor per foot, in the robot's FOOT_BODY_NAMES order."""

    # -- commands (sampled per episode) --
    cmd_vx_range: tuple[float, float] | None = None
    """Forward speed command [m/s]."""
    gait_freq_range: tuple[float, float] | None = None
    """Gait cycle frequency [Hz] (one cycle = two steps); drives the gait clock observation."""
    heading_offset_range: tuple[float, float] = (0.0, 0.0)
    """Target heading minus initial heading [rad]. Zero = walk straight ahead."""

    # -- initial state randomisation --
    init_yaw_range: tuple[float, float] = (0.0, 0.0)
    """Initial heading offset [rad] from world +X. Default 0: every robot starts facing its forward axis
    along world +X (user decision 2026-09-18)."""

    # -- spawn layout: robots side by side in rows, all walking toward +X --
    lateral_spacing: float | None = None
    """Distance between neighbouring robots in a row (world Y) [m]."""
    robots_per_row: int = 64
    row_spacing: float | None = None
    """Distance between rows (world X) [m]; must exceed the longest possible walk in one episode."""
    init_joint_noise: float = 0.05
    """Uniform noise on the action joint positions at reset [rad]."""
    init_phase_random: bool = False
    """Start the gait clock at a random phase instead of 0."""
    settle_time: float = 0.1
    """Touchdowns during the first ``settle_time`` seconds (initial landing) are not counted as steps [s]."""
    min_step_air_time: float = 0.03
    """A touchdown counts as a step only after at least this long in the air [s] (>= 2 control steps);
    filters contact chatter of a sole rocking on its edge."""

    # -- disturbances (training only; evaluation uses the push test in simulation/rollout.py) --
    push_interval_s: tuple[float, float] | None = None
    """Time between random pushes [s], sampled per push; None disables pushes (default)."""
    push_speed: float = 0.2
    """Horizontal velocity kick of a push [m/s], random direction (user decision 2026-09-22: light pushes in PPO)."""

    # -- termination --
    fall_com_height: float | None = None
    """Episode ends when the base CoM drops below this height [m]."""
    fall_height_ref: str = "spawn"
    """What ``fall_com_height`` is measured from. "spawn" (default; every scenario before stress_v2): the fixed spawn
    height, correct on flat ground and small bumps but wrong on slopes / steps (walking 10 cm downhill counts as a fall).
    "feet": the ground under the robot, estimated as the lower foot body minus ``fall_foot_height`` (evaluation/generalization,
    2026-10-09)."""
    fall_foot_height: float = 0.026
    """Foot body origin above the ground while standing [m] (custom_biped v1, measured on flat_v1: 0.025-0.029)."""
    fall_tilt_deg: float = 40.0
    """Episode ends when the base tilts more than this [deg]."""

    # -- rewards --
    rewards: RewardWeightsCfg = RewardWeightsCfg()
    sigma_vx: float = 0.1
    sigma_heading: float = 0.2
    lateral_vel_tau: float = 1.0
    """Time constant of the low-pass filter of the ``lateral_vel_avg`` reward [s]."""
    target_air_time: float | None = None
    gait_swing_start: float = 0.075
    """Clock phase at which the left foot is expected to leave the ground (right foot: + 0.5) [cycles].
    Experiment 1 value (measured on its teacher); re-measure on the experiment 2 teacher before training."""
    gait_swing_frac: float = 0.2
    """Length of the expected swing window [cycles]."""
    contact_force_limit: float | None = None

    # -- sim-to-real randomisation (control/sim2real; all off by default) --
    scenario: str = "flat_v1"
    """Name of the scenario this config was built from (``simulation/env/scenarios.py``); recorded in every run config."""
    dr: DomainRandCfg = DomainRandCfg()
    gait_var: GaitVariationCfg = GaitVariationCfg()
    obs_version: int = 1
    """1 = the 44-dim v1 observation (02 / 03). 2 = ``core/interface_v2.py``: groups ``policy`` / ``critic`` =
    privileged teacher observation, ``student`` = deployable history (control/sim2real)."""

    # -- bookkeeping --
    record_episodes: bool = False
    """Keep per-episode metric summaries in ``env.completed_episodes`` (evaluation scripts only)."""

    def __post_init__(self):
        rb = get_robot(self.robot_name)
        for k, v in rb.ENV_DEFAULTS.items():
            if k in ("viewer_eye", "viewer_lookat"):
                setattr(self.viewer, k.split("_")[1], v)
            elif getattr(self, k) is None:
                setattr(self, k, v)
        self.robot = rb.make_articulation_cfg("/World/envs/env_.*/Robot")
        self.foot_sensors = [
            ContactSensorCfg(
                prim_path=f"/World/envs/env_.*/Robot/{path}", history_length=3, update_period=0.0, track_air_time=True
            )
            for path in rb.FOOT_PRIM_PATHS
        ]
        j = len(rb.ACTION_JOINT_NAMES)
        self.action_space = j
        self.observation_space = itf.ObsLayout(j).dim
