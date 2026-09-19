"""Configuration of the straight-walking environment (robot-agnostic, robot chosen by name)."""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg, ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim import SimulationCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils.configclass import configclass

from .. import interface as itf
from ..robots import get_robot


@configclass
class RewardWeightsCfg:
    """Reward weights (initial values; tuned in the PPO phase). Each term is multiplied by ``step_dt``."""

    track_vx: float = 1.5
    """exp(-((v_fwd - cmd_vx) / sigma_vx)^2)"""
    track_heading: float = 0.5
    """exp(-(heading_error / sigma_heading)^2)"""
    lateral_vel: float = -1.0
    """v_lateral^2 [m^2/s^2]"""
    lateral_deviation: float = -10.0
    """lateral offset from the straight line^2 [m^2]"""
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


@configclass
class BipedWalkEnvCfg(DirectRLEnvCfg):
    """Straight walking on flat ground with a forward-speed command and a fixed target heading."""

    robot_name: str = "ref_biped"

    # -- timing / spaces (see shared/interface.py) --
    decimation: int = itf.DECIMATION
    episode_length_s: float = 20.0
    action_space: int = 8
    observation_space: int = 38
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
    eval_camera: bool = False
    """Spawn a follow camera for evaluation videos (needs ``--enable_cameras``); see
    :meth:`BipedWalkEnv.capture_frame`."""
    follow_cam_offset: tuple[float, float, float] = (0.6, -1.3, 0.12)
    """Camera position relative to the tracked robot's base CoM, in its target-heading frame
    (x forward, y left, z up) [m]."""

    # filled in __post_init__ from the robot module
    robot: ArticulationCfg | None = None
    foot_sensors: list[ContactSensorCfg] | None = None
    """One contact sensor per foot, in the robot's FOOT_BODY_NAMES order."""

    # -- commands (sampled per episode) --
    cmd_vx_range: tuple[float, float] = (0.05, 0.20)
    """Forward speed command [m/s]."""
    gait_freq_range: tuple[float, float] = (0.8, 1.2)
    """Gait cycle frequency [Hz] (one cycle = two steps); drives the gait clock observation."""
    heading_offset_range: tuple[float, float] = (0.0, 0.0)
    """Target heading minus initial heading [rad]. Zero = walk straight ahead."""

    # -- initial state randomisation --
    init_yaw_range: tuple[float, float] = (0.0, 0.0)
    """Initial heading offset [rad] from world +X. Default 0: every robot starts facing its forward axis
    (URDF +Y for ref_biped) along world +X (user decision 2026-09-18)."""

    # -- spawn layout: robots side by side in rows, all walking toward +X --
    lateral_spacing: float = 1.0
    """Distance between neighbouring robots in a row (world Y) [m]."""
    robots_per_row: int = 64
    row_spacing: float = 10.0
    """Distance between rows (world X) [m]; must exceed the longest possible walk in one episode."""
    init_joint_noise: float = 0.05
    """Uniform noise on leg joint positions at reset [rad]."""
    init_phase_random: bool = False
    """Start the gait clock at a random phase instead of 0."""
    settle_time: float = 0.1
    """Touchdowns during the first ``settle_time`` seconds (initial landing) are not counted as steps [s]."""
    min_step_air_time: float = 0.03
    """A touchdown counts as a step only after at least this long in the air [s] (>= 2 control steps);
    filters contact chatter of a sole rocking on its edge."""

    # -- termination --
    fall_com_height: float = 0.22
    """Episode ends when the base CoM drops below this height [m]."""
    fall_tilt_deg: float = 40.0
    """Episode ends when the base tilts more than this [deg]."""

    # -- rewards --
    rewards: RewardWeightsCfg = RewardWeightsCfg()
    sigma_vx: float = 0.1
    sigma_heading: float = 0.2
    target_air_time: float = 0.3
    contact_force_limit: float = 400.0

    # -- bookkeeping --
    record_episodes: bool = False
    """Keep per-episode metric summaries in ``env.completed_episodes`` (evaluation scripts only)."""

    def __post_init__(self):
        rb = get_robot(self.robot_name)
        self.robot = rb.make_articulation_cfg("/World/envs/env_.*/Robot")
        self.foot_sensors = [
            ContactSensorCfg(
                prim_path=f"/World/envs/env_.*/Robot/{path}", history_length=3, update_period=0.0, track_air_time=True
            )
            for path in rb.FOOT_PRIM_PATHS
        ]
        j = len(rb.LEG_JOINT_NAMES)
        self.action_space = j
        self.observation_space = itf.ObsLayout(j).dim
