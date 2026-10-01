"""Experiment 2 robot (tail biped, 4 DOF per leg + 2 DOF tail) wrapped for Isaac Lab.

All numbers come from the model version's ``01_robot_construction/model/v<N>/params.yaml`` and its
generated reports; nothing robot-specific is typed here twice. The version is :data:`ROBOT_VERSION`
(override with the environment variable ``CUSTOM_BIPED_VERSION``). Every downstream output must carry
it: ``outputs/robot_v<N>/...``.

Robot frame: +X forward, +Y left, +Z up (no root rotation needed). The base link origin is the midpoint
of the two hip_roll axes. Joint signs follow the pipeline's canonical convention directly:
roll + = foot moves left, pitch joints + = rotate backward (hip: leg back, knee: flex, ankle: toes down),
tail_yaw + = tail tip moves right (-Y), tail_pitch + = tail tip moves up.

Fields required by the pipeline are listed in ``shared/robots/README.md``. Compared with experiment 1 the
pipeline reads :data:`ACTION_JOINT_NAMES` (legs then tail) instead of ``LEG_JOINT_NAMES`` for the
observation / action order, :data:`ENV_DEFAULTS` for the robot-scale environment settings and the
``STANCE_*`` / ``MAX_*`` gait geometry for the rule-based defaults.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
ROBOT_NAME = "custom_biped"
ROBOT_VERSION = int(os.environ.get("CUSTOM_BIPED_VERSION", "1"))
MODEL_DIR = PROJECT_ROOT / "01_robot_construction" / "model" / f"v{ROBOT_VERSION}"
URDF_PATH = MODEL_DIR / "urdf" / f"{ROBOT_NAME}_v{ROBOT_VERSION}.urdf"
BUILD_DIR = Path(__file__).resolve().parent / "usd" / f"{ROBOT_NAME}_v{ROBOT_VERSION}"
"""Converted USD (not in git): ``shared/robots/usd/custom_biped_v<N>/``."""
TEST_ROOT = PROJECT_ROOT / "outputs" / f"robot_v{ROBOT_VERSION}"
"""Output root of this robot version (``outputs/robot_v<N>/<stage>/<run>/``)."""

_P = yaml.safe_load((MODEL_DIR / "params.yaml").read_text(encoding="utf-8"))
_MP = json.loads((MODEL_DIR / "reports" / "mass_properties.json").read_text(encoding="utf-8"))
_ST = json.loads((MODEL_DIR / "reports" / "static_check.json").read_text(encoding="utf-8"))
SERVO_MAP: list[dict] = json.loads((MODEL_DIR / "reports" / "servo_map.json").read_text(encoding="utf-8"))
"""Real robot: servo ID, joint, bus and ``direction`` (joint angle = direction x (position - 2048) x 2pi/4096).
Derived by the generator from the horn directions; the sign convention must be verified on hardware."""
_MM = 1e-3

# --------------------------------------------------------------------------------------
# Kinematic layout
# --------------------------------------------------------------------------------------
_ROLES = ("hip_roll", "hip_pitch", "knee", "ankle_pitch")
LEG_JOINT_NAMES: tuple[str, ...] = tuple(f"{s}_{r}" for s in ("left", "right") for r in _ROLES)
"""Actuated leg joints in the canonical observation / action order."""
LEG_JOINT_LAYOUT: tuple[tuple[str, str], ...] = tuple((s, r) for s in ("left", "right") for r in _ROLES)
TAIL_JOINT_NAMES: tuple[str, ...] = ("tail_yaw", "tail_pitch")
ACTUATED_JOINT_NAMES: tuple[str, ...] = LEG_JOINT_NAMES + TAIL_JOINT_NAMES
"""Legs (8) then tail (2)."""
ACTION_JOINT_NAMES: tuple[str, ...] = ACTUATED_JOINT_NAMES
"""Joints in the observation / action vectors, in order (user decision 2026-09-30: the tail is part of
the interface, J = 10)."""

JOINT_AXIS_SIGN: dict[str, float] = {r: 1.0 for r in _ROLES}
"""URDF axes already follow the canonical convention (roll about +x, pitch about +y)."""


def _lim(role: str) -> tuple[float, float]:
    j = _P["joints"][role]
    return math.radians(j["lower"]), math.radians(j["upper"])


LEG_JOINT_LIMITS: dict[str, tuple[float, float]] = {r: _lim(r) for r in _ROLES}
TAIL_JOINT_LIMITS: dict[str, tuple[float, float]] = {r: _lim(r) for r in TAIL_JOINT_NAMES}
"""Joint position limits [rad] (URDF values)."""

BASE_BODY_NAME = "base"
FOOT_BODY_NAMES: tuple[str, str] = ("left_foot", "right_foot")
FOOT_PRIM_PATHS: tuple[str, str] = tuple(
    f"Geometry/base/{s}_hip/{s}_thigh/{s}_shin/{s}_foot" for s in ("left", "right"))
"""Foot bodies relative to the robot prim (the 3.0 importer nests links along the chain)."""

# --------------------------------------------------------------------------------------
# Geometry [m] (zero pose)
# --------------------------------------------------------------------------------------
_L = _P["legs"]
THIGH_LENGTH = _L["thigh"] * _MM
SHIN_LENGTH = _L["shin"] * _MM
ANKLE_HEIGHT = _L["ankle_height"] * _MM
HIP_ROLL_TO_PITCH = (0.0, 0.0, -_L["hip_roll_to_pitch"] * _MM)
HIP_LATERAL_OFFSET = _L["hip_spacing"] / 2 * _MM
HIP_ROLL_HEIGHT = (_L["ankle_height"] + _L["shin"] + _L["thigh"] + _L["hip_roll_to_pitch"]) * _MM
FOOT_LENGTH_FWD = FOOT_LENGTH_BACK = _L["foot"]["length"] / 2 * _MM
FOOT_WIDTH = _L["foot"]["width"] * _MM
TAIL_LENGTH = _P["tail"]["length"] * _MM
TOTAL_MASS = _MP["total_mass_g"] * 1e-3
FOOT_MASS = _MP["links"]["left_foot"]["mass_g"] * 1e-3

# --------------------------------------------------------------------------------------
# Stance (params.stance; hip / ankle come from check_static.py so the CoM is above the feet)
# --------------------------------------------------------------------------------------
_S = _P["stance"]
DEFAULT_JOINT_POS: dict[str, float] = {}
for _s in ("left", "right"):
    DEFAULT_JOINT_POS.update({f"{_s}_hip_roll": 0.0, f"{_s}_hip_pitch": math.radians(_S["hip_pitch"]),
                              f"{_s}_knee": math.radians(_S["knee"]), f"{_s}_ankle_pitch": math.radians(_S["ankle_pitch"])})
DEFAULT_JOINT_POS.update({"tail_yaw": math.radians(_S["tail_yaw"]), "tail_pitch": math.radians(_S["tail_pitch"])})

INIT_BASE_HEIGHT = _ST["stance"]["base_height_mm"] * _MM + 0.003
"""Root z [m] at spawn for DEFAULT_JOINT_POS: soles 3 mm above the ground."""
INIT_YAW = 0.0
INIT_ROT_XYZW = (0.0, 0.0, 0.0, 1.0)
FORWARD_AXIS = "+x"
NOMINAL_COM_HEIGHT = _ST["stance"]["com_mm"][2] * _MM
"""Whole-body CoM height [m] in the default pose."""

# --------------------------------------------------------------------------------------
# Gait geometry for the rule-based defaults (reports/static_check.json)
# --------------------------------------------------------------------------------------
_R = _ST["reach"]
STANCE_LEG_LENGTH = _R["hip_to_ankle_mm"] * _MM
"""Hip pitch axis -> ankle axis distance in the default pose [m]."""
STANCE_FOOT_X = _ST["stance"]["ankle_x_mm"] * _MM
"""Ankle x relative to the hip pitch axes in the default pose [m] (negative: the tail pulls the CoM back,
the feet sit under it)."""
MAX_STEP_X = _R["max_step_mm"] * _MM
"""Largest ankle excursion from below the hip with the sole flat [m]."""
MAX_LIFT = _R["max_lift_mm"] * _MM
"""Largest ankle lift from the stance height [m]."""

ENV_DEFAULTS: dict = {
    "cmd_vx_range": (0.05, 0.15),  # user decision 2026-09-30
    "gait_freq_range": (1.2, 2.0),  # user decision 2026-09-30
    "fall_com_height": round(0.6 * NOMINAL_COM_HEIGHT, 3),
    "lateral_spacing": 1.0,
    "row_spacing": 5.0,
    "follow_cam_offset": (0.3, -0.5, 0.04),
    "follow_cam_target_drop": 0.06,
    "viewer_eye": (0.8, -1.3, 0.5),
    "viewer_lookat": (0.0, 0.0, 0.12),
    "contact_force_limit": round(1.5 * TOTAL_MASS * 9.81, 1),
    "target_air_time": 0.15,
}
"""Robot-scale environment settings, applied by ``BipedWalkEnvCfg.__post_init__`` to every field left at
None (see ``shared/env/biped_env_cfg.py`` for the meaning of each)."""

# --------------------------------------------------------------------------------------
# Actuators: Feetech HD-1910-C001, conservative 6 V numbers (params.actuator)
# --------------------------------------------------------------------------------------
_A = _P["actuator"]
LEG_EFFORT_LIMIT = _A["effort_limit"]
"""[N m] largest torque the DCMotorCfg can apply (continuous clip); the stall value ``saturation_effort``
only shapes the torque-speed curve. Torque saturation is detected as applied != computed torque."""
LEG_VELOCITY_LIMIT = _A["velocity_limit"]
LEG_STIFFNESS = _A["stiffness"]
LEG_DAMPING = _A["damping"]
LEG_ARMATURE = _P["joints"]["armature"]
LEG_FRICTION = _P["joints"]["friction"]


def usd_path() -> Path:
    """Converted asset (``01_robot_construction/model/generator/convert_usd.py``)."""
    matches = sorted((BUILD_DIR / "usd").rglob("*.usd*"))
    top = [m for m in matches if m.parent.parent == BUILD_DIR / "usd"]
    if not top:
        raise FileNotFoundError(f"No converted USD in {BUILD_DIR / 'usd'}; run 01_robot_construction/model/generator/convert_usd.py")
    return top[0]


def make_articulation_cfg(prim_path: str = "/World/envs/env_.*/Robot"):
    """Build the :class:`ArticulationCfg`. Imports inside the function: needs a running app."""
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import DCMotorCfg
    from isaaclab.assets.articulation import ArticulationCfg

    def motor(names):
        return DCMotorCfg(
            joint_names_expr=list(names),
            saturation_effort=_A["saturation_effort"],
            effort_limit=_A["effort_limit"],
            velocity_limit=_A["velocity_limit"],
            stiffness=LEG_STIFFNESS,
            damping=LEG_DAMPING,
            armature=LEG_ARMATURE,
            friction=LEG_FRICTION,
        )

    return ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(usd_path()),
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False, retain_accelerations=False, linear_damping=0.0, angular_damping=0.0,
                max_linear_velocity=100.0, max_angular_velocity=100.0, max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=1
            ),
            joint_drive_props=sim_utils.JointDrivePropertiesCfg(ensure_drives_exist=True),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, INIT_BASE_HEIGHT), rot=INIT_ROT_XYZW,
            joint_pos=dict(DEFAULT_JOINT_POS), joint_vel={".*": 0.0},
        ),
        soft_joint_pos_limit_factor=0.95,
        actuators={"legs": motor(LEG_JOINT_NAMES), "tail": motor(TAIL_JOINT_NAMES)},
    )
