"""Reference biped (``references/.../雙足機器人_髖關節位置修正``) wrapped for Isaac Lab.

The reference URDF is read-only, so every fix-up lives here:

* the URDF is copied to an ASCII-only build directory before conversion (the Omniverse
  importer does not reliably handle the non-ASCII source path);
* the robot's native forward axis (:data:`FORWARD_AXIS`) is rotated onto world +X via the root
  yaw in :data:`INIT_ROT_XYZW`;
* actuator limits / PD gains replace the URDF placeholder values (effort=velocity=100);
* mesh collisions are simplified to convex hulls by the converter.

Robot frame of the URDF (from the design sheet): +X right, +Y forward, +Z up. The base_link
origin lies on the ground plane when all joints are zero (foot soles at z = 0).

Only this module (and the other files in ``shared/robots``) may contain robot-specific
numbers. Pipeline code must read them from here.
"""

from __future__ import annotations

import math
import shutil
from pathlib import Path

TEST_ROOT = Path(__file__).resolve().parents[2]
"""``archive/exp1_reference`` directory."""
PROJECT_ROOT = TEST_ROOT.parents[1]  # archive/exp1_reference -> project root (moved 2026-10-15)

REFERENCE_URDF_DIR = (
    PROJECT_ROOT / "references" / "雙足機器人" / "雙足機器人_髖關節位置修正" / "Biped_Robot_Master_URDF_unity_urdf"
)
REFERENCE_URDF_NAME = "Biped_Robot_Master_URDF.urdf"

BUILD_DIR = Path(__file__).resolve().parent / "usd" / "ref_biped"
"""Generated files (not in git): ``src/`` = ASCII copy of the URDF + meshes, ``usd/`` = converted asset."""

ROBOT_NAME = "ref_biped"

# --------------------------------------------------------------------------------------
# Kinematic layout
# --------------------------------------------------------------------------------------

LEG_JOINT_NAMES: tuple[str, ...] = (
    "J01_Left_Hip_Roll_Y",
    "J02_Left_Hip_Pitch_X",
    "J03_Left_Knee_Pitch_X",
    "J04_Left_Ankle_Pitch_X",
    "J05_Right_Hip_Roll_Y",
    "J06_Right_Hip_Pitch_X",
    "J07_Right_Knee_Pitch_X",
    "J08_Right_Ankle_Pitch_X",
)
"""Actuated leg joints, in the canonical order used by observations and actions."""

LEG_JOINT_LAYOUT: tuple[tuple[str, str], ...] = (
    ("left", "hip_roll"), ("left", "hip_pitch"), ("left", "knee"), ("left", "ankle_pitch"),
    ("right", "hip_roll"), ("right", "hip_pitch"), ("right", "knee"), ("right", "ankle_pitch"),
)
"""(side, role) of each entry of :data:`LEG_JOINT_NAMES`; lets controllers find joints without hard-coded indices."""

JOINT_AXIS_SIGN: dict[str, float] = {"hip_roll": -1.0, "hip_pitch": 1.0, "knee": 1.0, "ankle_pitch": 1.0}
"""Joint angle = sign x canonical angle, where the canonical angle is a right-handed rotation about the
canonical base axis (roll: +x forward, pitch: +y left). Canonical meaning: roll + moves the foot left;
pitch joints + rotate backward (hip: leg back, knee: shin back, ankle: toes down)."""

LEG_JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "hip_roll": (-0.349066, 0.349066),
    "hip_pitch": (-0.785398, 0.785398),
    "knee": (-0.785398, 0.785398),
    "ankle_pitch": (-0.349066, 0.349066),
}
"""Joint position limits [rad] from the URDF (same for both legs, in joint coordinates)."""

TORSO_JOINT_NAMES: tuple[str, ...] = ("Torso_Yaw_Joint", "Torso_Pitch_Joint")
"""Torso joints: held at 0 by a stiff PD, excluded from observations and actions."""

FOOT_BODY_NAMES: tuple[str, str] = ("Left_Foot_Link_1", "Right_Foot_Link_1")
BASE_BODY_NAME = "base_link"
FOOT_PRIM_PATHS: tuple[str, str] = (
    "Geometry/base_link/Left_Hip_Roll_Link_1/Left_Thigh_Link_1/Left_Shin_Link_1/Left_Foot_Link_1",
    "Geometry/base_link/Right_Hip_Roll_Link_1/Right_Thigh_Link_1/Right_Shin_Link_1/Right_Foot_Link_1",
)
"""Foot rigid bodies relative to the robot prim, same order as :data:`FOOT_BODY_NAMES`. The 3.0 importer
nests links along the chain; a single regex over both feet only resolves the first one, so the env
creates one contact sensor per foot."""

NOMINAL_COM_HEIGHT = 0.36
"""Approximate base CoM height [m] in the default pose (base_link CoM is 0.370 m above the soles at q = 0)."""

FORWARD_AXIS = "+y"
"""Robot-frame axis that points in the walking direction ("+y" per the design sheet; "-y" is the
alternative if the foot geometry suggests the toes point the other way)."""

_YAW_TO_WORLD_X = {"+y": -math.pi / 2, "-y": math.pi / 2, "+x": 0.0, "-x": math.pi}
INIT_YAW = _YAW_TO_WORLD_X[FORWARD_AXIS]
"""Root yaw [rad] that maps the robot forward axis onto world +X."""
INIT_ROT_XYZW = (0.0, 0.0, math.sin(INIT_YAW / 2), math.cos(INIT_YAW / 2))
"""Root orientation quaternion, **XYZW** (Isaac Lab 3.0 convention)."""

# Joint sign conventions (verified in sim by scripts/check_ref_biped.py, +0.3 rad probes):
#   hip roll  + : foot swings to the robot's right (both legs, same sign)
#   hip pitch + : leg swings backward (hip extension)
#   knee      + : shin swings backward (human-like knee flexion)
#   ankle     + : toes down (plantar flexion)

# Leg geometry from the URDF joint origins [m], expressed in the robot frame at q = 0
# (+X right, +Y forward, +Z up). Needed by IK in the rule-based controller.
HIP_ROLL_HEIGHT = 0.3166
"""Hip roll axis height above the sole plane (zero pose)."""
HIP_LATERAL_OFFSET = 0.11
"""|x| of the hip roll axes; the legs are 2 * 0.11 m apart."""
HIP_ROLL_TO_PITCH = (0.011, 0.0178, -0.0608)
THIGH_LENGTH = 0.100
"""Hip pitch axis -> knee axis (vertical at q = 0; the 0.8 mm x offset is ignored)."""
SHIN_LENGTH = 0.090
"""Knee axis -> ankle axis."""
ANKLE_HEIGHT = 0.0658
"""Ankle axis height above the sole."""
FOOT_LENGTH_FWD = 0.055
FOOT_LENGTH_BACK = 0.125
"""Sole extent in front of / behind the ankle axis along +Y (from the foot mesh)."""
FOOT_WIDTH = 0.120
FOOT_MASS = 6.034
"""[kg] each; the feet carry 42 % of the total mass, which matters for swing dynamics."""
TOTAL_MASS = 28.4
"""[kg]"""

_HIP_PITCH, _KNEE, _ANKLE = -0.25, 0.50, -0.25
DEFAULT_JOINT_POS: dict[str, float] = {name: 0.0 for name in LEG_JOINT_NAMES + TORSO_JOINT_NAMES}
DEFAULT_JOINT_POS.update({
    "J02_Left_Hip_Pitch_X": _HIP_PITCH, "J03_Left_Knee_Pitch_X": _KNEE, "J04_Left_Ankle_Pitch_X": _ANKLE,
    "J06_Right_Hip_Pitch_X": _HIP_PITCH, "J07_Right_Knee_Pitch_X": _KNEE, "J08_Right_Ankle_Pitch_X": _ANKLE,
})
"""Nominal standing pose [rad]: slight crouch (straight knees are singular for IK), sole flat."""

_CROUCH_DROP = THIGH_LENGTH * (1 - math.cos(_HIP_PITCH)) + SHIN_LENGTH * (1 - math.cos(_HIP_PITCH + _KNEE))
INIT_BASE_HEIGHT = 0.003 - _CROUCH_DROP
"""Root z [m] at spawn for :data:`DEFAULT_JOINT_POS`: soles 3 mm above ground (they are at base z = 0
in the zero pose; the crouch lifts them by ``_CROUCH_DROP`` relative to the base)."""

# --------------------------------------------------------------------------------------
# Actuators (user decision 2026-09-18: all leg joints 80 N m)
# --------------------------------------------------------------------------------------

LEG_EFFORT_LIMIT = 80.0
"""[N m]"""
LEG_VELOCITY_LIMIT = 20.0
"""[rad/s]"""
LEG_STIFFNESS = 300.0
"""[N m/rad]"""
LEG_DAMPING = 10.0
"""[N m s/rad]"""
LEG_ARMATURE = 0.01
"""[kg m^2] reflected rotor inertia; small value for numerical stability."""
TORSO_STIFFNESS = 1000.0
TORSO_DAMPING = 50.0


def prepare_urdf() -> Path:
    """Copy the reference URDF and meshes to the ASCII build directory.

    Returns:
        Path of the copied URDF file.
    """
    src = BUILD_DIR / "src"
    if src.exists():
        shutil.rmtree(src)
    shutil.copytree(REFERENCE_URDF_DIR, src)
    return src / REFERENCE_URDF_NAME


def usd_path() -> Path:
    """Path of the converted asset (created by ``scripts/convert_ref_biped.py``)."""
    matches = sorted((BUILD_DIR / "usd").rglob("*.usd*"))
    top = [m for m in matches if m.parent.parent == BUILD_DIR / "usd"]
    if not top:
        raise FileNotFoundError(f"No converted USD in {BUILD_DIR / 'usd'}; run archive/exp1_reference/scripts/convert_ref_biped.py")
    return top[0]


def make_articulation_cfg(prim_path: str = "/World/envs/env_.*/Robot"):
    """Build the :class:`ArticulationCfg`. Import inside the function: needs a running app."""
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets.articulation import ArticulationCfg

    return ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(usd_path()),
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=100.0,
                max_angular_velocity=100.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=1
            ),
            joint_drive_props=sim_utils.JointDrivePropertiesCfg(ensure_drives_exist=True),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, INIT_BASE_HEIGHT),
            rot=INIT_ROT_XYZW,
            joint_pos=dict(DEFAULT_JOINT_POS),
            joint_vel={".*": 0.0},
        ),
        soft_joint_pos_limit_factor=0.95,
        actuators={
            "legs": ImplicitActuatorCfg(
                joint_names_expr=list(LEG_JOINT_NAMES),
                effort_limit_sim=LEG_EFFORT_LIMIT,
                velocity_limit_sim=LEG_VELOCITY_LIMIT,
                stiffness=LEG_STIFFNESS,
                damping=LEG_DAMPING,
                armature=LEG_ARMATURE,
            ),
            "torso": ImplicitActuatorCfg(
                joint_names_expr=list(TORSO_JOINT_NAMES),
                effort_limit_sim=LEG_EFFORT_LIMIT,
                velocity_limit_sim=LEG_VELOCITY_LIMIT,
                stiffness=TORSO_STIFFNESS,
                damping=TORSO_DAMPING,
                armature=LEG_ARMATURE,
            ),
        },
    )
