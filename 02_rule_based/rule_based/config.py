"""Tunable parameters of the rule-based walking controller (experiment 2, ported from ``00_test/rule_based``).

Every value here is robot-agnostic in meaning (lengths in m, angles in rad, gains in SI units).
Robot geometry (leg lengths, joint signs/limits, default pose) comes from ``shared/robots/<robot>.py``.
Entries that depend on the robot's size default to None and are filled by :meth:`RuleBasedCfg.resolve` from
the robot module (``STANCE_LEG_LENGTH``, ``STANCE_FOOT_X``, ``MAX_STEP_X``, ``MAX_LIFT``, ``HIP_LATERAL_OFFSET``).

Per-environment parameters: the controller keeps each entry of :class:`GaitParams` as a tensor
``[num_envs]``. At every reset it takes the default below, or samples uniformly from
``RuleBasedCfg.param_ranges[name]`` when present (data diversity for BC). Sweep scripts can pin
per-env values with :meth:`RuleBasedBipedController.override`.

Changes from experiment 1: robot-scale defaults come from the robot module; the tail parameters
(``tail_*``, ``k_tail_*``) are new and only used when the robot has ``tail_yaw`` / ``tail_pitch`` joints.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields


@dataclass
class GaitParams:
    # -- timing (the gait clock itself comes from the observation) --
    swing_frac: float = 0.35
    """Fraction of the gait cycle a leg spends in swing (<= 0.5; the rest of 0.5 is double support)."""
    swing_start: float = 0.0
    """Leg-phase at which swing starts. Left leg phase = clock phase, right = clock phase + 0.5."""
    ramp_cycles: float = 1.5
    """Step length, lift and sway grow linearly from 0 over this many gait cycles after reset."""

    # -- sagittal trajectory --
    step_scale: float = 1.0
    """Step length = step_scale * cmd_vx / (2 f) (two steps per cycle)."""
    stance_leg_length: float | None = None
    """Hip-pitch-axis to ankle distance of the stance leg [m]; the hip height follows sqrt(r^2 - x^2).
    Default: the robot's ``STANCE_LEG_LENGTH``."""
    stance_extend: float = 0.5
    """0..1: how much a trailing stance leg straightens toward the IK reach limit."""
    lift_height: float | None = None
    """Desired swing ankle lift [m] (clipped by the knee limit inside the IK). Default: 0.35 x ``MAX_LIFT``."""
    foot_x_offset: float | None = None
    """Constant forward offset of both feet relative to the hip pitch axis [m]. Default: the robot's
    ``STANCE_FOOT_X`` (feet under the whole-body CoM in the default pose)."""

    # -- lateral sway (hip roll, same angle on both legs = parallelogram) --
    sway_amp: float = 0.12
    """Hip roll amplitude [rad] shifting the pelvis over the stance foot."""
    sway_lead: float = 0.05
    """Phase lead of the sway relative to the middle of the opposite leg's swing [cycles]."""
    sway_shape: float = 1.0
    """Sway waveform exponent: sign(s)|s|^p with s a sinusoid; p < 1 flattens it toward a square wave."""
    swing_abduct: float = 0.0
    """Extra outward hip roll of the swing leg at mid-swing [rad] (negative = inward)."""
    swing_x_delay: float = 0.0
    """Fraction of the swing spent lifting before the foot starts moving forward (0 = move from liftoff)."""

    # -- tail (new in experiment 2) --
    tail_yaw_amp: float = 0.35
    """Tail yaw amplitude [rad]: the tail swings toward the stance side with the sway waveform, moving the
    whole-body CoM over the stance foot (30 deg shifts it 10.9 mm on robot v1)."""
    tail_yaw_lead: float = 0.0
    """Phase lead of the tail swing relative to the hip roll sway [cycles]."""
    tail_pitch_offset: float = 0.0
    """Constant tail pitch relative to the default pose [rad] (+ = tail tip up)."""
    k_tail_roll: float = 0.5
    """Tail yaw posture gain [rad/rad]: a lean to one side swings the tail toward the other side."""
    k_tail_pitch: float = 0.0
    """Tail pitch posture gain [rad/rad]: tail pitch += k * base pitch (+ pitch = leaning forward)."""

    # -- balance feedback --
    k_vx_place: float = 0.10
    """Sagittal Raibert gain [s]: landing x += k * (v_x - cmd_vx)."""
    k_vy_place: float = 0.05
    """Lateral Raibert gain [s]: landing y += k * (v_y - v_y_des)."""
    k_lat_pos: float = 0.5
    """Lateral drift correction [1/s]: v_y_des = -k * (integrated lateral offset estimate)."""
    k_heading: float = 0.5
    """Differential step length per rad of heading error (outer leg steps longer)."""
    k_pitch_hip: float = 0.3
    """Stance hip pitch posture gain [rad/rad]."""
    k_roll_hip: float = 0.3
    """Stance hip roll posture gain [rad/rad]."""
    k_pitch_ankle: float = 0.0
    """Stance ankle posture gain [rad/rad] (ankle strategy)."""
    swing_tilt_comp: float = 1.0
    """0..1: how much of the measured base tilt the swing leg compensates (1 = foot placed in the gravity frame)."""


@dataclass
class RuleBasedCfg:
    gait: GaitParams = field(default_factory=GaitParams)
    param_ranges: dict[str, tuple[float, float]] = field(default_factory=dict)
    """Per-episode uniform sampling ranges for entries of :class:`GaitParams` (empty = use defaults)."""
    max_step_x: float | None = None
    """Clamp of the landing position relative to the hip [m]. Default: the robot's ``MAX_STEP_X``."""
    max_place_y: float | None = None
    """Clamp of the lateral landing correction [m]. Default: 0.5 x ``HIP_LATERAL_OFFSET``."""
    knee_margin: float = 0.95
    """Fraction of the knee limit the IK may use."""
    min_knee: float = 0.08
    """Smallest knee flexion the IK uses [rad] (keeps the leg away from the straight-knee singularity)."""

    def resolve(self, rb) -> "RuleBasedCfg":
        """Fill every None entry from the robot module (in place; returns self)."""
        derived = {
            "stance_leg_length": rb.STANCE_LEG_LENGTH,
            "lift_height": 0.35 * rb.MAX_LIFT,
            "foot_x_offset": rb.STANCE_FOOT_X,
        }
        for f in fields(self.gait):
            if getattr(self.gait, f.name) is None:
                setattr(self.gait, f.name, float(derived[f.name]))
        if self.max_step_x is None:
            self.max_step_x = float(rb.MAX_STEP_X)
        if self.max_place_y is None:
            self.max_place_y = float(0.5 * rb.HIP_LATERAL_OFFSET)
        return self

    def to_dict(self) -> dict:
        return asdict(self)


def load_cfg(rb, params_path: str | None = None, overrides: list[str] | None = None) -> RuleBasedCfg:
    """Config with gait parameters from a JSON file (``params/*.json``) and ``name=value`` overrides."""
    import json
    from pathlib import Path

    cfg = RuleBasedCfg()
    if params_path:
        for k, v in json.loads(Path(params_path).read_text(encoding="utf-8")).items():
            setattr(cfg.gait, k, float(v))
    for s in overrides or []:
        k, v = s.split("=", 1)
        setattr(cfg.gait, k.strip(), float(v))
    return cfg.resolve(rb)
