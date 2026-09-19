"""Tunable parameters of the rule-based walking controller.

Every value here is robot-agnostic in meaning (lengths in m, angles in rad, gains in SI units).
Robot geometry (leg lengths, joint signs/limits, default pose) comes from ``shared/robots/<robot>.py``.

Per-environment parameters: the controller keeps each entry of :class:`GaitParams` as a tensor
``[num_envs]``. At every reset it takes the default below, or samples uniformly from
``RuleBasedCfg.param_ranges[name]`` when present (data diversity for BC). Sweep scripts can pin
per-env values with :meth:`RuleBasedBipedController.override`.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field


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
    stance_leg_length: float = 0.186
    """Hip-pitch-axis to ankle distance of the stance leg [m]; the hip height follows sqrt(r^2 - x^2)."""
    stance_extend: float = 1.0
    """0..1: how much a trailing stance leg straightens toward the IK reach limit."""
    lift_height: float = 0.012
    """Desired swing ankle lift [m] (clipped by the knee limit inside the IK)."""
    foot_x_offset: float = 0.015
    """Constant forward offset of both feet relative to the hip pitch axis [m] (the heel is much longer
    than the toe, so the support area lies behind the ankle)."""

    # -- lateral sway (hip roll, same angle on both legs = parallelogram) --
    sway_amp: float = 0.06
    """Hip roll amplitude [rad] shifting the pelvis over the stance foot."""
    sway_lead: float = 0.05
    """Phase lead of the sway relative to the middle of the opposite leg's swing [cycles]."""
    sway_shape: float = 1.0
    """Sway waveform exponent: sign(s)|s|^p with s a sinusoid; p < 1 flattens it toward a square wave."""
    swing_abduct: float = 0.0
    """Extra outward hip roll of the swing leg at mid-swing [rad] (negative = inward)."""
    swing_x_delay: float = 0.0
    """Fraction of the swing spent lifting before the foot starts moving forward (0 = move from liftoff)."""

    # -- balance feedback --
    k_vx_place: float = 0.10
    """Sagittal Raibert gain [s]: landing x += k * (v_x - cmd_vx)."""
    k_vy_place: float = 0.10
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
    max_step_x: float = 0.08
    """Clamp of the landing position relative to the hip [m]."""
    max_place_y: float = 0.06
    """Clamp of the lateral landing correction [m]."""
    knee_margin: float = 0.95
    """Fraction of the knee limit the IK may use."""
    min_knee: float = 0.08
    """Smallest knee flexion the IK uses [rad] (keeps the leg away from the straight-knee singularity)."""

    def to_dict(self) -> dict:
        return asdict(self)
