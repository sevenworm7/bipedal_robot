"""Named, versioned environment scenarios (03_training/sim2real, user rule 2026-10-06: never overwrite an old scene).

A scenario is a set of ``BipedWalkEnvCfg`` overrides. Rules:

* ``flat_v1`` is the 03 environment (no overrides) and must stay bit-identical to it; every change to
  ``shared/env`` is checked by re-running the 03 final model on ``flat_v1`` (``03_training/sim2real/evaluate.py --compare``).
* Once a scenario has been used by a recorded run it is **frozen**: never edit its entry. To change anything, add a
  new entry with the next version suffix (``dr_v2``) and keep the old one.
* Every run records ``scenario`` (written into ``BipedWalkEnvCfg.scenario`` and the run config).

Ranges follow the user decisions of 2026-10-06 (03_training/sim2real/README.md): delay 0-40 ms, kp / kd +-30 %, effort +-20 %,
friction 0.4-1.2, mass +-15 %; actuator ranges are wide estimates until one HD-1910 has been measured.
"""

from __future__ import annotations

import numpy as np

_MODERATE_DR = {
    "action_delay_phys_steps": (0, 8),  # 0-40 ms in 5 ms physics steps
    "kp_scale": (0.7, 1.3),
    "kd_scale": (0.7, 1.3),
    "effort_scale": (0.8, 1.2),
    "foot_friction": (0.4, 1.2),
    "mass_scale": (0.85, 1.15),
    "base_com_offset": 0.01,
    "obs_noise_ang_vel": 0.1,
    "obs_noise_gravity": 0.03,
    "obs_noise_joint_pos": 0.01,
    "obs_noise_joint_vel": 0.3,
    "heading_bias": 0.035,  # +-2 deg
}

_LIGHT_DR = {
    "action_delay_phys_steps": (0, 4),  # 0-20 ms
    "kp_scale": (0.85, 1.15),
    "kd_scale": (0.85, 1.15),
    "effort_scale": (0.9, 1.1),
    "foot_friction": (0.6, 1.0),
    "mass_scale": (0.93, 1.07),
    "base_com_offset": 0.005,
    "obs_noise_ang_vel": 0.05,
    "obs_noise_gravity": 0.015,
    "obs_noise_joint_pos": 0.005,
    "obs_noise_joint_vel": 0.15,
    "heading_bias": 0.0175,  # +-1 deg
}

_ROUGH_TRAIN = {  # +-8 mm bumps at the hardest of 10 levels (user decision 2026-10-06), curriculum.
    # 16 x 10 tiles at 5 cm: 32 columns (~12 M triangles) failed PhysX mesh cooking (no ground collision).
    "amplitude": 0.008, "levels": 10, "cols": 16, "tile_size": (6.0, 8.0), "robots_per_tile": 8,
    "horizontal_scale": 0.05, "init_level_max": 2, "curriculum": True,
}
_ROUGH_TRAIN_V2 = {**_ROUGH_TRAIN, "init_level_max": 6}
"""Round 3 (user decision 2026-10-06): robots start on levels 0-6 so the harder levels get more samples."""
_ROUGH_EVAL = {  # evaluation: every robot on the hardest level (+-8 mm), no curriculum
    "amplitude": 0.008, "levels": 10, "cols": 16, "tile_size": (6.0, 8.0), "robots_per_tile": 8,
    "horizontal_scale": 0.05, "fixed_level": 9,
}

_GAIT_VAR = {  # user decision 2026-10-06: standing, stepping in place, start / stop, speed and clearance changes
    "resample_interval_s": (3.0, 6.0), "zero_speed_prob": 0.1, "stand_prob": 0.1, "clearance_range": (0.010, 0.035),
}

SCENARIOS: dict[str, dict] = {
    # 03 environment: flat ground, ideal actuators, no noise. Frozen 2026-10-06.
    "flat_v1": {},
    # flat ground, half of the moderate ranges (evaluation level "light")
    "dr_light_v1": {"dr": _LIGHT_DR},
    # flat ground, moderate ranges (user decision 2026-10-06), no terrain
    "dr_v1": {"dr": _MODERATE_DR},
    # uneven ground only (ideal actuators), hardest level, for evaluation
    "rough_eval_v1": {"dr": {"rough_terrain": _ROUGH_EVAL}},
    # moderate randomisation + uneven ground at the hardest level, for evaluation
    "dr_rough_eval_v1": {"dr": {**_MODERATE_DR, "rough_terrain": _ROUGH_EVAL}},
    # training: moderate randomisation + uneven ground with a curriculum
    "dr_rough_train_v1": {"dr": {**_MODERATE_DR, "rough_terrain": _ROUGH_TRAIN}},
    # flat ground, ideal actuators, changing commands (stand / in place / start-stop / clearance); evaluation of diversity
    "gait_eval_v1": {"gait_var": _GAIT_VAR},
    # training: moderate randomisation + curriculum terrain + changing commands (phases 1 + 3)
    "dr_rough_gait_train_v1": {"dr": {**_MODERATE_DR, "rough_terrain": _ROUGH_TRAIN}, "gait_var": _GAIT_VAR},
    # training round 3: as v1, but initial terrain levels 0-6 (more samples on the harder levels)
    "dr_rough_gait_train_v2": {"dr": {**_MODERATE_DR, "rough_terrain": _ROUGH_TRAIN_V2}, "gait_var": _GAIT_VAR},
}
"""Scenario name -> overrides. ``dr`` / ``gait_var`` entries go to ``BipedWalkEnvCfg.dr`` / ``.gait_var``, every other
key to the env config."""

SINGLE_ITEM_PROBES: dict[str, dict] = {k: {"dr": {k: v}} for k, v in _MODERATE_DR.items()}
"""One moderate-range item at a time (diagnostic only, not a training scenario): which item hurts a policy most."""


def apply_scenario(env_cfg, name: str) -> None:
    """Set the overrides of scenario ``name`` (or ``probe:<item>``) on a ``BipedWalkEnvCfg`` in place."""
    if name.startswith("probe:"):
        overrides = SINGLE_ITEM_PROBES[name.split(":", 1)[1]]
    elif name.startswith("stress_v1/"):
        overrides = STRESS_V1[name.split("/", 1)[1]]
    elif name.startswith("stress_v2/"):
        overrides = STRESS_V2[name.split("/", 1)[1]]
    else:
        overrides = SCENARIOS[name]
    for k, v in overrides.items():
        if k.startswith("_"):  # runner-only keys (stress tests)
            continue
        if k in ("dr", "gait_var"):
            sub = getattr(env_cfg, k)
            for dk, dv in v.items():
                if not hasattr(sub, dk):
                    raise KeyError(f"unknown {k} field {dk!r}")
                setattr(sub, dk, dv)
        else:
            if not hasattr(env_cfg, k):
                raise KeyError(f"unknown env field {k!r}")
            setattr(env_cfg, k, v)
    env_cfg.scenario = name


# ----------------------------------------------------------------------------------------------------------------------
# Stress tests v1 (04_generalization, 2026-10-09): evaluation only, frozen once used. One factor at a time on top of flat_v1
# (flat ground, ideal actuators, no noise). Names "stress_v1/<id>"; metadata in STRESS_V1_META.
# ----------------------------------------------------------------------------------------------------------------------
_TILE = {"levels": 1, "cols": 13, "tile_size": (8.0, 8.0), "robots_per_tile": 8, "horizontal_scale": 0.05,
         "fixed_level": 0, "exact": True}
STRESS_V1: dict[str, dict] = {}
STRESS_V1_META: dict[str, dict] = {}
"""id -> {test, title, value, unit, in_training}; ``_push_speed`` / ``_video_cmd`` / ``_video_freq`` keys are for the runner."""


def _add(test: str, title: str, value, unit: str, in_training: bool, overrides: dict, tag: str):
    key = f"{test}_{tag}"
    STRESS_V1[key] = overrides
    STRESS_V1_META[key] = {"test": test, "title": title, "value": value, "unit": unit, "in_training": in_training}


for ms in (0, 10, 20, 30, 40, 60, 80, 100):
    _add("01_delay", "伺服延遲", ms, "ms", ms <= 40, {"dr": {"action_delay_phys_steps": (ms // 5, ms // 5)}}, f"{ms:03d}ms")
_add("02_bumps", "凸起高度", 0, "mm", True, {}, "00mm")
for mm in (4, 8, 10, 12, 16):
    _add("02_bumps", "凸起高度", mm, "mm", mm <= 8,
         {"dr": {"rough_terrain": {**_TILE, "amplitude": mm / 1000}}}, f"{mm:02d}mm")
for cm in (5, 10, 20):
    _add("03_bump_spacing", "凸起間距（高度 8 mm）", cm, "cm", cm == 10,
         {"dr": {"rough_terrain": {**_TILE, "amplitude": 0.008, "bump_spacing": cm / 100}}}, f"{cm:02d}cm")
for mu in (0.2, 0.3, 0.4, 0.8, 1.2, 1.5):
    _add("04_friction", "地面摩擦", mu, "-", 0.4 <= mu <= 1.2, {"dr": {"foot_friction": (mu, mu)}}, f"{mu:.1f}")
for pct in (50, 65, 80, 100):
    _add("05_motor", "馬達力量（扭矩上限）", pct, "%", pct >= 80, {"dr": {"effort_scale": (pct / 100, pct / 100)}}, f"{pct:03d}pct")
for kind in ("up", "down"):
    for deg in (3, 5, 8):
        _add("06_slope", f"斜坡（{'上' if kind == 'up' else '下'}坡）", deg if kind == "up" else -deg, "deg", False,
             {"dr": {"rough_terrain": {**_TILE, "amplitude": 0.0, "spawn_x": 0.0, "spawn_z_abs": 0.0,
                                       "profile": {"kind": f"ramp_{kind}", "slope_deg": float(deg), "flat_until": 0.3}}}},
             f"{kind}_{deg}deg")
for kind in ("up", "down"):
    for mm in (5, 10):
        _add("07_steps", f"台階（{'上' if kind == 'up' else '下'}）", mm if kind == "up" else -mm, "mm", False,
             {"dr": {"rough_terrain": {**_TILE, "amplitude": 0.0, "spawn_x": 0.0, "spawn_z_abs": 0.0, "slope_threshold": 0.15,
                                       "profile": {"kind": f"steps_{kind}", "step_height": mm / 1000, "step_length": 0.25,
                                                   "flat_until": 0.3}}}},
             f"{kind}_{mm:02d}mm")
for m in (1.0, 1.15, 1.3, 1.5):
    _add("08_mass", "整機質量", m, "x", m <= 1.15, {"dr": {"mass_scale": (m, m)}} if m != 1.0 else {}, f"{int(m * 100):03d}")
for v in (0.1, 0.15, 0.25, 0.35, 0.45):
    _add("09_push", "推擠強度", v, "m/s", v <= 0.25, {"_push_speed": v}, f"{int(v * 100):02d}cms")
for k in (1, 2, 3):
    _add("10_imu_noise", "IMU 雜訊倍數", k, "x", k == 1,
         {"dr": {"obs_noise_ang_vel": 0.1 * k, "obs_noise_gravity": 0.03 * k, "obs_noise_joint_pos": 0.01 * k,
                 "obs_noise_joint_vel": 0.3 * k}}, f"x{k}")
for d in (2, 5, 10):
    _add("11_heading_bias", "IMU 方向偏差（±）", d, "deg", d <= 2, {"dr": {"heading_bias": float(np.radians(d))}}, f"{d:02d}deg")
for v in (0.18, 0.22):
    _add("12_command", "速度指令", v, "m/s", False, {"cmd_vx_range": (v, v), "_video_cmd": v}, f"speed_{int(v * 100):02d}cms")
for f in (1.0, 2.3):
    _add("12_command", "步頻指令", f, "Hz", False, {"gait_freq_range": (f, f), "_video_freq": f}, f"freq_{int(f * 10):02d}")

# ----------------------------------------------------------------------------------------------------------------------
# Stress tests v2 (04_generalization, 2026-10-09): stress_v1 measured the fall height from the fixed spawn height, so
# walking ~10 cm down a slope or steps counted as a fall (and a fall on an up-slope could be missed). v2 repeats the slope
# and step conditions (plus the flat baseline) with the fall height measured from the ground under the feet
# (``fall_height_ref="feet"``). Names "stress_v2/<id>" (same ids as v1). Frozen once used.
# ----------------------------------------------------------------------------------------------------------------------
STRESS_V2: dict[str, dict] = {k: {**v, "fall_height_ref": "feet"} for k, v in STRESS_V1.items()
                              if k == "02_bumps_00mm" or k.startswith(("06_", "07_"))}
STRESS_V2_META: dict[str, dict] = {k: STRESS_V1_META[k] for k in STRESS_V2}
