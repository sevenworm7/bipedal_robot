"""Named, versioned environment scenarios (04_sim2real, user rule 2026-10-06: never overwrite an old scene).

A scenario is a set of ``BipedWalkEnvCfg`` overrides. Rules:

* ``flat_v1`` is the 03 environment (no overrides) and must stay bit-identical to it; every change to
  ``shared/env`` is checked by re-running the 03 final model on ``flat_v1`` (``04_sim2real/sim2real/regress.py``).
* Once a scenario has been used by a recorded run it is **frozen**: never edit its entry. To change anything, add a
  new entry with the next version suffix (``dr_v2``) and keep the old one.
* Every run records ``scenario`` (written into ``BipedWalkEnvCfg.scenario`` and the run config).

Ranges follow the user decisions of 2026-10-06 (04_sim2real/README.md): delay 0-40 ms, kp / kd +-30 %, effort +-20 %,
friction 0.4-1.2, mass +-15 %; actuator ranges are wide estimates until one HD-1910 has been measured.
"""

from __future__ import annotations

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

_ROUGH_TRAIN = {  # +-8 mm bumps at the hardest of 10 levels (user decision 2026-10-06), curriculum
    "amplitude": 0.008, "levels": 10, "cols": 32, "tile_size": (6.0, 8.0), "robots_per_tile": 8,
    "horizontal_scale": 0.05, "init_level_max": 2, "curriculum": True,
}
_ROUGH_EVAL = {  # evaluation: every robot on the hardest level (+-8 mm), no curriculum
    "amplitude": 0.008, "levels": 10, "cols": 16, "tile_size": (6.0, 8.0), "robots_per_tile": 8,
    "horizontal_scale": 0.05, "fixed_level": 9,
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
}
"""Scenario name -> overrides. ``dr`` entries go to ``BipedWalkEnvCfg.dr``, every other key to the env config."""

SINGLE_ITEM_PROBES: dict[str, dict] = {k: {"dr": {k: v}} for k, v in _MODERATE_DR.items()}
"""One moderate-range item at a time (diagnostic only, not a training scenario): which item hurts a policy most."""


def apply_scenario(env_cfg, name: str) -> None:
    """Set the overrides of scenario ``name`` (or ``probe:<item>``) on a ``BipedWalkEnvCfg`` in place."""
    if name.startswith("probe:"):
        overrides = SINGLE_ITEM_PROBES[name.split(":", 1)[1]]
    else:
        overrides = SCENARIOS[name]
    for k, v in overrides.items():
        if k == "dr":
            for dk, dv in v.items():
                if not hasattr(env_cfg.dr, dk):
                    raise KeyError(f"unknown dr field {dk!r}")
                setattr(env_cfg.dr, dk, dv)
        else:
            if not hasattr(env_cfg, k):
                raise KeyError(f"unknown env field {k!r}")
            setattr(env_cfg, k, v)
    env_cfg.scenario = name
