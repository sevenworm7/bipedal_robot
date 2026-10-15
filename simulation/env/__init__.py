"""Straight-walking environment for experiment 2 (ported from ``archive/exp1_reference/shared/env``).

Registered gym id: ``Biped-Walk-v0`` (robot selected by ``BipedWalkEnvCfg.robot_name``).
Import this package only after the Isaac Sim app has been launched.
"""

import gymnasium as gym

from .biped_env import BipedWalkEnv
from .biped_env_cfg import BipedWalkEnvCfg, RewardWeightsCfg

TASK_ID = "Biped-Walk-v0"

if TASK_ID not in gym.registry:
    gym.register(
        id=TASK_ID,
        entry_point=f"{__name__}.biped_env:BipedWalkEnv",
        disable_env_checker=True,
        kwargs={"env_cfg_entry_point": f"{__name__}.biped_env_cfg:BipedWalkEnvCfg"},
    )

__all__ = ["BipedWalkEnv", "BipedWalkEnvCfg", "RewardWeightsCfg", "TASK_ID"]
