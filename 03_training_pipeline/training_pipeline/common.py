"""Helpers shared by the stage scripts: run folders, config dumps, env construction.

Only ``make_env`` needs a running Isaac Sim app (the env is imported lazily), so plain-Python stages
such as ``filter.py`` can use the rest.
"""

from __future__ import annotations

import json
import time
from dataclasses import asdict, is_dataclass
from pathlib import Path

import yaml

from .config import PipelineCfg


def run_dir(cfg: PipelineCfg, iteration: int, stage: str, tag: str = "") -> Path:
    """``outputs/robot_v<N>/training/iter<N>_<stage>[_<tag>]_<YYYYMMDD_HHMMSS>/`` (never overwritten)."""
    from shared.robots import get_robot

    rb = get_robot(cfg.robot)
    name = f"iter{iteration}_{stage}" + (f"_{tag}" if tag else "") + f"_{time.strftime('%Y%m%d_%H%M%S')}"
    path = rb.TEST_ROOT / "training" / name
    path.mkdir(parents=True, exist_ok=False)
    return path


def dump_config(path: Path, cfg: PipelineCfg, args: dict | None = None, env_cfg=None, extra: dict | None = None) -> None:
    """Write ``config.yaml`` with the pipeline config, CLI arguments and the relevant env settings."""
    data = {"pipeline": cfg.to_dict(), "args": args or {}}
    if env_cfg is not None:
        data["env"] = env_summary(env_cfg)
    if extra:
        data.update(extra)
    path.mkdir(parents=True, exist_ok=True)
    (path / "config.yaml").write_text(yaml.safe_dump(json.loads(json.dumps(data, default=str)), sort_keys=False,
                                                     allow_unicode=True), encoding="utf-8")


def env_summary(env_cfg) -> dict:
    return {
        "robot_name": env_cfg.robot_name,
        "num_envs": env_cfg.scene.num_envs,
        "episode_length_s": env_cfg.episode_length_s,
        "cmd_vx_range": list(env_cfg.cmd_vx_range),
        "gait_freq_range": list(env_cfg.gait_freq_range),
        "init_joint_noise": env_cfg.init_joint_noise,
        "push_interval_s": list(env_cfg.push_interval_s) if env_cfg.push_interval_s else None,
        "push_speed": env_cfg.push_speed,
        "rewards": env_cfg.rewards.to_dict(),
        "sigma_vx": env_cfg.sigma_vx,
        "sigma_heading": env_cfg.sigma_heading,
        "target_air_time": env_cfg.target_air_time,
        "gait_swing_start": env_cfg.gait_swing_start,
        "gait_swing_frac": env_cfg.gait_swing_frac,
        "contact_force_limit": env_cfg.contact_force_limit,
        "fall_com_height": env_cfg.fall_com_height,
        "min_step_air_time": env_cfg.min_step_air_time,
        "seed": env_cfg.seed,
    }


def make_env(cfg: PipelineCfg, num_envs: int, episode_s: float, seed: int, record: bool = True, **overrides):
    """``BipedWalkEnv`` for ``cfg.robot``; ``cfg.env_overrides`` then ``overrides`` are set on ``BipedWalkEnvCfg``."""
    from shared.env import BipedWalkEnv, BipedWalkEnvCfg

    env_cfg = BipedWalkEnvCfg()
    env_cfg.robot_name = cfg.robot
    env_cfg.__post_init__()
    env_cfg.scene.num_envs = num_envs
    env_cfg.episode_length_s = episode_s
    env_cfg.record_episodes = record
    env_cfg.seed = seed
    for k, v in {**cfg.env_overrides, **overrides}.items():
        setattr(env_cfg, k, v)
    for k, v in cfg.reward_overrides.items():
        assert hasattr(env_cfg.rewards, k), k
        setattr(env_cfg.rewards, k, v)
    return BipedWalkEnv(env_cfg)


def write_json(path: Path, data) -> None:
    def default(o):
        if is_dataclass(o):
            return asdict(o)
        return str(o)

    path.write_text(json.dumps(data, indent=2, default=default), encoding="utf-8")
