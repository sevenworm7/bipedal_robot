"""Helpers of the sim-to-real stage. Reuses the 03 pipeline modules (policy loading, fixed evaluation set) instead of
copying them; adds named scenarios (``shared/env/scenarios.py``) and the ``outputs/robot_v<N>/03_training/sim2real/`` run
folders."""

from __future__ import annotations

import json
import subprocess
import time
from pathlib import Path

import yaml


def git_commit(root: Path) -> str:
    try:
        out = subprocess.run(["git", "-C", str(root), "rev-parse", "--short", "HEAD"], capture_output=True, text=True)
        dirty = subprocess.run(["git", "-C", str(root), "status", "--porcelain"], capture_output=True, text=True).stdout.strip()
        return out.stdout.strip() + ("+dirty" if dirty else "")
    except OSError:
        return "unknown"


def run_dir(rb, stage: str, tag: str = "") -> Path:
    """``outputs/robot_v<N>/03_training/sim2real/<stage>[_<tag>]_<YYYYMMDD_HHMMSS>/`` (never overwritten)."""
    name = stage + (f"_{tag}" if tag else "") + f"_{time.strftime('%Y%m%d_%H%M%S')}"
    path = Path(rb.TEST_ROOT) / "03_training" / "sim2real" / name
    path.mkdir(parents=True, exist_ok=False)
    return path


def make_env(pipe_cfg, scenario: str, num_envs: int, episode_s: float, seed: int, record: bool = True, **overrides):
    """``BipedWalkEnv`` with the 03 pipeline overrides (gait window, reward weights), then the scenario, then ``overrides``."""
    from shared.env import BipedWalkEnv, BipedWalkEnvCfg
    from shared.env.scenarios import apply_scenario

    env_cfg = BipedWalkEnvCfg()
    env_cfg.robot_name = pipe_cfg.robot
    env_cfg.__post_init__()
    env_cfg.scene.num_envs = num_envs
    env_cfg.episode_length_s = episode_s
    env_cfg.record_episodes = record
    env_cfg.seed = seed
    for k, v in pipe_cfg.env_overrides.items():
        setattr(env_cfg, k, v)
    for k, v in pipe_cfg.reward_overrides.items():
        setattr(env_cfg.rewards, k, v)
    apply_scenario(env_cfg, scenario)
    for k, v in overrides.items():
        setattr(env_cfg, k, v)
    return BipedWalkEnv(env_cfg)


def dump_config(path: Path, data: dict) -> None:
    (path / "config.yaml").write_text(yaml.safe_dump(json.loads(json.dumps(data, default=str)), sort_keys=False,
                                                     allow_unicode=True), encoding="utf-8")
