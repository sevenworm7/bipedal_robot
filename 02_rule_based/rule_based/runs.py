"""Run folders of experiment 2: ``outputs/robot_v<N>/rule_based/<name>_<YYYYMMDD_HHMMSS>/``, never overwritten."""

from __future__ import annotations

import time
from pathlib import Path


def new_run_dir(rb, name: str) -> Path:
    """Create a new run folder under the robot version's output root; stops if it already exists."""
    out = Path(rb.TEST_ROOT) / "rule_based" / f"{name}_{time.strftime('%Y%m%d_%H%M%S')}"
    out.mkdir(parents=True, exist_ok=False)
    return out
