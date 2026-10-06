"""Sim-to-real success criterion (user decision 2026-10-06), computed from an evaluation's ``episodes.pt``.

An episode succeeds when the robot did not fall, ran the whole episode, its mean forward speed is within
+-``speed_tol`` of the command and its maximum heading error stays below ``heading_deg``. Unlike the rule-based
criterion of 02 / 03 (distance >= 0.8 x steps x step length, unchanged there), extra corrective steps are allowed;
shuffling is still caught by the speed condition. Episodes whose command changes mid-run (``gait_eval_*``) or is 0
skip the speed condition. ``steps_per_clock_step`` stays a reference metric.
"""

from __future__ import annotations

import torch

SPEED_TOL = 0.25
HEADING_DEG = 10.0


def sim2real_success(ep: dict, episode_s: float, dt: float = 0.02, speed_check: bool = True) -> dict[str, torch.Tensor]:
    fell = ep["fell"] > 0.5
    full = ep["time"] >= episode_s - 2 * dt
    cmd = ep["cmd_vx"]
    speed_ok = (ep["mean_vx"] / cmd.clamp_min(1e-3) - 1.0).abs() < SPEED_TOL
    if not speed_check:
        speed_ok = torch.ones_like(speed_ok)
    speed_ok = speed_ok | (cmd <= 1e-6)
    head_ok = ep["max_abs_heading_error"] * 57.29578 < HEADING_DEG
    return {"not_fallen_full": ~fell & full, "speed_within_25pct": speed_ok, "heading_below_10deg": head_ok,
            "success": ~fell & full & speed_ok & head_ok}


def summarize(ep: dict, episode_s: float, speed_check: bool = True) -> dict:
    s = sim2real_success(ep, episode_s, speed_check=speed_check)
    out = {k: round(float(v.float().mean()), 4) for k, v in s.items()}
    out["fall_rate"] = round(float((ep["fell"] > 0.5).float().mean()), 4)
    clock = (2 * ep["gait_freq"] * ep["time"]).clamp_min(1e-6)
    ok = (ep["fell"] < 0.5) & (ep["gait_freq"] > 0)
    if bool(ok.any()):
        out["steps_per_clock_step_p50"] = round(float((ep["steps"] / clock)[ok].median()), 3)
    return out
