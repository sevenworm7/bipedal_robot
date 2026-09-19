"""Batched rollout helpers for the rule-based controller (no simulator imports; works with any env
that follows ``shared/env/biped_env.py``).

``run_first_episodes`` runs every environment until its first episode ends and returns one summary
per environment (later episodes of fast-falling envs are ignored), plus fall-direction and torque
statistics.
"""

from __future__ import annotations

import itertools
import math

import torch

from shared.metrics import rule_based_success


def parse_sweep(specs: list[str]) -> tuple[list[str], list[tuple[float, ...]]]:
    """``["a=1,2", "b=0.1,0.2"]`` -> names, Cartesian product of values."""
    names, values = [], []
    for spec in specs:
        k, v = spec.split("=", 1)
        names.append(k.strip())
        values.append([float(x) for x in v.split(",")])
    return names, list(itertools.product(*values)) if names else [()]


def run_first_episodes(env, ctrl, push=None, torque_every: int = 0, frame_cb=None) -> dict:
    """Run until every env has finished one episode.

    Args:
        env: a ``BipedWalkEnv`` with ``cfg.record_episodes = True``.
        ctrl: object with ``reset(env_ids)`` and ``compute_action(obs)``.
        push: optional callable ``push(env, step)`` applying disturbances before each step.
        torque_every: if > 0, sample |leg torque| of all envs every this many steps (for percentiles).
        frame_cb: optional callable ``frame_cb(env, step, terminated)`` called after each step (videos).

    Returns:
        dict of per-env tensors (first episode of each env): the ``WalkTracker`` summary keys plus
        ``success`` and ``fall_dir`` (0 none, 1 forward, 2 backward, 3 left, 4 right), and
        ``torque_samples`` [S, J] when requested.
    """
    n = env.num_envs
    dev = env.device
    env.completed_episodes.clear()
    obs = env.reset()[0]["policy"]
    ctrl.reset()
    seen = torch.zeros(n, dtype=torch.bool, device=dev)
    first: dict[str, torch.Tensor] = {}
    fall_dir = torch.zeros(n, device=dev)
    torque = []
    step = 0
    max_steps = int(env.max_episode_length) + 5
    while not bool(seen.all()) and step < max_steps:
        if push is not None:
            push(env, step)
        act = ctrl.compute_action(obs)
        prev_g = obs[:, 6:9]
        out = env.step(act)
        obs, term, trunc = out[0]["policy"], out[2], out[3]
        done = term | trunc
        if bool(term.any()):
            # direction of the fall from the tilt one control step earlier
            g = prev_g[term]
            d = torch.where(g[:, 0].abs() >= g[:, 1].abs(),
                            torch.where(g[:, 0] > 0, 1.0, 2.0), torch.where(g[:, 1] < 0, 3.0, 4.0))
            ids = term.nonzero().flatten()
            new = ~seen[ids]
            fall_dir[ids[new]] = d[new]
        if torque_every and step % torque_every == 0:
            torque.append(env.leg_torque.abs()[~seen].clone())
        if frame_cb is not None:
            frame_cb(env, step, term)
        for ep in env.completed_episodes:
            ids = ep["env_ids"]
            new = ~seen[ids]
            if bool(new.any()):
                for k, v in ep.items():
                    if k not in first:
                        first[k] = torch.zeros((n,) + tuple(v.shape[1:]), dtype=v.dtype, device=dev)
                    first[k][ids[new]] = v[new]
                seen[ids[new]] = True
        env.completed_episodes.clear()
        if bool(done.any()):
            ctrl.reset(done.nonzero().flatten())
        step += 1
    first["success"] = rule_based_success(first).float()
    first["fall_dir"] = fall_dir
    if torque:
        first["torque_samples"] = torch.cat(torque, 0)
    return first


def aggregate(ep: dict[str, torch.Tensor], mask: torch.Tensor | None = None) -> dict:
    """Scalar statistics over the episodes selected by ``mask``."""
    if mask is None:
        mask = torch.ones_like(ep["fell"], dtype=torch.bool)
    m = mask.bool()
    k = int(m.sum())
    if k == 0:
        return {"episodes": 0}

    def mean(key):
        return round(float(ep[key][m].float().mean()), 4)

    fell = ep["fell"][m] > 0.5
    fd = ep["fall_dir"][m]
    out = {
        "episodes": k,
        "success_rate": mean("success"),
        "fall_rate": round(float(fell.float().mean()), 4),
        "mean_steps": mean("steps"),
        "mean_distance_m": mean("distance"),
        "mean_time_s": mean("time"),
        "mean_vx": mean("mean_vx"),
        "mean_cmd_vx": mean("cmd_vx"),
        "mean_max_abs_lateral_m": mean("max_abs_lateral"),
        "mean_abs_final_lateral_m": round(float(ep["lateral"][m].abs().mean()), 4),
        "mean_max_abs_heading_err_deg": round(math.degrees(float(ep["max_abs_heading_error"][m].mean())), 2),
        "torque_sat_frac": mean("torque_sat_frac"),
        "fall_dir": {name: int((fd == i).sum()) for i, name in ((1, "forward"), (2, "backward"), (3, "left"), (4, "right"))},
    }
    ok = ~fell
    if bool(ok.any()):
        v_err = (ep["mean_vx"][m][ok] - ep["cmd_vx"][m][ok]).abs() / ep["cmd_vx"][m][ok].clamp_min(1e-3)
        out["mean_rel_speed_err_non_fallen"] = round(float(v_err.mean()), 4)
    return out


def torque_stats(ep: dict[str, torch.Tensor], joint_names) -> dict:
    out = {}
    samples = ep.get("torque_samples")
    for j, name in enumerate(joint_names):
        row = {"mean": round(float(ep["torque_abs_mean"][:, j].mean()), 2),
               "max": round(float(ep["torque_abs_max"][:, j].max()), 2)}
        if samples is not None and samples.numel():
            row["p95"] = round(float(torch.quantile(samples[:, j].float()[:100000], 0.95)), 2)
        out[name] = row
    return out
