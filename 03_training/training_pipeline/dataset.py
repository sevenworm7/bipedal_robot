"""(state, action) datasets: parallel collection in the sim and HDF5 storage.

HDF5 layout (one file per collection run)::

    /obs        [T, obs_dim] float32   observation (shared/interface.py order)
    /action     [T, act_dim] float32   label: clean action of the agent (teacher / policy mean)
    /exec       [T, act_dim] float32   action actually executed (label + exploration noise)
    /episodes/  one entry per episode, arrays of length E:
        start, length                  row range in /obs
        env_id, cmd_vx, gait_freq, step_length_cmd
        fell, success, steps, distance, lateral, max_abs_lateral, max_abs_heading_error,
        mean_vx, time, torque_sat_frac, ret (sum of env rewards), rew_<term> (per-term sums)
    attrs: obs_dim, act_dim, robot, source, created, config (json)

Episodes are complete: every env runs until its first episode ends (fall or timeout), so episode
boundaries are exact and the train/val split is done by episode.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import h5py
import numpy as np
import torch

from shared.metrics import rule_based_success

EPISODE_KEYS = ("cmd_vx", "gait_freq", "step_length_cmd", "fell", "steps", "distance", "lateral", "max_abs_lateral",
                "max_abs_heading_error", "mean_vx", "time", "torque_sat_frac")


@torch.no_grad()
def collect(env, agent, exec_noise_std: float = 0.0, max_steps: int | None = None) -> dict:
    """Run every env until its first episode ends and record (obs, label, executed action).

    ``agent.compute_action`` returns the executed action; the label is ``agent.last_mean`` (clean action).
    Teacher agents return the clean action, so ``exec_noise_std`` adds DART-style noise on top.
    Returns a dict of CPU numpy arrays in the HDF5 layout.
    """
    n, dev = env.num_envs, env.device
    t_max = int(max_steps or env.max_episode_length) + 2
    obs_dim = env.layout.dim
    act_dim = env.num_legs
    buf_obs = torch.zeros(t_max, n, obs_dim, device=dev)
    buf_lab = torch.zeros(t_max, n, act_dim, device=dev)
    buf_exe = torch.zeros(t_max, n, act_dim, device=dev)
    length = torch.zeros(n, dtype=torch.long, device=dev)
    ret = torch.zeros(n, device=dev)
    done_first = torch.zeros(n, dtype=torch.bool, device=dev)
    summaries: dict[str, torch.Tensor] = {}

    # reset first, then clear: the reset ends episodes still running from a previous call, and their partial
    # summaries must not count as first episodes of this call (bug fixed 2026-10-06; inherited from 00_test)
    obs = env.reset()[0]["policy"]
    env.completed_episodes.clear()
    agent.reset()
    t = 0
    while not bool(done_first.all()) and t < t_max:
        act = agent.compute_action(obs)
        label = agent.last_mean if agent.last_mean is not None else act
        if exec_noise_std > 0:
            act = act + exec_noise_std * torch.randn_like(act)
        active = ~done_first
        buf_obs[t] = obs
        buf_lab[t] = label
        buf_exe[t] = act
        o, rew, term, trunc, _ = env.step(act)
        ret += rew * active
        length += active.long()
        obs = o["policy"]
        for ep in env.completed_episodes:
            ids = ep["env_ids"]
            new = ~done_first[ids]
            for k, v in ep.items():
                if k not in summaries:
                    summaries[k] = torch.zeros((n,) + tuple(v.shape[1:]), dtype=v.dtype, device=dev)
                summaries[k][ids[new]] = v[new]
            done_first[ids[new]] = True
        env.completed_episodes.clear()
        done = term | trunc
        if bool(done.any()):
            agent.reset(done.nonzero().flatten())
        t += 1

    keep = done_first.nonzero().flatten()
    lens = length[keep]
    rows_obs, rows_lab, rows_exe = [], [], []
    for i in keep.tolist():
        L = int(length[i])
        rows_obs.append(buf_obs[:L, i])
        rows_lab.append(buf_lab[:L, i])
        rows_exe.append(buf_exe[:L, i])
    start = torch.cumsum(lens, 0) - lens
    keys = list(EPISODE_KEYS) + sorted(k for k in summaries if k.startswith("rew_"))
    episodes = {k: summaries[k][keep].cpu().numpy() for k in keys}
    ep_t = {k: summaries[k][keep] for k in ("fell", "steps", "distance", "step_length_cmd")}
    episodes["success"] = rule_based_success(ep_t).float().cpu().numpy()
    episodes["ret"] = ret[keep].cpu().numpy()
    episodes["env_id"] = keep.cpu().numpy()
    episodes["start"] = start.cpu().numpy()
    episodes["length"] = lens.cpu().numpy()
    return {
        "obs": torch.cat(rows_obs).cpu().numpy(),
        "action": torch.cat(rows_lab).cpu().numpy(),
        "exec": torch.cat(rows_exe).cpu().numpy(),
        "episodes": episodes,
    }


def merge(parts: list[dict]) -> dict:
    """Concatenate several ``collect`` results (episode starts are re-based). Only episode fields present in
    every part are kept (e.g. datasets collected before a reward term was added lack its ``rew_<term>``)."""
    out = {k: np.concatenate([p[k] for p in parts]) for k in ("obs", "action", "exec")}
    eps, offset = [], 0
    for p in parts:
        e = dict(p["episodes"])
        e["start"] = e["start"] + offset
        offset += len(p["obs"])
        eps.append(e)
    common = [k for k in eps[0] if all(k in e for e in eps)]
    out["episodes"] = {k: np.concatenate([e[k] for e in eps]) for k in common}
    return out


def select_episodes(data: dict, idx: np.ndarray) -> dict:
    """Subset of episodes (rows copied, starts re-based)."""
    ep = data["episodes"]
    rows = np.concatenate([np.arange(ep["start"][i], ep["start"][i] + ep["length"][i]) for i in idx]) if len(idx) else np.zeros(0, int)
    out = {k: data[k][rows] for k in ("obs", "action", "exec")}
    e = {k: v[idx] for k, v in ep.items()}
    e["start"] = np.cumsum(e["length"]) - e["length"]
    out["episodes"] = e
    return out


def save_h5(path: Path, data: dict, attrs: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(path, "w") as f:
        for k in ("obs", "action", "exec"):
            f.create_dataset(k, data=data[k].astype(np.float32), compression="lzf")
        g = f.create_group("episodes")
        for k, v in data["episodes"].items():
            g.create_dataset(k, data=v)
        f.attrs["obs_dim"] = data["obs"].shape[1]
        f.attrs["act_dim"] = data["action"].shape[1]
        f.attrs["created"] = time.strftime("%Y-%m-%d %H:%M:%S")
        for k, v in attrs.items():
            f.attrs[k] = v if isinstance(v, (int, float, str)) else json.dumps(v)


def load_h5(path: Path) -> dict:
    with h5py.File(path, "r") as f:
        data = {k: f[k][()] for k in ("obs", "action", "exec")}
        data["episodes"] = {k: f["episodes"][k][()] for k in f["episodes"]}
        data["attrs"] = dict(f.attrs)
    return data


def episode_stats(data: dict) -> dict:
    """Summary of a dataset (for stats.json)."""
    e = data["episodes"]
    ok = e["fell"] < 0.5
    speed_ratio = e["mean_vx"][ok] / np.maximum(e["cmd_vx"][ok], 1e-3)
    return {
        "episodes": int(len(e["length"])),
        "steps": int(e["length"].sum()),
        "fall_rate": round(float(1 - ok.mean()), 4),
        "success_rate": round(float(e["success"].mean()), 4),
        "mean_return": round(float(e["ret"].mean()), 3),
        "mean_distance_m": round(float(e["distance"].mean()), 3),
        "speed_ratio_median": round(float(np.median(speed_ratio)), 3) if ok.any() else None,
        "max_abs_lateral_median_m": round(float(np.median(e["max_abs_lateral"])), 4),
        "cmd_vx_range": [round(float(e["cmd_vx"].min()), 3), round(float(e["cmd_vx"].max()), 3)],
        "gait_freq_range": [round(float(e["gait_freq"].min()), 3), round(float(e["gait_freq"].max()), 3)],
        "nan_rows": int(np.isnan(data["obs"]).any(1).sum() + np.isnan(data["action"]).any(1).sum()),
        "action_abs_max": round(float(np.abs(data["action"]).max()), 3),
        "reward_terms_mean": {k[4:]: round(float(v.mean()), 4) for k, v in e.items() if k.startswith("rew_")},
    }
