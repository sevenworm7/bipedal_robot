"""Fixed evaluation set shared by every stage (teacher, BC, PPO, distilled BC).

Same protocol and ``stats.json`` schema as ``rule_based/evaluate.py`` so all numbers are comparable:
``EvalCfg.episodes`` parallel episodes of ``EvalCfg.episode_s`` with the env's command / gait-frequency /
initial-pose randomisation and a fixed seed, plus a separate push test (reference only).
"""

from __future__ import annotations

import torch

from simulation.rollout import aggregate, binned, make_push, run_first_episodes, torque_stats

from .config import EvalCfg


def evaluate_agent(env, agent, cfg: EvalCfg, push: bool = True) -> tuple[dict, dict]:
    """Returns (stats, per-episode tensors of the nominal run)."""
    torch.manual_seed(cfg.seed)
    ep = run_first_episodes(env, agent, torque_every=5)
    lo, hi = env.cfg.cmd_vx_range
    flo, fhi = env.cfg.gait_freq_range
    ok = ep["fell"] < 0.5
    speed_ratio = (ep["mean_vx"] / ep["cmd_vx"].clamp_min(1e-3))[ok]
    lat = ep["max_abs_lateral"]
    head = ep["max_abs_heading_error"] * 57.2958

    def pct(x):
        return [round(float(torch.quantile(x.float(), q)), 4) for q in (0.1, 0.5, 0.9)] if x.numel() else None

    stats = {
        "env": {"episodes": env.num_envs, "episode_s": env.cfg.episode_length_s, "cmd_vx_range": [lo, hi],
                "gait_freq_range": [flo, fhi], "init_joint_noise": env.cfg.init_joint_noise, "seed": cfg.seed},
        "nominal": aggregate(ep),
        "percentiles_p10_p50_p90": {
            "speed_ratio": pct(speed_ratio),
            "max_abs_lateral_m": pct(lat),
            "max_abs_heading_err_deg": pct(head),
            "steps_per_clock_step": pct((ep["steps"] / (2 * ep["gait_freq"] * ep["time"].clamp_min(1e-3)))[ok]),
        },
        "by_cmd_vx": binned(ep, "cmd_vx", [lo, lo + (hi - lo) / 3, lo + 2 * (hi - lo) / 3, hi]),
        "by_gait_freq": binned(ep, "gait_freq", [flo, flo + (fhi - flo) / 3, flo + 2 * (fhi - flo) / 3, fhi]),
        "torque_Nm": torque_stats(ep, env.rb.ACTION_JOINT_NAMES),
        "reward_terms_mean": {k[4:]: round(float(v.mean()), 4) for k, v in ep.items() if k.startswith("rew_")},
    }
    stats["nominal"]["passes_rule_based_gate"] = stats["nominal"]["success_rate"] >= 0.70
    if push:
        ep_push = run_first_episodes(env, agent, push=make_push(cfg.push_speed, list(cfg.push_times_s)))
        stats["push_test"] = {"push_speed_mps": cfg.push_speed, "push_times_s": list(cfg.push_times_s),
                              "note": "reference only", **aggregate(ep_push)}
    return stats, ep


def summary_line(stats: dict) -> str:
    n = stats["nominal"]
    p = stats["percentiles_p10_p50_p90"]
    s = (f"success {n['success_rate']:.3f} fall {n['fall_rate']:.3f} dist {n['mean_distance_m']:.2f} m "
         f"vx {n['mean_vx']:.3f}/{n['mean_cmd_vx']:.3f} speed-ratio p50 {p['speed_ratio'][1] if p['speed_ratio'] else None} "
         f"lat p50 {p['max_abs_lateral_m'][1]:.3f} m heading p50 {p['max_abs_heading_err_deg'][1]:.1f} deg")
    if "push_test" in stats:
        s += f" | push success {stats['push_test']['success_rate']:.3f} fall {stats['push_test']['fall_rate']:.3f}"
    return s
