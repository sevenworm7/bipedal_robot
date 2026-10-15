"""Cross-entropy (CEM) search over the rule-based gait parameters in parallel envs (development step 4).

Each generation samples ``--pop`` parameter vectors, runs each on ``--envs_per`` environments with
commands and gait frequencies sampled from the full env ranges, scores them and refits a Gaussian to
the elite fraction. At the end, the best candidates are re-evaluated on more environments.

Per-env score (higher is better):
    success (official rule_based_success)
    + clip(distance / (cmd_vx * T), -0.5, 1.2)          speed tracking over the full episode
    - 0.5 * clip(steps / (2 f T) - 1.15, 0, 2)          chatter: extra touchdowns beyond the clock rate
    - 0.5 * fell
    - w_speed * clip(|mean_vx / cmd_vx - 1|, 0, 1.5)      speed tracking error
    - w_lat * clip(max |lateral| / lat_norm, 0, 3)        straightness (lat_norm 0.10 m by default)
    - w_heading * clip(max |heading error| / 10 deg, 0, 3)
The chatter term keeps the search away from gaits that bounce on the sole edges; the progress term
keeps it away from shuffling (few steps, sliding feet).

Output: ``outputs/robot_v<N>/control/rule_based/<tag>_<timestamp>/{history.json, best_params.json}``.

Usage:
    tools\\isaacpy.cmd control\rule_based\\rule_based\\optimize.py --tag cem1 --gens 25 --pop 256 --envs_per 12
"""

import argparse
import json
import sys
import time
from pathlib import Path

# project root (for shared) and control/rule_based (for the rule_based package)
sys.path[:0] = [str(Path(__file__).resolve().parents[2]), str(Path(__file__).resolve().parents[1])]

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--tag", default="cem")
parser.add_argument("--gens", type=int, default=25)
parser.add_argument("--pop", type=int, default=256)
parser.add_argument("--envs_per", type=int, default=12)
parser.add_argument("--episode_s", type=float, default=15.0)
parser.add_argument("--elite_frac", type=float, default=0.1)
parser.add_argument("--final_top", type=int, default=8)
parser.add_argument("--final_envs_per", type=int, default=256)
parser.add_argument("--init", type=str, default=None, help="JSON file with initial mean parameters.")
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--init_std", type=float, default=0.25, help="Initial std as a fraction of each range.")
parser.add_argument("--prog_cap", type=float, default=1.2, help="Upper clip of the progress term.")
parser.add_argument("--w_speed", type=float, default=0.0)
parser.add_argument("--w_lat", type=float, default=0.0)
parser.add_argument("--w_heading", type=float, default=0.0)
parser.add_argument("--lat_norm", type=float, default=0.10, help="Max lateral offset [m] that costs w_lat.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import torch  # noqa: E402

from rule_based import RuleBasedBipedController, RuleBasedCfg  # noqa: E402
from rule_based.runs import new_run_dir  # noqa: E402
from core.robots import get_robot  # noqa: E402
from simulation.rollout import aggregate, run_first_episodes  # noqa: E402
from simulation.env import BipedWalkEnv, BipedWalkEnvCfg  # noqa: E402

RB = get_robot("custom_biped")
_X0, _R0 = RB.STANCE_FOOT_X, RB.STANCE_LEG_LENGTH
BOUNDS = {
    "swing_frac": (0.15, 0.45),
    "sway_amp": (0.0, 0.25),
    "sway_lead": (-0.5, 0.2),
    "sway_shape": (0.3, 1.5),
    "swing_abduct": (-0.10, 0.15),
    "swing_x_delay": (0.0, 0.5),
    "step_scale": (0.5, 2.0),
    "stance_leg_length": (0.9 * _R0, 1.04 * _R0),
    "stance_extend": (0.0, 1.0),
    "lift_height": (0.1 * RB.MAX_LIFT, 0.6 * RB.MAX_LIFT),
    "foot_x_offset": (_X0 - 0.02, _X0 + 0.02),
    "k_vx_place": (0.0, 0.3),
    "k_vy_place": (0.0, 0.3),
    "k_lat_pos": (0.0, 2.0),
    "k_heading": (0.0, 3.0),
    "k_pitch_hip": (0.0, 1.0),
    "k_roll_hip": (0.0, 1.0),
    "k_pitch_ankle": (0.0, 0.5),
    "swing_tilt_comp": (0.0, 1.0),
    "tail_yaw_amp": (0.0, 0.8),
    "tail_yaw_lead": (-0.25, 0.25),
    "tail_pitch_offset": (-0.3, 0.5),
    "k_tail_roll": (0.0, 2.0),
    "k_tail_pitch": (-1.0, 1.0),
}
"""Search ranges; lengths scale with the robot (``STANCE_*``, ``MAX_LIFT`` of the robot module)."""
INIT: dict[str, float] = {}
"""Initial mean overrides (empty: the config defaults resolved for the robot)."""


def score_envs(ep: dict, episode_s: float) -> torch.Tensor:
    cmd = ep["cmd_vx"].clamp_min(1e-3)
    a = args_cli
    prog = (ep["distance"] / (cmd * episode_s)).clamp(-0.5, a.prog_cap)
    rate = ep["steps"] / (2.0 * ep["gait_freq"] * episode_s)
    chatter = (rate - 1.15).clamp(0.0, 2.0)
    speed_err = (ep["mean_vx"] / cmd - 1.0).abs().clamp(0.0, 1.5)
    lat = (ep["max_abs_lateral"] / a.lat_norm).clamp(0.0, 3.0)
    head = (ep["max_abs_heading_error"] / 0.1745).clamp(0.0, 3.0)
    return (ep["success"] + prog - 0.5 * chatter - 0.5 * ep["fell"]
            - a.w_speed * speed_err - a.w_lat * lat - a.w_heading * head)


def evaluate(env, ctrl, names, cand: torch.Tensor, envs_per: int, episode_s: float):
    """cand [P, D] -> per-candidate mean score [P] and the per-env episode dict."""
    p = cand.shape[0]
    cid = torch.arange(env.num_envs, device=env.device) // envs_per
    cid = cid.clamp_max(p - 1)
    for d, name in enumerate(names):
        ctrl.override(name, cand[cid, d])
    ep = run_first_episodes(env, ctrl)
    s = score_envs(ep, episode_s)
    used = torch.arange(env.num_envs, device=env.device) < p * envs_per
    scores = torch.zeros(p, device=env.device).index_add_(0, cid[used], s[used]) / envs_per
    return scores, ep, cid, used


def main():
    torch.manual_seed(args_cli.seed)
    names = list(BOUNDS)
    lo = torch.tensor([BOUNDS[n][0] for n in names])
    hi = torch.tensor([BOUNDS[n][1] for n in names])
    base = RuleBasedCfg().resolve(RB).gait
    init = dict(INIT)
    if args_cli.init:
        init.update(json.loads(Path(args_cli.init).read_text(encoding="utf-8")))
    mean = torch.tensor([float(init.get(n, getattr(base, n))) for n in names]).clamp(lo, hi)
    std = args_cli.init_std * (hi - lo)

    num_envs = max(args_cli.pop * args_cli.envs_per, args_cli.final_top * args_cli.final_envs_per)
    env_cfg = BipedWalkEnvCfg()
    env_cfg.scene.num_envs = num_envs
    env_cfg.episode_length_s = args_cli.episode_s
    env_cfg.record_episodes = True
    env = BipedWalkEnv(env_cfg)
    dev = env.device
    ctrl = RuleBasedBipedController(env.rb, env.num_envs, dev, RuleBasedCfg())
    lo_d, hi_d = lo.to(dev), hi.to(dev)

    out_dir = new_run_dir(env.rb, args_cli.tag)
    (out_dir / "objective.json").write_text(json.dumps({**vars(args_cli), "bounds": BOUNDS, "init_mean": dict(zip(names, mean.tolist())),
                                                        "cmd_vx_range": env_cfg.cmd_vx_range, "gait_freq_range": env_cfg.gait_freq_range},
                                                       indent=2), encoding="utf-8")
    history, archive = [], []
    n_elite = max(2, int(args_cli.pop * args_cli.elite_frac))
    for g in range(args_cli.gens):
        t0 = time.time()
        cand = (mean.to(dev) + std.to(dev) * torch.randn(args_cli.pop, len(names), device=dev)).clamp(lo_d, hi_d)
        cand[0] = mean.to(dev).clamp(lo_d, hi_d)  # always re-evaluate the current mean
        scores, ep, cid, used = evaluate(env, ctrl, names, cand, args_cli.envs_per, args_cli.episode_s)
        order = torch.argsort(scores, descending=True)
        elite = cand[order[:n_elite]].cpu()
        mean = 0.3 * mean + 0.7 * elite.mean(0)
        std = (0.3 * std + 0.7 * elite.std(0)).clamp_min(0.02 * (hi - lo))
        best = order[0]
        m_best = used & (cid == best)
        row = {
            "gen": g, "time_s": round(time.time() - t0, 1),
            "score_mean_candidate": round(float(scores[0]), 4),
            "score_best": round(float(scores[best]), 4),
            "score_pop_mean": round(float(scores.mean()), 4),
            "all_envs": aggregate(ep, used),
            "best_candidate_envs": aggregate(ep, m_best),
            "mean": {n: round(float(v), 4) for n, v in zip(names, mean)},
            "std": {n: round(float(v), 4) for n, v in zip(names, std)},
        }
        history.append(row)
        for i in order[: n_elite].tolist():
            archive.append((float(scores[i]), {n: float(v) for n, v in zip(names, cand[i].cpu())}))
        (out_dir / "history.json").write_text(json.dumps(history, indent=2), encoding="utf-8")
        a = row["all_envs"]
        b = row["best_candidate_envs"]
        print(f"[cem] gen {g:2d} | best {row['score_best']:.3f} mean-cand {row['score_mean_candidate']:.3f} "
              f"pop {row['score_pop_mean']:.3f} | pop succ {a['success_rate']:.3f} fall {a['fall_rate']:.3f} "
              f"| best succ {b['success_rate']:.2f} dist {b['mean_distance_m']:.2f} steps {b['mean_steps']:.1f} "
              f"| {row['time_s']} s", flush=True)

    # final re-evaluation of the best archived candidates + the final mean on more envs
    archive.sort(key=lambda x: x[0], reverse=True)
    finals = [{n: float(v) for n, v in zip(names, mean)}] + [a[1] for a in archive[: args_cli.final_top - 1]]
    cand = torch.tensor([[f[n] for n in names] for f in finals], device=dev)
    scores, ep, cid, used = evaluate(env, ctrl, names, cand, args_cli.final_envs_per, args_cli.episode_s)
    final = []
    for i, f in enumerate(finals):
        final.append({"params": f, "score": round(float(scores[i]), 4), **aggregate(ep, used & (cid == i))})
    final.sort(key=lambda r: r["score"], reverse=True)
    (out_dir / "final.json").write_text(json.dumps(final, indent=2), encoding="utf-8")
    (out_dir / "best_params.json").write_text(json.dumps(final[0]["params"], indent=2), encoding="utf-8")
    for r in final:
        print(f"[cem-final] score {r['score']:.3f} succ {r['success_rate']:.3f} fall {r['fall_rate']:.3f} "
              f"dist {r['mean_distance_m']:.2f} steps {r['mean_steps']:.1f} lat {r['mean_max_abs_lateral_m']:.3f}")
    print(f"[cem] output: {out_dir}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
