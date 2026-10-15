"""Run the rule-based controller in many parallel envs, optionally sweeping gait parameters.

Development steps 2-4 of ``rule_based/CLAUDE.md`` (marching in place, fixed-parameter walking,
feedback + parameter sweep). Every parameter combination gets ``num_envs / combos`` environments;
each environment runs one episode. Results (per combination, sorted by success rate) are written to
``archive/exp1_reference/outputs/rule_based/<tag>_<timestamp>/results.json``.

Examples:
    # step 2: march in place (cmd 0), 1 Hz, 10 s
    tools\\isaacpy.cmd archive\exp1_reference\\rule_based\\sweep.py --tag march --cmd 0 --freq 1.0 --episode_s 10
    # step 4: sweep two parameters over the full command range
    tools\\isaacpy.cmd archive\exp1_reference\\rule_based\\sweep.py --tag sw1 --num_envs 4096 --sweep sway_amp=0,0.05,0.1 --sweep swing_frac=0.3,0.4,0.5
"""

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--tag", default="sweep")
parser.add_argument("--num_envs", type=int, default=512)
parser.add_argument("--episode_s", type=float, default=20.0)
parser.add_argument("--cmd", type=str, default=None, help="Fixed cmd_vx or 'lo,hi' range (default: env range).")
parser.add_argument("--freq", type=str, default=None, help="Fixed gait frequency or 'lo,hi' range (default: env range).")
parser.add_argument("--set", action="append", default=[], help="Gait parameter override name=value (repeatable).")
parser.add_argument("--sweep", action="append", default=[], help="name=v1,v2,... (repeatable, Cartesian product).")
parser.add_argument("--by_cmd", type=int, default=0, help="Also report success per command bin (number of bins).")
parser.add_argument("--top", type=int, default=15, help="Number of combinations printed.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import torch  # noqa: E402

from rule_based import RuleBasedBipedController, RuleBasedCfg  # noqa: E402
from shared.rollout import aggregate, parse_sweep, run_first_episodes, torque_stats  # noqa: E402
from shared.env import BipedWalkEnv, BipedWalkEnvCfg  # noqa: E402


def as_range(text: str | None, default):
    if text is None:
        return tuple(default)
    vals = [float(v) for v in text.split(",")]
    return (vals[0], vals[-1])


def main():
    env_cfg = BipedWalkEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.episode_length_s = args_cli.episode_s
    env_cfg.record_episodes = True
    env_cfg.cmd_vx_range = as_range(args_cli.cmd, env_cfg.cmd_vx_range)
    env_cfg.gait_freq_range = as_range(args_cli.freq, env_cfg.gait_freq_range)
    env = BipedWalkEnv(env_cfg)

    cfg = RuleBasedCfg()
    for s in args_cli.set:
        k, v = s.split("=", 1)
        setattr(cfg.gait, k.strip(), float(v))
    ctrl = RuleBasedBipedController(env.rb, env.num_envs, env.device, cfg)
    names, combos = parse_sweep(args_cli.sweep)
    combo_id = torch.arange(env.num_envs, device=env.device) % len(combos)
    for i, name in enumerate(names):
        vals = torch.tensor([c[i] for c in combos], device=env.device)
        ctrl.override(name, vals[combo_id])

    t0 = time.time()
    ep = run_first_episodes(env, ctrl, torque_every=10)
    elapsed = time.time() - t0

    results = []
    for ci, combo in enumerate(combos):
        row = {"params": dict(zip(names, combo))}
        row.update(aggregate(ep, combo_id == ci))
        if args_cli.by_cmd:
            lo, hi = env_cfg.cmd_vx_range
            edges = torch.linspace(lo, hi, args_cli.by_cmd + 1, device=env.device)
            row["success_by_cmd"] = {}
            for b in range(args_cli.by_cmd):
                m = (combo_id == ci) & (ep["cmd_vx"] >= edges[b]) & (ep["cmd_vx"] <= edges[b + 1])
                if bool(m.any()):
                    row["success_by_cmd"][f"{float(edges[b]):.3f}-{float(edges[b + 1]):.3f}"] = round(float(ep["success"][m].mean()), 3)
        results.append(row)
    results.sort(key=lambda r: (r.get("success_rate", 0), r.get("mean_time_s", 0), r.get("mean_distance_m", 0)), reverse=True)

    out_dir = env.rb.TEST_ROOT / "outputs" / "rule_based" / f"{args_cli.tag}_{time.strftime('%Y%m%d_%H%M%S')}"
    out_dir.mkdir(parents=True, exist_ok=True)
    march = env_cfg.cmd_vx_range == (0.0, 0.0)
    report = {
        "args": vars(args_cli),
        "env": {"num_envs": env.num_envs, "episode_s": env_cfg.episode_length_s, "cmd_vx_range": env_cfg.cmd_vx_range,
                "gait_freq_range": env_cfg.gait_freq_range},
        "controller_cfg": cfg.to_dict(),
        "wall_time_s": round(elapsed, 1),
        "overall": aggregate(ep),
        "torque_Nm": torque_stats(ep, env.rb.LEG_JOINT_NAMES),
        "combos": results,
    }
    if march:
        # step counting check: touchdowns per second vs. two steps per gait cycle
        rate = ep["steps"] / ep["time"].clamp_min(1e-3)
        expected = 2.0 * ep["gait_freq"]
        ok = ep["fell"] < 0.5
        report["step_count_check"] = {
            "mean_touchdowns_per_s": round(float(rate[ok].mean()), 3) if bool(ok.any()) else None,
            "mean_expected_per_s": round(float(expected[ok].mean()), 3) if bool(ok.any()) else None,
        }
    (out_dir / "results.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps({k: report[k] for k in ("overall", "wall_time_s")} | ({"step_count_check": report["step_count_check"]} if march else {}), indent=2))
    for r in results[: args_cli.top]:
        print(f"{r['params']} | succ {r.get('success_rate', 0):.2f} fall {r.get('fall_rate', 0):.2f} "
              f"steps {r.get('mean_steps', 0):5.1f} dist {r.get('mean_distance_m', 0):6.3f} t {r.get('mean_time_s', 0):5.2f} "
              f"lat {r.get('mean_max_abs_lateral_m', 0):.3f} dir {r.get('fall_dir')}"
              + (f" bins {r['success_by_cmd']}" if 'success_by_cmd' in r else ""))
    print(f"[sweep] results: {out_dir / 'results.json'}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
