"""Stage 4: select high-quality episodes from policy rollouts for self-distillation.

Hard thresholds (every one must pass; values in ``FilterCfg``, decided with the user):
    not fallen and the full episode was run; steps >= min_steps and distance >= coef * steps * step_length;
    max |lateral| < max_lateral_m[iter - 1]; max |heading error| < max_heading_deg;
    |mean_vx / cmd_vx - 1| < max_speed_err_frac; torque saturation fraction < max_torque_sat_frac.
Ranking: inside each command bin, keep the best ``top_frac_per_bin`` of the passing episodes by return.

``--stats_only`` (required before the first real filtering, user rule): distributions and percentiles
of every metric plus the rejection rate of each threshold, without writing a dataset.

Stop rule (user): if fewer than ``stop_if_kept_below`` of the episodes pass the hard thresholds, or a
command bin ends up empty, the report is written with ``"stop": true`` and the script exits with code 2
without producing a dataset -> discuss the thresholds with the user.

Output: ``iter<N>_filter_<time>/{filter_report.json | stats_report.json, dataset.h5, config.yaml}``.

Usage:
    tools\\isaacpy.cmd 03_training_pipeline\\training_pipeline\\filter.py --iter 1 --data <iter1_collect_ppo>/dataset.h5 --stats_only
    tools\\isaacpy.cmd 03_training_pipeline\\training_pipeline\\filter.py --iter 1 --data <iter1_collect_ppo>/dataset.h5
"""

import argparse
import sys
from pathlib import Path

# project root (shared), 02_rule_based (rule_based teacher), 03_training_pipeline (training_pipeline)
_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_ROOT), str(_ROOT / "02_rule_based"), str(_ROOT / "03_training_pipeline")]

import numpy as np  # noqa: E402
import yaml  # noqa: E402

from training_pipeline.config import PipelineCfg  # noqa: E402
from training_pipeline.dataset import load_h5, save_h5, select_episodes  # noqa: E402


def metrics(e: dict, episode_s: float) -> dict[str, np.ndarray]:
    cmd = np.maximum(e["cmd_vx"], 1e-3)
    return {
        "fell": e["fell"],
        "time": e["time"],
        "steps": e["steps"],
        "distance_ratio": e["distance"] / np.maximum(e["steps"] * e["step_length_cmd"], 1e-6),
        "max_abs_lateral_m": e["max_abs_lateral"],
        "max_abs_heading_deg": np.degrees(e["max_abs_heading_error"]),
        "speed_err_frac": np.abs(e["mean_vx"] / cmd - 1.0),
        "torque_sat_frac": e["torque_sat_frac"],
        "ret": e["ret"],
    }


def hard_checks(m: dict, f, it: int, episode_s: float, dt: float) -> dict[str, np.ndarray]:
    """Boolean pass mask of each hard threshold."""
    lat = f.max_lateral_m[min(it, len(f.max_lateral_m)) - 1]
    return {
        "not_fallen_full_episode": (m["fell"] < 0.5) & (m["time"] >= episode_s - 2 * dt),
        f"steps>={f.min_steps}_and_distance>={f.distance_coef}*steps*step_length":
            (m["steps"] >= f.min_steps) & (m["distance_ratio"] >= f.distance_coef),
        f"max_lateral<{lat}m": m["max_abs_lateral_m"] < lat,
        f"max_heading<{f.max_heading_deg}deg": m["max_abs_heading_deg"] < f.max_heading_deg,
        f"speed_err<{f.max_speed_err_frac}": m["speed_err_frac"] < f.max_speed_err_frac,
        f"torque_sat<{f.max_torque_sat_frac}": m["torque_sat_frac"] < f.max_torque_sat_frac,
    }


def pct(x: np.ndarray) -> dict:
    qs = (0, 10, 25, 50, 75, 90, 100)
    return {f"p{q}": round(float(np.percentile(x, q)), 4) for q in qs} if len(x) else {}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iter", type=int, required=True, help="Distillation iteration (1..3).")
    ap.add_argument("--data", type=str, required=True)
    ap.add_argument("--stats_only", action="store_true")
    ap.add_argument("--tag", type=str, default="")
    args = ap.parse_args()

    cfg = PipelineCfg()
    f = cfg.filter
    from shared import interface as itf
    from training_pipeline.common import run_dir, write_json

    data = load_h5(Path(args.data))
    data.pop("attrs", None)
    e = data["episodes"]
    n = len(e["length"])
    m = metrics(e, cfg.collect.episode_s)
    checks = hard_checks(m, f, args.iter, cfg.collect.episode_s, itf.CONTROL_DT)
    passed = np.all(np.stack(list(checks.values())), axis=0)
    lo, hi = float(e["cmd_vx"].min()), float(e["cmd_vx"].max())
    edges = np.linspace(lo, hi, f.cmd_bins + 1)
    bins = np.clip(np.digitize(e["cmd_vx"], edges[1:-1]), 0, f.cmd_bins - 1)

    report = {
        "data": args.data, "iteration": args.iter, "episodes": n, "steps": int(e["length"].sum()),
        "thresholds": {k: None for k in checks},
        "rejected_frac_per_threshold": {k: round(float(1 - v.mean()), 4) for k, v in checks.items()},
        "only_failed_this_threshold_frac": {
            k: round(float((~v & np.all(np.stack([w for kk, w in checks.items() if kk != k]), axis=0)).mean()), 4)
            for k, v in checks.items()},
        "passed_all_frac": round(float(passed.mean()), 4),
        "cmd_bins": [f"{edges[b]:.3f}-{edges[b + 1]:.3f}" for b in range(f.cmd_bins)],
        "passed_per_bin": [int((passed & (bins == b)).sum()) for b in range(f.cmd_bins)],
        "episodes_per_bin": [int((bins == b).sum()) for b in range(f.cmd_bins)],
    }
    report.pop("thresholds")
    if args.stats_only:
        report["distributions"] = {k: pct(v) for k, v in m.items()}
        report["distributions_non_fallen"] = {k: pct(v[m["fell"] < 0.5]) for k, v in m.items() if k != "fell"}
        lat = m["max_abs_lateral_m"]
        report["lateral_threshold_scan"] = {f"<{t:.2f}m": round(float((lat < t).mean()), 4)
                                            for t in (0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10, 0.15)}
        head = m["max_abs_heading_deg"]
        report["heading_threshold_scan"] = {f"<{t}deg": round(float((head < t).mean()), 4) for t in (5, 8, 10, 15, 20)}
        spd = m["speed_err_frac"]
        report["speed_threshold_scan"] = {f"<{t}": round(float((spd < t).mean()), 4) for t in (0.1, 0.15, 0.25, 0.35, 0.5)}
        out = run_dir(cfg, args.iter, "filterstats", args.tag)
        write_json(out / "stats_report.json", report)
        print(yaml.safe_dump({k: report[k] for k in ("episodes", "passed_all_frac", "rejected_frac_per_threshold",
                                                     "passed_per_bin", "lateral_threshold_scan")}, sort_keys=False))
        print(f"[filter] stats-only report: {out / 'stats_report.json'}")
        return

    keep = []
    for b in range(f.cmd_bins):
        idx = np.nonzero(passed & (bins == b))[0]
        if len(idx) == 0:
            continue
        order = idx[np.argsort(-e["ret"][idx])]
        keep.extend(order[: max(1, int(np.ceil(f.top_frac_per_bin * len(order))))].tolist())
    keep = np.sort(np.array(keep, dtype=int))
    report["kept_per_bin"] = [int(np.isin(np.nonzero(bins == b)[0], keep).sum()) for b in range(f.cmd_bins)]
    report["kept_episodes"] = int(len(keep))
    report["kept_steps"] = int(e["length"][keep].sum()) if len(keep) else 0
    report["kept_frac"] = round(len(keep) / max(n, 1), 4)
    stop = report["passed_all_frac"] < f.stop_if_kept_below or min(report["passed_per_bin"]) == 0
    report["stop"] = bool(stop)
    report["stop_rule"] = (f"stop if fewer than {f.stop_if_kept_below:.0%} of the episodes pass the hard thresholds "
                           f"or a command bin is empty")
    out = run_dir(cfg, args.iter, "filter", args.tag)
    (out / "config.yaml").write_text(yaml.safe_dump({"filter": cfg.filter.__dict__, "args": vars(args)}, sort_keys=False),
                                     encoding="utf-8")
    write_json(out / "filter_report.json", report)
    print(yaml.safe_dump({k: report[k] for k in ("episodes", "passed_all_frac", "rejected_frac_per_threshold", "passed_per_bin",
                                                 "kept_per_bin", "kept_episodes", "kept_steps", "stop")}, sort_keys=False))
    if stop:
        print("[filter] STOP: retention below the agreed limit or an empty command bin -> discuss thresholds with the user")
        sys.exit(2)
    save_h5(out / "dataset.h5", select_episodes(data, keep), {"robot": cfg.robot, "source": args.data,
                                                              "iteration": args.iter, "filter": cfg.filter.__dict__})
    print(f"[filter] output: {out}")


if __name__ == "__main__":
    main()
