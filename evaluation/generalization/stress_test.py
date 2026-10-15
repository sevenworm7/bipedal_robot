r"""Stress tests of a trained policy (no training): one factor at a time, ``simulation/env/scenarios.py`` ``STRESS_V1`` / ``STRESS_V2``.

v2 (2026-10-09) repeats the slope / step conditions of v1 with the fall height measured from the ground under the feet
(v1 measured it from the spawn height: walking ~10 cm downhill counted as a fall). Select with ``--suite``.

Stages (each condition is a separate Isaac Lab process; finished conditions are skipped, so batches can be resumed):

* ``stats``  : 100 episodes x 20 s per condition (``evaluate.py``; push tests run only the push test)
* ``videos`` : one 10 s clip of the medium-speed robot per condition (4 robots recorded, quality 5) ->
               ``videos/medium/<id>.mp4``
* ``report`` : README.md with tables, failure-boundary plots and video links (sim-to-real success criterion)

Output: ``outputs/robot_v<N>/evaluation/generalization/stress_v<K>_<date>/{README.md, plots/, videos/medium/, runs/<id>/, video_runs/<id>/}``.
The policies come from ``control``; this step never trains.
This script itself does not start Isaac Sim (run it with ``tools\isaacpy.cmd``; it calls ``tools\isaacpy.cmd`` per condition).

Usage:
    tools\isaacpy.cmd evaluation\generalization\stress_test.py --out outputs\robot_v1\evaluation\generalization\stress_v1_20261009 --stage stats --only 01_ 02_
    tools\isaacpy.cmd evaluation\generalization\stress_test.py --out outputs\robot_v1\evaluation\generalization\stress_v1_20261009 --stage report
    tools\isaacpy.cmd evaluation\generalization\stress_test.py --suite v2 --out outputs\robot_v1\evaluation\generalization\stress_v2_20261009 --stage stats
"""

import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "control")]  # core / simulation, sim2real (scoring)

from simulation.env.scenarios import STRESS_V1, STRESS_V1_META, STRESS_V2, STRESS_V2_META  # noqa: E402

SUITES = {"v1": (STRESS_V1, STRESS_V1_META), "v2": (STRESS_V2, STRESS_V2_META)}
SUITE = "v1"  # set from --suite in main()

DEFAULT_AGENT = "student:outputs/robot_v1/control/sim2real/student_v5_20261007_003835/student.pt"
"""Final deployable model of control/sim2real (student v5)."""
ISAACPY = str(ROOT / "tools" / "isaacpy.cmd")


def run(cmd: list[str], log: Path) -> int:
    with open(log, "w", encoding="utf-8", errors="ignore") as f:
        return subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=ROOT).returncode


def stage_stats(out: Path, ids: list[str], agent: str):
    for i in ids:
        d = out / "runs" / i
        if (d / "stats.json").exists():
            continue
        if d.exists():
            shutil.rmtree(d)  # unfinished attempt of this runner
        ov = SUITES[SUITE][0][i]
        cmd = [ISAACPY, "control/sim2real/evaluate.py", "--scenario", f"stress_{SUITE}/{i}", "--agent", agent,
               "--out_dir", str(d)]
        cmd += ["--push_only", "--push_speed", str(ov["_push_speed"])] if "_push_speed" in ov else ["--no_push"]
        t0 = time.time()
        rc = run(cmd, out / "logs" / f"stats_{i}.log")
        print(f"[stress] stats {i}: rc {rc} ({time.time() - t0:.0f} s)", flush=True)


def stage_videos(out: Path, ids: list[str], agent: str):
    dst = out / "videos" / "medium"
    dst.mkdir(parents=True, exist_ok=True)
    for i in ids:
        if (dst / f"{i}.mp4").exists():
            continue
        d = out / "video_runs" / i
        if d.exists():
            shutil.rmtree(d)
        ov = SUITES[SUITE][0][i]
        cmd = [ISAACPY, "control/sim2real/record_videos.py", "--scenario", f"stress_{SUITE}/{i}", "--agent", agent,
               "--tag", i, "--num_envs", "4", "--episode_s", "10", "--quality", "5", "--keep_all", "--out_dir", str(d),
               "--enable_cameras"]
        if "_video_cmd" in ov:
            v = ov["_video_cmd"]
            cmd += ["--speeds", f"{v},{v},{v}"]
        if "_video_freq" in ov:
            cmd += ["--freq", str(ov["_video_freq"])]
        if "_push_speed" in ov:
            cmd += ["--push_speed", str(ov["_push_speed"])]
        t0 = time.time()
        rc = run(cmd, out / "logs" / f"video_{i}.log")
        med = sorted((d / "videos").glob("*_medium_eval_*.mp4")) if (d / "videos").exists() else []
        if med:
            shutil.copy2(med[0], dst / f"{i}.mp4")
            for f in (d / "videos").glob("*.mp4"):  # only the medium clip is kept (user decision 2026-10-09)
                f.unlink()
        print(f"[stress] video {i}: rc {rc}, medium {'ok' if med else 'MISSING'} ({time.time() - t0:.0f} s)", flush=True)


def stage_report(out: Path, ids: list[str]):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import torch

    from sim2real.scoring import summarize

    rows = {}
    for i in ids:
        d = out / "runs" / i
        if not (d / "stats.json").exists():
            continue
        st = json.loads((d / "stats.json").read_text(encoding="utf-8"))
        ep = torch.load(d / "episodes.pt")
        crit = summarize(ep, st["env"]["episode_s"])
        n = st["nominal"]
        q = st.get("percentiles_p10_p50_p90", {})
        lo, hi = 0.05, 0.15
        bins = {}
        for name, a, b in (("slow", lo, lo + (hi - lo) / 3), ("medium", lo + (hi - lo) / 3, lo + 2 * (hi - lo) / 3),
                           ("fast", lo + 2 * (hi - lo) / 3, hi + 1e-6)):
            m = (ep["cmd_vx"] >= a) & (ep["cmd_vx"] < b)
            if int(m.sum()):
                sub = {k: v[m] for k, v in ep.items() if hasattr(v, "shape") and v.shape[:1] == m.shape}
                bins[name] = summarize(sub, st["env"]["episode_s"])["success"]
        rows[i] = {**SUITES[SUITE][1][i], "success": crit["success"], "fall": crit["fall_rate"],
                   "speed_p50": (q.get("speed_ratio") or [None, None])[1],
                   "heading_p50": (q.get("max_abs_heading_err_deg") or [None, None])[1],
                   "lateral_p50": (q.get("max_abs_lateral_m") or [None, None])[1],
                   "dist": n.get("mean_distance_m"), "bins": bins,
                   "video": (out / "videos" / "medium" / f"{i}.mp4").exists()}
    (out / "results.json").write_text(json.dumps(rows, indent=2, ensure_ascii=False), encoding="utf-8")

    plots = out / "plots"
    plots.mkdir(exist_ok=True)
    tests = {}
    for i, r in rows.items():
        tests.setdefault(r["test"], []).append((i, r))
    lines = [f"# Stress tests {SUITE}：student v5 在訓練範圍內外的表現", "",
             f"- 模型：`{DEFAULT_AGENT.split(':', 1)[1]}`（不重新訓練）",
             "- 方法：每次只改一個條件，其餘為理想平地（`flat_v1`）；每個條件 100 episode × 20 s、seed 42、指令 0.05–0.15 m/s、1.2–2.0 Hz。",
             "- 成功＝未跌倒、走完、平均速度在指令 ±25% 內、最大方向誤差 < 10°（`sim2real/scoring.py`）。推擠測試：第 5、10、15 秒各推一次。",
             "- 部署門檻：平地與中等隨機化成功率 ≥ 95%、不規則地形 ≥ 85%、跌倒 ≤ 5%。",
             "- 影片：每個條件一段中速（0.10 m/s）10 秒片段，`videos/medium/<條件>.mp4`；成功率與跌倒率以 20 秒統計為準。"]
    if SUITE == "v1":
        lines += ["- **注意（2026-10-09）**：v1 的跌倒高度以出生點高度為基準，斜坡（06）與台階（07）往下走約 10 cm 就被誤判為跌倒、"
                  "往上走則可能漏判。06、07 以 [`stress_v2_20261009`](../stress_v2_20261009/README.md) 的結果為準（跌倒高度改以腳下地面為基準）。"]
    else:
        lines += ["- v2：重做 v1 的斜坡（06）與台階（07），跌倒高度改以腳下地面為基準（`fall_height_ref=\"feet\"`）；"
                  "`02_bumps_00mm` 為平地對照。其餘條件見 [`stress_v1_20261009`](../stress_v1_20261009/README.md)。"]
    lines.append("")
    for t, items in sorted(tests.items()):
        title, unit = items[0][1]["title"], items[0][1]["unit"]
        lines += [f"## {t}：{title}", "",
                  f"| 數值 ({unit}) | 訓練範圍內 | 成功率 | 跌倒率 | 慢／中／快 成功率 | 速度比 p50 | 方向 p50 | 側向 p50 | 影片 |",
                  "|---|---|---|---|---|---|---|---|---|"]
        for i, r in items:
            b = r["bins"]
            bin_s = "／".join(f"{b[k]:.0%}" if k in b else "—" for k in ("slow", "medium", "fast"))
            fmt = lambda v, f: (f.format(v) if v is not None else "—")  # noqa: E731
            vid = f"[{i}](videos/medium/{i}.mp4)" if r["video"] else "—"
            lines.append(f"| {r['value']} | {'是' if r['in_training'] else '**否**'} | {r['success']:.0%} | {r['fall']:.0%} | {bin_s} | "
                         f"{fmt(r['speed_p50'], '{:.2f}')} | {fmt(r['heading_p50'], '{:.1f}°')} | {fmt(r['lateral_p50'], '{:.2f} m')} | {vid} |")
        xs = [r["value"] for _, r in items]
        fig, ax = plt.subplots(figsize=(6, 3.4))
        ax.plot(xs, [100 * r["success"] for _, r in items], "o-", color="#2a7ab9", label="success %")
        ax.plot(xs, [100 * r["fall"] for _, r in items], "s--", color="#c0392b", label="falls %")
        for x, (_, r) in zip(xs, items):
            if not r["in_training"]:
                ax.axvspan(x - 1e-9, x + 1e-9, color="0.8")
        tr = [x for x, (_, r) in zip(xs, items) if r["in_training"]]
        if tr:
            ax.axvspan(min(tr), max(tr), color="#2ecc71", alpha=0.12, label="training range")
        ax.axhline(85, color="0.5", lw=0.8, ls=":")
        ax.axhline(5, color="0.5", lw=0.8, ls=":")
        ax.set(ylim=(-3, 103), xlabel=f"{t} [{unit}]", ylabel="%", title=t)
        ax.legend(fontsize=7, loc="center left")
        fig.tight_layout()
        fig.savefig(plots / f"{t}.png", dpi=110)
        plt.close(fig)
        lines += ["", f"![{t}](plots/{t}.png)", ""]
    (out / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[stress] report: {out / 'README.md'} ({len(rows)} conditions)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stage", required=True, choices=("stats", "videos", "report"))
    ap.add_argument("--only", nargs="*", default=[], help="Condition id prefixes (default: all).")
    ap.add_argument("--agent", default=DEFAULT_AGENT)
    ap.add_argument("--suite", default="v1", choices=tuple(SUITES))
    a = ap.parse_args()
    global SUITE
    SUITE = a.suite
    out = ROOT / a.out
    (out / "logs").mkdir(parents=True, exist_ok=True)
    ids = [i for i in SUITES[SUITE][0] if not a.only or any(i.startswith(p) for p in a.only)]
    if a.stage == "stats":
        stage_stats(out, ids, a.agent)
    elif a.stage == "videos":
        stage_videos(out, ids, a.agent)
    else:
        stage_report(out, ids)


if __name__ == "__main__":
    main()
