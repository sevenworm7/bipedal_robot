r"""Servo load analysis v1 (no training): runs every case of ``cases.py`` (``record_load.py``, one Isaac Lab process per
case and stage) and writes the report. Finished cases are skipped, so batches can be resumed.

Stages:
* ``stats``  : 128 robots x 20 s per case -> ``cases/<id>/load.npz`` (applied torque, joint speed, contact)
* ``videos`` : one 10 s clip per case with a live per-joint torque panel -> ``videos/<id>.mp4``
* ``report`` : README.md, results.json, plots/ (per-joint RMS / p99 heat maps, time series, current histograms)

Metrics (first episode of every robot, after 0.5 s of settling): per joint mean |tau|, RMS tau (heating; compared with
the rated 0.294 N m), p95 / p99 / max |tau|, time above rated and at the simulated limit 0.8 N m, mean |tau omega|;
whole robot: estimated servo current (6 V, linear in torque) mean / p99 / max.

Output: ``outputs/robot_v<N>/evaluation/load_analysis/load_v1_<date>/``.

Usage:
    tools\isaacpy.cmd evaluation\load_analysis\run_load.py --out outputs\robot_v1\evaluation\load_analysis\load_v1_20261015 --stage stats
"""

import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

from cases import CASES_V1, SERVO_6V  # noqa: E402

ISAACPY = str(ROOT / "tools" / "isaacpy.cmd")
SETTLE_S = 0.5


def run(cmd, log):
    with open(log, "w", encoding="utf-8", errors="ignore") as f:
        return subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=ROOT).returncode


def stage_record(out: Path, ids, mode: str):
    for i in ids:
        if mode == "stats":
            d, done = out / "cases" / i, (out / "cases" / i / "load.npz").exists()
        else:
            d, done = out / "video_runs" / i, (out / "videos" / f"{i}.mp4").exists()
        if done:
            continue
        if d.exists():
            shutil.rmtree(d)
        cmd = [ISAACPY, "evaluation/load_analysis/record_load.py", "--case", i, "--mode", mode, "--out_dir", str(d)]
        if mode == "video":
            cmd.append("--enable_cameras")
        t0 = time.time()
        rc = run(cmd, out / "logs" / f"{mode}_{i}.log")
        if mode == "video":
            clips = sorted(d.glob("*.mp4"))
            if clips:
                (out / "videos").mkdir(exist_ok=True)
                shutil.copy2(clips[0], out / "videos" / f"{i}.mp4")
            ok = bool(clips)
        else:
            ok = (d / "load.npz").exists()
        print(f"[load] {mode} {i}: rc {rc}, {'ok' if ok else 'MISSING'} ({time.time() - t0:.0f} s)", flush=True)


def current(tau_abs):
    import numpy as np
    s = SERVO_6V
    return s["noload_current"] + np.minimum(tau_abs, s["stall_torque"]) * (s["stall_current"] - s["noload_current"]) / s["stall_torque"]


def stage_report(out: Path, ids):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    plots = out / "plots"
    plots.mkdir(exist_ok=True)
    rated, limit = SERVO_6V["rated_torque"], SERVO_6V["sim_limit"]
    res, names, series = {}, None, {}
    for i in ids:
        f = out / "cases" / i / "load.npz"
        if not f.exists():
            continue
        z = np.load(f)
        tau, vel, alive, con = np.abs(z["torque"]), z["joint_vel"], z["alive"], z["contact"]  # [T, N, J], [T, N]
        names = [str(x) for x in z["joint_names"]]
        dt = float(z["step_dt"])
        t0 = int(round(SETTLE_S / dt))
        m = alive.copy()
        m[:t0] = False
        sel = tau[m]  # [samples, J]
        rms_env = np.sqrt((tau ** 2 * m[..., None]).sum(0) / np.maximum(m.sum(0)[:, None], 1))  # [N, J]
        cur = current(tau).sum(-1)  # [T, N]
        cur_sel = cur[m]
        cur_env = (cur * m).sum(0) / np.maximum(m.sum(0), 1)
        summ = json.loads((out / "cases" / i / "summary.json").read_text(encoding="utf-8"))
        res[i] = {
            "title": CASES_V1[i]["title"], "fall_rate": summ["fall_rate"], "samples": int(m.sum()),
            "joints": {nm: {"mean": float(sel[:, j].mean()), "rms": float(np.sqrt((sel[:, j] ** 2).mean())),
                            "rms_p90_robot": float(np.percentile(rms_env[:, j], 90)),
                            "p95": float(np.percentile(sel[:, j], 95)), "p99": float(np.percentile(sel[:, j], 99)),
                            "max": float(sel[:, j].max()), "frac_above_rated": float((sel[:, j] > rated).mean()),
                            "frac_at_limit": float((sel[:, j] >= limit * 0.99).mean()),
                            "mean_power_w": float(np.abs(tau[..., j] * vel[..., j])[m].mean())}
                       for j, nm in enumerate(names)},
            "current_a": {"mean": float(cur_sel.mean()), "p99": float(np.percentile(cur_sel, 99)),
                          "max": float(cur_sel.max()), "robot_mean_p90": float(np.percentile(cur_env, 90))},
        }
        # one representative robot (first that lived the whole episode) for the time-series plot
        full = np.where(alive.all(0))[0]
        k = int(full[0]) if len(full) else 0
        series[i] = (tau[:, k], con[:, k], cur[:, k], dt)
    (out / "results.json").write_text(json.dumps(res, indent=2, ensure_ascii=False), encoding="utf-8")
    cases = list(res)
    short = [n.replace("left_", "L_").replace("right_", "R_").replace("_pitch", "_p").replace("_roll", "_r") for n in names]

    def heat(key, title, fname, vmax):
        a = np.array([[res[c]["joints"][nm][key] for nm in names] for c in cases])
        fig, ax = plt.subplots(figsize=(11, 0.45 * len(cases) + 1.6))
        im = ax.imshow(a, cmap="RdYlGn_r", vmin=0, vmax=vmax, aspect="auto")
        ax.set_xticks(range(len(names)), short, rotation=35, ha="right", fontsize=8)
        ax.set_yticks(range(len(cases)), cases, fontsize=8)
        for r in range(a.shape[0]):
            for c in range(a.shape[1]):
                ax.text(c, r, f"{a[r, c]:.2f}", ha="center", va="center", fontsize=7)
        fig.colorbar(im, ax=ax, label="N m")
        ax.set_title(title, fontsize=10)
        fig.tight_layout()
        fig.savefig(plots / fname, dpi=110)
        plt.close(fig)

    heat("rms", f"RMS torque per joint [N m] (heating; rated {rated} N m)", "rms_heatmap.png", 2 * rated)
    heat("p99", f"p99 |torque| per joint [N m] (sim limit {limit}, stall {SERVO_6V['stall_torque']})", "p99_heatmap.png", limit)

    for i in [c for c in ("04_flat_medium", "01_stand", "07_bumps_8mm", "10_steps_up_10mm") if c in series]:
        tau, con, cur, dt = series[i]
        t = np.arange(len(tau)) * dt
        w = (t >= 4.0) & (t < 8.0)
        fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
        for ax, idx, side in ((axes[0], range(0, 4), "left leg"), (axes[1], range(4, 8), "right leg")):
            for j in idx:
                ax.plot(t[w], tau[w, j], label=short[j], lw=1.2)
            c = con[w, 0 if side == "left leg" else 1]
            ax.fill_between(t[w], 0, limit, where=c > 0.5, color="0.85", step="mid", label="foot contact")
            ax.axhline(rated, color="#d4a017", ls="--", lw=0.8)
            ax.axhline(limit, color="#c0392b", ls=":", lw=0.8)
            ax.set(ylabel=f"|tau| {side} [N m]", ylim=(0, limit * 1.05))
            ax.legend(fontsize=7, ncol=5, loc="upper right")
        axes[2].plot(t[w], cur[w], color="#2a7ab9")
        axes[2].set(ylabel="total current [A]", xlabel="time [s]")
        fig.suptitle(f"{i}: one robot, 4 s (yellow dashed = rated, red dotted = sim limit)", fontsize=10)
        fig.tight_layout()
        fig.savefig(plots / f"timeseries_{i}.png", dpi=110)
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 3.6))
    ax.bar(range(len(cases)), [res[c]["current_a"]["mean"] for c in cases], color="#2a7ab9", label="mean")
    ax.scatter(range(len(cases)), [res[c]["current_a"]["p99"] for c in cases], color="#d4a017", zorder=3, label="p99")
    ax.scatter(range(len(cases)), [res[c]["current_a"]["max"] for c in cases], color="#c0392b", marker="x", zorder=3, label="max")
    ax.set_xticks(range(len(cases)), cases, rotation=35, ha="right", fontsize=8)
    ax.set(ylabel="total servo current [A] (6 V est.)")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(plots / "current.png", dpi=110)
    plt.close(fig)

    L = ["# 馬達負載分析 v1：student v5 在各情境下的伺服出力", "",
         "- 模型：`outputs/robot_v1/control/sim2real/student_v5_20261007_003835/student.pt`（不重新訓練）。",
         "- 每個情境 128 台 × 20 s、seed 42，所有機器人同一個指令；統計每台的第一個 episode，略過前 0.5 s。",
         f"- 伺服 HD-1910（6 V 保守值）：額定 {rated} N·m／{SERVO_6V['rated_current']} A、模擬上限 {limit} N·m、"
         f"堵轉 {SERVO_6V['stall_torque']} N·m／{SERVO_6V['stall_current']} A、空載 {SERVO_6V['noload_current']} A。",
         "- 電流是估計值：每顆 = 空載電流 ＋ |扭矩| ×（堵轉電流 − 空載電流）／堵轉扭矩，10 顆相加；模擬的扭矩來自 PD 模型，實機需以台架量測校正。",
         "- **RMS 扭矩**代表發熱：長時間 RMS 超過額定值，實機馬達會過熱、降扭或觸發保護。", "",
         "## 總覽", "",
         "| 情境 | 跌倒 | 最大 RMS 關節（RMS／額定） | 最大 p99 關節 | 超過額定的時間（最高關節） | 整機電流 平均／p99／最大 | 影片 |",
         "|---|---|---|---|---|---|---|"]
    for c in cases:
        r = res[c]
        jr = max(r["joints"], key=lambda nm: r["joints"][nm]["rms"])
        jp = max(r["joints"], key=lambda nm: r["joints"][nm]["p99"])
        ja = max(r["joints"], key=lambda nm: r["joints"][nm]["frac_above_rated"])
        vid = f"[影片](videos/{c}.mp4)" if (out / "videos" / f"{c}.mp4").exists() else "—"
        L.append(f"| {c}：{r['title']} | {r['fall_rate']:.0%} | {jr} {r['joints'][jr]['rms']:.3f}（{r['joints'][jr]['rms'] / rated:.0%}） | "
                 f"{jp} {r['joints'][jp]['p99']:.2f} | {ja} {r['joints'][ja]['frac_above_rated']:.0%} | "
                 f"{r['current_a']['mean']:.2f}／{r['current_a']['p99']:.2f}／{r['current_a']['max']:.2f} A | {vid} |")
    L += ["", "![RMS](plots/rms_heatmap.png)", "", "![p99](plots/p99_heatmap.png)", "", "![current](plots/current.png)", ""]
    for i in series:
        if (plots / f"timeseries_{i}.png").exists():
            L += [f"![{i}](plots/timeseries_{i}.png)", ""]
    L += ["## 各情境的關節明細（RMS／p99／最大，N·m；超過額定的時間比例）", ""]
    for c in cases:
        r = res[c]
        L += [f"### {c}：{r['title']}", "", "| 關節 | 平均 | RMS | 各機器人 RMS p90 | p95 | p99 | 最大 | > 額定 | 達上限 | 平均機械功率 |",
              "|---|---|---|---|---|---|---|---|---|---|"]
        for nm, v in r["joints"].items():
            L.append(f"| {nm} | {v['mean']:.3f} | {v['rms']:.3f} | {v['rms_p90_robot']:.3f} | {v['p95']:.2f} | {v['p99']:.2f} | "
                     f"{v['max']:.2f} | {v['frac_above_rated']:.1%} | {v['frac_at_limit']:.2%} | {v['mean_power_w']:.3f} W |")
        L.append("")
    (out / "README.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print(f"[load] report: {out / 'README.md'} ({len(cases)} cases)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stage", required=True, choices=("stats", "videos", "report"))
    ap.add_argument("--only", nargs="*", default=[])
    a = ap.parse_args()
    out = ROOT / a.out
    (out / "logs").mkdir(parents=True, exist_ok=True)
    ids = [i for i in CASES_V1 if not a.only or any(i.startswith(p) for p in a.only)]
    if a.stage == "report":
        stage_report(out, ids)
    else:
        stage_record(out, ids, "stats" if a.stage == "stats" else "video")


if __name__ == "__main__":
    main()
