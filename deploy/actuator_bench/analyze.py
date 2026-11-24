r"""Analyse an actuator-bench session (run_bench.py): register dump (E0), step response (E1), sine sweep (E2).

  tools\isaacpy.cmd deploy\actuator_bench\analyze.py [session_dir] [session_dir ...]

PC only (numpy + matplotlib from the Isaac Lab Python; nothing from Isaac is imported). Default: the newest session.
Writes <session>/bench_analysis/: summary.json, regdump.csv, step_metrics.csv, sweep_bode.csv, step_*.png, bode.png.

Step metrics (per edge, the goal jumps at t_edge; y = (pos - from) / (to - from)):
  dead_ms    first sample with y > max(5 %, 2 counts) after the edge (servo + bus latency until motion is visible)
  t50_ms     y crosses 0.5
  rise_ms    10 % -> 90 %
  overshoot  max(y) - 1 [%]
  settle_ms  last time |y - 1| > max(5 %, 2 counts) inside the hold
  ss_err_deg mean(pos - goal) over the last 200 ms of the hold [deg]
  vmax_dps   peak |velocity| from the position samples (5-sample central difference) [deg/s]
Sweep (log chirp, see SWEEP in main_rp2350.c): at each analysis frequency, goal and position are fitted with
sin / cos of the known chirp phase over a window of 2 periods (>= 0.3 s); H = P / G gives the gain and the phase lag;
delay_ms = phase lag / (360 f).
"""
from __future__ import annotations

import csv
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SESSIONS = ROOT / "outputs" / "robot_v2" / "deploy" / "sessions"
DEG = 360.0 / 4096.0

STEP_START = re.compile(r"STEP start joint=(\S+) id=(\d+) q0=(\S+)deg delta=(\S+)deg c0=(-?\d+) c1=(-?\d+) hold_ms=(\d+)")
SWEEP_START = re.compile(r"SWEEP start joint=(\S+) id=(\d+) q0=(\S+)deg amp=(\S+)deg f0=(\S+) f1=(\S+) dur=(\S+)s c0=(-?\d+)")
DONE = re.compile(r"(STEP|SWEEP) done (\S+) fails=(\d+) torque_off=(\S+)")
BS = re.compile(r"BS (\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (\d+) (\d+)$")

# register names (address: name, bytes), HLS / STS memory table (hardware/design_refs/hd1910/docs)
REGS = {0: ("fw_main", 1), 1: ("fw_sub", 1), 3: ("model", 2), 5: ("id", 1), 6: ("baud", 1), 8: ("resp_level", 1),
        9: ("min_angle", 2), 11: ("max_angle", 2), 13: ("max_temp_c", 1), 14: ("max_volt_0.1v", 1),
        15: ("min_volt_0.1v", 1), 16: ("max_torque_0.1pct", 2), 18: ("phase", 1), 19: ("unload_cond", 1),
        20: ("led_alarm", 1), 21: ("p_gain", 1), 22: ("d_gain", 1), 23: ("i_gain", 1), 24: ("min_start_torque", 1),
        25: ("i_limit", 1), 26: ("deadband_cw", 1), 27: ("deadband_ccw", 1), 28: ("protect_current_6.5ma", 2),
        30: ("angle_resolution", 1), 31: ("pos_offset", 2), 33: ("mode", 1), 34: ("hold_torque_pct", 1),
        35: ("protect_time_10ms", 1), 36: ("overload_torque_pct", 1), 37: ("speed_p", 1),
        38: ("overcurrent_time_10ms", 1), 39: ("speed_i", 1), 40: ("torque_enable", 1), 41: ("acc", 1),
        46: ("run_speed", 2), 48: ("torque_limit_0.1pct", 2), 50: ("sram_p", 1), 51: ("sram_d", 1), 52: ("sram_i", 1),
        55: ("lock", 1), 62: ("volt_0.1v", 1), 63: ("temp_c", 1), 65: ("status", 1)}


def parse(path: Path) -> dict:
    regs, runs, cur = {}, [], None
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw.split(" ", 1)[1] if " " in raw else raw
        if line.startswith("REGDUMP id="):
            p = line.split()
            regs[int(p[1].split("=")[1])] = [int(x) for x in p[2:]]
        elif m := STEP_START.match(line):
            cur = {"kind": "step", "joint": m[1], "id": int(m[2]), "q0": float(m[3]), "delta": float(m[4]),
                   "c0": int(m[5]), "c1": int(m[6]), "hold_ms": int(m[7]), "s": []}
        elif m := SWEEP_START.match(line):
            cur = {"kind": "sweep", "joint": m[1], "id": int(m[2]), "q0": float(m[3]), "amp": float(m[4]),
                   "f0": float(m[5]), "f1": float(m[6]), "dur": float(m[7]), "c0": int(m[8]), "s": []}
        elif cur is not None and (m := BS.match(line)):
            cur["s"].append([int(x) for x in m.groups()])
        elif cur is not None and (m := DONE.match(line)):
            cur["status"], cur["fails"], cur["torque_off"] = m[2], int(m[3]), m[4]
            cur["s"] = np.array(cur["s"], dtype=float).reshape(-1, 8)
            runs.append(cur)
            cur = None
    return {"regs": regs, "runs": runs}


def decode_regs(regs: dict) -> list[dict]:
    rows = []
    for sid, r in sorted(regs.items()):
        row = {"servo_id": sid}
        for a, (name, n) in REGS.items():
            if a + n <= len(r):
                v = r[a] | (r[a + 1] << 8) if n == 2 else r[a]
                if name == "pos_offset" and v & 0x8000:
                    v = -(v & 0x7FFF)
                row[name] = v
        rows.append(row)
    return rows


def velocity(t_s: np.ndarray, pos: np.ndarray) -> np.ndarray:
    v = np.zeros_like(pos)
    if len(pos) > 4:
        v[2:-2] = (pos[4:] - pos[:-4]) / np.maximum(t_s[4:] - t_s[:-4], 1e-6)
    return v


def step_edges(r: dict) -> list[dict]:
    s = r["s"]
    t, goal, pos = s[:, 0] * 1e-6, s[:, 1], s[:, 2]
    out = []
    change = np.nonzero(np.diff(goal) != 0)[0] + 1
    for k, i0 in enumerate(change):
        i1 = change[k + 1] if k + 1 < len(change) else len(t)
        frm, to = goal[i0 - 1], goal[i0]
        d = to - frm
        tt, pp = t[i0:i1] - t[i0], pos[i0:i1]
        y = (pp - frm) / d
        tol = max(0.05, 2.0 / abs(d))

        def first(cond):
            idx = np.nonzero(cond)[0]
            return float(tt[idx[0]] * 1e3) if len(idx) else float("nan")

        late = tt > tt[-1] - 0.2
        outside = np.nonzero(np.abs(y - 1.0) > tol)[0]
        v = velocity(tt, pp) * DEG
        out.append({
            "joint": r["joint"], "id": r["id"], "q0_deg": r["q0"], "edge": "out" if k == 0 else "back",
            "step_deg": float(d * DEG), "samples": int(i1 - i0),
            "dt_ms_median": float(np.median(np.diff(tt)) * 1e3) if i1 - i0 > 1 else float("nan"),
            "dead_ms": first(y > tol), "t50_ms": first(y > 0.5), "rise_ms": first(y > 0.9) - first(y > 0.1),
            "overshoot_pct": float((y.max() - 1.0) * 100), "settle_ms": float(tt[outside[-1]] * 1e3) if len(outside) else 0.0,
            "ss_err_deg": float(np.mean(pp[late] - to) * DEG) if late.any() else float("nan"),
            "vmax_dps": float(np.max(np.abs(v))), "current_raw_max": float(np.max(np.abs(r["s"][i0:i1, 5]))),
            "volt_min": float(np.min(r["s"][i0:i1, 6]) / 10.0),
        })
    return out


def sweep_bode(r: dict, n_freq: int = 14) -> list[dict]:
    s = r["s"]
    t = s[:, 0] * 1e-6 - 0.3  # chirp time (0.3 s rest before)
    g, p = s[:, 1] - r["c0"], s[:, 2] - r["c0"]
    f0, f1, T = r["f0"], r["f1"], r["dur"]
    k = f1 / f0
    ph = 2 * math.pi * f0 * T * (np.power(k, np.clip(t, 0, T) / T) - 1) / math.log(k)
    rows = []
    for f in np.geomspace(f0 * 1.15, f1 / 1.1, n_freq):
        tc = T * math.log(f / f0) / math.log(k)
        half = max(1.0 / f, 0.15)
        m = (t > max(tc - half, 0.5)) & (t < min(tc + half, T - 0.5))
        if m.sum() < 20:
            continue
        X = np.column_stack([np.sin(ph[m]), np.cos(ph[m]), np.ones(m.sum())])
        cg = np.linalg.lstsq(X, g[m], rcond=None)[0]
        cp = np.linalg.lstsq(X, p[m], rcond=None)[0]
        H = complex(cp[0], cp[1]) / complex(cg[0], cg[1])
        lag = -math.degrees(math.atan2(H.imag, H.real))
        rows.append({"joint": r["joint"], "id": r["id"], "amp_deg": r["amp"], "freq_hz": float(f), "gain": abs(H),
                     "gain_db": 20 * math.log10(max(abs(H), 1e-9)), "phase_lag_deg": lag,
                     "delay_ms": lag / (360.0 * f) * 1e3, "samples": int(m.sum())})
    return rows


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def plots(out: Path, runs: list[dict], bode: list[dict]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for joint in sorted({r["joint"] for r in runs if r["kind"] == "step"}):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for r in (r for r in runs if r["kind"] == "step" and r["joint"] == joint):
            s = r["s"]
            i0 = int(np.argmax(s[:, 1] != s[0, 1])) if (s[:, 1] != s[0, 1]).any() else 0
            t = (s[:, 0] - s[i0, 0]) * 1e-3
            m = (t > -20) & (t < 300)
            y = (s[m, 2] - r["c0"]) / (r["c1"] - r["c0"])
            ax.plot(t[m], y, lw=1, label=f"{r['delta']:+g} deg")
        ax.axhline(1, color="0.6", lw=0.8)
        ax.set(xlabel="time after goal jump [ms]", ylabel="normalised position", title=f"step response {joint}")
        ax.legend(fontsize=8, ncol=2)
        fig.tight_layout()
        fig.savefig(out / f"step_{joint}.png", dpi=110)
        plt.close(fig)
    if bode:
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        for key in sorted({(b["joint"], b["amp_deg"]) for b in bode}):
            bb = [b for b in bode if (b["joint"], b["amp_deg"]) == key]
            f = [b["freq_hz"] for b in bb]
            a1.semilogx(f, [b["gain_db"] for b in bb], "o-", ms=3, label=f"{key[0]} {key[1]:g} deg")
            a2.semilogx(f, [b["phase_lag_deg"] for b in bb], "o-", ms=3)
        a1.set(ylabel="gain [dB]", title="frequency response (goal -> position)")
        a2.set(xlabel="frequency [Hz]", ylabel="phase lag [deg]")
        a1.legend(fontsize=8)
        for a in (a1, a2):
            a.grid(True, which="both", alpha=0.3)
        fig.tight_layout()
        fig.savefig(out / "bode.png", dpi=110)
        plt.close(fig)


def main() -> None:
    dirs = [Path(a) for a in sys.argv[1:]] or [sorted(SESSIONS.glob("*"))[-1]]
    for d in dirs:
        d = d.resolve()
        data = parse(d / "console.log")
        out = d / "bench_analysis"
        out.mkdir(exist_ok=True)
        regs = decode_regs(data["regs"])
        steps = [e for r in data["runs"] if r["kind"] == "step" for e in step_edges(r)]
        bode = [b for r in data["runs"] if r["kind"] == "sweep" for b in sweep_bode(r)]
        write_csv(out / "regdump.csv", regs)
        write_csv(out / "step_metrics.csv", steps)
        write_csv(out / "sweep_bode.csv", bode)
        plots(out, data["runs"], bode)
        summary = {"session": d.name, "runs": [{k: r[k] for k in ("kind", "joint", "status", "fails", "torque_off")}
                                                | {"samples": len(r["s"])} for r in data["runs"]],
                   "regs_identical": len({json.dumps({k: v for k, v in r.items() if k not in
                                                      ("servo_id", "id", "pos_offset", "volt_0.1v", "temp_c")})
                                          for r in regs}) <= 1 if regs else None}
        (out / "summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
        print(f"[bench] {d.name}: {len(regs)} register dumps, {len(steps)} step edges, {len(bode)} bode points -> {out}")
        if regs:
            r0 = regs[0]
            print("  regs: " + " ".join(f"{k}={r0[k]}" for k in ("p_gain", "d_gain", "i_gain", "max_torque_0.1pct",
                                                              "protect_current_6.5ma", "unload_cond", "acc", "run_speed")
                                        if k in r0) + f" | identical across servos: {summary['regs_identical']}")
        for e in steps:
            print(f"  STEP {e['joint']:>16} {e['step_deg']:+6.1f} deg {e['edge']:>4}: dead {e['dead_ms']:5.1f} t50 "
                  f"{e['t50_ms']:5.1f} rise {e['rise_ms']:5.1f} ms, overshoot {e['overshoot_pct']:5.1f} %, settle "
                  f"{e['settle_ms']:5.0f} ms, ss_err {e['ss_err_deg']:+5.2f} deg, vmax {e['vmax_dps']:4.0f} deg/s, "
                  f"dt {e['dt_ms_median']:.2f} ms")
        for b in bode:
            print(f"  SWEEP {b['joint']:>16} {b['amp_deg']:g} deg {b['freq_hz']:5.2f} Hz: gain {b['gain']:.3f} "
                  f"lag {b['phase_lag_deg']:6.1f} deg delay {b['delay_ms']:5.1f} ms")


if __name__ == "__main__":
    main()
