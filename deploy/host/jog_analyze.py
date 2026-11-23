"""Analyse a JOG run (single-servo bench move) from a robot_cli session log.

  .venv_fw\\Scripts\\python.exe deploy\\host\\jog_analyze.py [session_dir_or_console.log] [--all]

Reads the "JOG start" header and the 100 Hz "JOG t_ms goal pos speed load volt" lines and reports:
  - direction: does the count move the same way as the goal (sign check of the calibration)
  - tracking: final hold error, max error during the ramps
  - latency: time shift that best aligns position to goal over the ramps (least squares, 0..200 ms)
  - speed register: ratio of speed_raw to the finite-difference velocity (counts/s), to find its unit
  - load and voltage range
Default: the newest session folder, every JOG in it (one result block per JOG).
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SESSIONS = ROOT / "outputs" / "robot_v2" / "deploy" / "sessions"
DEG_PER_COUNT = 360.0 / 4096.0
START = re.compile(r"JOG start joint=(\S+) id=(\d+) q0=(\S+)deg target=(\S+)deg c0=(-?\d+) c1=(-?\d+) speed=(\S+)deg/s")
LINE = re.compile(r"JOG (\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (\d+)$")


def runs(text: str) -> list[dict]:
    out, cur = [], None
    for line in text.splitlines():
        line = line.strip()
        if "] " in line[:40] and line.startswith("["):
            line = line.split("] ", 1)[1]  # robot_cli prefix, if any
        m = START.search(line)
        if m:
            cur = {"joint": m[1], "id": int(m[2]), "c0": int(m[5]), "c1": int(m[6]), "speed": float(m[7]), "rows": []}
            out.append(cur)
            continue
        if cur is not None:
            if "JOG done" in line:
                cur["done"] = line[line.index("JOG done"):]
                cur = None
                continue
            m = LINE.search(line)
            if m:
                cur["rows"].append(tuple(int(x) for x in m.groups()))
    return out


def analyse(r: dict) -> list[str]:
    rows = r["rows"]
    if len(rows) < 20:
        return [f"{r['joint']}: only {len(rows)} samples"]
    t = [x[0] / 1000.0 for x in rows]
    goal = [x[1] for x in rows]
    pos = [x[2] for x in rows]
    spd = [x[3] for x in rows]
    load = [x[4] for x in rows]
    volt = [x[5] / 10.0 for x in rows]
    span = r["c1"] - r["c0"]
    moved = max(pos) - min(pos)
    # direction: position at the end of the outward hold vs start
    i_out = max(range(len(goal)), key=lambda k: abs(goal[k] - r["c0"]))
    dpos = pos[i_out] - pos[0]
    same_dir = (dpos > 0) == (span > 0) and abs(dpos) > 0.3 * abs(span)
    # hold errors: last 0.3 s of the outward hold and of the final hold
    def hold_err(t_end: float) -> float:
        e = [pos[k] - goal[k] for k in range(len(t)) if t_end - 0.3 <= t[k] <= t_end]
        return sum(e) / len(e) if e else float("nan")
    ramp = abs(span) / (r["speed"] / DEG_PER_COUNT)
    e_out = hold_err(ramp + 1.0)
    e_end = hold_err(t[-1])
    ramp_err = max(abs(pos[k] - goal[k]) for k in range(len(t)))
    # latency: shift position back by d and compare with goal, on samples where the goal is moving
    moving = [k for k in range(1, len(t)) if goal[k] != goal[k - 1]]
    best = (float("inf"), 0.0)
    for d_ms in range(0, 205, 5):
        d = d_ms / 1000.0
        sse, n, j = 0.0, 0, 0
        for k in moving:
            tk = t[k] + d
            while j + 1 < len(t) and t[j + 1] <= tk:
                j += 1
            if j + 1 >= len(t):
                break
            a = (tk - t[j]) / (t[j + 1] - t[j]) if t[j + 1] > t[j] else 0.0
            p = pos[j] + a * (pos[j + 1] - pos[j])
            sse += (p - goal[k]) ** 2
            n += 1
        if n and sse / n < best[0]:
            best = (sse / n, d_ms)
    # speed register vs finite difference (counts/s), over the ramps
    ratios = []
    for k in range(2, len(t) - 2):
        dt = t[k + 2] - t[k - 2]
        v = (pos[k + 2] - pos[k - 2]) / dt if dt > 0 else 0.0
        if abs(v) > 40 and spd[k] != 0:
            ratios.append(spd[k] / v)
    ratios.sort()
    ratio = ratios[len(ratios) // 2] if ratios else float("nan")
    dt_ms = [1000 * (t[k] - t[k - 1]) for k in range(1, len(t))]
    return [
        f"{r['joint']} (id {r['id']}): goal {span:+d} counts ({span * DEG_PER_COUNT:+.1f} deg) at {r['speed']:.0f} deg/s; "
        f"{len(rows)} samples, period mean {sum(dt_ms) / len(dt_ms):.1f} ms",
        f"  direction       : position moved {dpos:+d} counts -> {'OK (same as goal)' if same_dir else 'WRONG / too small'}; "
        f"total travel {moved} counts ({moved * DEG_PER_COUNT:.1f} deg)",
        f"  hold error      : outward {e_out:+.1f} counts ({e_out * DEG_PER_COUNT:+.2f} deg), "
        f"final {e_end:+.1f} counts ({e_end * DEG_PER_COUNT:+.2f} deg)",
        f"  max |pos-goal|  : {ramp_err} counts ({ramp_err * DEG_PER_COUNT:.2f} deg)",
        f"  latency (fit)   : {best[1]:.0f} ms (rms residual {best[0] ** 0.5:.1f} counts)",
        f"  speed register  : median speed_raw / finite-diff = {ratio:.3f} ({len(ratios)} samples)",
        f"  load raw        : {min(load)} .. {max(load)};  voltage {min(volt):.1f} .. {max(volt):.1f} V",
        f"  {r.get('done', 'JOG done line missing')}",
    ]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", type=Path)
    a = ap.parse_args()
    p = a.path or sorted(d for d in SESSIONS.iterdir() if d.is_dir())[-1]
    log = p / "console.log" if p.is_dir() else p
    rs = runs(log.read_text(encoding="utf-8", errors="replace"))
    if not rs:
        print(f"no JOG runs in {log}")
        return 1
    print(f"[jog] {log}")
    for r in rs:
        print("\n".join(analyse(r)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
