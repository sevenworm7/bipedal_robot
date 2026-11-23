"""Hand-move probe: which servo ID belongs to which joint, and which way its count moves (torque OFF, read only).

The operator moves one joint at a time, in the order of `ORDER`, first in the direction given by `FIRST_DIR`
(+1 = the joint's positive direction, -1 = negative; ~30 deg), back, then holds still >= 2 s before the next joint.
Both hip rolls are moved OUTWARD first (the two hips sit close together and collide when moved inward), i.e. right
hip roll negative, left hip roll positive. The board (v2 runtime, `SCAN` command) is polled
for raw positions; motion segments are cut automatically and each one is assigned to the servo that moved most.

  .venv_fw\\Scripts\\python.exe deploy\\calibration\\joint_probe.py [--port COM3] [--max-seconds 300]
  .venv_fw\\Scripts\\python.exe deploy\\calibration\\joint_probe.py --analyze <probe dir>   (re-run the analysis)
  .venv_fw\\Scripts\\python.exe deploy\\calibration\\joint_probe.py --joints left_ankle_pitch,tail_yaw   (subset)
  .venv_fw\\Scripts\\python.exe deploy\\calibration\\joint_probe.py --capture <label> [--seconds 3]
      static pose capture (zero pose, check poses): mean / std / min / max count per ID ->
      outputs/robot_v2/deploy/calibration/capture_<ts>_<label>/capture.json

Output: outputs/robot_v2/deploy/calibration/probe_<ts>/ (samples.csv, report.txt, report.json). No writes to servos.
Positive directions (URDF axes, +X forward, +Y left, +Z up):
  hip_roll +X: foot swings to the robot's LEFT     hip_pitch +Y: leg swings BACKWARD
  knee +Y: bends, shank swings BACKWARD (heel toward the seat, like a human knee; 0..110 deg)
  ankle_pitch +Y: toes DOWN
  tail_yaw +Z: tail tip to the robot's RIGHT        tail_pitch +Y: tail tip UP
"""
from __future__ import annotations

import argparse
import csv
import json
import re
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CALIB = ROOT / "deploy" / "calibration" / "robot_v2.json"
ORDER = ["right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle_pitch",
         "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle_pitch", "tail_yaw", "tail_pitch"]
# direction of the FIRST hand move per joint, in joint-angle sign (hip rolls outward: right -, left +)
FIRST_DIR = {"right_hip_roll": -1, "left_hip_roll": +1}
SCAN_RE = re.compile(r"SCAN id=(\d+) .*?pos=(-?\d+)")
MOVE_COUNTS = 25        # counts change within WINDOW_S that marks "moving" (~2.2 deg)
WINDOW_S = 0.3
MERGE_GAP_S = 1.0       # active periods closer than this belong to one segment
FIRST_EXCURSION = 40    # counts away from the segment start that fix the direction
MIN_SEG_DEG = 20.0      # smaller motions (handling the robot) are not counted as a joint (2026-11-23 run)


def poll(port: str, max_s: float, out: Path, ids: list[int], order: list[str], auto_stop: bool
         ) -> list[tuple[float, dict[int, int]]]:
    import serial  # pyserial from .venv_fw

    rows: list[tuple[float, dict[int, int]]] = []
    with serial.Serial(port, 115200, timeout=0.5) as sp, open(out / "samples.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t"] + [f"id{i}" for i in ids])
        time.sleep(0.3)
        sp.reset_input_buffer()
        t0 = time.time()
        last_motion = None
        segs_seen = 0
        moving = False
        last_warn = 0.0
        print(f"[probe] polling {port}; move the joints now in this order: {', '.join(order)}", flush=True)
        while time.time() - t0 < max_s:
            sp.write(f"SCAN {max(ids)}\n".encode())
            got: dict[int, int] = {}
            deadline = time.time() + 1.0
            while time.time() < deadline:
                line = sp.readline().decode(errors="replace").strip()
                if line.startswith("SCAN done"):
                    break
                m = SCAN_RE.match(line)
                if m:
                    got[int(m.group(1))] = int(m.group(2))
            if not all(i in got for i in ids):
                missing = [i for i in ids if i not in got]
                if time.time() - last_warn > 2.0:  # cable / power dropout (2026-11-23: bus lost for 87 s)
                    print(f"[probe] WARNING t={time.time() - t0:5.1f}s servos not answering: {missing}", flush=True)
                    last_warn = time.time()
                continue
            t = time.time() - t0
            rows.append((t, got))
            w.writerow([f"{t:.3f}"] + [got[i] for i in ids])
            f.flush()
            # live progress: count segments, stop 4 s after the last expected one
            past = [r for r in rows if t - WINDOW_S - 0.15 <= r[0] <= t - WINDOW_S + 0.15]
            if past:
                d = max(abs(got[i] - past[0][1][i]) for i in ids)
                if d > MOVE_COUNTS:
                    if not moving and (last_motion is None or t - last_motion > MERGE_GAP_S):
                        segs_seen += 1
                        print(f"[probe] t={t:5.1f}s motion #{segs_seen} (expected: "
                              f"{order[segs_seen - 1] if segs_seen <= len(order) else 'extra'})", flush=True)
                    moving, last_motion = True, t
                else:
                    moving = False
            if auto_stop and segs_seen >= len(order) and last_motion is not None and t - last_motion > 4.0:
                break
        sp.write(b"STOP\n")
    return rows


def load(probe: Path) -> tuple[list[int], list[tuple[float, dict[int, int]]]]:
    with open(probe / "samples.csv") as f:
        r = csv.reader(f)
        head = next(r)
        ids = [int(h[2:]) for h in head[1:]]
        rows = [(float(x[0]), {i: int(v) for i, v in zip(ids, x[1:])}) for x in r]
    return ids, rows


def analyze(ids: list[int], rows: list[tuple[float, dict[int, int]]], order: list[str]) -> dict:
    ts = [r[0] for r in rows]
    active = []
    for k, (t, p) in enumerate(rows):
        j = k
        while j > 0 and t - ts[j] < WINDOW_S:
            j -= 1
        active.append(max(abs(p[i] - rows[j][1][i]) for i in ids) > MOVE_COUNTS)
    segs: list[list[int]] = []
    for k, a in enumerate(active):
        if not a:
            continue
        if segs and ts[k] - ts[segs[-1][1]] <= MERGE_GAP_S:
            segs[-1][1] = k
        else:
            segs.append([k, k])
    calib = {j["joint"]: j for j in json.loads(CALIB.read_text(encoding="utf-8"))["joints"]}
    out = []
    n = -1
    for a, b in segs:
        a0 = max(a - 3, 0)  # include the rest just before the motion
        b1 = min(b + 3, len(rows) - 1)
        start = rows[a0][1]
        rng = {i: max(r[1][i] for r in rows[a0:b1 + 1]) - min(r[1][i] for r in rows[a0:b1 + 1]) for i in ids}
        dom = max(ids, key=lambda i: rng[i])
        if rng[dom] * 360 / 4096 < MIN_SEG_DEG:
            continue
        n += 1
        direction = 0
        for r in rows[a0:b1 + 1]:
            dv = r[1][dom] - start[dom]
            if abs(dv) > FIRST_EXCURSION:
                direction = 1 if dv > 0 else -1
                break
        expect = order[n] if n < len(order) else None
        c = calib.get(expect, {}) if expect else {}
        others = {i: rng[i] for i in ids if i != dom and rng[i] > 0.3 * rng[dom]}
        first = FIRST_DIR.get(expect, 1) if expect else 1
        sign = direction * first  # count change per positive joint angle
        out.append({
            "segment": n + 1, "t": [round(ts[a], 2), round(ts[b], 2)], "expected_joint": expect,
            "moved_id": dom, "range_counts": rng[dom], "range_deg": round(rng[dom] * 360 / 4096, 1),
            "first_move_joint_dir": first, "first_direction_counts": direction, "measured_sign": sign,
            "also_moved": others, "calib_id": c.get("id"), "calib_sign": c.get("sign"),
            "id_match": c.get("id") == dom if c else None,
            "sign_match": c.get("sign") == sign if c and sign else None,
            "rest_count_before": start[dom],
        })
    return {"segments": out, "n_segments": len(out), "n_expected": len(order), "order": order, "samples": len(rows),
            "duration_s": round(ts[-1], 1) if ts else 0}


def report(res: dict, probe: Path) -> None:
    lines = [f"samples {res['samples']} over {res['duration_s']} s, segments {res['n_segments']} "
             f"(expected {res['n_expected']})", ""]
    lines.append(f"{'#':>2} {'t[s]':>11}  {'expected joint':18} {'moved id':>8} {'calib id':>8} {'range':>7} "
                 f"{'sign':>4} {'calib sign':>10}  notes")
    for s in res["segments"]:
        note = []
        if s["id_match"] is False:
            note.append("ID DIFFERS")
        if s["sign_match"] is False:
            note.append("SIGN DIFFERS")
        if s["also_moved"]:
            note.append("also moved " + ", ".join(f"id{i}:{v}" for i, v in s["also_moved"].items()))
        lines.append(f"{s['segment']:>2} {s['t'][0]:5.1f}-{s['t'][1]:5.1f}  {str(s['expected_joint']):18} "
                     f"{s['moved_id']:>8} {str(s['calib_id']):>8} {s['range_deg']:6.1f}d {s['measured_sign']:>+4} "
                     f"{str(s['calib_sign']):>10}  {'; '.join(note)}")
    text = "\n".join(lines)
    print(text)
    (probe / "report.txt").write_text(text + "\n", encoding="utf-8")
    (probe / "report.json").write_text(json.dumps(res, indent=1), encoding="utf-8")


def capture(port: str, label: str, seconds: float, ids: list[int]) -> int:
    import serial

    probe = ROOT / "outputs" / "robot_v2" / "deploy" / "calibration" / f"capture_{datetime.now():%Y%m%d_%H%M%S}_{label}"
    probe.mkdir(parents=True, exist_ok=False)
    samples: dict[int, list[int]] = {i: [] for i in ids}
    with serial.Serial(port, 115200, timeout=0.5) as sp:
        time.sleep(0.3)
        sp.reset_input_buffer()
        t0 = time.time()
        while time.time() - t0 < seconds:
            sp.write(f"SCAN {max(ids)}\n".encode())
            got: dict[int, int] = {}
            deadline = time.time() + 1.0
            while time.time() < deadline:
                line = sp.readline().decode(errors="replace").strip()
                if line.startswith("SCAN done"):
                    break
                m = SCAN_RE.match(line)
                if m:
                    got[int(m.group(1))] = int(m.group(2))
            if all(i in got for i in ids):
                for i in ids:
                    samples[i].append(got[i])
        sp.write(b"STOP\n")
    n = len(samples[ids[0]])
    if n == 0:
        print("[capture] no complete scan (servos not answering?)", file=sys.stderr)
        return 1
    res = {"label": label, "samples": n, "seconds": seconds, "ids": {}}
    for i in ids:
        v = samples[i]
        m = sum(v) / n
        sd = (sum((x - m) ** 2 for x in v) / n) ** 0.5
        res["ids"][str(i)] = {"mean": round(m, 1), "std": round(sd, 2), "min": min(v), "max": max(v)}
        print(f"  id{i:<3} mean {m:8.1f}  std {sd:5.2f}  range {max(v) - min(v):4d}")
    (probe / "capture.json").write_text(json.dumps(res, indent=1), encoding="utf-8")
    print(f"[capture] {n} scans -> {probe}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--max-seconds", type=float, default=300)
    ap.add_argument("--ids", default="1,2,3,4,5,6,7,8,9,10")
    ap.add_argument("--analyze", type=Path, help="re-analyze an existing probe folder")
    ap.add_argument("--joints", help="comma-separated subset/order of joints to probe (default: all 10, ORDER)")
    ap.add_argument("--capture", metavar="LABEL", help="static pose capture instead of a hand-move probe")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--auto-stop", action="store_true", help="stop 4 s after the expected number of motions")
    a = ap.parse_args()
    order = a.joints.split(",") if a.joints else ORDER
    bad = [j for j in order if j not in ORDER]
    if bad:
        ap.error(f"unknown joints {bad}")
    if a.analyze:
        ids, rows = load(a.analyze)
        order_file = a.analyze / "order.json"
        if order_file.exists() and not a.joints:
            order = json.loads(order_file.read_text())
        report(analyze(ids, rows, order), a.analyze)
        return 0
    ids = [int(x) for x in a.ids.split(",")]
    if a.capture:
        return capture(a.port, a.capture, a.seconds, ids)
    probe = ROOT / "outputs" / "robot_v2" / "deploy" / "calibration" / f"probe_{datetime.now():%Y%m%d_%H%M%S}"
    probe.mkdir(parents=True, exist_ok=False)
    (probe / "order.json").write_text(json.dumps(order))
    rows = poll(a.port, a.max_seconds, probe, ids, order, a.auto_stop)
    if not rows:
        print("[probe] no samples", file=sys.stderr)
        return 1
    report(analyze(ids, rows, order), probe)
    print(f"[probe] {probe}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
