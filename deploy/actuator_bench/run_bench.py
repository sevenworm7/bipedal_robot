r"""Run the single-servo actuator bench (E0 register dump, E1 steps, E2 sine sweeps) on the robot through robot_cli.

  .venv_fw\Scripts\python.exe deploy\actuator_bench\run_bench.py --suite regdump
  .venv_fw\Scripts\python.exe deploy\actuator_bench\run_bench.py --suite step --joints left_knee left_hip_pitch
  .venv_fw\Scripts\python.exe deploy\actuator_bench\run_bench.py --suite sweep --joints left_knee [--dry-run]

The firmware commands (deploy/firmware/src/main_rp2350.c) are blocking and any byte from the host aborts them, so
every command gets a host-side WAIT long enough for it to finish. Each command switches on ONE servo only, starting
from where that joint hangs (torque off); targets outside the joint range are rejected by the firmware (ERROR line,
the run continues). Safety: the robot hangs free and is held; the other nine joints stay limp.

Output: a robot_cli session folder outputs/robot_v2/deploy/sessions/<time>/console.log (all lines, "BS ..." samples
included) plus bench_plan.json (suite, commands). Analyse it with analyze.py.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "deploy" / "host" / "robot_cli.py"
SESSIONS = ROOT / "outputs" / "robot_v2" / "deploy" / "sessions"

LEGS = ["left_knee", "left_hip_pitch", "left_ankle_pitch", "left_hip_roll",
        "right_knee", "right_hip_pitch", "right_ankle_pitch", "right_hip_roll"]
TAIL = ["tail_yaw", "tail_pitch"]

# E1: step sizes [deg], both directions (the firmware rejects the ones that leave the joint range)
STEP_DEG = [2.0, 5.0, 10.0, 20.0]
STEP_HOLD_MS = 800
# E2: (amplitude deg, f0 Hz, f1 Hz, duration s); 2 pi amp f1 <= 400 deg/s (firmware limit)
SWEEPS = [(5.0, 0.5, 8.0, 20.0), (10.0, 0.5, 6.0, 20.0)]
# every command first moves the joint from where it hangs to the stance angle at 20 deg/s and holds 0.5 s
# (firmware); allow up to 70 deg of travel
MOVE_S = 4.0


def plan(suite: str, joints: list[str]) -> list[str]:
    run: list[str] = ["ALIGN"]
    if suite in ("regdump", "all"):
        run += [f"REGDUMP {i}" for i in range(1, 11)]
    if suite in ("step", "all"):
        for j in joints:
            for d in STEP_DEG:
                for s in (1, -1):
                    run += [f"STEP {j} {s * d:g} {STEP_HOLD_MS}", f"WAIT {2 * STEP_HOLD_MS / 1000 + 1.0 + MOVE_S:g}"]
    if suite in ("sweep", "all"):
        for j in joints:
            for amp, f0, f1, dur in SWEEPS:
                run += [f"SWEEP {j} {amp:g} {f0:g} {f1:g} {dur:g}", f"WAIT {dur + 2.0 + MOVE_S:g}"]
    return run


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", choices=["regdump", "step", "sweep", "all"], required=True)
    ap.add_argument("--joints", nargs="+", default=["left_knee", "left_hip_pitch"], choices=LEGS + TAIL)
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--dry-run", action="store_true", help="print the command list only")
    a = ap.parse_args()
    run = plan(a.suite, a.joints)
    host_s = sum(float(c.split()[1]) for c in run if c.startswith("WAIT ")) + sum(1 for c in run if not c.startswith("WAIT "))
    print(f"[bench] suite={a.suite} joints={' '.join(a.joints)} commands={sum(1 for c in run if not c.startswith('WAIT '))} "
          f"about {host_s / 60:.1f} min")
    if a.dry_run:
        print("\n".join(run))
        return
    before = set(SESSIONS.glob("*")) if SESSIONS.exists() else set()
    py = ROOT / ".venv_fw" / "Scripts" / "python.exe"
    p = subprocess.Popen([str(py), str(CLI), "--port", a.port, "--run", *run, "--exit-after", "2"],
                         stdout=subprocess.PIPE, text=True, encoding="ascii", errors="replace")
    n_bs = 0
    for line in p.stdout:  # the BS samples go to console.log only
        if line.startswith("BS "):
            n_bs += 1
            continue
        print(line, end="")
    p.wait()
    new = sorted(set(SESSIONS.glob("*")) - before)
    if new:
        (new[-1] / "bench_plan.json").write_text(json.dumps({"suite": a.suite, "joints": a.joints, "commands": run,
                                                              "step_deg": STEP_DEG, "sweeps": SWEEPS}, indent=1))
        print(f"[bench] {n_bs} samples; session {new[-1]}")
    sys.exit(p.returncode)


if __name__ == "__main__":
    main()
