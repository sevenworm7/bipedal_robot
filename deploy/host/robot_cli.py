r"""PC console for the robot v2 runtime firmware (deploy/firmware) over USB.

Type firmware commands (PING, STATUS, BENCH, BUSTEST 100, IMUCAL, INIT, WALK 0.1 1.6, SET ..., STAND, STOP, TELEM 1).
Ctrl-C or typing STOP sends STOP (torque off: the robot is no longer held -- support it first).
Every line from the board is saved to outputs/robot_v2/deploy/sessions/<timestamp>/console.log; telemetry lines
("T ...", see main_rp2350.c telemetry_line) go to telemetry.csv.

Usage (project venv with pyserial):
    .venv_fw\Scripts\python.exe deploy\host\robot_cli.py            # finds the board by its USB product name
    .venv_fw\Scripts\python.exe deploy\host\robot_cli.py --port COM5
    .venv_fw\Scripts\python.exe deploy\host\robot_cli.py --run "PING" "STATUS" "BENCH" --exit-after 3
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

ROOT = Path(__file__).resolve().parents[2]
JOINTS = ["left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle_pitch", "right_hip_roll", "right_hip_pitch",
          "right_knee", "right_ankle_pitch", "tail_yaw", "tail_pitch"]
HEADER = (["t_ms", "state", "loop_us", "bus_us", "ctl_us", "wx", "wy", "wz", "gx", "gy", "gz", "heading", "phase"]
          + [f"q_{j}" for j in JOINTS] + [f"target_{j}" for j in JOINTS] + [f"speed_raw_{j}" for j in JOINTS]
          + ["v_min", "temp_max"] + [f"load_raw_{j}" for j in JOINTS] + [f"current_raw_{j}" for j in JOINTS])


def find_port() -> str:
    for p in list_ports.comports():
        if "BIPED V2 RUNTIME" in (p.product or "") or "BIPED V2 RUNTIME" in (p.description or ""):
            return p.device
    ports = ", ".join(f"{p.device} ({p.description})" for p in list_ports.comports()) or "none"
    sys.exit(f"board not found (ports: {ports}); use --port COMx")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--run", nargs="*", default=[],
                    help="commands sent one per second at start; 'WAIT <s>' pauses on the host side")
    ap.add_argument("--exit-after", type=float, default=None, help="seconds after the --run commands, then exit")
    a = ap.parse_args()
    port = a.port or find_port()
    out = ROOT / "outputs" / "robot_v2" / "deploy" / "sessions" / datetime.now().strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=False)
    log = (out / "console.log").open("w", encoding="utf-8", buffering=1)
    csv = (out / "telemetry.csv").open("w", encoding="utf-8", buffering=1)
    csv.write(",".join(HEADER) + "\n")
    ser = serial.Serial(port, 115200, timeout=0.1)
    print(f"[cli] {port}, session {out}")
    stop = threading.Event()

    def reader() -> None:
        buf = b""
        while not stop.is_set():
            buf += ser.read(4096)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").rstrip("\r")
                log.write(f"{time.time():.3f} {line}\n")
                if line.startswith("T "):
                    csv.write(",".join(line.split()[1:]) + "\n")
                else:
                    print(line)

    th = threading.Thread(target=reader, daemon=True)
    th.start()

    def send(cmd: str) -> None:
        log.write(f"{time.time():.3f} >> {cmd}\n")
        ser.write((cmd.strip() + "\n").encode("ascii"))

    try:
        for cmd in a.run:
            if cmd.upper().startswith("WAIT "):  # host-side pause, e.g. "WAIT 12" until INIT reached the stance
                time.sleep(float(cmd.split()[1]))
                continue
            send(cmd)
            time.sleep(1.0)
        if a.exit_after is not None:
            time.sleep(a.exit_after)
        else:
            for line in sys.stdin:
                if line.strip():
                    send(line)
    except KeyboardInterrupt:
        pass
    finally:
        send("STOP")
        time.sleep(0.5)
        stop.set()
        th.join(timeout=1)
        ser.close()
        print(f"[cli] STOP sent; log in {out}")


if __name__ == "__main__":
    main()
