r"""Staged hanging test of robot v2 with per-servo voltage / current / load / temperature tables (2026-11-25).
Replaces demo_hanging.ps1 (2026-11-23): same two sequences, plus firmware / servo checks and a report.

Modes:
  play : open-loop slowed preview of the simulated gait: INIT -> PLAY <rate> <loops> -> back to stance -> STOP
  walk : closed-loop policy: INIT -> WALK <vx> <freq> for N s -> STAND -> STOP
Stages (one, or `all` in order):
  check  : board answers, flashed firmware build / model, SCAN finds the 10 servos (ID, voltage, temperature); torque off
  flash  : flash the model's pinned UF2 when the flashed build differs (deploy/firmware/flash_firmware.ps1), check again
  run    : the mode's sequence, then STOP (torque off)
  report : per-servo table (tracking error, voltage, temperature, current / load registers) -> report.md

  .venv_fw\Scripts\python.exe deploy\field_tests\hanging_test.py all                                   # play, v2ft, 0.5x
  .venv_fw\Scripts\python.exe deploy\field_tests\hanging_test.py all --mode walk --model v2slow       # WALK 0.05 1.0, 6 s
  .venv_fw\Scripts\python.exe deploy\field_tests\hanging_test.py all --mode walk --model v2ft --vstop 5.5
  .venv_fw\Scripts\python.exe deploy\field_tests\hanging_test.py report [--session <dir>]

Firmware 0.2 builds (outputs/robot_v2/deploy/firmware_build/): v2ft 20261125_155757 (export 20261123_103850; playback =
simulated v2ft gait, vx 0.1 m/s, 1.6 Hz), v2slow 20261125_155809 (export 20261124_193736; playback vx 0.05, 1.0 Hz).
PLAY 0.5 1 = 1.5 s ramp + 8.1 s gait + 1.5 s ramp. INIT can take 11 s (tail hanging at -75 deg); we poll STATUS.
Safety: the robot must hang free and be held; STOP at the end switches torque off (legs and tail drop). Any byte sent
during PLAY aborts it with torque off, so nothing is sent until "PLAY done". --vstop below 6.5 V: hanging only.
Units: volt_raw 0.1 V; current_raw x 6.5 mA is the HLS-series factor, NOT verified for the HD-1910 (backlog #21, E4).
Output: outputs/robot_v2/deploy/sessions/<time>/ console.log, servo_telemetry.csv (per tick, per servo), report.md.
"""
from __future__ import annotations

import argparse
import csv
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "host"))
from robot_cli import HEADER as T_HEADER  # noqa: E402  (firmware "T" telemetry columns, deploy/host)

ROOT = Path(__file__).resolve().parents[2]
DEPLOY_OUT = ROOT / "outputs" / "robot_v2" / "deploy"
JOINTS = ["left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle_pitch", "right_hip_roll", "right_hip_pitch",
          "right_knee", "right_ankle_pitch", "tail_yaw", "tail_pitch"]
SERVO_IDS = list(range(1, 11))
MODELS = {  # firmware build, default WALK frequency (inside the model's trained range)
    "v2ft": {"build": "20261125_155757", "freq": 1.6},
    "v2slow": {"build": "20261125_155809", "freq": 1.0},
}
CURRENT_MA_PER_RAW = 6.5  # not verified for HD-1910
V_WARN = 7.0              # idle bus voltage below this: warn before the run
FIELDS = ["q", "target", "volt_raw", "temp", "load_raw", "current_raw"]
SERVO_HEADER = ["t_ms", "state"] + [f"{f}_{j}" for j in JOINTS for f in FIELDS]
STATE_FAULT = "4"         # controller.c: OFF INIT STAND WALK FAULT


def find_port() -> str:
    for p in list_ports.comports():
        if "BIPED V2 RUNTIME" in (p.product or "") or "BIPED V2 RUNTIME" in (p.description or ""):
            return p.device
    rp = [p.device for p in list_ports.comports() if p.vid == 0x2E8A]  # Raspberry Pi USB VID (RP2350 board)
    if len(rp) == 1:
        return rp[0]
    sys.exit("[check] FAIL board not found over USB (is the MANGO X4 plugged in?); use --port COMx")


class Board:
    """Serial link; every line goes to console.log when a log file is given."""

    def __init__(self, port: str, log=None) -> None:
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.log = log
        self.buf = b""
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def send(self, cmd: str) -> None:
        if self.log:
            self.log.write(f"{time.time():.3f} >> {cmd}\n")
        self.ser.write((cmd + "\n").encode("ascii"))

    def lines(self, seconds: float):
        """Yield board lines for up to `seconds` (the caller may stop early)."""
        end = time.time() + seconds
        while time.time() < end:
            self.buf += self.ser.read(8192)
            while b"\n" in self.buf:
                raw, self.buf = self.buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").rstrip("\r")
                if self.log:
                    self.log.write(f"{time.time():.3f} {line}\n")
                yield line

    def ask(self, cmd: str, until: str, seconds: float = 5.0) -> list[str]:
        self.send(cmd)
        out = []
        for line in self.lines(seconds):
            out.append(line)
            if line.startswith(until):
                break
        return out

    def close(self) -> None:
        self.ser.close()


# ---------------------------------------------------------------- stages

def stage_check(port: str, build: str) -> dict:
    print("\n=== Stage 1: board, firmware, servos (torque stays off) ===")
    b = Board(port)
    try:
        status = b.ask("STATUS", "IMU")
        scan = b.ask("SCAN", "SCAN done", 15.0)
    finally:
        b.close()
    info = {"build": None, "model": None, "playback": None, "servos": {}}
    for line in status:
        if line.startswith("STATUS"):
            m = re.search(r"state=(\S+) fault=(\S+)", line)
            info["state"], info["fault"] = (m.group(1), m.group(2)) if m else ("?", "?")
            info["fw"] = " ".join(line.split()[1:3])
        if line.startswith("MODEL"):
            info["model"] = line.split()[1]
            m = re.search(r"build=(\S+)", line)
            info["build"] = m.group(1) if m else None
            m = re.search(r"playback=(\d)", line)
            info["playback"] = m.group(1) == "1" if m else None
    for line in scan:
        m = re.match(r"SCAN id=(\d+) .*volt=([\d.]+) temp=(\d+)", line)
        if m:
            info["servos"][int(m.group(1))] = (float(m.group(2)), int(m.group(3)))
    print(f"firmware : {info.get('fw', '?')}  build={info['build']}  (want {build})")
    print(f"model    : {info['model']}  playback={'yes' if info['playback'] else 'no/unknown'}")
    print(f"state    : {info.get('state', '?')}  fault={info.get('fault', '?')}")
    print("\n| ID | joint | voltage V | temp C |\n|---|---|---|---|")
    for sid, name in zip(SERVO_IDS, JOINTS):
        v = info["servos"].get(sid)
        print(f"| {sid} | {name} | {v[0]:.1f} | {v[1]} |" if v else f"| {sid} | {name} | MISSING | - |")
    missing = [s for s in SERVO_IDS if s not in info["servos"]]
    info["servos_ok"] = not missing and len(info["servos"]) == len(SERVO_IDS)
    info["build_ok"] = info["build"] == build
    volts = [v for v, _ in info["servos"].values()]
    if not info["servos"]:
        print("[check] FAIL no servo answers: servo power off, or servo cable not on the UART1 (GPIO8/9) connector")
    elif missing:
        print(f"[check] FAIL missing servo IDs {missing}: check the daisy-chain cable before / at those servos")
    else:
        print(f"[check] servos OK: 10/10, voltage {min(volts):.1f}-{max(volts):.1f} V")
        if min(volts) < V_WARN:
            print(f"[check] WARNING idle voltage {min(volts):.1f} V < {V_WARN} V: supply is weak")
    print(f"[check] firmware {'OK' if info['build_ok'] else 'NEEDS FLASH'}")
    return info


def stage_flash(port: str, build: str) -> None:
    uf2 = DEPLOY_OUT / "firmware_build" / build / "biped_runtime.uf2"
    if not uf2.exists():
        sys.exit(f"[flash] FAIL {uf2} not found")
    print(f"\n=== Stage 2: flash {uf2.parent.name} ===")
    script = ROOT / "deploy" / "firmware" / "flash_firmware.ps1"
    r = subprocess.run(["powershell", "-ExecutionPolicy", "Bypass", "-File", str(script), "-Uf2", str(uf2),
                        "-Port", port])
    if r.returncode != 0:
        sys.exit("[flash] FAIL flashing did not complete (see output above)")
    time.sleep(2.0)


def t_to_servo_row(line: str) -> list[str] | None:
    """Firmware 0.2 'T' line -> SERVO_HEADER row (None for older firmware without per-servo voltage)."""
    v = dict(zip(T_HEADER, line.split()[1:]))
    if f"temp_{JOINTS[-1]}" not in v:
        return None
    row = [v["t_ms"], v["state"]]
    for j in JOINTS:
        row += [v[f"q_{j}"], v[f"target_{j}"], v[f"volt_raw_{j}"], v[f"temp_{j}"], v[f"load_raw_{j}"],
                v[f"current_raw_{j}"]]
    return row


def init_to_stand(b: Board, pre: list[str]) -> None:
    for cmd in ["ALIGN"] + pre + ["INIT"]:
        b.send(cmd)
        for line in b.lines(1.0):
            print(line)
            if line.startswith("ERROR"):
                raise RuntimeError(line)
    t_end = time.time() + 25.0
    while time.time() < t_end:
        b.send("STATUS")
        for line in b.lines(1.0):
            if line.startswith("ERROR") or ("fault=" in line and "fault=NONE" not in line):
                raise RuntimeError(line)
            if line.startswith("STATUS"):
                print(f"[run] INIT ... {line.split()[3]}")
                if "state=STAND" in line:
                    return
    raise RuntimeError("INIT did not reach STAND in 25 s")


def stage_run(port: str, a) -> Path:
    if a.mode == "play":
        gait_s = 202 / 50.0 / a.rate * a.loops
        desc = f"PLAY {a.rate} x{a.loops}: gait {gait_s:.1f} s, about {gait_s + 3:.1f} s with the ramps"
    else:
        desc = f"WALK vx={a.vx} m/s freq={a.freq} Hz for {a.seconds} s, voltage stop {a.vstop or 6.5} V"
    print(f"\n=== Stage 3: {a.model} {desc} ===")
    if a.vstop and a.vstop < 6.5:
        print(f"[run] WARNING voltage stop lowered to {a.vstop} V: hanging tests only")
    if not a.yes:
        ans = input("Robot hanging free (feet and tail touch nothing) and held by someone? Type yes to start: ")
        if ans.strip().lower() != "yes":
            sys.exit("[run] cancelled")
    out = DEPLOY_OUT / "sessions" / datetime.now().strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=False)
    log = (out / "console.log").open("w", encoding="utf-8", buffering=1)
    (out / "test.txt").write_text(f"hanging_test {a.mode} model={a.model} build={a.build} {desc}\n", encoding="utf-8")
    fcsv = (out / "servo_telemetry.csv").open("w", encoding="utf-8", newline="")
    w = csv.writer(fcsv)
    w.writerow(SERVO_HEADER)
    b = Board(port, log)
    print(f"[run] session {out}")
    result = "no result"
    try:
        init_to_stand(b, [f"VSTOP {a.vstop}"] if a.vstop else [])
        if a.mode == "play":
            b.send("TELEM 1")  # PLAY prints one PLT line per tick while telemetry is on
            list(b.lines(0.3))
            b.send(f"PLAY {a.rate} {a.loops}")
            for line in b.lines(202 / 50.0 / a.rate * a.loops + 13.0):
                if line.startswith("PLT "):
                    p = line.split()
                    w.writerow([p[1], "PLAY"] + p[3:])
                elif line.startswith("PL "):
                    p = line.split()
                    print(f"\r[run] t={int(p[1]) / 1000:5.1f} s  frame={p[2]:>3}  v_min={p[3]} V  "
                          f"max_err={p[4]} deg ({p[5]})   ", end="", flush=True)
                elif not line.startswith("T "):
                    print("\n" + line)
                if line.startswith(("PLAY done", "ERROR")):
                    result = line
                    break
        else:
            b.send("TELEM 1")
            list(b.lines(0.3))
            b.send(f"WALK {a.vx} {a.freq}")
            t_walk = time.time()
            faulted = False
            for line in b.lines(a.seconds):
                if line.startswith("T "):
                    row = t_to_servo_row(line)
                    if row is None:
                        raise RuntimeError("telemetry without per-servo voltage: firmware older than 0.2")
                    w.writerow(row)
                    vmin = min(int(row[4 + 6 * k]) for k in range(len(JOINTS))) / 10
                    print(f"\r[run] t={time.time() - t_walk:4.1f} s  v_min={vmin:.1f} V   ", end="", flush=True)
                    if row[1] == STATE_FAULT:
                        faulted = True
                else:
                    print("\n" + line)
                    if line.startswith("ERROR"):
                        raise RuntimeError(line)
                if faulted:
                    break
            if faulted:
                result = f"FAULT after {time.time() - t_walk:.2f} s (see console.log)"
            else:
                b.send("STAND")
                for line in b.lines(1.5):
                    if line.startswith("T "):
                        row = t_to_servo_row(line)
                        if row:
                            w.writerow(row)
                result = f"WALK {a.seconds} s done, STAND"
    except KeyboardInterrupt:
        result = "interrupted (Ctrl-C)"
    except RuntimeError as e:
        result = f"stopped: {e}"
    finally:
        b.send("STOP")
        list(b.lines(0.5))
        b.close()
        fcsv.close()
        log.write(f"{time.time():.3f} ## RESULT {result}\n")
        log.close()
    print(f"\n[run] {result}")
    print("[run] STOP sent: torque off")
    return out


def stage_report(session: Path) -> None:
    print(f"\n=== Stage 4: report {session} ===")
    path = session / "servo_telemetry.csv"
    with path.open(encoding="utf-8") as f:
        rows = [r for r in csv.DictReader(f) if r.get("t_ms")]
    if not rows:
        sys.exit(f"[report] no telemetry rows in {path}")
    test = (session / "test.txt").read_text(encoding="utf-8").strip() if (session / "test.txt").exists() else ""
    result = next((l.split("## RESULT ", 1)[1] for l in (session / "console.log").read_text(encoding="utf-8")
                   .splitlines() if "## RESULT " in l), "no result line")
    dur = (int(rows[-1]["t_ms"]) - int(rows[0]["t_ms"])) / 1000
    k = CURRENT_MA_PER_RAW / 1000

    def col(j: str, f: str) -> list[float]:
        return [float(r[f"{f}_{j}"]) for r in rows]

    def mean(xs: list[float]) -> float:
        return sum(xs) / len(xs)

    lines = [f"# Hanging test {session.name}", "", f"- {test}", f"- result: {result}",
             f"- samples: {len(rows)} over {dur:.1f} s",
             f"- A* = current_raw x {CURRENT_MA_PER_RAW} mA (HLS factor, **not verified** for HD-1910)", "",
             "| joint | max err deg | mean err deg | V min | V mean | temp max C | I mean raw | I peak raw "
             "| I mean A* | I peak A* | load mean | load peak |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    tot = [0.0] * len(rows)
    vmin_tick = [99.0] * len(rows)
    for j in JOINTS:
        err = [abs(x - y) * 57.2958 for x, y in zip(col(j, "q"), col(j, "target"))]
        v = [x * 0.1 for x in col(j, "volt_raw")]
        cur = [abs(x) for x in col(j, "current_raw")]
        load = [abs(x) for x in col(j, "load_raw")]
        for i, c in enumerate(cur):
            tot[i] += c
            vmin_tick[i] = min(vmin_tick[i], v[i])
        lines.append(f"| {j} | {max(err):.2f} | {mean(err):.2f} | {min(v):.1f} | {mean(v):.2f} "
                     f"| {max(col(j, 'temp')):.0f} | {mean(cur):.0f} | {max(cur):.0f} | {mean(cur) * k:.2f} "
                     f"| {max(cur) * k:.2f} | {mean(load):.0f} | {max(load):.0f} |")
    lines += ["", "| whole robot | value |", "|---|---|",
              f"| bus voltage min / mean (lowest servo each tick) | {min(vmin_tick):.1f} / {mean(vmin_tick):.2f} V |",
              f"| summed current mean / peak (raw) | {mean(tot):.0f} / {max(tot):.0f} |",
              f"| summed current mean / peak (A*, unverified) | {mean(tot) * k:.2f} / {max(tot) * k:.2f} A |"]
    text = "\n".join(lines) + "\n"
    (session / "report.md").write_text(text, encoding="utf-8")
    print(text)
    print(f"[report] saved {session / 'report.md'}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=["all", "check", "flash", "run", "report"])
    ap.add_argument("--mode", choices=["play", "walk"], default="play")
    ap.add_argument("--model", choices=sorted(MODELS), default="v2ft")
    ap.add_argument("--build", help="override the model's firmware_build/<name>")
    ap.add_argument("--port")
    ap.add_argument("--rate", type=float, default=0.5, help="play: 0.05-1.0")
    ap.add_argument("--loops", type=int, default=1, choices=range(1, 6), help="play: gait repetitions")
    ap.add_argument("--vx", type=float, default=0.05, help="walk: m/s")
    ap.add_argument("--freq", type=float, help="walk: Hz (default v2ft 1.6, v2slow 1.0)")
    ap.add_argument("--seconds", type=float, default=6.0, help="walk: 1-30 s")
    ap.add_argument("--vstop", type=float, help="voltage stop 5.5-7.0 V (firmware default 6.5 V)")
    ap.add_argument("--session", type=Path, help="report: session folder (default newest)")
    ap.add_argument("--yes", action="store_true", help="skip the 'robot is hanging' question")
    a = ap.parse_args()
    a.build = a.build or MODELS[a.model]["build"]
    a.freq = a.freq or MODELS[a.model]["freq"]
    if not 0.05 <= a.rate <= 1.0 or not 1 <= a.seconds <= 30 or (a.vstop and not 5.5 <= a.vstop <= 7.0):
        sys.exit("ranges: --rate 0.05-1.0, --seconds 1-30, --vstop 5.5-7.0")

    if a.stage == "report":
        s = a.session or max((p.parent for p in (DEPLOY_OUT / "sessions").glob("*/servo_telemetry.csv")),
                             key=lambda p: p.name)
        stage_report(s)
        return
    port = a.port or find_port()
    if a.stage == "flash":
        stage_flash(port, a.build)
        stage_check(port, a.build)
        return
    info = stage_check(port, a.build)
    if a.stage == "check":
        return
    if not info["build_ok"]:
        if a.stage != "all":
            sys.exit("[run] firmware is not the expected build: run the flash stage first (or `all`)")
        stage_flash(port, a.build)
        port = a.port or find_port()
        info = stage_check(port, a.build)
        if not info["build_ok"]:
            sys.exit("[flash] FAIL board still reports another build")
    if not info["servos_ok"]:
        sys.exit("[run] not started: servos missing (stage 1)")
    session = stage_run(port, a)
    with (session / "servo_telemetry.csv").open(encoding="utf-8") as f:
        has_rows = sum(1 for _ in f) > 1
    if has_rows:
        stage_report(session)


if __name__ == "__main__":
    main()
