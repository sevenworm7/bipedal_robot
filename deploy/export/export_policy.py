r"""Export a deployable student (interface v2, 210 -> MLP -> 10) for the MANGO X4 (RP2350) firmware in ``deploy/firmware``.

Writes a new run folder ``outputs/robot_v<N>/deploy/export_<student run>_<timestamp>/`` (never overwritten):

    robot_config.h      joint order, servo IDs / signs / zero counts (from the calibration JSON), default pose,
                        joint limits, action scale / clip, control period, history length
    policy_fp32.h       float32 weights + input normaliser (flash-resident, ~1.1 MB)
    policy_int8.h       int8 weights with one float scale per output row (~0.27 MB, fits the 520 KB SRAM)
    golden_mlp.bin      test vectors: int32 N, then N x (210 obs, 10 fp32 action, 10 int8 action) float32
    student_int8.pt     checkpoint with the de-quantised int8 weights (closed-loop check in simulation)
    meta.json           sources, SHA256, sizes, quantisation error

Why int8: RP2350 has 520 KB SRAM; float32 weights (1.09 MB) must stay in flash, and streaming them over QSPI each
control step would take about 30 ms (> the 20 ms period). int8 weights fit in SRAM. Whether int8 keeps the gait is
checked in simulation with ``student_int8.pt`` (see deploy/export/README.md).

Usage (robot v2):
    set CUSTOM_BIPED_VERSION=2
    tools\isaacpy.cmd deploy\export\export_policy.py
    tools\isaacpy.cmd deploy\export\export_policy.py --student <student.pt> --calib deploy\calibration\robot_v2.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "control")]

import torch  # noqa: E402
import yaml  # noqa: E402

from core import interface as itf  # noqa: E402
from core import interface_v2 as itf2  # noqa: E402
from core.robots import custom_biped as rb  # noqa: E402
from flat_pipeline.config import ActorCfg  # noqa: E402
from flat_pipeline.policy import actor_mean, load_actor  # noqa: E402

DEFAULT_STUDENT = "outputs/robot_v2/control/sim2real/student_v2ft_20261118_230657/student.pt"
NORM_EPS = 1e-2  # rsl_rl EmpiricalNormalization: (x - mean) / (std + eps)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def c_floats(name: str, t: torch.Tensor, qual: str = "static const") -> str:
    v = t.detach().cpu().float().flatten().tolist()
    rows = [",".join(f"{x:.9e}f" for x in v[i:i + 8]) for i in range(0, len(v), 8)]
    return f"{qual} float {name}[{len(v)}] = {{\n" + ",\n".join(rows) + "\n};\n"


def c_int8(name: str, t: torch.Tensor, qual: str) -> str:
    v = t.detach().cpu().to(torch.int8).flatten().tolist()
    rows = [",".join(str(x) for x in v[i:i + 32]) for i in range(0, len(v), 32)]
    return f"{qual} int8_t {name}[{len(v)}] = {{\n" + ",\n".join(rows) + "\n};\n"


def quantise(w: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """Symmetric per-output-row int8: w ~= q * s, q in [-127, 127]."""
    s = w.abs().amax(dim=1).clamp_min(1e-12) / 127.0
    q = torch.round(w / s.unsqueeze(1)).clamp(-127, 127)
    return q, s


def mlp_int8(x: torch.Tensor, layers: list, mean: torch.Tensor, inv_std: torch.Tensor) -> torch.Tensor:
    """Reference of the firmware's int8 path (float accumulation of q * x, then * scale + bias)."""
    h = (x - mean) * inv_std
    for i, (q, s, b) in enumerate(layers):
        h = (h @ q.T) * s + b
        if i < len(layers) - 1:
            h = torch.nn.functional.elu(h)
    return h


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--student", default=DEFAULT_STUDENT)
    p.add_argument("--calib", default=f"deploy/calibration/robot_v{rb.ROBOT_VERSION}.json")
    p.add_argument("--samples", type=int, default=256, help="random test vectors (normal around the normaliser stats)")
    a = p.parse_args()

    student = (ROOT / a.student).resolve()
    calib_path = (ROOT / a.calib).resolve()
    calib = json.loads(calib_path.read_text(encoding="utf-8"))
    run = student.parent.name
    out = ROOT / "outputs" / f"robot_v{rb.ROBOT_VERSION}" / "deploy" / f"export_{run}_{datetime.now():%Y%m%d_%H%M%S}"
    out.mkdir(parents=True, exist_ok=False)

    # ---------------------------------------------------------------- robot config
    names = list(rb.ACTION_JOINT_NAMES)
    J = len(names)
    lay = itf2.LayoutV2(J)
    by_joint = {j["joint"]: j for j in calib["joints"]}
    assert sorted(by_joint) == sorted(names), f"calibration joints {sorted(by_joint)} != {sorted(names)}"
    ids = [int(by_joint[n]["id"]) for n in names]
    assert sorted(ids) == list(range(1, J + 1)), f"servo IDs must be 1..{J}: {ids}"
    lim = {**{f"{s}_{r}": v for s in ("left", "right") for r, v in rb.LEG_JOINT_LIMITS.items()}, **rb.TAIL_JOINT_LIMITS}
    rng = {n: ([math.radians(x) for x in by_joint[n]["range_check_deg"]] if "range_check_deg" in by_joint[n] else lim[n])
           for n in lim}
    default = [rb.DEFAULT_JOINT_POS[n] for n in names]
    imu = calib["imu"]["sensor_to_body"]
    vel_src = {"finite_difference": 0, "servo_speed": 1}[calib["joint_velocity_source"]]
    # command ranges the student was trained on (config.yaml next to student.pt); the firmware rejects WALK / SET
    # outside them. vx may also be 0 (training samples zero-speed commands).
    tcfg = yaml.safe_load((student.parent / "config.yaml").read_text(encoding="utf-8"))
    vx_rng = [float(x) for x in tcfg["env"]["cmd_vx_range"]]
    fq_rng = [float(x) for x in tcfg["env"]["gait_freq_range"]]
    clr_rng = [float(x) for x in tcfg["gait_var"]["clearance_range"]]

    def arr(vals, fmt):
        return "{" + ", ".join(fmt(v) for v in vals) + "}"

    cfg_h = [
        "/* Generated by deploy/export/export_policy.py -- do not edit. */",
        "#pragma once", "#include <stdint.h>",
        f"#define ROBOT_VERSION {rb.ROBOT_VERSION}",
        f"#define NUM_JOINTS {J}",
        f"#define OBS_HISTORY {lay.history}",
        f"#define OBS_FRAME_DIM {lay.frame_dim}",
        f"#define OBS_DIM {lay.student_dim}",
        f"#define CONTROL_DT_S {itf.CONTROL_DT:.6f}f",
        f"#define ACTION_SCALE {itf.ACTION_SCALE:.6f}f",
        f"#define ACTION_CLIP {itf.ACTION_CLIP:.6f}f",
        f"#define CALIB_VERIFIED {1 if calib['verified'] else 0}",
        f"#define IMU_MAP_VERIFIED {1 if calib['imu']['verified'] else 0}",
        f"#define JOINT_VEL_SOURCE {vel_src} /* 0 finite difference, 1 servo speed register */",
        f"#define POLICY_NAME \"{run}\"",
        f"#define CMD_VX_MIN {vx_rng[0]:.4f}f /* training range; 0 is also accepted */",
        f"#define CMD_VX_MAX {vx_rng[1]:.4f}f",
        f"#define GAIT_FREQ_MIN {fq_rng[0]:.4f}f",
        f"#define GAIT_FREQ_MAX {fq_rng[1]:.4f}f",
        f"#define CLEARANCE_MAX {clr_rng[1]:.4f}f /* 0 = default clearance */",
        "static const char *const JOINT_NAMES[NUM_JOINTS] = " + arr(names, lambda s: f'"{s}"') + ";",
        "static const uint8_t SERVO_ID[NUM_JOINTS] = " + arr(ids, str) + ";",
        "static const int8_t SERVO_SIGN[NUM_JOINTS] = " + arr([int(by_joint[n]["sign"]) for n in names], str) + ";",
        "static const int32_t SERVO_ZERO[NUM_JOINTS] = " + arr([int(by_joint[n]["zero_count"]) for n in names], str) + ";",
        "static const float DEFAULT_POS[NUM_JOINTS] = " + arr(default, lambda v: f"{v:.9e}f") + ";",
        "static const float JOINT_LOWER[NUM_JOINTS] = " + arr([lim[n][0] for n in names], lambda v: f"{v:.9e}f") + ";",
        "static const float JOINT_UPPER[NUM_JOINTS] = " + arr([lim[n][1] for n in names], lambda v: f"{v:.9e}f") + ";",
        "/* safety range checks (turn alignment, INIT, runtime fault, JOG); = training limits unless the calibration gives",
        "   range_check_deg (2026-11-23: tail, which has no collision risk). Targets are still clamped to JOINT_LOWER/UPPER. */",
        "static const float RANGE_LOWER[NUM_JOINTS] = " + arr([rng[n][0] for n in names], lambda v: f"{v:.9e}f") + ";",
        "static const float RANGE_UPPER[NUM_JOINTS] = " + arr([rng[n][1] for n in names], lambda v: f"{v:.9e}f") + ";",
        "static const float IMU_SENSOR_TO_BODY[3][3] = {" + ", ".join(arr(r, lambda v: f"{float(v):.1f}f") for r in imu) + "};",
        "",
    ]
    (out / "robot_config.h").write_text("\n".join(cfg_h), encoding="ascii")

    # ---------------------------------------------------------------- policy
    actor = load_actor(student, ActorCfg(), "cpu")
    sd = actor.state_dict()
    wk = sorted((k for k in sd if k.startswith("mlp.") and k.endswith(".weight")), key=lambda k: int(k.split(".")[1]))
    dims = [sd[wk[0]].shape[1]] + [sd[k].shape[0] for k in wk]
    assert dims[0] == lay.student_dim and dims[-1] == J, dims
    assert actor.obs_normalization
    mean = sd["obs_normalizer._mean"].flatten()
    std = sd["obs_normalizer._std"].flatten()
    inv_std = 1.0 / (std + NORM_EPS)
    Ws = [sd[k] for k in wk]
    Bs = [sd[k.replace("weight", "bias")] for k in wk]

    def dims_h(prefix):
        lines = [f"#define {prefix}_LAYERS {len(Ws)}"]
        lines += [f"#define {prefix}_D{i} {d}" for i, d in enumerate(dims)]
        return lines

    h = ["/* Generated by deploy/export/export_policy.py -- do not edit. float32 weights (flash). */",
         "#pragma once", f"/* source {student.relative_to(ROOT).as_posix()} sha256 {sha256(student)} */", *dims_h("POL"),
         c_floats("POL_MEAN", mean), c_floats("POL_INV_STD", inv_std)]
    for i, (w, b) in enumerate(zip(Ws, Bs)):
        h += [c_floats(f"POL_W{i}", w), c_floats(f"POL_B{i}", b)]
    (out / "policy_fp32.h").write_text("\n".join(h), encoding="ascii")

    qlayers = []
    sram = "static POL_SRAM"
    hq = ["/* Generated by deploy/export/export_policy.py -- do not edit. int8 weights, one scale per output row.",
          "   POL_SRAM (deploy/firmware/src/platform.h) places them in .data, which the RP2350 start-up code copies",
          "   into SRAM; without it GCC turns the never-written arrays into flash constants. */",
          "#pragma once", "#include <stdint.h>", "#ifndef POL_SRAM", "#define POL_SRAM", "#endif",
          f"/* source {student.relative_to(ROOT).as_posix()} sha256 {sha256(student)} */", *dims_h("POL"),
          c_floats("POL_MEAN", mean, sram), c_floats("POL_INV_STD", inv_std, sram)]
    qerr = []
    for i, (w, b) in enumerate(zip(Ws, Bs)):
        q, s = quantise(w)
        qlayers.append((q, s, b))
        qerr.append(float(((q * s.unsqueeze(1)) - w).abs().max()))
        hq += [c_int8(f"POL_Q{i}", q, sram), c_floats(f"POL_S{i}", s, sram), c_floats(f"POL_B{i}", b, sram)]
    (out / "policy_int8.h").write_text("\n".join(hq), encoding="ascii")

    # de-quantised checkpoint for the closed-loop simulation check
    ck = torch.load(student, map_location="cpu", weights_only=False)
    sdq = dict(ck["actor_state_dict"])
    for k, (q, s, _) in zip(wk, qlayers):
        sdq[k] = (q * s.unsqueeze(1)).to(sdq[k].dtype)
    torch.save({**ck, "actor_state_dict": sdq}, out / "student_int8.pt")

    # ---------------------------------------------------------------- golden vectors
    g = torch.Generator().manual_seed(0)
    obs = mean + std * torch.randn(a.samples, dims[0], generator=g)
    obs[0] = mean  # exactly the normaliser mean
    with torch.no_grad():
        act32 = actor_mean(actor, obs)
        act8 = mlp_int8(obs, qlayers, mean, inv_std)
        ref32 = mlp_int8(obs, [(w, torch.ones(w.shape[0]), b) for w, b in zip(Ws, Bs)], mean, inv_std)
    assert torch.allclose(ref32, act32, atol=1e-4), "normaliser / ELU reference does not match the rsl_rl actor"
    # on-board self-test (BENCH command): action for obs = normaliser mean
    for name, act in (("policy_fp32.h", act32), ("policy_int8.h", act8)):
        with (out / name).open("a", encoding="ascii") as f:
            f.write("/* action for obs = POL_MEAN (firmware BENCH self-test) */\n" + c_floats("POL_TEST_ACT", act[0]))
    with (out / "golden_mlp.bin").open("wb") as f:
        f.write(struct.pack("<i", a.samples))
        f.write(torch.cat((obs, act32, act8), dim=1).float().numpy().astype("<f4").tobytes())

    params = sum(w.numel() for w in Ws)
    meta = {
        "student": student.relative_to(ROOT).as_posix(), "student_sha256": sha256(student),
        "calibration": calib_path.relative_to(ROOT).as_posix(), "calibration_sha256": sha256(calib_path),
        "calibration_verified": bool(calib["verified"]), "robot_version": rb.ROBOT_VERSION,
        "cmd_vx_range": vx_rng, "gait_freq_range": fq_rng, "clearance_range": clr_rng,
        "dims": dims, "activation": "elu", "normaliser": f"(x - mean) / (std + {NORM_EPS})",
        "joint_order": names, "servo_ids": ids, "default_pos_rad": default,
        "weights_fp32_bytes": params * 4, "weights_int8_bytes": params + 4 * sum(dims[1:]),
        "int8_max_weight_error": qerr,
        "int8_vs_fp32_action_abs_err": {"max": float((act8 - act32).abs().max()),
                                        "mean": float((act8 - act32).abs().mean())},
        "int8_vs_fp32_target_err_deg_max": math.degrees(float((act8 - act32).abs().max()) * itf.ACTION_SCALE),
        "samples": a.samples, "created": datetime.now().isoformat(timespec="seconds"),
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(json.dumps({k: meta[k] for k in ("dims", "weights_fp32_bytes", "weights_int8_bytes",
                                            "int8_vs_fp32_action_abs_err", "int8_vs_fp32_target_err_deg_max")}, indent=1))
    print(f"[export] {out}")


if __name__ == "__main__":
    main()
