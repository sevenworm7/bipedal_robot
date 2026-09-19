"""Step 1 of the rule-based plan: offline check of the leg IK and of the controller's joint targets.

No simulator is started. Checks:
  1. FK(IK(x, z)) round trip over the reachable workspace, and the default pose.
  2. The controller over one full gait cycle for a grid of (cmd_vx, gait frequency) with an upright,
     static base: joint targets within limits, reach error (requested step/lift not achievable),
     sole kept parallel to the ground.
Writes ``00_test/outputs/rule_based/check_ik_<date>/report.json`` and ``trajectories.png``.

Usage:
    tools\\isaacpy.cmd 00_test\\rule_based\\check_ik.py
"""

import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import torch  # noqa: E402

from rule_based import RuleBasedBipedController, RuleBasedCfg  # noqa: E402
from rule_based.leg_kinematics import SagittalLegIK  # noqa: E402
from shared import interface as itf  # noqa: E402
from shared.robots import get_robot  # noqa: E402


def synthetic_obs(n: int, cmd: torch.Tensor, phase: torch.Tensor, j: int, freq: torch.Tensor) -> torch.Tensor:
    z3 = torch.zeros(n, 3)
    g = torch.tensor([0.0, 0.0, -1.0]).expand(n, 3)
    zj = torch.zeros(n, j)
    return itf.build_observation(
        base_lin_vel=torch.stack((cmd, torch.zeros(n), torch.zeros(n)), -1), base_ang_vel=z3, projected_gravity=g,
        cmd_vx=cmd, gait_freq=freq, heading_error=torch.zeros(n), joint_pos_rel=zj, joint_vel=zj, last_action=zj, gait_phase=phase,
    )


def main():
    rb = get_robot("ref_biped")
    cfg = RuleBasedCfg()
    ik = SagittalLegIK(rb, cfg.knee_margin, cfg.min_knee)
    report: dict = {"thigh": ik.l1, "shin": ik.l2, "knee_range_used": [ik.knee_min, ik.knee_max],
                    "reach_min": ik.r_min, "reach_max": ik.r_max}

    # 1. round trip over the reachable annulus
    r = torch.linspace(ik.r_min, ik.r_max, 50)
    ang = torch.linspace(-0.6, 0.6, 50)
    rr, aa = torch.meshgrid(r, ang, indexing="ij")
    x, z = rr * torch.sin(aa), -rr * torch.cos(aa)
    hip, knee, err = ik.ik(x.flatten(), z.flatten())
    xf, zf = ik.fk(hip, knee)
    report["roundtrip_max_err_m"] = float(torch.hypot(xf - x.flatten(), zf - z.flatten()).max())
    report["roundtrip_reach_err_max_m"] = float(err.max())
    names = list(rb.LEG_JOINT_NAMES)
    q0 = {role: rb.DEFAULT_JOINT_POS[names[i]] * rb.JOINT_AXIS_SIGN[role]
          for i, (side, role) in enumerate(rb.LEG_JOINT_LAYOUT) if side == "left"}
    x0, z0 = ik.fk(torch.tensor([q0["hip_pitch"]]), torch.tensor([q0["knee"]]))
    h0, k0, _ = ik.ik(x0, z0)
    report["default_pose"] = {"ankle_x": float(x0), "ankle_z": float(z0), "hip_ik": float(h0), "knee_ik": float(k0),
                              "hip_ref": q0["hip_pitch"], "knee_ref": q0["knee"]}

    # 2. controller over one gait cycle for a (cmd, freq) grid, ramp already finished
    cmds = torch.tensor([0.0, 0.05, 0.10, 0.15, 0.20])
    freqs = torch.tensor([0.8, 1.0, 1.2])
    cc, ff = torch.meshgrid(cmds, freqs, indexing="ij")
    cc, ff = cc.flatten(), ff.flatten()
    n = cc.numel()
    ctrl = RuleBasedBipedController(rb, n, "cpu", cfg)
    j = len(names)
    steps = int(round(1.0 / (ff.min().item() * itf.CONTROL_DT))) * 4
    t = torch.zeros(n)
    rows = {"q": [], "reach": [], "phase": []}
    for _ in range(steps):
        phase = torch.remainder(ff * t, 1.0)
        a = ctrl.compute_action(synthetic_obs(n, cc, phase, j, ff))
        q = itf.action_to_joint_targets(a, ctrl.q_default)
        rows["q"].append(q)
        rows["reach"].append(ctrl.debug["reach_err"].max(-1).values)
        rows["phase"].append(phase)
        t = t + itf.CONTROL_DT
    q = torch.stack(rows["q"], 1)  # [n, T, J]
    reach = torch.stack(rows["reach"], 1)
    last = int(math.ceil(max(1.0 / f for f in freqs.tolist()) / itf.CONTROL_DT))  # final full cycle only
    q_c, reach_c = q[:, -last:], reach[:, -last:]
    at_limit = ((q_c <= ctrl.q_lo + 1e-5) | (q_c >= ctrl.q_hi - 1e-5)).any(1)  # [n, J]
    # sole pitch relative to the pelvis for each leg (canonical): hip + knee + ankle
    canon = q_c * ctrl.sign
    sole = torch.stack([canon[..., ctrl.index[s, 1:]].sum(-1) for s in range(2)], -1)
    grid = []
    for i in range(n):
        grid.append({
            "cmd_vx": round(float(cc[i]), 3), "freq": round(float(ff[i]), 2),
            "step_length": round(float(cc[i] / (2 * ff[i])), 4),
            "max_reach_err_mm": round(float(reach_c[i].max()) * 1000, 2),
            "joints_at_limit": [names[k] for k in range(j) if at_limit[i, k]],
            "max_abs_sole_pitch_rad": round(float(sole[i].abs().max()), 4),
            "q_min": [round(v, 3) for v in q_c[i].min(0).values.tolist()],
            "q_max": [round(v, 3) for v in q_c[i].max(0).values.tolist()],
        })
    report["gait_grid"] = grid
    report["gait_params"] = cfg.to_dict()

    out = rb.TEST_ROOT / "outputs" / "rule_based" / f"check_ik_{time.strftime('%Y%m%d')}"
    out.mkdir(parents=True, exist_ok=True)
    (out / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    # foot trajectories (left leg) for the fastest command at each frequency
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    li = ctrl.index[0]
    for i in range(n):
        if not any(abs(float(cc[i]) - v) < 1e-4 for v in (0.0, 0.2)):
            continue
        hq, kq = q_c[i, :, li[1]] * ctrl.sign[li[1]], q_c[i, :, li[2]] * ctrl.sign[li[2]]
        xs, zs = ik.fk(hq, kq)
        label = f"cmd {float(cc[i]):.2f} m/s, f {float(ff[i]):.1f} Hz"
        axes[0].plot(xs * 1000, zs * 1000, label=label)
        axes[1].plot(q_c[i, :, li[0]].numpy(), label=label)
    axes[0].set(xlabel="ankle x rel. hip [mm]", ylabel="ankle z rel. hip [mm]", title="left ankle path (1 cycle)")
    axes[0].axis("equal")
    axes[1].set(xlabel="control step", ylabel="left hip roll [rad]", title="left hip roll target")
    axes[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out / "trajectories.png", dpi=110)

    summary = {k: report[k] for k in ("reach_min", "reach_max", "roundtrip_max_err_m", "default_pose")}
    print(json.dumps(summary, indent=2))
    for row in grid:
        print(f"cmd {row['cmd_vx']:.2f} f {row['freq']:.1f} L {row['step_length']:.3f} | reach err "
              f"{row['max_reach_err_mm']:5.1f} mm | sole {row['max_abs_sole_pitch_rad']:.3f} | limits {row['joints_at_limit']}")
    print(f"[check_ik] report: {out}")


if __name__ == "__main__":
    main()
