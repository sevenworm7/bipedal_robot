"""Time-series diagnostics of the rule-based controller on a few environments.

Logs, for environment 0 (and optional others): clock phase, base roll/pitch, lateral/forward velocity,
foot heights and contact states, and leg joint targets vs. measured angles. Writes ``timeseries.png``
and ``timeseries.npz`` (and ``diagnose.mp4`` with ``--video --enable_cameras``) to
``archive/exp1_reference/outputs/rule_based/diag_<tag>_<timestamp>/``.

Usage:
    tools\\isaacpy.cmd archive\exp1_reference\\rule_based\\diagnose.py --cmd 0 --freq 1 --seconds 6 --set sway_amp=0.2
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--tag", default="diag")
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--seconds", type=float, default=6.0)
parser.add_argument("--cmd", type=float, default=0.0)
parser.add_argument("--freq", type=float, default=1.0)
parser.add_argument("--set", action="append", default=[], help="Gait parameter override name=value.")
parser.add_argument("--video", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import numpy as np  # noqa: E402
import torch  # noqa: E402

from rule_based import RuleBasedBipedController, RuleBasedCfg  # noqa: E402
from shared import interface as itf  # noqa: E402
from shared.env import BipedWalkEnv, BipedWalkEnvCfg  # noqa: E402
from shared.video_overlay import EvalVideoRecorder, HudState  # noqa: E402


def main():
    env_cfg = BipedWalkEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.episode_length_s = args_cli.seconds + 1.0
    env_cfg.cmd_vx_range = (args_cli.cmd, args_cli.cmd)
    env_cfg.gait_freq_range = (args_cli.freq, args_cli.freq)
    env_cfg.init_joint_noise = 0.0
    env_cfg.eval_camera = args_cli.video
    env = BipedWalkEnv(env_cfg)
    cfg = RuleBasedCfg()
    for s in args_cli.set:
        k, v = s.split("=", 1)
        setattr(cfg.gait, k.strip(), float(v))
    ctrl = RuleBasedBipedController(env.rb, env.num_envs, env.device, cfg)
    foot_ids = [env.robot.find_bodies(n)[0][0] for n in env.rb.FOOT_BODY_NAMES]

    out = env.rb.TEST_ROOT / "outputs" / "rule_based" / f"diag_{args_cli.tag}_{time.strftime('%Y%m%d_%H%M%S')}"
    out.mkdir(parents=True, exist_ok=True)
    rec = EvalVideoRecorder(out, exp="exp1", tag=f"diag_{args_cli.tag}") if args_cli.video else None

    obs = env.reset()[0]["policy"]
    ctrl.reset()
    log = {k: [] for k in ("t", "phase", "roll", "pitch", "vx", "vy", "foot_z", "contact", "force", "first", "air", "swing", "q_target", "q", "steps", "lat")}
    traj = []
    n = int(args_cli.seconds / env.step_dt)
    for i in range(n):
        act = ctrl.compute_action(obs)
        o, _, term, _, _ = env.step(act)
        obs = o["policy"]
        d = ctrl.debug
        log["t"].append(i * env.step_dt)
        log["phase"].append(float(d["phase"][0]))
        log["roll"].append(float(d["roll"][0]))
        log["pitch"].append(float(d["pitch"][0]))
        log["vx"].append(float(env.base_lin_vel[0, 0]))
        log["vy"].append(float(env.base_lin_vel[0, 1]))
        log["foot_z"].append(env.robot.data.body_pos_w.torch[0, foot_ids, 2].tolist())
        log["contact"].append(env.foot_in_contact()[0].float().tolist())
        log["force"].append(env.foot_forces()[0, :, 2].tolist())
        log["first"].append(env.first_contact[0].float().tolist())
        log["air"].append(env.foot_last_air_time()[0].tolist())
        log["swing"].append(d["in_swing"][0].float().tolist())
        log["q_target"].append(env.q_target[0].tolist())
        log["q"].append(env.leg_pos[0].tolist())
        log["steps"].append(float(env.tracker.steps[0]))
        log["lat"].append(float(env.tracker.lateral[0]))
        if rec is not None:
            traj.append((float(env.tracker.distance[0]), float(env.tracker.lateral[0])))
            if rec.wants_frame(i * env.step_dt):
                hud = HudState(label=f"exp1 / rule_based / diag {args_cli.tag}", sim_time=i * env.step_dt,
                               steps=int(env.tracker.steps[0]), distance=float(env.tracker.distance[0]),
                               lateral=float(env.tracker.lateral[0]), vx=float(env.base_lin_vel[0, 0]),
                               status="FALLEN" if bool(term[0]) else "WALKING", trajectory=list(traj))
                rec.add_frame(env.capture_frame(0), hud)
        if bool(term[0]):
            print(f"[diagnose] env 0 fell at t = {i * env.step_dt:.2f} s")
            break
    arr = {k: np.asarray(v) for k, v in log.items()}
    np.savez(out / "timeseries.npz", **arr)
    if rec is not None:
        rec.close()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = env.rb.LEG_JOINT_NAMES
    t = arr["t"]
    fig, ax = plt.subplots(6, 1, figsize=(12, 15), sharex=True)
    ax[0].plot(t, arr["phase"], label="clock phase")
    ax[0].plot(t, arr["contact"][:, 0] * 0.9, label="L contact")
    ax[0].plot(t, arr["contact"][:, 1] * 0.8, label="R contact")
    ax[0].fill_between(t, 0, arr["swing"][:, 0] * 0.45, alpha=0.2, label="L swing (ctrl)")
    ax[0].fill_between(t, 0.5, 0.5 + arr["swing"][:, 1] * 0.45, alpha=0.2, label="R swing (ctrl)")
    ax[0].plot(t, arr["steps"] / max(1.0, arr["steps"].max()), label=f"steps (max {arr['steps'].max():.0f})")
    ax[1].plot(t, (arr["foot_z"] - arr["foot_z"][:5].mean(0)) * 1000)
    ax[1].set_ylabel("foot z - initial [mm]")
    ax[1].legend(["left", "right"])
    ax[2].plot(t, np.degrees(arr["roll"]), label="roll (+ = lean right) [deg]")
    ax[2].plot(t, np.degrees(arr["pitch"]), label="pitch (+ = lean fwd) [deg]")
    ax[3].plot(t, arr["force"][:, 0] / 300, label="L Fz / 300 N")
    ax[3].plot(t, arr["force"][:, 1] / 300, label="R Fz / 300 N")
    ax[3].plot(t, arr["vx"], label="vx")
    ax[3].plot(t, arr["vy"], label="vy")
    ax[3].plot(t, arr["lat"], label="lateral offset [m]")
    for j in (0, 4):
        ax[4].plot(t, arr["q_target"][:, j], "--", label=f"{names[j]} target")
        ax[4].plot(t, arr["q"][:, j], label=f"{names[j]}")
    for j in (1, 2, 3):
        ax[5].plot(t, arr["q_target"][:, j], "--", label=f"{names[j]} target")
        ax[5].plot(t, arr["q"][:, j], label=f"{names[j]}")
    for a in ax:
        a.legend(fontsize=7, loc="upper right")
        a.grid(alpha=0.3)
    ax[-1].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(out / "timeseries.png", dpi=90)
    print(f"[diagnose] output: {out}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
