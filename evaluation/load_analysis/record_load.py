r"""Record the servo load of one case (``cases.py``): applied torque, joint speed and foot contact of every robot for
its first episode (``--mode stats``), or a 10 s follow-camera clip of one robot with a live torque panel
(``--mode video``). One Isaac Lab process per call; ``run_load.py`` runs all cases and writes the report.

Usage:
    tools\isaacpy.cmd evaluation\load_analysis\record_load.py --case 04_flat_medium --mode stats --out_dir <dir>
    tools\isaacpy.cmd evaluation\load_analysis\record_load.py --case 04_flat_medium --mode video --out_dir <dir> --enable_cameras
"""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "control"), str(Path(__file__).resolve().parent)]

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--case", required=True)
parser.add_argument("--mode", required=True, choices=("stats", "video"))
parser.add_argument("--out_dir", required=True)
parser.add_argument("--agent", default="student:outputs/robot_v1/control/sim2real/student_v5_20261007_003835/student.pt")
parser.add_argument("--num_envs", type=int, default=128)
parser.add_argument("--episode_s", type=float, default=20.0)
parser.add_argument("--seed", type=int, default=42)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import numpy as np  # noqa: E402
import torch  # noqa: E402

from cases import CASE_SETS, SERVO_6V  # noqa: E402
CASES_ALL = {k: v for s in CASE_SETS.values() for k, v in s.items()}  # every case id is unique across sets
from flat_pipeline.config import PipelineCfg  # noqa: E402
from sim2real.agents import load_agent_v2  # noqa: E402
from sim2real.common import dump_config, git_commit, make_env  # noqa: E402
from simulation.video_overlay import EvalVideoRecorder, HudState  # noqa: E402

SHORT = {"left_hip_roll": "L hip roll", "left_hip_pitch": "L hip pitch", "left_knee": "L knee",
         "left_ankle_pitch": "L ankle", "right_hip_roll": "R hip roll", "right_hip_pitch": "R hip pitch",
         "right_knee": "R knee", "right_ankle_pitch": "R ankle", "tail_yaw": "tail yaw", "tail_pitch": "tail pitch"}


def servo_current(tau_abs: torch.Tensor) -> torch.Tensor:
    """Linear estimate per servo [A]: no-load current + |torque| x (stall - no-load current) / stall torque (6 V)."""
    s = SERVO_6V
    return s["noload_current"] + tau_abs.clamp(max=s["stall_torque"]) * (s["stall_current"] - s["noload_current"]) / s["stall_torque"]


def set_command(env, case):
    env.cmd_vx[:] = case["vx"]
    env.gait_freq[:] = case["freq"]
    return env._get_observations()["policy"]  # refresh command / clock in the observations


def main():
    case = CASES_ALL[args_cli.case]
    out = Path(args_cli.out_dir)
    out.mkdir(parents=True, exist_ok=False)
    video = args_cli.mode == "video"
    n = 4 if video else args_cli.num_envs
    episode_s = 10.0 if video else args_cli.episode_s
    torch.manual_seed(args_cli.seed)
    cfg = PipelineCfg()
    extra = {"push_interval_s": (5.0, 5.0), "push_speed": case["push_speed"]} if "push_speed" in case else {}
    if case["scenario"].startswith(("stress_v2/",)):
        extra["fall_height_ref"] = "feet"
    env = make_env(cfg, case["scenario"], n, episode_s, args_cli.seed, eval_camera=video, obs_version=2, **extra)
    agent = load_agent_v2(args_cli.agent, env, cfg.actor, ROOT)
    dump_config(out, {"args": vars(args_cli), "case": case, "servo_6v": SERVO_6V, "git_commit": git_commit(ROOT)})
    names = list(env.rb.ACTION_JOINT_NAMES)

    env.reset()
    env.completed_episodes.clear()
    obs = set_command(env, case)
    agent.reset()
    active = torch.ones(n, dtype=torch.bool, device=env.device)
    tau_l, vel_l, con_l, alive_l = [], [], [], []
    rec = EvalVideoRecorder(out, exp="exp2", tag=f"load_{args_cli.case}", quality=5) if video else None
    steps = int(round(episode_s / env.step_dt))
    fell = torch.zeros(n, dtype=torch.bool, device=env.device)
    for step in range(steps):
        act = agent.compute_action(obs)
        o, _, term, trunc, _ = env.step(act)
        obs = o["policy"]
        tau = env.leg_torque.clone()
        tau_l.append(tau.cpu().numpy().astype(np.float32))
        vel_l.append(env.leg_vel.cpu().numpy().astype(np.float32))
        con_l.append(env.foot_in_contact().cpu().numpy())
        alive_l.append(active.cpu().numpy())
        done = term | trunc
        fell |= term & active
        if rec is not None and bool(active[0]) and not bool(trunc[0]):  # the time-out step already shows the reset robot
            t = (step + 1) * env.step_dt
            if rec.wants_frame(t):
                amps = float(servo_current(tau[0].abs()).sum())
                hud = HudState(label=f"exp2 / load / {args_cli.case}", sim_time=t, steps=int(env.tracker.steps[0]),
                               distance=float(env.tracker.distance[0]), lateral=float(env.tracker.lateral[0]),
                               vx=float(env.base_lin_vel[0, 0]), status="FALLEN" if bool(term[0]) else "WALKING",
                               command="STAND" if case["freq"] == 0 else ("IN PLACE" if case["vx"] == 0 else f"{case['vx']:.2f} m/s"),
                               joint_load=[(SHORT.get(nm, nm), float(v)) for nm, v in zip(names, tau[0].tolist())],
                               load_refs=(SERVO_6V["rated_torque"], SERVO_6V["sim_limit"], SERVO_6V["stall_torque"]),
                               load_note=f"total servo current ~ {amps:.1f} A (6 V est.)")
                rec.add_frame(env.capture_frame(0), hud)
        active &= ~done
        if not bool(active.any()):
            break
    if rec is not None:
        print(f"[load] video: {rec.close()}")
        env.close()
        return
    np.savez_compressed(out / "load.npz", torque=np.stack(tau_l), joint_vel=np.stack(vel_l),
                        contact=np.stack(con_l), alive=np.stack(alive_l), step_dt=env.step_dt,
                        joint_names=np.array(names))
    summary = {"case": args_cli.case, "num_envs": n, "steps": len(tau_l), "fall_rate": float(fell.float().mean())}
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"[load] {args_cli.case}: {len(tau_l)} steps x {n} robots, falls {summary['fall_rate']:.0%} -> {out}")
    env.close()


if __name__ == "__main__":
    main()
