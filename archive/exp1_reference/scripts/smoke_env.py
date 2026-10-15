"""Smoke test of the shared env + interface + metrics + video HUD (phase B of archive/exp1_reference).

Runs the env with zero actions (robot holds the default pose) and checks:
  * observation / action dimensions match shared/interface.py, no NaN/Inf
  * standing robots do not fall; foot contact forces carry the robot weight
  * episodes time out and produce metric summaries
  * throughput (env steps per second)
  * with --video: an mp4 with the HUD, plus 3 extracted frames for inspection

Usage:
    tools\\isaacpy.cmd archive\exp1_reference\\scripts\\smoke_env.py --num_envs 1024
    tools\\isaacpy.cmd archive\exp1_reference\\scripts\\smoke_env.py --num_envs 4 --video --enable_cameras
"""

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--num_envs", type=int, default=256)
parser.add_argument("--episode_s", type=float, default=4.0)
parser.add_argument("--episodes", type=int, default=2, help="Number of episode lengths to simulate.")
parser.add_argument("--video", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import cv2  # noqa: E402
import torch  # noqa: E402

from shared import interface as itf  # noqa: E402
from shared.env import BipedWalkEnv, BipedWalkEnvCfg  # noqa: E402
from shared.video_overlay import EvalVideoRecorder, HudState  # noqa: E402


def main():
    cfg = BipedWalkEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.episode_length_s = args_cli.episode_s
    cfg.record_episodes = True
    cfg.eval_camera = args_cli.video
    env = BipedWalkEnv(cfg)
    out_dir = env.rb.TEST_ROOT / "outputs" / "smoke" / time.strftime("%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)

    obs, _ = env.reset()
    obs = obs["policy"]
    layout = itf.ObsLayout(env.num_legs)
    report = {
        "num_envs": env.num_envs,
        "obs_dim": int(obs.shape[1]),
        "obs_dim_expected": layout.dim,
        "action_dim": env.num_legs,
        "control_dt": env.step_dt,
        "leg_joints": list(env.rb.LEG_JOINT_NAMES),
    }
    actions = torch.zeros(env.num_envs, env.num_legs, device=env.device)
    rec = EvalVideoRecorder(out_dir, exp="exp1", tag="smoke") if args_cli.video else None
    traj: list[tuple[float, float]] = []

    n_steps = int(args_cli.episodes * args_cli.episode_s / env.step_dt)
    bad = 0
    force_ratio = []
    weight = float(env.robot.data.body_mass.torch[0].sum()) * 9.81
    t0 = time.time()
    for i in range(n_steps):
        obs, rew, term, trunc, _ = env.step(actions)
        obs = obs["policy"]
        bad += int((~torch.isfinite(obs)).any().item())
        if i == 50:
            f = env.foot_forces().norm(dim=-1).sum(-1)
            force_ratio = (f / weight).tolist()
        if rec is not None:
            t_sim = float(env.episode_length_buf[0]) * env.step_dt
            tr = env.tracker
            if env.episode_length_buf[0] <= 1:
                traj.clear()
            traj.append((float(tr.distance[0]), float(tr.lateral[0])))
            if rec.wants_frame(i * env.step_dt):
                hud = HudState(
                    label="exp1 / smoke", sim_time=t_sim, steps=int(tr.steps[0]), distance=float(tr.distance[0]),
                    lateral=float(tr.lateral[0]), vx=float(env.base_lin_vel[0, 0]),
                    status="FALLEN" if bool(term[0]) else "WALKING", trajectory=list(traj),
                )
                rec.add_frame(env.capture_frame(0), hud)
    elapsed = time.time() - t0

    eps = env.completed_episodes
    cat = {k: torch.cat([e[k] for e in eps]) for k in eps[0]} if eps else {}
    report.update({
        "nan_or_inf_steps": bad,
        "env_steps_per_s": round(n_steps * env.num_envs / elapsed, 1),
        "robot_weight_N": round(weight, 1),
        "foot_force_over_weight_mean": round(sum(force_ratio) / max(1, len(force_ratio)), 3),
        "episodes_completed": int(cat["fell"].numel()) if cat else 0,
        "fall_rate": round(float(cat["fell"].mean()), 3) if cat else None,
        "mean_steps": round(float(cat["steps"].mean()), 2) if cat else None,
        "mean_abs_distance_m": round(float(cat["distance"].abs().mean()), 4) if cat else None,
        "max_torque_Nm_per_joint": [round(v, 2) for v in cat["torque_abs_max"].max(0).values.tolist()] if cat else None,
    })
    if rec is not None:
        path = rec.close()
        report["video"] = str(path)
        cap = cv2.VideoCapture(str(path))
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        report["video_frames"] = n
        report["video_size"] = [int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))]
        report["video_fps"] = cap.get(cv2.CAP_PROP_FPS)
        for k, idx in enumerate((0, n // 2, n - 1)):
            cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
            ok, frame = cap.read()
            if ok:
                cv2.imwrite(str(out_dir / f"frame_{k}.png"), frame)
        cap.release()
    (out_dir / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"[smoke_env] report: {out_dir / 'report.json'}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
