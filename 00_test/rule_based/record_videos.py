"""Evaluation videos of the rule-based controller (development step 6), in the ``/eval-video`` format.

Runs ``--num_envs`` robots side by side (slow / medium / fast commands, the remaining envs random) and
records one follow-camera stream per robot with the HUD. Afterwards it keeps one video per speed class
(the successful episode closest to the class's command) plus one failure case if any robot fell, and
deletes the rest (without falls, an unsuccessful episode is kept as the failure case). Episodes are not restarted: recording of a robot stops when its first episode ends.

Usage:
    tools\\isaacpy.cmd 00_test\\rule_based\\record_videos.py --params <best_params.json> --enable_cameras
"""

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--tag", default="rule_based")
parser.add_argument("--params", type=str, default=None)
parser.add_argument("--num_envs", type=int, default=12)
parser.add_argument("--episode_s", type=float, default=20.0)
parser.add_argument("--speeds", type=str, default="0.07,0.125,0.18", help="Slow, medium, fast cmd_vx [m/s].")
parser.add_argument("--freq", type=float, default=1.0, help="Gait frequency of the recorded robots [Hz].")
parser.add_argument("--seed", type=int, default=7)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import torch  # noqa: E402

from rule_based import RuleBasedBipedController, RuleBasedCfg  # noqa: E402
from shared.env import BipedWalkEnv, BipedWalkEnvCfg  # noqa: E402
from shared.metrics import rule_based_success  # noqa: E402
from shared.video_overlay import EvalVideoRecorder, HudState  # noqa: E402


def main():
    torch.manual_seed(args_cli.seed)
    speeds = [float(s) for s in args_cli.speeds.split(",")]
    env_cfg = BipedWalkEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.episode_length_s = args_cli.episode_s
    env_cfg.record_episodes = True
    env_cfg.eval_camera = True
    env = BipedWalkEnv(env_cfg)
    cfg = RuleBasedCfg()
    if args_cli.params:
        for k, v in json.loads(Path(args_cli.params).read_text(encoding="utf-8")).items():
            setattr(cfg.gait, k, float(v))
    ctrl = RuleBasedBipedController(env.rb, env.num_envs, env.device, cfg)
    n = env.num_envs
    per = max(1, n // (len(speeds) + 1))
    cls = [i // per if i // per < len(speeds) else -1 for i in range(n)]  # -1 = random command

    obs = env.reset()[0]["policy"]
    for i, c in enumerate(cls):
        if c >= 0:
            env.cmd_vx[i] = speeds[c]
            env.gait_freq[i] = args_cli.freq
    obs = env._get_observations()["policy"]  # refresh the command / clock in the observation
    ctrl.reset()

    out_dir = env.rb.TEST_ROOT / "outputs" / "rule_based" / f"videos_{args_cli.tag}_{time.strftime('%Y%m%d_%H%M%S')}"
    out_dir.mkdir(parents=True, exist_ok=True)
    date = time.strftime("%Y%m%d")
    vid_dir = out_dir / "videos"
    recs = [EvalVideoRecorder(vid_dir / "all", exp="exp1", tag=f"{args_cli.tag}_env{i:02d}") for i in range(n)]
    trajs = [[] for _ in range(n)]
    last_frame = [None] * n  # the env auto-resets on done, so the final hold reuses the last live frame
    active = torch.ones(n, dtype=torch.bool)
    summaries: dict[int, dict] = {}
    labels = {-1: "random"} | {i: name for i, name in enumerate(("slow", "medium", "fast"))}
    for step in range(int(env.max_episode_length) + 2):
        act = ctrl.compute_action(obs)
        o, _, term, trunc, _ = env.step(act)
        obs = o["policy"]
        done = (term | trunc).cpu()
        for ep in env.completed_episodes:
            for j, eid in enumerate(ep["env_ids"].tolist()):
                if eid not in summaries:
                    summaries[eid] = {k: v[j].item() if v[j].numel() == 1 else v[j].tolist() for k, v in ep.items()}
        env.completed_episodes.clear()
        t = (step + 1) * env.step_dt
        for i in range(n):
            if not active[i]:
                continue
            tr = env.tracker
            if bool(done[i]):
                s = summaries.get(i, {})
                hud_steps, hud_dist, hud_lat = s.get("steps", 0), s.get("distance", 0.0), s.get("lateral", 0.0)
                status = "FALLEN" if bool(term[i]) else "FINISHED"
            else:
                hud_steps, hud_dist, hud_lat = float(tr.steps[i]), float(tr.distance[i]), float(tr.lateral[i])
                status = "WALKING"
            trajs[i].append((hud_dist, hud_lat))
            if recs[i].wants_frame(t) or status != "WALKING":
                label = f"exp1 / rule_based / {labels[cls[i]]} cmd {float(env.cmd_vx[i]) if status == 'WALKING' else s.get('cmd_vx', 0):.3f} m/s"
                hud = HudState(label=label, sim_time=t, steps=int(hud_steps), distance=hud_dist, lateral=hud_lat,
                               vx=float(env.base_lin_vel[i, 0]), status=status, trajectory=list(trajs[i]))
                if status == "WALKING" or last_frame[i] is None:
                    last_frame[i] = env.capture_frame(i)
                frame = last_frame[i]
                recs[i].add_frame(frame, hud)
                if status != "WALKING":
                    for _ in range(30):  # hold the last frame for 1 s
                        recs[i].add_frame(frame, hud)
            if bool(done[i]):
                active[i] = False
        if not bool(active.any()):
            break

    paths = [r.close() for r in recs]
    # choose the videos to keep
    keep, report = {}, []
    for i in range(n):
        s = summaries.get(i)
        if s is None:
            continue
        ep = {k: torch.tensor([v]) for k, v in s.items() if not isinstance(v, list)}
        ok = bool(rule_based_success(ep)[0])
        report.append({"env": i, "class": labels[cls[i]], "success": ok, **{k: s[k] for k in
                       ("cmd_vx", "steps", "distance", "lateral", "max_abs_lateral", "time", "fell", "mean_vx")}})
        if cls[i] >= 0 and ok and labels[cls[i]] not in keep:
            keep[labels[cls[i]]] = i
        if s["fell"] > 0.5 and "failure" not in keep:
            keep["failure"] = i
    if "failure" not in keep:  # no fall: keep an unsuccessful (e.g. too short / too slow) episode instead
        bad = [r["env"] for r in report if not r["success"]]
        if bad:
            keep["failure"] = bad[0]
    final = {}
    for k, (name, i) in enumerate(keep.items()):
        dst = vid_dir / f"exp1_rule_based_{name}_eval_{date}.mp4"
        Path(paths[i]).replace(dst)
        final[name] = {"env": i, "video": str(dst)}
    for p in (vid_dir / "all").glob("*.mp4"):
        p.unlink()
    (vid_dir / "all").rmdir()
    (out_dir / "videos.json").write_text(json.dumps({"kept": final, "episodes": report}, indent=2), encoding="utf-8")
    for r in report:
        print(f"[videos] env {r['env']:2d} {r['class']:6s} cmd {r['cmd_vx']:.3f} steps {r['steps']:.0f} "
              f"dist {r['distance']:.2f} fell {r['fell']:.0f} success {r['success']}")
    print(f"[videos] kept: {final}")
    print(f"[videos] output: {out_dir}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
