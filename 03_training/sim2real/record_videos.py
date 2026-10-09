r"""Evaluation videos of any agent in a named scenario (``/eval-video`` format, shared recorder ``shared/eval_videos.py``).

Robots side by side with slow / medium / fast commands (``EvalCfg.video_speeds``), one follow camera each; keeps one
successful video per speed class plus a failure case when there is one. The HUD label is
``exp2 / robot_v<N> / sim2real / <tag> / <scenario>``.

Output: ``outputs/robot_v<N>/sim2real/videos_<scenario>_<tag>_<time>/videos/exp2_sim2real_<tag>_<scenario>_<class>_eval_<date>.mp4``.

Usage:
    tools\isaacpy.cmd 03_training\sim2real\record_videos.py --scenario dr_rough_eval_v1 --agent student:<student.pt> --tag student_v3 --enable_cameras
"""

import argparse
import json
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_ROOT), str(_ROOT / "02_rule_based"), str(_ROOT / "03_training")]

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--scenario", required=True)
parser.add_argument("--agent", required=True)
parser.add_argument("--tag", required=True, help="Label of the policy, e.g. student_v3.")
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--seed", type=int, default=7)
parser.add_argument("--episode_s", type=float, default=None, help="Episode / clip length [s] (default EvalCfg.episode_s = 20).")
parser.add_argument("--quality", type=int, default=8, help="H.264 quality 0-10 (default 8 as in 02 / 03; stress tests use 5).")
parser.add_argument("--speeds", type=str, default=None, help="Slow,medium,fast commands [m/s] (default EvalCfg.video_speeds).")
parser.add_argument("--freq", type=float, default=None, help="Gait frequency of the class robots [Hz] (default EvalCfg.video_freq).")
parser.add_argument("--keep_all", action="store_true", help="Keep the first robot of every class even if it failed (stress tests).")
parser.add_argument("--out_dir", type=str, default=None, help="Exact output folder; must not exist.")
parser.add_argument("--push_speed", type=float, default=None, help="Push every robot every 5 s with this speed [m/s].")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import torch  # noqa: E402

from shared.eval_videos import record_class_videos  # noqa: E402
from sim2real.agents import load_agent_v2, obs_version_for  # noqa: E402
from sim2real.common import dump_config, git_commit, make_env, run_dir  # noqa: E402
from sim2real.scoring import sim2real_success  # noqa: E402
from training_pipeline.config import PipelineCfg  # noqa: E402


def main():
    cfg = PipelineCfg()
    e = cfg.eval
    torch.manual_seed(args_cli.seed)
    obs_version = obs_version_for(args_cli.agent)
    if args_cli.episode_s:
        e.episode_s = args_cli.episode_s
    pushes = {"push_interval_s": (5.0, 5.0), "push_speed": args_cli.push_speed} if args_cli.push_speed else {}
    env = make_env(cfg, args_cli.scenario, args_cli.num_envs, e.episode_s, args_cli.seed, eval_camera=True,
                   obs_version=obs_version, **pushes)
    agent = load_agent_v2(args_cli.agent, env, cfg.actor, env.rb.PROJECT_ROOT)
    safe = args_cli.scenario.replace(":", "-")
    if args_cli.out_dir:
        out = Path(args_cli.out_dir)
        out.mkdir(parents=True, exist_ok=False)
    else:
        out = run_dir(env.rb, f"videos_{safe}", args_cli.tag)
    dump_config(out, {"args": vars(args_cli), "scenario": args_cli.scenario, "obs_version": obs_version,
                      "git_commit": git_commit(env.rb.PROJECT_ROOT)})
    label = f"exp2 / robot_v{env.rb.ROBOT_VERSION} / sim2real / {args_cli.tag} / {args_cli.scenario}"
    res = record_class_videos(env, agent, out, exp="exp2", label=label, tag=f"sim2real_{args_cli.tag}_{safe}",
                              speeds=[float(v) for v in args_cli.speeds.split(",")] if args_cli.speeds else list(e.video_speeds),
                              freq=args_cli.freq or e.video_freq,
                              success_fn=(lambda ep: torch.ones(1, dtype=torch.bool)) if args_cli.keep_all else
                              (lambda ep: sim2real_success(ep, e.episode_s,
                                                           speed_check=not args_cli.scenario.startswith("gait"))["success"]),
                              quality=args_cli.quality)
    for r in res["episodes"]:
        print(f"[videos] env {r['env']:2d} {r['class']:6s} cmd {r['cmd_vx']:.3f} steps {r['steps']:.0f} "
              f"dist {r['distance']:.2f} lat {r['lateral']:+.3f} fell {r['fell']:.0f}")
    print(f"[videos] kept: {json.dumps({k: Path(v['video']).name for k, v in res['kept'].items()})}")
    print(f"[videos] output: {out}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
