"""Fixed-set evaluation (and optional videos) of any agent: teacher, BC, PPO or distilled BC.

Writes ``iter<N>_eval_<stage>_<time>/{stats.json, episodes.pt}`` (same schema as ``rule_based/evaluate.py``).
Videos (``/eval-video`` format) are recorded by a separate command with ``--video_only --enable_cameras``
into ``iter<N>_videos_<stage>_<time>/videos/`` (small env with a follow camera per robot).

Usage:
    tools\\isaacpy.cmd 03_training_pipeline\\training_pipeline\\evaluate.py --iter 1 --stage ppo --agent policy:<model_1499.pt>
    tools\\isaacpy.cmd 03_training_pipeline\\training_pipeline\\evaluate.py --iter 1 --stage ppo --agent policy:<ckpt> --video_only --enable_cameras
"""

import argparse
import sys
from pathlib import Path

# project root (shared), 02_rule_based (rule_based teacher), 03_training_pipeline (training_pipeline)
_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_ROOT), str(_ROOT / "02_rule_based"), str(_ROOT / "03_training_pipeline")]

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--iter", type=int, required=True)
parser.add_argument("--stage", type=str, required=True, help="Label: teacher, bc, ppo, distill.")
parser.add_argument("--agent", type=str, required=True, help="teacher[:params.json] or policy:<ckpt>.")
parser.add_argument("--no_push", action="store_true")
parser.add_argument("--video_only", action="store_true", help="Only record the evaluation videos.")
parser.add_argument("--video_envs", type=int, default=16)
parser.add_argument("--speeds", type=str, default=None, help="Default: EvalCfg.video_speeds.")
parser.add_argument("--freq", type=float, default=None, help="Default: EvalCfg.video_freq.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import torch  # noqa: E402

from shared.eval_videos import record_class_videos  # noqa: E402
from training_pipeline.common import dump_config, make_env, run_dir, write_json  # noqa: E402
from training_pipeline.config import PipelineCfg  # noqa: E402
from training_pipeline.evaluation import evaluate_agent, summary_line  # noqa: E402
from training_pipeline.policy import load_agent  # noqa: E402


def main():
    cfg = PipelineCfg()
    e = cfg.eval
    kind = "videos" if args_cli.video_only else "eval"
    out = run_dir(cfg, args_cli.iter, f"{kind}_{args_cli.stage}")
    if args_cli.video_only:
        torch.manual_seed(e.seed)
        env = make_env(cfg, args_cli.video_envs, e.episode_s, e.seed, eval_camera=True)
        agent = load_agent(args_cli.agent, env, cfg.actor, env.rb.PROJECT_ROOT)
        dump_config(out, cfg, vars(args_cli), env.cfg)
        speeds = [float(s) for s in args_cli.speeds.split(",")] if args_cli.speeds else list(e.video_speeds)
        label = f"{cfg.exp} / robot_v{env.rb.ROBOT_VERSION} / iter{args_cli.iter} / {args_cli.stage}"
        res = record_class_videos(env, agent, out, exp=cfg.exp, label=label, tag=f"iter{args_cli.iter}_{args_cli.stage}",
                                  speeds=speeds, freq=args_cli.freq or e.video_freq)
        for r in res["episodes"]:
            print(f"[videos] env {r['env']:2d} {r['class']:6s} cmd {r['cmd_vx']:.3f} steps {r['steps']:.0f} "
                  f"dist {r['distance']:.2f} fell {r['fell']:.0f} success {r['success']}")
        print(f"[videos] kept: {list(res['kept'])}")
    else:
        env = make_env(cfg, e.episodes, e.episode_s, e.seed)
        agent = load_agent(args_cli.agent, env, cfg.actor, env.rb.PROJECT_ROOT)
        dump_config(out, cfg, vars(args_cli), env.cfg)
        stats, ep = evaluate_agent(env, agent, e, push=not args_cli.no_push)
        stats = {"agent": args_cli.agent, "iteration": args_cli.iter, "stage": args_cli.stage, **stats}
        write_json(out / "stats.json", stats)
        torch.save({k: v.cpu() for k, v in ep.items() if k != "torque_samples"}, out / "episodes.pt")
        print(f"[eval] iter {args_cli.iter} {args_cli.stage}: {summary_line(stats)}")
    print(f"[eval] output: {out}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
