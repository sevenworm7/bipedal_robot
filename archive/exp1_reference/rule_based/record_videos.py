"""Evaluation videos of the rule-based controller (development step 6), in the ``/eval-video`` format.

Runs ``--num_envs`` robots side by side (slow / medium / fast commands, the remaining envs random) and
keeps one video per speed class plus one failure case; the recording logic is shared with the training
pipeline (``shared/eval_videos.py``).

Usage:
    tools\\isaacpy.cmd archive\exp1_reference\\rule_based\\record_videos.py --params <best_params.json> --enable_cameras
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
from shared.eval_videos import record_class_videos  # noqa: E402


def main():
    torch.manual_seed(args_cli.seed)
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
    out_dir = env.rb.TEST_ROOT / "outputs" / "rule_based" / f"videos_{args_cli.tag}_{time.strftime('%Y%m%d_%H%M%S')}"
    res = record_class_videos(env, ctrl, out_dir, exp="exp1", label="exp1 / rule_based", tag=args_cli.tag,
                              speeds=[float(s) for s in args_cli.speeds.split(",")], freq=args_cli.freq)
    for r in res["episodes"]:
        print(f"[videos] env {r['env']:2d} {r['class']:6s} cmd {r['cmd_vx']:.3f} steps {r['steps']:.0f} "
              f"dist {r['distance']:.2f} fell {r['fell']:.0f} success {r['success']}")
    print(f"[videos] kept: {res['kept']}")
    print(f"[videos] output: {out_dir}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
