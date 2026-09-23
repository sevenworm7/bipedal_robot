"""Stage 1: collect (state, action) data with an agent and store it as HDF5.

* Teacher data (iteration 0): the rule-based controller, with DART-style Gaussian noise on the executed
  action while the label stays the clean teacher action.
* Distillation rollouts (iteration >= 1): a PPO policy; the executed action is mean + noise_scale * std
  * N(0, 1) and the label is the policy mean (see ``filter.py`` for the selection).

Each env runs complete episodes (``--rounds`` batches of ``--num_envs`` episodes). Output:
``00_test/outputs/training/iter<N>_collect[_<tag>]_<time>/{dataset.h5, stats.json, config.yaml}``.

Usage:
    tools\\isaacpy.cmd 00_test\\training_pipeline\\collect.py --iter 0
    tools\\isaacpy.cmd 00_test\\training_pipeline\\collect.py --iter 1 --agent policy:<model.pt> --noise 0.3 --tag ppo
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--iter", type=int, default=0)
parser.add_argument("--agent", type=str, default=None, help="teacher[:params.json] (default) or policy:<ckpt>.")
parser.add_argument("--num_envs", type=int, default=None)
parser.add_argument("--rounds", type=int, default=1, help="Number of batches of num_envs episodes.")
parser.add_argument("--noise", type=float, default=None,
                    help="Teacher: executed-action noise std; policy: noise scale x policy std.")
parser.add_argument("--tag", type=str, default="")
parser.add_argument("--seed", type=int, default=None)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import torch  # noqa: E402

from training_pipeline.common import dump_config, make_env, run_dir, write_json  # noqa: E402
from training_pipeline.config import PipelineCfg  # noqa: E402
from training_pipeline.dataset import collect, episode_stats, merge, save_h5  # noqa: E402
from training_pipeline.policy import PolicyAgent, load_agent  # noqa: E402


def main():
    cfg = PipelineCfg()
    c = cfg.collect
    seed = args_cli.seed if args_cli.seed is not None else c.seed + args_cli.iter
    torch.manual_seed(seed)
    num_envs = args_cli.num_envs or c.num_envs
    spec = args_cli.agent or f"teacher:{c.teacher_params}"
    env = make_env(cfg, num_envs, c.episode_s, seed)
    agent = load_agent(spec, env, cfg.actor, env.rb.TEST_ROOT)
    is_policy = isinstance(agent, PolicyAgent)
    noise = args_cli.noise if args_cli.noise is not None else (cfg.filter.rollout_noise_scale if is_policy else c.exec_noise_std)
    if is_policy:
        agent.noise_scale = noise

    out = run_dir(cfg, args_cli.iter, "collect", args_cli.tag)
    dump_config(out, cfg, vars(args_cli), env.cfg, {"agent": spec, "noise": noise, "seed": seed})
    t0 = time.time()
    parts = []
    for r in range(args_cli.rounds):
        parts.append(collect(env, agent, exec_noise_std=0.0 if is_policy else noise))
        print(f"[collect] round {r + 1}/{args_cli.rounds}: {len(parts[-1]['episodes']['length'])} episodes, "
              f"{len(parts[-1]['obs'])} steps", flush=True)
    data = merge(parts) if len(parts) > 1 else parts[0]
    save_h5(out / "dataset.h5", data, {"robot": cfg.robot, "source": spec, "noise": noise, "iteration": args_cli.iter,
                                       "config": cfg.to_dict()})
    stats = {"agent": spec, "noise": noise, "wall_time_s": round(time.time() - t0, 1), **episode_stats(data)}
    write_json(out / "stats.json", stats)
    print(f"[collect] episodes {stats['episodes']} steps {stats['steps']} fall {stats['fall_rate']:.3f} "
          f"success {stats['success_rate']:.3f} return {stats['mean_return']:.2f} nan_rows {stats['nan_rows']}")
    print(f"[collect] reward terms (mean per episode): {stats['reward_terms_mean']}")
    print(f"[collect] output: {out}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
