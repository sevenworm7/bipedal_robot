"""Fixed-set evaluation of any agent in a named scenario (same protocol and stats.json schema as 02 / 03).

``--compare <stats.json>`` turns the run into a regression check: the nominal and push-test statistics must be
identical to the reference (exit code 3 otherwise). Used after every change of ``shared/env``: the 03 final model in
``flat_v1`` must reproduce ``outputs/robot_v1/training/iter3_distill_20261001_132024/stats.json`` exactly.

Output: ``outputs/robot_v<N>/sim2real/eval_<scenario>_<tag>_<time>/{stats.json, episodes.pt, config.yaml}``.

Usage:
    tools\isaacpy.cmd 04_sim2real\sim2real\evaluate.py --scenario flat_v1 --agent policy:<model.pt> --compare <stats.json>
    tools\isaacpy.cmd 04_sim2real\sim2real\evaluate.py --scenario dr_v1 --agent policy:<model.pt>
"""

import argparse
import json
import sys
from pathlib import Path

# project root (shared), 02_rule_based (teacher), 03_training_pipeline (pipeline modules), 04_sim2real (sim2real)
_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_ROOT), str(_ROOT / "02_rule_based"), str(_ROOT / "03_training_pipeline"), str(_ROOT / "04_sim2real")]

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--scenario", type=str, required=True, help="Scenario name or probe:<dr item>.")
parser.add_argument("--agent", type=str, required=True,
                    help="teacher[:params.json], policy:<ckpt> (03 model or v2 teacher) or student:<ckpt>.")
parser.add_argument("--tag", type=str, default="")
parser.add_argument("--no_push", action="store_true")
parser.add_argument("--compare", type=str, default=None, help="Reference stats.json for the regression check.")
parser.add_argument("--episodes", type=int, default=None, help="Override the fixed set size (default EvalCfg.episodes = 100).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import torch  # noqa: E402

from sim2real.common import dump_config, git_commit, make_env, run_dir  # noqa: E402
from training_pipeline.common import env_summary, write_json  # noqa: E402
from training_pipeline.config import PipelineCfg  # noqa: E402
from training_pipeline.evaluation import evaluate_agent, summary_line  # noqa: E402
from sim2real.agents import load_agent_v2, obs_version_for  # noqa: E402


def main():
    cfg = PipelineCfg()
    e = cfg.eval
    if args_cli.episodes:
        e.episodes = args_cli.episodes
    obs_version = obs_version_for(args_cli.agent)
    env = make_env(cfg, args_cli.scenario, e.episodes, e.episode_s, e.seed, obs_version=obs_version)
    agent = load_agent_v2(args_cli.agent, env, cfg.actor, env.rb.PROJECT_ROOT)
    safe = args_cli.scenario.replace(":", "-")
    out = run_dir(env.rb, f"eval_{safe}", args_cli.tag)
    dump_config(out, {"args": vars(args_cli), "scenario": args_cli.scenario, "obs_version": obs_version, "dr": env.cfg.dr.to_dict(),
                      "env": env_summary(env.cfg), "pipeline": cfg.to_dict(), "git_commit": git_commit(env.rb.PROJECT_ROOT)})
    stats, ep = evaluate_agent(env, agent, e, push=not args_cli.no_push)
    stats = {"agent": args_cli.agent, "scenario": args_cli.scenario, **stats}
    write_json(out / "stats.json", stats)
    torch.save({k: v.cpu() for k, v in ep.items() if k != "torque_samples"}, out / "episodes.pt")
    print(f"[eval] {args_cli.scenario}: {summary_line(stats)}")
    print(f"[eval] output: {out}")
    code = 0
    if args_cli.compare:
        ref = json.loads(Path(args_cli.compare).read_text(encoding="utf-8"))
        ref = ref.get("eval", ref)
        keys = ["nominal"] + (["push_test"] if not args_cli.no_push else [])
        diff = {k: (stats[k], ref.get(k)) for k in keys if stats[k] != ref.get(k)}
        print(f"[regress] {'IDENTICAL' if not diff else 'MISMATCH'} vs {args_cli.compare}")
        for k, (a, b) in diff.items():
            for kk in a:
                if a[kk] != (b or {}).get(kk):
                    print(f"[regress]   {k}.{kk}: {a[kk]} != {(b or {}).get(kk)}")
        code = 3 if diff else 0
    env.close()
    return code


if __name__ == "__main__":
    rc = main()
    app.close()
    sys.exit(rc)
