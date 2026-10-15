"""Stage 3: PPO fine-tuning (rsl_rl OnPolicyRunner) initialised from a BC / distilled checkpoint.

1. Build the env (``PPOCfg.num_envs``, light random pushes) and the rsl_rl runner with an actor that is
   identical to the BC network (``ActorCfg``) and a fresh critic.
2. Load the actor weights (incl. observation normaliser and action std) and verify that the PPO actor
   reproduces the BC outputs; freeze the actor's normaliser.
3. Critic warm-up: ``PPOCfg.critic_warmup_iters`` iterations with the actor frozen, then reset the
   learning rate and train everything for the remaining iterations.

Reward terms and weights: ``BipedWalkEnvCfg.rewards`` (written to config.yaml); TensorBoard logs every
reward term (Episode/Episode_Reward/*), episode length, fall rate and walking metrics (Episode/Metrics/*).

Output: ``iter<N>_ppo_<time>/{model_<it>.pt, config.yaml, stats.json, events.* (TensorBoard)}``.

Usage:
    tools\\isaacpy.cmd control\\flat_pipeline\\ppo.py --iter 1 --init <iter0_bc>/model.pt
"""

import argparse
import sys
import time
from pathlib import Path

# project root (shared), control/rule_based (rule_based teacher), control (flat_pipeline)
_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_ROOT), str(_ROOT / "control")]  # project root (core, simulation), control (rule_based, flat_pipeline, sim2real)

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--iter", type=int, required=True)
parser.add_argument("--init", type=str, required=True, help="BC / distilled checkpoint (model.pt).")
parser.add_argument("--max_iterations", type=int, default=None)
parser.add_argument("--num_envs", type=int, default=None)
parser.add_argument("--tag", type=str, default="")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

from importlib import metadata  # noqa: E402

import torch  # noqa: E402
from isaaclab_rl.rsl_rl import (  # noqa: E402
    RslRlMLPModelCfg,
    RslRlOnPolicyRunnerCfg,
    RslRlPpoAlgorithmCfg,
    RslRlVecEnvWrapper,
    handle_deprecated_rsl_rl_cfg,
)
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from flat_pipeline.common import dump_config, make_env, run_dir, write_json  # noqa: E402
from flat_pipeline.config import PipelineCfg  # noqa: E402
from flat_pipeline.policy import actor_mean, freeze_normalizer, load_actor  # noqa: E402


def runner_cfg(cfg: PipelineCfg, max_iterations: int) -> RslRlOnPolicyRunnerCfg:
    p, a = cfg.ppo, cfg.actor
    return RslRlOnPolicyRunnerCfg(
        seed=p.seed,
        device="cuda:0",
        num_steps_per_env=p.num_steps_per_env,
        max_iterations=max_iterations,
        save_interval=p.save_interval,
        experiment_name=f"{cfg.exp}_ppo",
        empirical_normalization=False,
        obs_groups={"actor": ["policy"], "critic": ["policy"]},
        clip_actions=None,
        actor=RslRlMLPModelCfg(
            hidden_dims=list(a.hidden_dims), activation=a.activation, obs_normalization=a.obs_normalization,
            distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=a.init_std),
        ),
        critic=RslRlMLPModelCfg(hidden_dims=list(p.critic_hidden_dims), activation=a.activation, obs_normalization=True),
        algorithm=RslRlPpoAlgorithmCfg(
            value_loss_coef=p.value_loss_coef, use_clipped_value_loss=True, clip_param=p.clip_param,
            entropy_coef=p.entropy_coef, num_learning_epochs=p.num_learning_epochs, num_mini_batches=p.num_mini_batches,
            learning_rate=p.learning_rate, schedule=p.schedule, gamma=p.gamma, lam=p.lam, desired_kl=p.desired_kl,
            max_grad_norm=p.max_grad_norm,
        ),
    )


def main():
    cfg = PipelineCfg()
    p = cfg.ppo
    total = args_cli.max_iterations or p.max_iterations
    env = make_env(cfg, args_cli.num_envs or p.num_envs, cfg.collect.episode_s, p.seed + args_cli.iter, record=False,
                   push_interval_s=p.push_interval_s, push_speed=p.push_speed)
    out = run_dir(cfg, args_cli.iter, "ppo", args_cli.tag)
    agent_cfg = handle_deprecated_rsl_rl_cfg(runner_cfg(cfg, total), metadata.version("rsl-rl-lib"))
    dump_config(out, cfg, vars(args_cli), env.cfg, {"rsl_rl": agent_cfg.to_dict()})
    wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(wrapped, agent_cfg.to_dict(), log_dir=str(out), device=agent_cfg.device)

    # load the BC actor and check that PPO reproduces it
    runner.load(args_cli.init, load_cfg={"actor": True}, strict=True, map_location=agent_cfg.device)
    actor = runner.alg.actor
    freeze_normalizer(actor)
    bc = load_actor(args_cli.init, cfg.actor, agent_cfg.device)
    obs = wrapped.get_observations()["policy"]
    with torch.no_grad():
        diff = float((actor_mean(actor, obs) - actor_mean(bc, obs)).abs().max())
    print(f"[ppo] loaded {args_cli.init}; max |a_ppo - a_bc| = {diff:.2e}; std {actor.distribution.std_param.mean().item():.3f}", flush=True)
    if diff > 1e-4:
        raise RuntimeError("PPO actor does not reproduce the BC checkpoint")
    del bc

    t0 = time.time()
    warm = min(p.critic_warmup_iters, total)
    if warm > 0:
        for prm in actor.parameters():
            prm.requires_grad_(False)
        print(f"[ppo] critic warm-up: {warm} iterations with the actor frozen", flush=True)
        runner.learn(num_learning_iterations=warm, init_at_random_ep_len=p.init_at_random_ep_len)
        for prm in actor.parameters():
            prm.requires_grad_(True)
        runner.alg.learning_rate = p.learning_rate  # the adaptive schedule raised it while KL was 0
        for g in runner.alg.optimizer.param_groups:
            g["lr"] = p.learning_rate
        runner.current_learning_iteration += 1
    if total - warm > 0:
        runner.learn(num_learning_iterations=total - warm, init_at_random_ep_len=warm == 0 and p.init_at_random_ep_len)
    ckpts = sorted(out.glob("model_*.pt"), key=lambda q: int(q.stem.split("_")[1]))
    stats = {"init": args_cli.init, "iterations": total, "critic_warmup_iters": warm, "wall_time_s": round(time.time() - t0, 1),
             "bc_reproduction_max_abs_diff": diff, "final_checkpoint": str(ckpts[-1]) if ckpts else None,
             "final_action_std": [round(v, 4) for v in actor.distribution.std_param.detach().cpu().tolist()]}
    write_json(out / "stats.json", stats)
    print(f"[ppo] done in {stats['wall_time_s']} s; final checkpoint {stats['final_checkpoint']}")
    print(f"[ppo] output: {out}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
