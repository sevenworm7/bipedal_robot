r"""Phase 4a: privileged teacher PPO on interface v2, initialised from the 03 final model.

1. Build the env (``TeacherCfg.scenario``, ``obs_version=2``): ``policy`` = ``critic`` = 55-dim teacher observation.
2. Teacher initialisation: the 03 actor (44 inputs) is copied into a 55-input actor whose extra first-layer columns are
   zero; the normaliser keeps the 03 statistics for the first 44 inputs and gets the mean / variance of the 11 new
   inputs from a short zero-action rollout. The script checks that the teacher reproduces the 03 actions (max |diff|).
3. PPO as in 03 (critic warm-up with the actor frozen, learning-rate reset, actor normaliser frozen).

Output: ``outputs/robot_v<N>/sim2real/teacher_<tag>_<time>/{teacher_init.pt, model_<it>.pt, config.yaml, stats.json, tb}``.

Usage:
    tools\isaacpy.cmd 04_sim2real\sim2real\teacher_ppo.py --tag v1
    tools\isaacpy.cmd 04_sim2real\sim2real\teacher_ppo.py --tag speed --max_iterations 20 --num_envs 4096
"""

import argparse
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_ROOT), str(_ROOT / "02_rule_based"), str(_ROOT / "03_training_pipeline"), str(_ROOT / "04_sim2real")]

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--tag", default="")
parser.add_argument("--scenario", default=None, help="Training scenario (default: the config's).")
parser.add_argument("--max_iterations", type=int, default=None)
parser.add_argument("--num_envs", type=int, default=None)
parser.add_argument("--init", type=str, default=None, help="Teacher checkpoint to resume from (55 inputs).")
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

from shared import interface_v2 as itf2  # noqa: E402
from sim2real.common import dump_config, git_commit, make_env, run_dir  # noqa: E402
from sim2real.config import Sim2RealCfg  # noqa: E402
from training_pipeline.common import env_summary, write_json  # noqa: E402
from training_pipeline.config import PipelineCfg  # noqa: E402
from training_pipeline.policy import actor_mean, freeze_normalizer, load_actor, make_actor, save_actor  # noqa: E402


def runner_cfg(t, a, max_iterations: int) -> RslRlOnPolicyRunnerCfg:
    return RslRlOnPolicyRunnerCfg(
        seed=t.seed, device="cuda:0", num_steps_per_env=t.num_steps_per_env, max_iterations=max_iterations,
        save_interval=t.save_interval, experiment_name="exp2_sim2real_teacher", empirical_normalization=False,
        obs_groups={"actor": ["policy"], "critic": ["critic"]}, clip_actions=None,
        actor=RslRlMLPModelCfg(hidden_dims=list(a.hidden_dims), activation=a.activation, obs_normalization=a.obs_normalization,
                               distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=t.init_std)),
        critic=RslRlMLPModelCfg(hidden_dims=list(t.critic_hidden_dims), activation=a.activation, obs_normalization=True),
        algorithm=RslRlPpoAlgorithmCfg(
            value_loss_coef=t.value_loss_coef, use_clipped_value_loss=True, clip_param=t.clip_param,
            entropy_coef=t.entropy_coef, num_learning_epochs=t.num_learning_epochs, num_mini_batches=t.num_mini_batches,
            learning_rate=t.learning_rate, schedule=t.schedule, gamma=t.gamma, lam=t.lam, desired_kl=t.desired_kl,
            max_grad_norm=t.max_grad_norm),
    )


@torch.no_grad()
def build_teacher_init(env, pipe, t, out: Path) -> tuple[Path, float]:
    """Zero-padded copy of the 03 actor (see module docstring); returns the checkpoint path and the reproduction error."""
    dev = env.device
    src = load_actor(_ROOT / t.init_checkpoint, pipe.actor, dev)
    lay = itf2.LayoutV2(env.num_legs)
    # normaliser statistics of the new inputs from a zero-action rollout (default pose, randomised scenario)
    obs = env.reset()[0]
    samples = []
    for _ in range(t.norm_stat_steps):
        samples.append(obs["policy"][:, lay.v1.dim:].clone())
        obs = env.step(torch.zeros(env.num_envs, env.num_legs, device=dev))[0]
    new = torch.cat(samples)
    teacher = make_actor(pipe.actor, lay.teacher_dim, env.num_legs, dev)
    sd_src, sd = src.state_dict(), teacher.state_dict()
    for k, v in sd_src.items():
        if k == "mlp.0.weight":
            sd[k].zero_()
            sd[k][:, : lay.v1.dim] = v
        elif k.startswith("obs_normalizer.") and v.ndim == 2:
            sd[k][:, : lay.v1.dim] = v
        elif k == "obs_normalizer.count":
            sd[k] = v
        else:
            sd[k] = v
    mean, var = new.mean(0), new.var(0, unbiased=False).clamp_min(1e-6)
    sd["obs_normalizer._mean"][:, lay.v1.dim:] = mean
    sd["obs_normalizer._var"][:, lay.v1.dim:] = var
    sd["obs_normalizer._std"][:, lay.v1.dim:] = var.sqrt()
    teacher.load_state_dict(sd)
    # reproduction check on fresh observations: teacher(55) vs 03 actor(44)
    tob = obs["policy"]
    diff = float((actor_mean(teacher, tob) - actor_mean(src, tob[:, : lay.v1.dim])).abs().max())
    path = out / "teacher_init.pt"
    save_actor(path, teacher, {"from": t.init_checkpoint, "zero_padded_inputs": lay.teacher_dim - lay.v1.dim,
                               "new_input_mean": mean.tolist(), "new_input_var": var.tolist()})
    return path, diff


def main():
    pipe, cfg = PipelineCfg(), Sim2RealCfg()
    t = cfg.teacher
    if args_cli.scenario:
        t.scenario = args_cli.scenario
    total = args_cli.max_iterations or t.max_iterations
    env = make_env(pipe, t.scenario, args_cli.num_envs or t.num_envs, pipe.collect.episode_s, t.seed, record=False,
                   obs_version=2, push_interval_s=t.push_interval_s, push_speed=t.push_speed)
    for k, v in t.reward_overrides.items():
        setattr(env.cfg.rewards, k, v)  # read every step by _get_rewards
    for k, v in t.env_overrides.items():
        setattr(env.cfg, k, v)
    out = run_dir(env.rb, "teacher", args_cli.tag)
    agent_cfg = handle_deprecated_rsl_rl_cfg(runner_cfg(t, pipe.actor, total), metadata.version("rsl-rl-lib"))
    if args_cli.init:
        init_path, diff = Path(args_cli.init), 0.0
    else:
        init_path, diff = build_teacher_init(env, pipe, t, out)
        print(f"[teacher] init from {t.init_checkpoint}: max |a_teacher - a_03| = {diff:.2e}", flush=True)
        if diff > 1e-4:
            raise RuntimeError("zero-padded teacher does not reproduce the 03 actor")
    dump_config(out, {"args": vars(args_cli), "sim2real": cfg.to_dict(), "pipeline": pipe.to_dict(), "scenario": t.scenario,
                      "dr": env.cfg.dr.to_dict(), "gait_var": env.cfg.gait_var.to_dict(), "env": env_summary(env.cfg),
                      "rewards": env.cfg.rewards.to_dict(), "rsl_rl": agent_cfg.to_dict(), "init": str(init_path),
                      "git_commit": git_commit(env.rb.PROJECT_ROOT)})
    wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(wrapped, agent_cfg.to_dict(), log_dir=str(out), device=agent_cfg.device)
    runner.load(str(init_path), load_cfg={"actor": True}, strict=True, map_location=agent_cfg.device)
    actor = runner.alg.actor
    freeze_normalizer(actor)

    t0 = time.time()
    warm = min(t.critic_warmup_iters, total)  # also when resuming: only the actor is loaded, the critic is new
    if warm > 0:
        for prm in actor.parameters():
            prm.requires_grad_(False)
        print(f"[teacher] critic warm-up: {warm} iterations with the actor frozen", flush=True)
        runner.learn(num_learning_iterations=warm, init_at_random_ep_len=True)
        for prm in actor.parameters():
            prm.requires_grad_(True)
        runner.alg.learning_rate = t.learning_rate
        for g in runner.alg.optimizer.param_groups:
            g["lr"] = t.learning_rate
        runner.current_learning_iteration += 1
    if total - warm > 0:
        runner.learn(num_learning_iterations=total - warm, init_at_random_ep_len=False)
    ckpts = sorted(out.glob("model_*.pt"), key=lambda q: int(q.stem.split("_")[1]))
    stats = {"init": str(init_path), "teacher_init_reproduction_max_abs_diff": diff, "iterations": total,
             "wall_time_s": round(time.time() - t0, 1), "final_checkpoint": str(ckpts[-1]) if ckpts else None,
             "final_action_std": [round(v, 4) for v in actor.distribution.std_param.detach().cpu().tolist()]}
    write_json(out / "stats.json", stats)
    print(f"[teacher] done in {stats['wall_time_s']} s; final checkpoint {stats['final_checkpoint']}")
    print(f"[teacher] output: {out}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
