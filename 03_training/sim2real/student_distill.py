r"""Phase 4b: distil the privileged teacher into the deployable student with DAgger.

Every round runs ``steps_per_round`` control steps in the training scenario. Per env, the teacher acts with probability
beta (decaying to 0 after ``beta_rounds``), otherwise the student; every visited state is labelled with the teacher's
mean action. The (student observation, teacher action) pairs are aggregated in a FIFO buffer and the student (rsl_rl
``MLPModel``, same network as 03, 210 inputs) is trained on it with MSE. The student's normaliser is fitted on the first
round and then frozen. Episode endings come from the env (falls reset as usual), so states after the student's mistakes
are part of the data.

Output: ``outputs/robot_v<N>/03_training/sim2real/student_<tag>_<time>/{student.pt, history.json, config.yaml, tb/}``.

Usage:
    tools\isaacpy.cmd 03_training\sim2real\student_distill.py --teacher <teacher model_*.pt> --tag v1
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_ROOT), str(_ROOT / "02_rule_based"), str(_ROOT / "03_training")]

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--teacher", required=True)
parser.add_argument("--tag", default="")
parser.add_argument("--scenario", default=None, help="Training scenario (default: the config's).")
parser.add_argument("--reward_preset", default=None, help="Key of TeacherCfg.reward_presets (round 5: obs_lateral).")
parser.add_argument("--symmetry", action="store_true", help="Also store mirrored samples.")
parser.add_argument("--rounds", type=int, default=None)
parser.add_argument("--num_envs", type=int, default=None)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import torch  # noqa: E402
from torch.utils.tensorboard import SummaryWriter  # noqa: E402

from shared import interface_v2 as itf2  # noqa: E402
from shared.symmetry import Mirror  # noqa: E402
from sim2real.common import dump_config, git_commit, make_env, run_dir  # noqa: E402
from sim2real.config import Sim2RealCfg  # noqa: E402
from training_pipeline.common import env_summary, write_json  # noqa: E402
from training_pipeline.config import PipelineCfg  # noqa: E402
from training_pipeline.policy import actor_mean, load_actor, make_actor, save_actor, set_normalizer  # noqa: E402


def main():
    pipe, cfg = PipelineCfg(), Sim2RealCfg()
    c = cfg.student
    if args_cli.scenario:
        c.scenario = args_cli.scenario
    if args_cli.symmetry:
        c.mirror_augment = True
    torch.manual_seed(c.seed)
    rounds = args_cli.rounds or c.rounds
    env = make_env(pipe, c.scenario, args_cli.num_envs or c.num_envs, pipe.collect.episode_s, c.seed, record=False,
                   obs_version=2, push_interval_s=c.push_interval_s, push_speed=c.push_speed)
    for k, v in cfg.teacher.reward_overrides.items():
        setattr(env.cfg.rewards, k, v)
    if args_cli.reward_preset:
        for k, v in cfg.teacher.reward_presets[args_cli.reward_preset].items():
            setattr(env.cfg.rewards, k, v)
    for k, v in cfg.teacher.env_overrides.items():
        setattr(env.cfg, k, v)
    dev, n = env.device, env.num_envs
    lay = itf2.LayoutV2(env.num_legs)
    teacher = load_actor(args_cli.teacher, pipe.actor, dev)
    student = make_actor(pipe.actor, lay.student_dim, env.num_legs, dev)
    opt = torch.optim.Adam(student.mlp.parameters(), lr=c.lr)
    total_updates = rounds * c.epochs_per_round * math.ceil(c.buffer_steps / c.batch_size)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=total_updates, eta_min=c.lr_min)
    out = run_dir(env.rb, "student", args_cli.tag)
    dump_config(out, {"args": vars(args_cli), "sim2real": cfg.to_dict(), "pipeline": pipe.to_dict(), "scenario": c.scenario,
                      "dr": env.cfg.dr.to_dict(), "gait_var": env.cfg.gait_var.to_dict(), "env": env_summary(env.cfg),
                      "teacher": args_cli.teacher, "git_commit": git_commit(env.rb.PROJECT_ROOT)})
    writer = SummaryWriter(str(out / "tb"))

    buf_x = torch.zeros(c.buffer_steps, lay.student_dim, device=dev)
    buf_y = torch.zeros(c.buffer_steps, env.num_legs, device=dev)
    filled, head = 0, 0
    mirror = Mirror(env.rb, dev) if c.mirror_augment else None
    obs = env.reset()[0]
    history, t0, loss = [], time.time(), torch.tensor(float("nan"))
    for r in range(rounds):
        beta = max(0.0, 1.0 - r / max(c.beta_rounds, 1))
        use_teacher = torch.rand(n, device=dev) < beta
        falls = dones = 0
        with torch.no_grad():
            for _ in range(c.steps_per_round):
                xs, xt = obs["student"], obs["policy"]
                a_t = actor_mean(teacher, xt)
                a_s = actor_mean(student, xs) if r > 0 else a_t
                pairs = [(xs, a_t)] + ([(mirror.student(xs), mirror.actions(a_t))] if mirror is not None else [])
                for px, py in pairs:
                    idx = (torch.arange(n, device=dev) + head) % c.buffer_steps
                    buf_x[idx], buf_y[idx] = px, py
                    head = (head + n) % c.buffer_steps
                    filled = min(filled + n, c.buffer_steps)
                act = torch.where(use_teacher.unsqueeze(-1), a_t, a_s)
                obs, _, term, trunc, _ = env.step(act)
                falls += int(term.sum())
                dones += int((term | trunc).sum())
        if r == 0:  # fit the student normaliser on the first round, then keep it fixed
            set_normalizer(student, buf_x[:filled].mean(0), buf_x[:filled].var(0, unbiased=False), filled)
        n_val = int(filled * c.val_frac)
        perm = torch.randperm(filled, device=dev)
        va, tr = perm[:n_val], perm[n_val:]
        student.train()
        for _ in range(c.epochs_per_round):
            order = tr[torch.randperm(len(tr), device=dev)]
            for i in range(0, len(order), c.batch_size):
                b = order[i:i + c.batch_size]
                loss = torch.nn.functional.mse_loss(actor_mean(student, buf_x[b]), buf_y[b])
                opt.zero_grad(set_to_none=True)
                loss.backward()
                opt.step()
                sched.step()
        student.eval()
        with torch.no_grad():
            val = float(torch.nn.functional.mse_loss(actor_mean(student, buf_x[va]), buf_y[va])) if n_val else float("nan")
        row = {"round": r, "beta": round(beta, 3), "buffer": filled, "val_mse": val, "falls": falls, "episodes_ended": dones,
               "train_loss_last": float(loss), "time_s": round(time.time() - t0, 1)}
        history.append(row)
        for kk in ("val_mse", "falls", "beta"):
            writer.add_scalar(f"student/{kk}", row[kk], r)
        print(f"[student] round {r:2d} beta {beta:.2f} buffer {filled} val_mse {val:.5f} falls {falls} "
              f"ended {dones} | {row['time_s']} s", flush=True)
        (out / "history.json").write_text(json.dumps(history, indent=2), encoding="utf-8")
    writer.close()
    save_actor(out / "student.pt", student, {"teacher": args_cli.teacher, "rounds": rounds, "interface": "v2 student",
                                             "inputs": lay.student_dim})
    write_json(out / "stats.json", {"teacher": args_cli.teacher, "rounds": rounds, "final": history[-1],
                                    "wall_time_s": round(time.time() - t0, 1)})
    print(f"[student] output: {out}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
