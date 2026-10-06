r"""Smoke test of the v2 observation groups (shapes, privileged values, history) in a scenario.

Usage:
    tools\isaacpy.cmd 04_sim2real\sim2real\check_env_v2.py --scenario dr_rough_train_v1 --num_envs 64
"""

import argparse
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_ROOT), str(_ROOT / "02_rule_based"), str(_ROOT / "03_training_pipeline"), str(_ROOT / "04_sim2real")]

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--scenario", default="dr_rough_train_v1")
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--seconds", type=float, default=8.0)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import torch  # noqa: E402

from shared import interface_v2 as itf2  # noqa: E402
from sim2real.common import make_env  # noqa: E402
from training_pipeline.config import PipelineCfg  # noqa: E402


def main():
    env = make_env(PipelineCfg(), args_cli.scenario, args_cli.num_envs, 20.0, 0, record=False, obs_version=2)
    lay = itf2.LayoutV2(env.num_legs)
    obs = env.reset()[0]
    print({k: tuple(v.shape) for k, v in obs.items()}, "expected teacher", lay.teacher_dim, "student", lay.student_dim)
    cmd0, f0 = env.cmd_vx.clone(), env.gait_freq.clone()
    ph_prev, f_prev = env.gait_phase.clone(), env.gait_freq.clone()
    frozen_ok, steps = True, int(args_cli.seconds / env.step_dt)
    for i in range(steps):
        obs = env.step(torch.zeros(env.num_envs, env.num_legs, device=env.device))[0]
        stand = (env.gait_freq == 0) & (f_prev == 0)  # standing during the whole step (not the switching step)
        ph = env.gait_phase
        reset = env.episode_length_buf <= 1
        frozen_ok &= bool(((ph - ph_prev).abs()[stand & ~reset] < 1e-6).all())
        ph_prev, f_prev = ph.clone(), env.gait_freq.clone()
        if i == 30:
            t, st = obs["policy"], obs["student"]
    print(f"commands at reset: stand {(f0 == 0).float().mean():.2f} in-place {((cmd0 == 0) & (f0 > 0)).float().mean():.2f} "
          f"walking {(cmd0 > 0).float().mean():.2f}")
    print(f"after {args_cli.seconds:.0f} s: cmd changed {(env.cmd_vx != cmd0).float().mean():.2f} "
          f"freq changed {(env.gait_freq != f0).float().mean():.2f}; clock frozen while standing: {frozen_ok}")
    print(f"clearance cmd range {env.clearance_cmd.min().item():.4f}-{env.clearance_cmd.max().item():.4f}")
    t, st = obs["policy"], obs["student"]
    for name, sl in lay.priv_slices().items():
        v = t[:, sl]
        print(f"{name:14s} min {v.min().item():.4f} mean {v.mean().item():.4f} max {v.max().item():.4f}")
    h = st.reshape(env.num_envs, lay.history, lay.frame_dim)
    print("history frames differ:", [(h[:, 0] - h[:, i]).abs().max().item() > 0 for i in range(1, lay.history)])
    print("student frame 0 == teacher v1 part minus lin vel (only with noise off):",
          torch.allclose(h[:, 0, :-1], t[:, 3:lay.v1.dim]))
    env.close()


if __name__ == "__main__":
    main()
    app.close()
