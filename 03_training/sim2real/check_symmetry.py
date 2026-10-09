r"""Checks of the left-right mirror (``shared/symmetry.py``) on real states.

1. Involution: mirror(mirror(x)) == x for teacher / student / action tensors.
2. Equivariance error of a policy, |pi(M s) - M pi(s)| / |pi(s)| on states visited in ``flat_v1``: small for a nearly
   symmetric gait, larger for an asymmetric one; a deliberately wrong map (hip roll not flipped) must be worse.

Usage:
    tools\isaacpy.cmd 03_training\sim2real\check_symmetry.py --agent policy:<03 model.pt> --agent policy:<teacher.pt>
"""

import argparse
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_ROOT), str(_ROOT / "02_rule_based"), str(_ROOT / "03_training")]

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--agent", action="append", required=True)
parser.add_argument("--steps", type=int, default=300)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import torch  # noqa: E402

from shared.symmetry import Mirror  # noqa: E402
from sim2real.common import make_env  # noqa: E402
from training_pipeline.config import PipelineCfg  # noqa: E402
from training_pipeline.policy import actor_mean, load_actor  # noqa: E402


def main():
    cfg = PipelineCfg()
    env = make_env(cfg, "flat_v1", 256, 20.0, 1, record=False, obs_version=2)
    m = Mirror(env.rb, env.device)
    print(f"[symmetry] involution max error {m.self_test():.2e}")
    wrong = Mirror(env.rb, env.device)
    wrong.sign = wrong.sign.abs()  # deliberately wrong: hip roll and tail yaw not flipped
    actors = [(spec, load_actor(spec.split(":", 1)[1], cfg.actor, env.device)) for spec in args_cli.agent]
    first = actors[0][1]
    obs = env.reset()[0]
    states = []
    with torch.no_grad():
        for i in range(args_cli.steps):
            t = obs["policy"]
            a = actor_mean(first, t[:, : m.lay.v1.dim]) if first.obs_normalizer._mean.shape[-1] == m.lay.v1.dim \
                else actor_mean(first, t)
            obs = env.step(a)[0]
            if i >= 100:
                states.append(obs["policy"].clone())
    s = torch.cat(states)
    with torch.no_grad():
        for spec, actor in actors:
            v1 = actor.obs_normalizer._mean.shape[-1] == m.lay.v1.dim
            for name, mm in (("correct", m), ("wrong", wrong)):
                x = s[:, : m.lay.v1.dim] if v1 else s
                fm = mm.obs_v1 if v1 else mm.teacher
                a, am = actor_mean(actor, x), actor_mean(actor, fm(x))
                err = (am - mm.actions(a)).norm(dim=-1) / a.norm(dim=-1).clamp_min(1e-6)
                print(f"[symmetry] {spec.split('/')[-2] if '/' in spec else spec} map={name}: "
                      f"rel equivariance error median {err.median():.3f} p90 {torch.quantile(err, 0.9):.3f}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
