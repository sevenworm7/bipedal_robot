"""Measure the swing window of an agent on the gait clock (sets the ``gait_contact`` reward window).

Runs the agent (default: the rule-based teacher) on ``--num_envs`` robots over the full command / gait-frequency
ranges, skips the first ``--skip_s`` seconds (ramp-up), and records P(foot in the air) per clock-phase bin for
each foot, aligned to its own leg phase (left = clock phase, right = clock phase + 0.5). The swing window is the
contiguous run of bins with P(air) > 0.5 around the maximum; copy it into ``PipelineCfg.env_overrides``
(``gait_swing_start``, ``gait_swing_frac``).

Output: ``iter0_gait_<time>/{gait.json, gait.png}``.

Usage:
    tools\\isaacpy.cmd control\\flat_pipeline\\measure_gait.py
"""

import argparse
import sys
from pathlib import Path

# project root (shared), control/rule_based (rule_based teacher), control (flat_pipeline)
_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_ROOT), str(_ROOT / "control")]  # project root (core, simulation), control (rule_based, flat_pipeline, sim2real)

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--agent", type=str, default=None, help="teacher[:params.json] (default) or policy:<ckpt>.")
parser.add_argument("--num_envs", type=int, default=512)
parser.add_argument("--seconds", type=float, default=12.0)
parser.add_argument("--skip_s", type=float, default=3.0)
parser.add_argument("--bins", type=int, default=40)
parser.add_argument("--seed", type=int, default=5)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import numpy as np  # noqa: E402
import torch  # noqa: E402

from flat_pipeline.common import dump_config, make_env, run_dir, write_json  # noqa: E402
from flat_pipeline.config import PipelineCfg  # noqa: E402
from flat_pipeline.policy import load_agent  # noqa: E402


def window(p_air: np.ndarray) -> tuple[float, float]:
    """(start, length) in cycles of the contiguous P(air) > 0.5 run that contains the maximum (circular)."""
    n = len(p_air)
    k = int(np.argmax(p_air))
    if p_air[k] <= 0.5:
        return float("nan"), 0.0
    lo = k
    while p_air[(lo - 1) % n] > 0.5 and (k - lo) < n:
        lo -= 1
    hi = k
    while p_air[(hi + 1) % n] > 0.5 and (hi - lo) < n:
        hi += 1
    return (lo % n) / n, (hi - lo + 1) / n


def main():
    cfg = PipelineCfg()
    torch.manual_seed(args_cli.seed)
    env = make_env(cfg, args_cli.num_envs, args_cli.seconds + 1.0, args_cli.seed, record=False, init_joint_noise=0.0)
    spec = args_cli.agent or f"teacher:{cfg.collect.teacher_params}"
    agent = load_agent(spec, env, cfg.actor, env.rb.PROJECT_ROOT)
    out = run_dir(cfg, 0, "gait")
    dump_config(out, cfg, vars(args_cli), env.cfg, {"agent": spec})

    b = args_cli.bins
    air = np.zeros((2, b))
    cnt = np.zeros((2, b))
    obs = env.reset()[0]["policy"]
    agent.reset()
    n_steps = int(args_cli.seconds / env.step_dt)
    for i in range(n_steps):
        obs, _, term, trunc, _ = env.step(agent.compute_action(obs))
        obs = obs["policy"]
        if bool((term | trunc).any()):
            agent.reset((term | trunc).nonzero().flatten())
        if i * env.step_dt < args_cli.skip_s:
            continue
        phase = env.gait_phase.cpu().numpy()
        in_air = (~env.foot_in_contact()).cpu().numpy()
        alive = (env.episode_length_buf.cpu().numpy() * env.step_dt) > args_cli.skip_s
        for f, off in enumerate((0.0, 0.5)):
            k = (np.floor(((phase + off) % 1.0) * b).astype(int)) % b
            np.add.at(air[f], k[alive], in_air[alive, f])
            np.add.at(cnt[f], k[alive], 1)
    p_air = air / np.maximum(cnt, 1)
    res = {"agent": spec, "bins": b, "p_air_left": p_air[0].round(3).tolist(), "p_air_right": p_air[1].round(3).tolist()}
    for f, name in enumerate(("left", "right")):
        s, L = window(p_air[f])
        res[f"{name}_swing_start"], res[f"{name}_swing_frac"] = round(s, 4), round(L, 4)
    res["suggested"] = {"gait_swing_start": round(float(np.nanmean([res["left_swing_start"], res["right_swing_start"]])), 4),
                        "gait_swing_frac": round(float(np.mean([res["left_swing_frac"], res["right_swing_frac"]])), 4)}
    write_json(out / "gait.json", res)

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    x = (np.arange(b) + 0.5) / b
    fig, ax = plt.subplots(figsize=(8, 3.5))
    ax.plot(x, p_air[0], label="left (leg phase = clock)")
    ax.plot(x, p_air[1], label="right (leg phase = clock + 0.5)")
    ax.axhline(0.5, color="gray", lw=0.8)
    ax.set(xlabel="leg phase [cycles]", ylabel="P(foot in the air)", ylim=(0, 1), title=spec)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "gait.png", dpi=110)
    print(f"[gait] left start {res['left_swing_start']} frac {res['left_swing_frac']} | right start {res['right_swing_start']} "
          f"frac {res['right_swing_frac']} | suggested {res['suggested']}")
    print(f"[gait] output: {out}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
