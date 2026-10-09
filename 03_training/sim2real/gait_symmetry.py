r"""Left / right gait symmetry of an agent (diagnostic for the "limp" / step-to gait seen in the 04 videos).

Runs the agent at one fixed command (no command changes, gait clock running) and records every touchdown after a
settling time. Per touchdown of foot F: step length = (x of F) - (x of the other foot) along the walking direction, and
the air time of that swing. A normal (step-through) gait has equal positive step lengths for both feet (about
cmd / (2 f)); a step-to gait has one long step and one step near 0 (the trailing foot only catches up).

Output: ``outputs/robot_v<N>/03_training/sim2real/symmetry_<tag>_<time>/symmetry.json`` and a printed summary.

Usage:
    tools\isaacpy.cmd 03_training\sim2real\gait_symmetry.py --agent student:<student.pt> --tag student_v3
"""

import argparse
import json
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_ROOT), str(_ROOT / "02_rule_based"), str(_ROOT / "03_training")]

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--agent", required=True)
parser.add_argument("--tag", required=True)
parser.add_argument("--scenario", default="flat_v1")
parser.add_argument("--cmd", type=float, default=0.10)
parser.add_argument("--freq", type=float, default=1.6)
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--seconds", type=float, default=12.0)
parser.add_argument("--settle_s", type=float, default=3.0)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import torch  # noqa: E402

from sim2real.agents import load_agent_v2, obs_version_for  # noqa: E402
from sim2real.common import make_env, run_dir  # noqa: E402
from training_pipeline.common import write_json  # noqa: E402
from training_pipeline.config import PipelineCfg  # noqa: E402


def q(x: torch.Tensor) -> list[float]:
    return [round(float(v), 4) for v in torch.quantile(x, torch.tensor([0.1, 0.5, 0.9]))] if x.numel() else []


def main():
    cfg = PipelineCfg()
    ver = obs_version_for(args_cli.agent)
    env = make_env(cfg, args_cli.scenario, args_cli.num_envs, args_cli.seconds + 1.0, 3, record=False, obs_version=ver,
                   cmd_vx_range=(args_cli.cmd, args_cli.cmd), gait_freq_range=(args_cli.freq, args_cli.freq),
                   init_joint_noise=0.0)
    agent = load_agent_v2(args_cli.agent, env, cfg.actor, env.rb.PROJECT_ROOT)
    feet = [env.robot.find_bodies(nm)[0][0] for nm in env.rb.FOOT_BODY_NAMES]
    obs = env.reset()[0]
    agent.reset()
    steps = {0: [], 1: []}
    air = {0: [], 1: []}
    width = []
    for i in range(int(args_cli.seconds / env.step_dt)):
        o, _, term, trunc, _ = env.step(agent.compute_action(obs["policy"] if isinstance(obs, dict) else obs))
        obs = o
        if i * env.step_dt < args_cli.settle_s:
            continue
        pos = env.robot.data.body_pos_w.torch[:, feet, :2]  # [N, 2 feet, xy], walking direction = +x
        td = env.first_contact
        alive = ~(term | trunc)
        for f in (0, 1):
            m = td[:, f] & alive
            if bool(m.any()):
                steps[f].append((pos[m, f, 0] - pos[m, 1 - f, 0]).cpu())
                air[f].append(env.foot_last_air_time()[m, f].cpu())
                width.append((pos[m, 0, 1] - pos[m, 1, 1]).cpu())
    res = {"agent": args_cli.agent, "scenario": args_cli.scenario, "cmd": args_cli.cmd, "freq": args_cli.freq,
           "nominal_step_length": round(args_cli.cmd / (2 * args_cli.freq), 4)}
    names = ("left", "right")
    for f in (0, 1):
        s = torch.cat(steps[f]) if steps[f] else torch.zeros(0)
        a = torch.cat(air[f]) if air[f] else torch.zeros(0)
        res[f"{names[f]}_touchdowns"] = int(s.numel())
        res[f"{names[f]}_step_length_p10_p50_p90"] = q(s)
        res[f"{names[f]}_air_time_p10_p50_p90"] = q(a)
    sl, sr = res["left_step_length_p10_p50_p90"], res["right_step_length_p10_p50_p90"]
    if sl and sr:
        mean = (abs(sl[1]) + abs(sr[1])) / 2
        res["step_length_asymmetry"] = round(abs(sl[1] - sr[1]) / max(mean, 1e-6), 3)
    w = torch.cat(width) if width else torch.zeros(0)
    res["foot_width_p50"] = round(float(w.median()), 4) if w.numel() else None
    out = run_dir(env.rb, "symmetry", args_cli.tag)
    write_json(out / "symmetry.json", res)
    print(f"[symmetry] {args_cli.tag}: nominal step {res['nominal_step_length']} | left step {sl} air "
          f"{res['left_air_time_p10_p50_p90']} n {res['left_touchdowns']} | right step {sr} air "
          f"{res['right_air_time_p10_p50_p90']} n {res['right_touchdowns']} | asym {res.get('step_length_asymmetry')}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
