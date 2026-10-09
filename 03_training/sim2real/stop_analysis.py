r"""Does a policy stop on its own? Separates commanded stops (speed command 0) from uncommanded ones.

Every control step records the commanded speed, gait frequency and the base forward velocity. With a 1 s moving average
of the forward velocity, a robot is "stalled" when the command asks to walk (> 0.03 m/s) but the averaged speed is below
``stall_ratio`` x command. Reports, over the time the command asks to walk: the stalled fraction, the number of stall
events per minute (entries into the stalled state), and the median stall duration; plus the fraction of time with a
commanded stop (stand or stepping in place). Falls end an env's record.

Output: ``outputs/robot_v<N>/sim2real/stops_<scenario>_<tag>_<time>/stops.json``.

Usage:
    tools\isaacpy.cmd 03_training\sim2real\stop_analysis.py --scenario gait_eval_v1 --agent student:<student.pt> --tag v4
"""

import argparse
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_ROOT), str(_ROOT / "02_rule_based"), str(_ROOT / "03_training")]

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--scenario", required=True)
parser.add_argument("--agent", required=True)
parser.add_argument("--tag", required=True)
parser.add_argument("--num_envs", type=int, default=128)
parser.add_argument("--seconds", type=float, default=20.0)
parser.add_argument("--stall_ratio", type=float, default=0.3)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import torch  # noqa: E402

from sim2real.agents import load_agent_v2, obs_version_for  # noqa: E402
from sim2real.common import make_env, run_dir  # noqa: E402
from training_pipeline.common import write_json  # noqa: E402
from training_pipeline.config import PipelineCfg  # noqa: E402


def main():
    cfg = PipelineCfg()
    ver = obs_version_for(args_cli.agent)
    env = make_env(cfg, args_cli.scenario, args_cli.num_envs, args_cli.seconds + 1.0, 42, record=False, obs_version=ver)
    agent = load_agent_v2(args_cli.agent, env, cfg.actor, env.rb.PROJECT_ROOT)
    obs = env.reset()[0]
    agent.reset()
    n, dev = env.num_envs, env.device
    T = int(args_cli.seconds / env.step_dt)
    cmd = torch.zeros(T, n, device=dev)
    vx = torch.zeros(T, n, device=dev)
    alive = torch.ones(T, n, dtype=torch.bool, device=dev)
    dead = torch.zeros(n, dtype=torch.bool, device=dev)
    with torch.no_grad():
        for t in range(T):
            o, _, term, trunc, _ = env.step(agent.compute_action(obs["policy"]))
            obs = o
            dead |= term | trunc
            cmd[t], vx[t], alive[t] = env.cmd_vx, env.base_lin_vel[:, 0], ~dead
    w = int(1.0 / env.step_dt)
    kernel = torch.ones(1, 1, w, device=dev) / w
    v_avg = torch.nn.functional.conv1d(vx.T.unsqueeze(1), kernel, padding=w - 1)[:, 0, : T].T  # causal 1 s average
    settle = torch.arange(T, device=dev).unsqueeze(1) >= 2 * w  # ignore the first 2 s (ramp-up)
    walk = (cmd > 0.03) & alive & settle
    stalled = walk & (v_avg < args_cli.stall_ratio * cmd)
    commanded_stop = (cmd <= 0.03) & alive & settle
    entries = (stalled[1:] & ~stalled[:-1]).sum()
    walk_min = float(walk.sum()) * env.step_dt / 60.0
    # stall durations
    durations = []
    for e in range(n):
        run = 0
        for t in range(T):
            if bool(stalled[t, e]):
                run += 1
            elif run:
                durations.append(run * env.step_dt)
                run = 0
        if run:
            durations.append(run * env.step_dt)
    d = torch.tensor(durations) if durations else torch.zeros(0)
    res = {
        "agent": args_cli.agent, "scenario": args_cli.scenario, "envs": n, "seconds": args_cli.seconds,
        "fell_frac": round(float(dead.float().mean()), 4),
        "walk_command_time_frac": round(float(walk.float().sum() / (alive & settle).float().sum().clamp_min(1)), 4),
        "commanded_stop_time_frac": round(float(commanded_stop.float().sum() / (alive & settle).float().sum().clamp_min(1)), 4),
        "stalled_frac_of_walk_time": round(float(stalled.float().sum() / walk.float().sum().clamp_min(1)), 4),
        "stall_events_per_min_walking": round(float(entries) / max(walk_min, 1e-6), 3),
        "stall_duration_s_p50_p90": [round(float(x), 2) for x in torch.quantile(d, torch.tensor([0.5, 0.9]))] if d.numel() else [],
        "robots_with_stall_frac": round(float(stalled.any(0).float().mean()), 4),
    }
    out = run_dir(env.rb, f"stops_{args_cli.scenario}", args_cli.tag)
    write_json(out / "stops.json", res)
    print(f"[stops] {args_cli.tag:12s} {args_cli.scenario:15s} stalled {res['stalled_frac_of_walk_time']:.3f} of walking time | "
          f"{res['stall_events_per_min_walking']:.2f} events/min | durations p50/p90 {res['stall_duration_s_p50_p90']} | "
          f"robots with a stall {res['robots_with_stall_frac']:.2f} | commanded stops {res['commanded_stop_time_frac']:.2f} | "
          f"fell {res['fell_frac']:.2f}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
