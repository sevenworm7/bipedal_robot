"""Formal evaluation of the rule-based controller (development step 5).

Runs ``--episodes`` parallel episodes (one per env) with commands, gait frequencies and initial joint
noise randomised over the env ranges, and writes ``stats.json`` with the acceptance numbers:
success rate (``rule_based_success``), fall rate, distance, speed, heading and lateral errors, results
per command / frequency bin, and per-joint torque (mean, p95, max).

With ``--push`` a second batch is run in which every robot receives velocity kicks (random horizontal
direction) at the given times; reported separately as a reference metric (not part of the gate).

Usage:
    tools\\isaacpy.cmd 00_test\\rule_based\\evaluate.py --params <best_params.json> --episodes 100 --push
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--tag", default="eval")
parser.add_argument("--params", type=str, default=None, help="JSON with gait parameters (default: config defaults).")
parser.add_argument("--episodes", type=int, default=100)
parser.add_argument("--episode_s", type=float, default=20.0)
parser.add_argument("--push", action="store_true", help="Also run the push test (reference only).")
parser.add_argument("--push_speed", type=float, default=0.3, help="Horizontal velocity kick [m/s].")
parser.add_argument("--push_times", type=str, default="5,10,15", help="Kick times [s].")
parser.add_argument("--seed", type=int, default=42)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import torch  # noqa: E402

from rule_based import RuleBasedBipedController, RuleBasedCfg  # noqa: E402
from rule_based.rollout import aggregate, run_first_episodes, torque_stats  # noqa: E402
from shared.env import BipedWalkEnv, BipedWalkEnvCfg  # noqa: E402


def load_cfg() -> RuleBasedCfg:
    cfg = RuleBasedCfg()
    if args_cli.params:
        for k, v in json.loads(Path(args_cli.params).read_text(encoding="utf-8")).items():
            setattr(cfg.gait, k, float(v))
    return cfg


def make_push(push_speed: float, times: list[float]):
    def push(env, step):
        t = step * env.step_dt
        if not any(abs(t - pt) < 0.5 * env.step_dt for pt in times):
            return
        vel = env.robot.data.root_vel_w.torch.clone()
        ang = torch.rand(env.num_envs, device=env.device) * 2 * math.pi
        vel[:, 0] += push_speed * torch.cos(ang)
        vel[:, 1] += push_speed * torch.sin(ang)
        ids = torch.arange(env.num_envs, device=env.device)
        env.robot.write_root_velocity_to_sim_index(root_velocity=vel, env_ids=ids)

    return push


def binned(ep: dict, key: str, edges: list[float]) -> dict:
    out = {}
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (ep[key] >= lo) & (ep[key] <= hi)
        a = aggregate(ep, m)
        out[f"{lo:.2f}-{hi:.2f}"] = {k: a.get(k) for k in ("episodes", "success_rate", "fall_rate", "mean_distance_m", "mean_vx",
                                                         "mean_cmd_vx", "mean_max_abs_lateral_m")}
    return out


def main():
    torch.manual_seed(args_cli.seed)
    env_cfg = BipedWalkEnvCfg()
    env_cfg.scene.num_envs = args_cli.episodes
    env_cfg.episode_length_s = args_cli.episode_s
    env_cfg.record_episodes = True
    env_cfg.seed = args_cli.seed
    env = BipedWalkEnv(env_cfg)
    cfg = load_cfg()
    ctrl = RuleBasedBipedController(env.rb, env.num_envs, env.device, cfg)

    out_dir = env.rb.TEST_ROOT / "outputs" / "rule_based" / f"{args_cli.tag}_{time.strftime('%Y%m%d_%H%M%S')}"
    out_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    ep = run_first_episodes(env, ctrl, torque_every=5)
    lo, hi = env_cfg.cmd_vx_range
    flo, fhi = env_cfg.gait_freq_range
    stats = {
        "controller": "RuleBasedBipedController",
        "robot": env.rb.ROBOT_NAME,
        "gate": {"criterion": "not fallen, steps >= 10, distance >= 0.8 * steps * step_length_cmd",
                 "required_success_rate": 0.70},
        "env": {"episodes": env.num_envs, "episode_s": args_cli.episode_s, "cmd_vx_range": [lo, hi],
                "gait_freq_range": [flo, fhi], "init_joint_noise": env_cfg.init_joint_noise, "seed": args_cli.seed},
        "gait_params": cfg.to_dict()["gait"],
        "nominal": aggregate(ep),
        "by_cmd_vx": binned(ep, "cmd_vx", [lo, lo + (hi - lo) / 3, lo + 2 * (hi - lo) / 3, hi]),
        "by_gait_freq": binned(ep, "gait_freq", [flo, flo + (fhi - flo) / 3, flo + 2 * (fhi - flo) / 3, fhi]),
        "torque_Nm": torque_stats(ep, env.rb.LEG_JOINT_NAMES),
    }
    stats["nominal"]["passes_gate"] = stats["nominal"]["success_rate"] >= 0.70
    torch.save({k: v.cpu() for k, v in ep.items() if k != "torque_samples"}, out_dir / "episodes_nominal.pt")
    if args_cli.push:
        times = [float(t) for t in args_cli.push_times.split(",")]
        ep_push = run_first_episodes(env, ctrl, push=make_push(args_cli.push_speed, times))
        stats["push_test"] = {"push_speed_mps": args_cli.push_speed, "push_times_s": times,
                              "note": "reference only, not part of the 70 % gate", **aggregate(ep_push)}
        torch.save({k: v.cpu() for k, v in ep_push.items()}, out_dir / "episodes_push.pt")
    stats["wall_time_s"] = round(time.time() - t0, 1)
    (out_dir / "stats.json").write_text(json.dumps(stats, indent=2), encoding="utf-8")
    n = stats["nominal"]
    print(f"[eval] success {n['success_rate']:.3f} fall {n['fall_rate']:.3f} dist {n['mean_distance_m']:.3f} m "
          f"vx {n['mean_vx']:.3f} (cmd {n['mean_cmd_vx']:.3f}) lat {n['mean_max_abs_lateral_m']:.3f} m "
          f"heading {n['mean_max_abs_heading_err_deg']:.1f} deg | gate {'PASS' if n['passes_gate'] else 'FAIL'}")
    for k, v in stats["by_cmd_vx"].items():
        print(f"[eval]   cmd {k}: success {v['success_rate']} fall {v['fall_rate']} dist {v['mean_distance_m']}")
    if args_cli.push:
        p = stats["push_test"]
        print(f"[eval] push {args_cli.push_speed} m/s: success {p['success_rate']:.3f} fall {p['fall_rate']:.3f}")
    print(f"[eval] output: {out_dir}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
