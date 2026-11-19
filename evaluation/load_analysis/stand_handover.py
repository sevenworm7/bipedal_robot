r"""Deploy check (2026-11-19): stand in a fixed stance (zero action = default joint targets, policy NOT running) for
``--stand_s``, then hand over to the policy with a random walking command (0.05-0.15 m/s, 1.2-2.0 Hz) for ``--walk_s``.

Why: robot v2 student v2ft presses its feet together under the stand command (left hip roll ~0.41 N m, 141% of rated),
so the real robot stands with a fixed stance and starts the policy only to walk. Prints the standing torque / foot
spacing and the walking result (success = alive, distance within +-25% of the command, max heading error < 10 deg after 1 s).

Usage (robot v2):
    set CUSTOM_BIPED_VERSION=2
    tools\isaacpy.cmd evaluation\load_analysis\stand_handover.py --agent student:<student.pt> --scenario dr_v1
"""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "control")]
from isaaclab.app import AppLauncher
import argparse
p = argparse.ArgumentParser()
p.add_argument("--agent", required=True); p.add_argument("--scenario", default="flat_v1")
p.add_argument("--stand_s", type=float, default=3.0); p.add_argument("--walk_s", type=float, default=20.0)
AppLauncher.add_app_launcher_args(p); a = p.parse_args(); a.headless = True
app = AppLauncher(a).app
import torch, numpy as np, math
from flat_pipeline.config import PipelineCfg
from sim2real.agents import load_agent_v2
from sim2real.common import make_env
torch.manual_seed(42)
cfg = PipelineCfg()
n = 128
env = make_env(cfg, a.scenario, n, a.stand_s + a.walk_s + 2.0, 42, obs_version=2)
agent = load_agent_v2(a.agent, env, cfg.actor, ROOT)
env.reset(); env.completed_episodes.clear(); agent.reset()
dev = env.device
names = list(env.rb.ACTION_JOINT_NAMES)
alive = torch.ones(n, dtype=torch.bool, device=dev)
fell_stand = torch.zeros(n, dtype=torch.bool, device=dev)
fell_walk = torch.zeros(n, dtype=torch.bool, device=dev)
env.cmd_vx[:] = 0.0; env.gait_freq[:] = 0.0
env._get_observations()
zero = torch.zeros(n, len(names), device=dev)
tau_stand = []
for k in range(int(round(a.stand_s / env.step_dt))):
    _, _, term, trunc, _ = env.step(zero)
    tau_stand.append(env.leg_torque.abs().clone())
    fell_stand |= term & alive; alive &= ~(term | trunc)
fid = env._foot_body_ids
fp = env.robot.data.body_pos_w.torch[:, fid, :]
dy = (fp[:, 0, 1] - fp[:, 1, 1]).abs() * 1000
T = torch.stack(tau_stand[10:])
rms = T.pow(2).mean(0).sqrt().mean(0)
print("[handover] stand: falls", int(fell_stand.sum()), "foot dy p10/p50/p90 mm", np.round(np.percentile(dy.cpu().numpy(), [10, 50, 90]), 1))
print("[handover] stand RMS tau:", {nm: round(float(v), 3) for nm, v in zip(names, rms)})
# hand over: walking command
vx = torch.empty(n, device=dev).uniform_(0.05, 0.15); fq = torch.empty(n, device=dev).uniform_(1.2, 2.0)
env.cmd_vx[:] = vx; env.gait_freq[:] = fq
obs = env._get_observations()["policy"]
x0 = env.com_pos[:, 0].clone(); y0 = env.com_pos[:, 1].clone()
max_head = torch.zeros(n, device=dev)
steps = int(round(a.walk_s / env.step_dt))
for k in range(steps):
    o, _, term, trunc, _ = env.step(agent.compute_action(obs)); obs = o["policy"]
    fell_walk |= term & alive
    if k * env.step_dt > 1.0:
        max_head = torch.where(alive, torch.maximum(max_head, env.heading_error.abs()), max_head)
    if k == steps - 1:
        dist = env.com_pos[:, 0] - x0; lat = (env.com_pos[:, 1] - y0).abs()
    alive &= ~(term | trunc)
ratio = dist / (vx * a.walk_s)
ok = alive & (ratio - 1).abs().le(0.25) & (max_head < math.radians(10))
v = (~fell_stand) & (~fell_walk)
print(f"[handover] {a.scenario}: n {n}, fell stand {int(fell_stand.sum())}, fell walk {int(fell_walk.sum())}, "
      f"success {ok.float().mean().item():.1%}, speed ratio p50 {ratio[v].median().item():.2f}, "
      f"max heading p50 {math.degrees(max_head[v].median().item()):.1f} deg, lateral p50 {lat[v].median().item():.3f} m, "
      f"speed ok {((ratio - 1).abs().le(0.25) & v).float().mean().item():.1%}, heading ok {((max_head < math.radians(10)) & v).float().mean().item():.1%}")
app.close()
