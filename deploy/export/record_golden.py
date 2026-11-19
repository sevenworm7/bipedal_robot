r"""Record a golden rollout for the firmware's observation builder (deploy/firmware/src/obs.c).

Same sequence as the real robot (decisions #24): fixed stance (zero action, cmd 0, gait frequency 0) for ``--stand_s``,
then the student walks with ``--vx`` / ``--freq`` for ``--walk_s``. Scenario ``flat_v1`` has no observation noise, so the
student observation is a deterministic function of the raw signals recorded here; the C test rebuilds every
observation from the raw signals and compares it (and the policy output) with these values.

Output (into an export folder of export_policy.py): ``golden_rollout.bin``
    int32 steps, int32 envs, int32 J, int32 obs_dim, then steps x envs records of float32:
    ang_vel(3) gravity(3) heading_error cmd_vx gait_freq gait_phase clearance  joint_pos(J, absolute rad)
    joint_vel(J) last_action(J, clipped)  student_obs(obs_dim)  policy_action(J, raw mean)

Usage (robot v2):
    set CUSTOM_BIPED_VERSION=2
    tools\isaacpy.cmd deploy\export\record_golden.py --export outputs\robot_v2\deploy\export_<...>
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "control")]
from isaaclab.app import AppLauncher  # noqa: E402
import argparse  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument("--export", required=True)
p.add_argument("--student", default="outputs/robot_v2/control/sim2real/student_v2ft_20261118_230657/student.pt")
p.add_argument("--envs", type=int, default=4)
p.add_argument("--stand_s", type=float, default=1.0)
p.add_argument("--walk_s", type=float, default=4.0)
p.add_argument("--vx", type=float, default=0.10)
p.add_argument("--freq", type=float, default=1.6)
AppLauncher.add_app_launcher_args(p)
a = p.parse_args()
a.headless = True
app = AppLauncher(a).app

import json  # noqa: E402
import struct  # noqa: E402

import torch  # noqa: E402

from flat_pipeline.config import PipelineCfg  # noqa: E402
from sim2real.agents import load_agent_v2  # noqa: E402
from sim2real.common import make_env  # noqa: E402

torch.manual_seed(0)
out = (ROOT / a.export).resolve()
dst = out / "golden_rollout.bin"
assert out.is_dir() and not dst.exists(), f"need an export folder without golden_rollout.bin: {out}"
cfg = PipelineCfg()
n = a.envs
env = make_env(cfg, "flat_v1", n, a.stand_s + a.walk_s + 2.0, 0, obs_version=2)
agent = load_agent_v2(f"student:{(ROOT / a.student).as_posix()}", env, cfg.actor, ROOT)
env.reset()
J = len(env.rb.ACTION_JOINT_NAMES)
env.cmd_vx[:] = 0.0
env.gait_freq[:] = 0.0
env._get_observations()

rows = []


def record(action_applied_next: torch.Tensor) -> None:
    o = env.obs_v2["student"]
    act = agent.compute_action(None)
    rows.append(torch.cat((
        env.base_ang_vel, env.projected_gravity, env.heading_error.unsqueeze(-1), env.cmd_vx.unsqueeze(-1),
        env.gait_freq.unsqueeze(-1), env.gait_phase.unsqueeze(-1), env.clearance_cmd.unsqueeze(-1),
        env.leg_pos, env.leg_vel, env.actions, o, act), dim=-1).cpu())
    return act


zero = torch.zeros(n, J, device=env.device)
stand_steps = int(round(a.stand_s / env.step_dt))
walk_steps = int(round(a.walk_s / env.step_dt))
record(zero)  # observation right after reset (history filled with the first frame)
for _ in range(stand_steps):
    env.step(zero)
    record(zero)
env.cmd_vx[:] = a.vx
env.gait_freq[:] = a.freq
env._get_observations()  # pushes one more history frame with the new command (the C test replays every row)
act = record(zero)
alive = torch.ones(n, dtype=torch.bool, device=env.device)
for _ in range(walk_steps):
    _, _, term, trunc, _ = env.step(act)
    alive &= ~(term | trunc)
    act = record(act)
assert bool(alive.all()), "a robot fell or reset during the golden rollout"

data = torch.stack(rows)  # steps, envs, fields
obs_dim = env.obs_v2["student"].shape[1]
with dst.open("wb") as f:
    f.write(struct.pack("<4i", data.shape[0], n, J, obs_dim))
    f.write(data.float().numpy().astype("<f4").tobytes())
meta = json.loads((out / "meta.json").read_text(encoding="utf-8"))
meta["golden_rollout"] = {"scenario": "flat_v1", "envs": n, "steps": data.shape[0], "stand_s": a.stand_s,
                          "walk_s": a.walk_s, "vx": a.vx, "freq": a.freq,
                          "distance_m": [round(float(x), 3) for x in env.com_pos[:, 0]]}
(out / "meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
print(f"[golden] {dst} steps {data.shape[0]} envs {n} distance {meta['golden_rollout']['distance_m']}")
app.close()
