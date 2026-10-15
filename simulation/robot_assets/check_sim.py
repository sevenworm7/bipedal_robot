"""In-sim sanity checks of a model version (Isaac Lab), after convert_usd.py.

Checks (-> outputs/robot_v<N>/simulation/robot_check/<YYYYmmdd_HHMMSS>/report.json + png):
  1. axes  : base held in place; each of the 10 joints driven +probe rad from the stance (right hip_roll: -probe,
             outward, to avoid the other foot); reports how the foot frame (ankles: toe, tail: tip) moves in the
             world frame. Expected: hip_roll + -> foot left,
             hip_pitch / knee + -> foot back, ankle + -> toe down, tail_yaw + -> tip right, tail_pitch + -> tip up.
  2. stand : free robot on the ground holding the stance for --stand_time s: base height drift < 2 cm,
             tilt < 5 deg, no fall; max |torque| per joint vs the 6 V stall torque.
  3. snapshots (needs --enable_cameras).

Usage:
    tools\\isaacpy.cmd hardware\\model\\generator\\check_sim.py --version 1 --enable_cameras
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--version", type=int, required=True)
parser.add_argument("--stand_time", type=float, default=5.0)
parser.add_argument("--probe", type=float, default=0.3, help="joint probe [rad]")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
os.environ["CUSTOM_BIPED_VERSION"] = str(args_cli.version)
app = AppLauncher(args_cli).app

import cv2  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.utils.math import quat_apply  # noqa: E402

from core.robots import custom_biped as cb  # noqa: E402

SIM_DT = 0.005
OUT_DIR = cb.TEST_ROOT / "simulation" / "robot_check" / time.strftime("%Y%m%d_%H%M%S")
FIXED_ORIGIN = (0.0, -0.6, 0.35)
FREE_ORIGIN = (0.0, 0.6, 0.0)


def build_scene():
    ground = sim_utils.GroundPlaneCfg(
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0, restitution=0.0))
    ground.func("/World/ground", ground)
    light = sim_utils.DomeLightCfg(intensity=2500.0, color=(0.8, 0.8, 0.8))
    light.func("/World/light", light)
    fixed_cfg = cb.make_articulation_cfg("/World/RobotFixed")
    fixed_cfg.init_state.pos = FIXED_ORIGIN
    free_cfg = cb.make_articulation_cfg("/World/RobotFree")
    free_cfg.init_state.pos = (FREE_ORIGIN[0], FREE_ORIGIN[1], cb.INIT_BASE_HEIGHT)
    camera = None
    if args_cli.enable_cameras:
        from isaaclab.sensors.camera import Camera, CameraCfg
        camera = Camera(CameraCfg(prim_path="/World/cam", update_period=0, height=720, width=1280, data_types=["rgb"],
                                  spawn=sim_utils.PinholeCameraCfg(focal_length=24.0, clipping_range=(0.02, 50.0))))
    return Articulation(fixed_cfg), Articulation(free_cfg), camera


def snapshot(sim, camera, eye, target, name):
    if camera is None:
        return None
    dev = sim.device
    camera.set_world_poses_from_view(torch.tensor([eye], device=dev), torch.tensor([target], device=dev))
    for _ in range(10):
        sim.render()
    camera.update(dt=0.0)
    rgb = camera.data.output["rgb"][0, ..., :3].cpu().numpy()
    path = OUT_DIR / f"{name}.png"
    cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    return str(path.relative_to(cb.PROJECT_ROOT))


def step(sim, robots, n):
    fixed = robots[0]
    for _ in range(n):
        fixed.write_root_pose_to_sim_index(root_pose=fixed.data.default_root_pose.torch)
        fixed.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(fixed.data.default_root_vel.torch))
        for r in robots:
            r.write_data_to_sim()
        sim.step()
        for r in robots:
            r.update(SIM_DT)


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=False)
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=SIM_DT, device=args_cli.device))
    fixed, free, camera = build_scene()
    sim.reset()
    robots = (fixed, free)
    report = {"sim_dt": SIM_DT, "robot": cb.ROBOT_NAME, "version": cb.ROBOT_VERSION,
              "total_mass_kg_urdf": cb.TOTAL_MASS}
    names = fixed.joint_names
    report["joint_names_sim_order"] = names
    report["body_names"] = fixed.body_names
    report["sim_total_mass_kg"] = round(float(fixed.data.body_mass.torch[0].sum()), 4)
    foot_ids, _ = fixed.find_bodies(list(cb.FOOT_BODY_NAMES), preserve_order=True)
    tail_id = fixed.find_bodies(["tail_pitch_link"])[0][0]

    default = fixed.data.default_joint_pos.torch.clone()
    for r in robots:
        r.set_joint_position_target_index(target=default)
    step(sim, robots, 100)

    def tip():
        q = fixed.data.body_quat_w.torch[0:1, tail_id]
        p = fixed.data.body_pos_w.torch[0, tail_id]
        return p + quat_apply(q, torch.tensor([[-cb.TAIL_LENGTH, 0.0, 0.0]], device=sim.device))[0]

    def point(jname):
        """Tail tip for tail joints, toe (front edge of the sole) for ankles, foot frame otherwise."""
        if jname.startswith("tail"):
            return tip()
        fid = foot_ids[0 if jname.startswith("left") else 1]
        p = fixed.data.body_pos_w.torch[0, fid]
        if "ankle" not in jname:
            return p
        q = fixed.data.body_quat_w.torch[0:1, fid]
        toe = torch.tensor([[cb.FOOT_LENGTH_FWD, 0.0, -cb.ANKLE_HEIGHT]], device=sim.device)
        return p + quat_apply(q, toe)[0]

    fo = FIXED_ORIGIN
    report["snapshots"] = {
        "fixed_side": snapshot(sim, camera, [fo[0] - 0.05, fo[1] - 0.9, fo[2] + 0.05], [fo[0] - 0.1, fo[1], fo[2] - 0.1], "fixed_side"),
        "fixed_front": snapshot(sim, camera, [fo[0] + 0.7, fo[1] - 0.4, fo[2] + 0.1], [fo[0] - 0.05, fo[1], fo[2] - 0.1], "fixed_front"),
    }

    axes = {}
    for jname in cb.ACTUATED_JOINT_NAMES:
        jid = names.index(jname)
        root = fixed.data.root_pos_w.torch[0]
        before = (point(jname) - root).clone()
        target = default.clone()
        # right hip_roll is probed outward (-): + would push the right foot into the left one (5 mm gap)
        sign = -1.0 if jname == "right_hip_roll" else 1.0
        target[0, jid] += sign * args_cli.probe
        fixed.set_joint_position_target_index(target=target[:1], env_ids=[0])
        step(sim, robots, 200)
        d = (point(jname) - fixed.data.root_pos_w.torch[0] - before).tolist()
        axes[jname] = {
            "probe_rad": sign * args_cli.probe,
            "reached_rad": round(float(fixed.data.joint_pos.torch[0, jid] - default[0, jid]), 3),
            "point_delta_world_xyz_mm": [round(v * 1000, 1) for v in d],
            "meaning": ("forward" if d[0] > 0.003 else "backward" if d[0] < -0.003 else "-") + " / "
                       + ("left" if d[1] > 0.003 else "right" if d[1] < -0.003 else "-") + " / "
                       + ("up" if d[2] > 0.003 else "down" if d[2] < -0.003 else "-"),
        }
        fixed.set_joint_position_target_index(target=default[:1], env_ids=[0])
        step(sim, robots, 200)
    report["axis_probe"] = axes

    n_steps = int(args_cli.stand_time / SIM_DT)
    z0 = float(free.data.root_pos_w.torch[0, 2])
    xy0 = free.data.root_pos_w.torch[0, :2].clone()
    max_tilt, max_tau = 0.0, torch.zeros(len(names), device=sim.device)
    z_min, z_max = z0, z0
    for _ in range(n_steps):
        step(sim, robots, 1)
        g = free.data.projected_gravity_b.torch[0]
        max_tilt = max(max_tilt, math.degrees(math.acos(max(-1.0, min(1.0, -float(g[2]))))))
        max_tau = torch.maximum(max_tau, free.data.applied_torque.torch[0].abs())
        z = float(free.data.root_pos_w.torch[0, 2])
        z_min, z_max = min(z_min, z), max(z_max, z)
    z1 = float(free.data.root_pos_w.torch[0, 2])
    xy = free.data.root_pos_w.torch[0, :2] - torch.tensor(FREE_ORIGIN[:2], device=sim.device)
    jerr = (free.data.joint_pos.torch[0] - default[0]).abs()
    stand = {
        "duration_s": args_cli.stand_time, "base_z_start_m": round(z0, 4), "base_z_end_m": round(z1, 4),
        "base_z_range_m": [round(z_min, 4), round(z_max, 4)], "base_xy_drift_since_spawn_m": round(float(xy.norm()), 4),
        "base_xy_drift_during_stand_m": round(float((free.data.root_pos_w.torch[0, :2] - xy0).norm()), 4),
        "max_tilt_deg": round(max_tilt, 2),
        "max_abs_torque_Nm": {n: round(float(t), 3) for n, t in zip(names, max_tau.tolist())},
        "final_joint_error_deg": {n: round(math.degrees(float(e)), 2) for n, e in zip(names, jerr.tolist())},
        "fell": abs(z1 - z0) > 0.05 or max_tilt > 30.0,
    }
    stand["pass"] = (not stand["fell"]) and abs(z1 - z0) < 0.02 and max_tilt < 5.0
    report["stand"] = stand
    fb = [FREE_ORIGIN[0] - 0.05, FREE_ORIGIN[1], 0.15]
    report["snapshots"]["stand_side"] = snapshot(sim, camera, [fb[0], fb[1] + 0.9, 0.25], fb, "stand_side")
    report["snapshots"]["stand_front"] = snapshot(sim, camera, [fb[0] + 0.7, fb[1] + 0.35, 0.3], fb, "stand_front")
    report["snapshots"]["stand_iso"] = snapshot(sim, camera, [fb[0] - 0.55, fb[1] + 0.55, 0.45], fb, "stand_iso")

    (OUT_DIR / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({k: report[k] for k in ("axis_probe", "stand")}, indent=1, ensure_ascii=False))
    print(f"[check_sim] report: {OUT_DIR / 'report.json'}")


if __name__ == "__main__":
    main()
    app.close()
