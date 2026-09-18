"""In-sim sanity checks for the reference biped (phase A of 00_test).

Checks (results -> 00_test/outputs/robot_check/<date>/report.json + png snapshots):
  1. axes  : fixed-base robot; each leg joint driven to +0.3 rad; reports the resulting foot
             displacement in the world frame (+X = intended forward, +Y = left, +Z = up).
  2. stand : free robot on the ground holding DEFAULT_JOINT_POS for --stand_time seconds;
             reports base height drift, max tilt, max |torque| per joint, fall flag.
  3. snapshots of both robots (needs --enable_cameras).

Usage:
    tools\\isaacpy.cmd 00_test\\scripts\\check_ref_biped.py --enable_cameras
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--stand_time", type=float, default=5.0, help="Standing test duration [s].")
parser.add_argument("--probe", type=float, default=0.3, help="Joint probe angle for the axis test [rad].")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import cv2  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402

from shared.robots import ref_biped as rb  # noqa: E402

SIM_DT = 0.005
OUT_DIR = rb.TEST_ROOT / "outputs" / "robot_check" / time.strftime("%Y%m%d_%H%M%S")
FIXED_ORIGIN = (0.0, -1.0, 0.6)
FREE_ORIGIN = (0.0, 1.0, 0.0)


def build_scene():
    ground = sim_utils.GroundPlaneCfg(
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0, restitution=0.0)
    )
    ground.func("/World/ground", ground)
    light = sim_utils.DomeLightCfg(intensity=2500.0, color=(0.8, 0.8, 0.8))
    light.func("/World/light", light)

    # fix_root_link=True clashes with the layered USD from the 3.0 URDF importer (two
    # ArticulationRootAPI prims), so the "fixed" robot is instead held in place every step.
    fixed_cfg = rb.make_articulation_cfg("/World/RobotFixed")
    fixed_cfg.init_state.pos = FIXED_ORIGIN
    free_cfg = rb.make_articulation_cfg("/World/RobotFree")
    free_cfg.init_state.pos = (FREE_ORIGIN[0], FREE_ORIGIN[1], rb.INIT_BASE_HEIGHT)

    camera = None
    if args_cli.enable_cameras:
        from isaaclab.sensors.camera import Camera, CameraCfg

        camera = Camera(
            CameraCfg(
                prim_path="/World/cam",
                update_period=0,
                height=720,
                width=1280,
                data_types=["rgb"],
                spawn=sim_utils.PinholeCameraCfg(focal_length=24.0, clipping_range=(0.05, 100.0)),
            )
        )
    return Articulation(fixed_cfg), Articulation(free_cfg), camera


def snapshot(sim, camera, eye, target, name):
    if camera is None:
        return None
    dev = sim.device
    camera.set_world_poses_from_view(torch.tensor([eye], device=dev), torch.tensor([target], device=dev))
    for _ in range(10):  # let the RTX renderer converge
        sim.render()
    camera.update(dt=0.0)
    rgb = camera.data.output["rgb"][0, ..., :3].cpu().numpy()
    path = OUT_DIR / f"{name}.png"
    cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    return str(path.relative_to(rb.PROJECT_ROOT))


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
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=SIM_DT, device=args_cli.device))
    fixed, free, camera = build_scene()
    sim.reset()
    robots = (fixed, free)
    report = {"sim_dt": SIM_DT, "robot": rb.ROBOT_NAME, "forward_axis": rb.FORWARD_AXIS}

    joint_names = fixed.joint_names
    leg_ids, _ = fixed.find_joints(list(rb.LEG_JOINT_NAMES), preserve_order=True)
    foot_ids, _ = fixed.find_bodies(list(rb.FOOT_BODY_NAMES), preserve_order=True)
    report["joint_names_sim_order"] = joint_names
    report["body_names"] = fixed.body_names

    default = fixed.data.default_joint_pos.torch.clone()
    for r in robots:
        r.set_joint_position_target_index(target=default)
    step(sim, robots, 100)

    # ---- zero-pose geometry -------------------------------------------------------
    base_w = fixed.data.root_pos_w.torch[0]
    feet_w = fixed.data.body_pos_w.torch[0, foot_ids]
    report["zero_pose"] = {
        "foot_frames_rel_base_world": {
            n: [round(v, 4) for v in (feet_w[i] - base_w).tolist()] for i, n in enumerate(rb.FOOT_BODY_NAMES)
        },
        "note": "left foot should have +Y (left) offset after the root yaw; otherwise the forward axis sign is wrong",
    }
    report["snapshots"] = {
        "fixed_side": snapshot(sim, camera, [0.0, -2.2, 0.8], list(FIXED_ORIGIN[:2]) + [0.35], "fixed_side"),
        "fixed_front": snapshot(sim, camera, [1.4, -1.0, 0.7], list(FIXED_ORIGIN[:2]) + [0.35], "fixed_front"),
    }

    # ---- 1. joint axis probe --------------------------------------------------------
    axes = {}
    for jid in leg_ids:
        name = joint_names[jid]
        side = 0 if "Left" in name else 1
        before = (fixed.data.body_pos_w.torch[0, foot_ids[side]] - fixed.data.root_pos_w.torch[0]).clone()
        target = default.clone()
        target[0, jid] += args_cli.probe
        fixed.set_joint_position_target_index(target=target[:1], env_ids=[0])
        step(sim, robots, 200)
        after = fixed.data.body_pos_w.torch[0, foot_ids[side]] - fixed.data.root_pos_w.torch[0]
        d = (after - before).tolist()
        axes[name] = {
            "reached_rad": round(float(fixed.data.joint_pos.torch[0, jid]), 3),
            "foot_delta_world_xyz_m": [round(v, 4) for v in d],
            "meaning": ("foot moves " + ("forward" if d[0] > 0.005 else "backward" if d[0] < -0.005 else "-")
                        + " / " + ("left" if d[1] > 0.005 else "right" if d[1] < -0.005 else "-")
                        + " / " + ("up" if d[2] > 0.005 else "down" if d[2] < -0.005 else "-")),
        }
        fixed.set_joint_position_target_index(target=default[:1], env_ids=[0])
        step(sim, robots, 200)
    report["axis_probe"] = axes

    # ---- 2. standing test ------------------------------------------------------------
    n_steps = int(args_cli.stand_time / SIM_DT)
    z0 = float(free.data.root_pos_w.torch[0, 2])
    max_tilt, max_tau = 0.0, torch.zeros(len(joint_names), device=sim.device)
    z_min, z_max = z0, z0
    for _ in range(n_steps):
        step(sim, robots, 1)
        g = free.data.projected_gravity_b.torch[0]
        max_tilt = max(max_tilt, math.degrees(math.acos(max(-1.0, min(1.0, -float(g[2]))))))
        max_tau = torch.maximum(max_tau, free.data.applied_torque.torch[0].abs())
        z = float(free.data.root_pos_w.torch[0, 2])
        z_min, z_max = min(z_min, z), max(z_max, z)
    z1 = float(free.data.root_pos_w.torch[0, 2])
    xy_drift = free.data.root_pos_w.torch[0, :2] - torch.tensor(FREE_ORIGIN[:2], device=sim.device)
    stand = {
        "duration_s": args_cli.stand_time,
        "base_z_start_m": round(z0, 4),
        "base_z_end_m": round(z1, 4),
        "base_z_range_m": [round(z_min, 4), round(z_max, 4)],
        "base_xy_drift_m": round(float(xy_drift.norm()), 4),
        "max_tilt_deg": round(max_tilt, 2),
        "max_abs_torque_Nm": {n: round(float(t), 2) for n, t in zip(joint_names, max_tau.tolist())},
        "fell": abs(z1 - z0) > 0.05 or max_tilt > 30.0,
    }
    stand["pass"] = (not stand["fell"]) and abs(z1 - z0) < 0.02 and max_tilt < 5.0
    report["stand"] = stand
    fb = [FREE_ORIGIN[0], FREE_ORIGIN[1], 0.25]
    report["snapshots"]["stand_side"] = snapshot(sim, camera, [0.0, 2.8, 0.6], fb, "stand_side")
    report["snapshots"]["stand_front"] = snapshot(sim, camera, [1.6, 1.0, 0.5], fb, "stand_front")
    # close-up from the robot's left side: image left = world +X = intended forward
    report["snapshots"]["stand_side_close"] = snapshot(
        sim, camera, [0.0, 1.75, 0.18], [0.0, 1.0, 0.15], "stand_side_close"
    )

    (OUT_DIR / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"[check_ref_biped] report: {OUT_DIR / 'report.json'}")


if __name__ == "__main__":
    main()
    app.close()
