"""Convert a model version's URDF into the USD asset used by Isaac Lab.

Usage (Isaac Lab Python, from the project root):
    tools\\isaacpy.cmd 01_robot_construction\\model\\generator\\convert_usd.py --version 1

Output: shared/robots/usd/custom_biped_v<N>/usd/<robot>/<robot>.usda (not in git).
PD gains are set to 0 here; the actuators in shared/robots/custom_biped.py own them.
"""

import argparse
import os
import shutil
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT_ROOT))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--version", type=int, required=True)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
os.environ["CUSTOM_BIPED_VERSION"] = str(args_cli.version)
app = AppLauncher(args_cli).app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: E402

from shared.robots import custom_biped as cb  # noqa: E402


def main() -> None:
    out_dir = cb.BUILD_DIR / "usd"
    if out_dir.exists():
        shutil.rmtree(out_dir)
    cfg = UrdfConverterCfg(
        asset_path=str(cb.URDF_PATH),
        usd_dir=str(out_dir),
        fix_base=False,
        merge_fixed_joints=False,
        force_usd_conversion=True,
        collision_from_visuals=False,
        collision_type="Convex Hull",
        self_collision=True,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            target_type="position",
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0),
        ),
    )
    converter = UrdfConverter(cfg)
    print(f"[convert_usd] source URDF : {cb.URDF_PATH}")
    print(f"[convert_usd] generated   : {converter.usd_path}")
    print(f"[convert_usd] usd_path()  : {cb.usd_path()}")


if __name__ == "__main__":
    main()
    app.close()
