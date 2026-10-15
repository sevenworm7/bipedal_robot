"""Convert the read-only reference URDF into a USD asset for Isaac Lab.

Usage (from the project root):
    tools\\isaacpy.cmd archive\exp1_reference\\scripts\\convert_ref_biped.py

Output: archive/exp1_reference/shared/robots/usd/ref_biped/usd/<robot_name>/<robot_name>.usda (not in git).
"""

import argparse
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: E402

from shared.robots import ref_biped  # noqa: E402


def main() -> None:
    urdf = ref_biped.prepare_urdf()
    out_dir = ref_biped.BUILD_DIR / "usd"
    if out_dir.exists():
        shutil.rmtree(out_dir)
    cfg = UrdfConverterCfg(
        asset_path=str(urdf),
        usd_dir=str(out_dir),
        fix_base=False,
        merge_fixed_joints=False,
        force_usd_conversion=True,
        collision_from_visuals=False,
        collision_type="Convex Hull",
        self_collision=False,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            target_type="position",
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0),
        ),
    )
    converter = UrdfConverter(cfg)
    print(f"[convert_ref_biped] source URDF : {urdf}")
    print(f"[convert_ref_biped] generated   : {converter.usd_path}")
    print(f"[convert_ref_biped] usd_path()  : {ref_biped.usd_path()}")


if __name__ == "__main__":
    main()
    app.close()
