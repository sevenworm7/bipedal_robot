"""Load a model version's params.yaml and derive the zero-pose layout.

Everything here is plain Python (no CAD), so the URDF writer and the static checks can share it.
Units: mm, g, deg (as in params.yaml). World frame at zero pose: +X forward, +Y left, +Z up,
origin on the ground below the hip_roll axes.
"""
from __future__ import annotations

import subprocess
from dataclasses import dataclass, field
from pathlib import Path

import yaml

MODEL_ROOT = Path(__file__).resolve().parents[1]      # hardware/ (robot_v<N>/ per version)
PROJECT_ROOT = MODEL_ROOT.parent

SIDES = {"left": 1, "right": -1}
LEG_ROLES = ("hip_roll", "hip_pitch", "knee", "ankle_pitch")
TAIL_ROLES = ("tail_yaw", "tail_pitch")


def version_dir(version: int) -> Path:
    return MODEL_ROOT / f"robot_v{version}"


def git_commit() -> str:
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=PROJECT_ROOT,
                             capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain"], cwd=PROJECT_ROOT,
                               capture_output=True, text=True, check=True).stdout.strip()
        return out + ("-dirty" if dirty else "")
    except Exception:
        return "unknown"


@dataclass
class Joint:
    name: str
    parent: str
    child: str
    origin: tuple[float, float, float]   # world position at zero pose (mm)
    axis: tuple[float, float, float]
    lower: float                         # deg
    upper: float


@dataclass
class Layout:
    p: dict
    version: int
    hip_roll_z: float
    joints: list[Joint] = field(default_factory=list)

    def joint(self, name: str) -> Joint:
        return next(j for j in self.joints if j.name == name)

    @property
    def link_origin(self) -> dict[str, tuple[float, float, float]]:
        """World position (zero pose) of every link frame; base sits on the hip_roll axis midpoint."""
        out = {"base": (0.0, 0.0, self.hip_roll_z)}
        for j in self.joints:
            out[j.child] = j.origin
        return out


def load(version: int) -> Layout:
    path = version_dir(version) / "params.yaml"
    p = yaml.safe_load(path.read_text(encoding="utf-8"))
    if p["robot"]["version"] != version:
        raise ValueError(f"{path}: robot.version is {p['robot']['version']}, expected {version}")
    lg, tl, jt = p["legs"], p["tail"], p["joints"]
    z_ankle = lg["ankle_height"]
    z_knee = z_ankle + lg["shin"]
    z_hp = z_knee + lg["thigh"]
    z_hr = z_hp + lg["hip_roll_to_pitch"]
    lay = Layout(p=p, version=version, hip_roll_z=z_hr)

    def lim(role):
        return jt[role]["lower"], jt[role]["upper"]

    for side, s in SIDES.items():
        y = s * lg["hip_spacing"] / 2
        chain = [("hip_roll", "base", f"{side}_hip", z_hr),
                 ("hip_pitch", f"{side}_hip", f"{side}_thigh", z_hp),
                 ("knee", f"{side}_thigh", f"{side}_shin", z_knee),
                 ("ankle_pitch", f"{side}_shin", f"{side}_foot", z_ankle)]
        for role, parent, child, z in chain:
            lay.joints.append(Joint(f"{side}_{role}", parent, child, (0.0, y, z),
                                    tuple(jt[role]["axis"]), *lim(role)))
    zt = z_hr + tl["z_rel"]
    lay.joints.append(Joint("tail_yaw", "base", "tail_yaw_link", (tl["yaw_x"], 0.0, zt),
                            tuple(jt["tail_yaw"]["axis"]), *lim("tail_yaw")))
    lay.joints.append(Joint("tail_pitch", "tail_yaw_link", "tail_pitch_link",
                            (tl["yaw_x"] - tl["yaw_to_pitch"], 0.0, zt),
                            tuple(jt["tail_pitch"]["axis"]), *lim("tail_pitch")))
    return lay
