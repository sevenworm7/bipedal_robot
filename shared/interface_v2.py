"""Observation interface v2 (04_sim2real, user decision 2026-10-06): a deployable student and a privileged teacher.

``shared/interface.py`` (v1, 44 dims for custom_biped) is unchanged and still used by 02 / 03. v2 adds three groups,
all float32, SI units, no normalisation (policies normalise their own inputs). J = len(ACTION_JOINT_NAMES) = 10.

``teacher`` (privileged; also the critic input) = v1 observation (14 + 3J) followed by :data:`PRIV_FIELDS`::

    idx      name               dim   meaning
    44       foot_friction      1     mean static friction of the foot shapes [-]
    45       mass_scale         1     total robot mass / nominal [-]
    46       kp_scale           1     actuator stiffness scale [-]
    47       kd_scale           1     actuator damping scale [-]
    48       effort_scale       1     actuator effort-limit scale [-]
    49       delay_s            1     joint-target delay [s]
    50       terrain_amp        1     bump amplitude of the current tile [m]
    51       base_height        1     base CoM height above the spawn origin [m]
    52       foot_contact       2     left / right foot in contact [0/1]
    54       clearance_cmd      1     commanded swing-foot apex height [m] (0 when gait_var.clearance_range is off)
    -> 14 + 3J + 11 = 55

    The v1 part is the *true* state (no observation noise, no heading bias). Because the first 44 entries are exactly
    the v1 observation, a v1 policy can initialise the teacher by zero-padding its first layer.

``student`` (deployable: IMU + servo readings only) = the last :data:`HISTORY` frames, newest first, each frame::

    idx   name               dim   meaning
    0     base_ang_vel       3     IMU angular velocity, canonical base frame [rad/s]
    3     projected_gravity  3     IMU gravity direction [-]
    6     cmd_vx             1     commanded forward speed [m/s]
    7     gait_freq          1     commanded gait frequency [Hz]
    8     heading_error      1     target heading - IMU heading [rad] (with the per-episode IMU bias)
    9     joint_pos_rel      J     servo positions - default [rad]
    9+J   joint_vel          J     servo velocities [rad/s]
    9+2J  last_action        J     previous normalised action [-]
    9+3J  gait_phase         2     (sin, cos) of the gait clock [-]
    11+3J clearance_cmd      1     commanded swing-foot apex height [m]
    -> frame 12 + 3J = 42, student 5 x 42 = 210

    = the v1 observation without ``base_lin_vel`` (not measurable on the real robot), with the scenario's
    observation noise, plus the clearance command. At reset the history is filled with the first frame.

Standing (``gait_var.stand_prob``) is commanded as cmd_vx = 0 and gait_freq = 0 (the clock stops).

Action, joint order, control rate and action scaling are those of ``shared/interface.py`` (unchanged).
"""

from __future__ import annotations

from dataclasses import dataclass

import torch

from .interface import ObsLayout

HISTORY = 5
"""Student history length [control steps] (0.1 s at 50 Hz)."""

PRIV_FIELDS: tuple[tuple[str, int], ...] = (
    ("foot_friction", 1), ("mass_scale", 1), ("kp_scale", 1), ("kd_scale", 1), ("effort_scale", 1),
    ("delay_s", 1), ("terrain_amp", 1), ("base_height", 1), ("foot_contact", 2), ("clearance_cmd", 1),
)
PRIV_DIM = sum(d for _, d in PRIV_FIELDS)


@dataclass(frozen=True)
class LayoutV2:
    num_joints: int
    history: int = HISTORY

    @property
    def v1(self) -> ObsLayout:
        return ObsLayout(self.num_joints)

    @property
    def teacher_dim(self) -> int:
        return self.v1.dim + PRIV_DIM

    @property
    def frame_dim(self) -> int:
        return self.v1.dim - 3 + 1

    @property
    def student_dim(self) -> int:
        return self.history * self.frame_dim

    def priv_slices(self) -> dict[str, slice]:
        out, i = {}, self.v1.dim
        for name, d in PRIV_FIELDS:
            out[name] = slice(i, i + d)
            i += d
        return out


def student_frame(v1_obs: torch.Tensor, clearance_cmd: torch.Tensor) -> torch.Tensor:
    """v1 observation ``[N, 14+3J]`` + clearance command ``[N]`` -> student frame ``[N, 12+3J]``
    (drops ``base_lin_vel``, the first 3 entries)."""
    return torch.cat((v1_obs[..., 3:], clearance_cmd.unsqueeze(-1)), dim=-1)
