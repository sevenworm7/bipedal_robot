"""Observation / action contract shared by the rule-based controller and every learned policy.

This is the single source of truth for the interface (see project CLAUDE.md, 跨流程介面契約).
Everything here is robot-agnostic; robot-specific numbers come from ``shared/robots/<robot>.py``.
Experiment 2 copy of ``00_test/shared/interface.py``: same structure and order rules; the joint block covers
the robot's ``ACTION_JOINT_NAMES`` (legs, then any other actuated joints such as the tail).

Canonical base frame
    All base-frame vectors are expressed in the *canonical* frame: +x forward (walking
    direction), +y left, +z up, attached to the base. The URDF root frame of a robot may differ
    (the reference biped walks along its URDF +Y); :func:`root_to_canonical` rotates root-frame
    vectors by the robot's ``INIT_YAW``.

Observation vector (float32, SI units, **no normalisation** — learned policies must apply their own
running normalisation, identically in BC and PPO):

    ====  =====================  ====  ============================================
    idx   name                   dim   meaning
    ====  =====================  ====  ============================================
    0     base_lin_vel           3     base CoM linear velocity, canonical frame [m/s]
    3     base_ang_vel           3     base angular velocity, canonical frame [rad/s]
    6     projected_gravity      3     unit gravity vector in canonical frame [-]
    9     cmd_vx                 1     commanded forward speed [m/s]
    10    gait_freq              1     commanded gait cycle frequency [Hz] (one cycle = two steps)
    11    heading_error          1     wrap(target_heading - current_heading) [rad]
    12    joint_pos_rel          J     joint position minus default pose [rad]
    12+J  joint_vel              J     joint velocity [rad/s]
    12+2J last_action            J     previous action (normalised, see below) [-]
    12+3J gait_phase             2     (sin, cos) of the gait clock phase [-]
    ====  =====================  ====  ============================================

    J = len(ACTION_JOINT_NAMES): 10 for ``custom_biped`` (8 leg + tail_yaw, tail_pitch) -> total 14 + 3J = 44.
    (Experiment 1, ``ref_biped``: J = 8 leg joints -> 38.)

    ``cmd_vx`` and ``gait_freq`` together are the command: step length = cmd_vx / (2 gait_freq).
    ``gait_freq`` was added on 2026-09-19 (user decision): without it a memoryless policy sees only the
    clock position, not its rate, and cannot tell which step length the teacher would take.

Action vector (float32, J): normalised joint position offsets, clipped to ``[-ACTION_CLIP, ACTION_CLIP]``:

    q_target = q_default + ACTION_SCALE * action          (ACTION_JOINT_NAMES order)

Joint order is the robot module's ``ACTION_JOINT_NAMES`` everywhere (never the simulator order).
Control rate: ``CONTROL_DT`` (physics ``SIM_DT`` x ``DECIMATION``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch

SIM_DT = 0.005
"""Physics step [s]."""
DECIMATION = 4
"""Physics steps per control step."""
CONTROL_DT = SIM_DT * DECIMATION
"""Control period [s] (50 Hz)."""

ACTION_SCALE = 0.25
"""[rad] per unit action."""
ACTION_CLIP = 5.0


@dataclass(frozen=True)
class ObsLayout:
    """Index slices of the observation vector for a robot with ``num_joints`` action joints."""

    num_joints: int

    @property
    def dim(self) -> int:
        return 14 + 3 * self.num_joints

    def slices(self) -> dict[str, slice]:
        j = self.num_joints
        return {
            "base_lin_vel": slice(0, 3),
            "base_ang_vel": slice(3, 6),
            "projected_gravity": slice(6, 9),
            "cmd_vx": slice(9, 10),
            "gait_freq": slice(10, 11),
            "heading_error": slice(11, 12),
            "joint_pos_rel": slice(12, 12 + j),
            "joint_vel": slice(12 + j, 12 + 2 * j),
            "last_action": slice(12 + 2 * j, 12 + 3 * j),
            "gait_phase": slice(12 + 3 * j, 14 + 3 * j),
        }

    def split(self, obs: torch.Tensor) -> dict[str, torch.Tensor]:
        """Split a batch of observations ``[N, dim]`` into named views."""
        return {k: obs[..., s] for k, s in self.slices().items()}


def wrap_angle(x: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(x), torch.cos(x))


def root_to_canonical(vec_root: torch.Tensor, init_yaw: float) -> torch.Tensor:
    """Rotate root-frame vectors ``[..., 3]`` into the canonical base frame (yaw by ``init_yaw``)."""
    c, s = math.cos(init_yaw), math.sin(init_yaw)
    x, y, z = vec_root[..., 0], vec_root[..., 1], vec_root[..., 2]
    return torch.stack((c * x - s * y, s * x + c * y, z), dim=-1)


def build_observation(
    *,
    base_lin_vel: torch.Tensor,
    base_ang_vel: torch.Tensor,
    projected_gravity: torch.Tensor,
    cmd_vx: torch.Tensor,
    gait_freq: torch.Tensor,
    heading_error: torch.Tensor,
    joint_pos_rel: torch.Tensor,
    joint_vel: torch.Tensor,
    last_action: torch.Tensor,
    gait_phase: torch.Tensor,
) -> torch.Tensor:
    """Assemble the observation in the documented order.

    Args:
        base_lin_vel: canonical-frame base CoM velocity [m/s], shape [N, 3].
        base_ang_vel: canonical-frame base angular velocity [rad/s], shape [N, 3].
        projected_gravity: canonical-frame unit gravity, shape [N, 3].
        cmd_vx: commanded forward speed [m/s], shape [N].
        gait_freq: commanded gait cycle frequency [Hz], shape [N].
        heading_error: target minus current heading [rad], shape [N].
        joint_pos_rel: joint positions minus default [rad], shape [N, J].
        joint_vel: joint velocities [rad/s], shape [N, J].
        last_action: previous normalised action, shape [N, J].
        gait_phase: gait clock phase in [0, 1), shape [N].
    """
    phase = 2.0 * math.pi * gait_phase
    return torch.cat(
        (
            base_lin_vel,
            base_ang_vel,
            projected_gravity,
            cmd_vx.unsqueeze(-1),
            gait_freq.unsqueeze(-1),
            heading_error.unsqueeze(-1),
            joint_pos_rel,
            joint_vel,
            last_action,
            torch.stack((torch.sin(phase), torch.cos(phase)), dim=-1),
        ),
        dim=-1,
    )


def action_to_joint_targets(action: torch.Tensor, default_pos: torch.Tensor) -> torch.Tensor:
    """Normalised action ``[N, J]`` -> joint position targets [rad]."""
    return default_pos + ACTION_SCALE * action.clamp(-ACTION_CLIP, ACTION_CLIP)


def joint_targets_to_action(q_target: torch.Tensor, default_pos: torch.Tensor) -> torch.Tensor:
    """Joint position targets [rad] ``[N, J]`` -> normalised action (inverse of the above, clipped)."""
    return ((q_target - default_pos) / ACTION_SCALE).clamp(-ACTION_CLIP, ACTION_CLIP)
