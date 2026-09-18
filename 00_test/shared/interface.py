"""Observation / action contract shared by the rule-based controller and every learned policy.

This is the single source of truth for the interface (see project CLAUDE.md, 跨流程介面契約).
Everything here is robot-agnostic; robot-specific numbers come from ``shared/robots/<robot>.py``.

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
    10    heading_error          1     wrap(target_heading - current_heading) [rad]
    11    joint_pos_rel          J     leg joint position minus default pose [rad]
    11+J  joint_vel              J     leg joint velocity [rad/s]
    11+2J last_action            J     previous action (normalised, see below) [-]
    11+3J gait_phase             2     (sin, cos) of the gait clock phase [-]
    ====  =====================  ====  ============================================

    J = number of leg joints (8 for ``ref_biped``) -> total 13 + 3J = 37.

Action vector (float32, J): normalised joint position offsets, clipped to ``[-ACTION_CLIP, ACTION_CLIP]``:

    q_target = q_default + ACTION_SCALE * action          (leg joints, canonical order)

Joint order is the robot module's ``LEG_JOINT_NAMES`` everywhere (never the simulator order).
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
    """Index slices of the observation vector for a robot with ``num_joints`` leg joints."""

    num_joints: int

    @property
    def dim(self) -> int:
        return 13 + 3 * self.num_joints

    def slices(self) -> dict[str, slice]:
        j = self.num_joints
        return {
            "base_lin_vel": slice(0, 3),
            "base_ang_vel": slice(3, 6),
            "projected_gravity": slice(6, 9),
            "cmd_vx": slice(9, 10),
            "heading_error": slice(10, 11),
            "joint_pos_rel": slice(11, 11 + j),
            "joint_vel": slice(11 + j, 11 + 2 * j),
            "last_action": slice(11 + 2 * j, 11 + 3 * j),
            "gait_phase": slice(11 + 3 * j, 13 + 3 * j),
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
        heading_error: target minus current heading [rad], shape [N].
        joint_pos_rel: leg joint positions minus default [rad], shape [N, J].
        joint_vel: leg joint velocities [rad/s], shape [N, J].
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
            heading_error.unsqueeze(-1),
            joint_pos_rel,
            joint_vel,
            last_action,
            torch.stack((torch.sin(phase), torch.cos(phase)), dim=-1),
        ),
        dim=-1,
    )


def action_to_joint_targets(action: torch.Tensor, default_pos: torch.Tensor) -> torch.Tensor:
    """Normalised action ``[N, J]`` -> leg joint position targets [rad]."""
    return default_pos + ACTION_SCALE * action.clamp(-ACTION_CLIP, ACTION_CLIP)


def joint_targets_to_action(q_target: torch.Tensor, default_pos: torch.Tensor) -> torch.Tensor:
    """Leg joint position targets [rad] ``[N, J]`` -> normalised action (inverse of the above, clipped)."""
    return ((q_target - default_pos) / ACTION_SCALE).clamp(-ACTION_CLIP, ACTION_CLIP)
