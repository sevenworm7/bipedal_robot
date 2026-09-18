"""Per-episode walking metrics shared by the env, the evaluation scripts and the video HUD.

All quantities are tracked per environment on the GPU and reported when an episode ends.
Distances use the base CoM projected on the ground, relative to the episode start and the
target heading (the "straight line" the robot should follow).
"""

from __future__ import annotations

import torch

EPISODE_KEYS = (
    "steps",  # foot touchdowns (left + right) [-]
    "distance",  # displacement along the target heading [m]
    "lateral",  # signed lateral offset from the straight line at episode end [m]
    "max_abs_lateral",  # [m]
    "time",  # simulated episode time [s]
    "mean_vx",  # distance / time [m/s]
    "cmd_vx",  # [m/s]
    "step_length_cmd",  # nominal step length implied by the command [m]
    "max_abs_heading_error",  # [rad]
    "fell",  # 1.0 if the episode ended by a fall
    "torque_sat_frac",  # fraction of control steps with any leg joint at its effort limit
)


class WalkTracker:
    """Accumulates :data:`EPISODE_KEYS` for ``num_envs`` environments."""

    def __init__(self, num_envs: int, num_joints: int, device: str):
        self.num_envs = num_envs
        self.device = device
        z = lambda: torch.zeros(num_envs, device=device)  # noqa: E731
        self.start_xy = torch.zeros(num_envs, 2, device=device)
        self.dir_xy = torch.zeros(num_envs, 2, device=device)
        self.steps = z()
        self.lateral = z()
        self.distance = z()
        self.max_abs_lateral = z()
        self.max_abs_heading_error = z()
        self.sat_steps = z()
        self.n = z()
        self.torque_abs_sum = torch.zeros(num_envs, num_joints, device=device)
        self.torque_abs_max = torch.zeros(num_envs, num_joints, device=device)

    def reset(self, env_ids: torch.Tensor, start_xy: torch.Tensor, heading: torch.Tensor) -> None:
        self.start_xy[env_ids] = start_xy
        self.dir_xy[env_ids] = torch.stack((torch.cos(heading), torch.sin(heading)), dim=-1)
        for buf in (self.steps, self.lateral, self.distance, self.max_abs_lateral, self.max_abs_heading_error,
                    self.sat_steps, self.n):
            buf[env_ids] = 0.0
        self.torque_abs_sum[env_ids] = 0.0
        self.torque_abs_max[env_ids] = 0.0

    def update(
        self,
        com_xy: torch.Tensor,
        touchdowns: torch.Tensor,
        heading_error: torch.Tensor,
        torque_abs: torch.Tensor,
        torque_saturated: torch.Tensor,
    ) -> None:
        """Advance one control step.

        Args:
            com_xy: base CoM ground position [m], shape [N, 2].
            touchdowns: number of feet that touched down this step, shape [N].
            heading_error: [rad], shape [N].
            torque_abs: |applied torque| of leg joints [N m], shape [N, J].
            torque_saturated: any leg joint at its effort limit, shape [N] (bool).
        """
        d = com_xy - self.start_xy
        self.distance = (d * self.dir_xy).sum(-1)
        self.lateral = self.dir_xy[:, 0] * d[:, 1] - self.dir_xy[:, 1] * d[:, 0]
        self.max_abs_lateral = torch.maximum(self.max_abs_lateral, self.lateral.abs())
        self.max_abs_heading_error = torch.maximum(self.max_abs_heading_error, heading_error.abs())
        self.steps += touchdowns
        self.sat_steps += torque_saturated.float()
        self.torque_abs_sum += torque_abs
        self.torque_abs_max = torch.maximum(self.torque_abs_max, torque_abs)
        self.n += 1.0

    def summary(self, env_ids: torch.Tensor, time_s: torch.Tensor, cmd_vx: torch.Tensor,
                step_length_cmd: torch.Tensor, fell: torch.Tensor) -> dict[str, torch.Tensor]:
        """Episode summary for ``env_ids`` (call before :meth:`reset`)."""
        n = self.n[env_ids].clamp_min(1.0)
        t = time_s.clamp_min(1e-6)
        return {
            "steps": self.steps[env_ids].clone(),
            "distance": self.distance[env_ids].clone(),
            "lateral": self.lateral[env_ids].clone(),
            "max_abs_lateral": self.max_abs_lateral[env_ids].clone(),
            "time": time_s.clone(),
            "mean_vx": self.distance[env_ids] / t,
            "cmd_vx": cmd_vx.clone(),
            "step_length_cmd": step_length_cmd.clone(),
            "max_abs_heading_error": self.max_abs_heading_error[env_ids].clone(),
            "fell": fell.float().clone(),
            "torque_sat_frac": self.sat_steps[env_ids] / n,
            "torque_abs_mean": self.torque_abs_sum[env_ids] / n.unsqueeze(-1),
            "torque_abs_max": self.torque_abs_max[env_ids].clone(),
        }


def rule_based_success(ep: dict[str, torch.Tensor], min_steps: int = 10, distance_coef: float = 0.8) -> torch.Tensor:
    """Episode success for the rule-based gate (user decision 2026-09-18).

    Not fallen, at least ``min_steps`` touchdowns within the episode, and
    ``distance >= distance_coef * steps * step_length_cmd`` using the *actual* number of steps, so
    stepping in place (many short steps) fails the distance bound.
    """
    return (
        (ep["fell"] < 0.5)
        & (ep["steps"] >= min_steps)
        & (ep["distance"] >= distance_coef * ep["steps"] * ep["step_length_cmd"])
    )
