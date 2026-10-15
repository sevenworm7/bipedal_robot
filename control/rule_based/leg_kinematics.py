"""Planar (sagittal) two-link leg kinematics, robot-agnostic.

Conventions (canonical frame, see ``core/interface.py``): x forward, z up, angles are right-handed
rotations about the canonical +y axis, so positive hip pitch swings the leg backward, positive knee
flexes the shin backward (human-like) and positive ankle points the toes down. The ankle position
is expressed relative to the hip pitch axis in the pelvis frame.

    thigh angle (forward from straight down)  phi1 = -hip
    shin angle                                phi2 = phi1 - knee
    ankle x =  L1 sin(phi1) + L2 sin(phi2)
    ankle z = -(L1 cos(phi1) + L2 cos(phi2))
    foot pitch (relative to the pelvis) = hip + knee + ankle
"""

from __future__ import annotations

import math

import torch


class SagittalLegIK:
    """Closed-form IK/FK for the thigh-shin chain of a robot module (``THIGH_LENGTH``, ``SHIN_LENGTH``)."""

    def __init__(self, rb, knee_margin: float = 0.95, min_knee: float = 0.08):
        self.l1 = float(rb.THIGH_LENGTH)
        self.l2 = float(rb.SHIN_LENGTH)
        sign = rb.JOINT_AXIS_SIGN["knee"]
        lo, hi = (v / sign for v in rb.LEG_JOINT_LIMITS["knee"])
        self.knee_max = min(max(lo, hi), math.pi) * knee_margin
        self.knee_min = min_knee
        self.r_min = self.reach(self.knee_max)
        self.r_max = self.reach(self.knee_min)

    def reach(self, knee: float) -> float:
        """Hip-to-ankle distance [m] for a knee angle [rad]."""
        return math.sqrt(self.l1**2 + self.l2**2 + 2 * self.l1 * self.l2 * math.cos(knee))

    def ik(self, x: torch.Tensor, z: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Ankle position (x, z) relative to the hip pitch axis -> (hip, knee, reach_error).

        Targets outside the reachable annulus are scaled radially onto it; ``reach_error`` [m] is
        the distance between the requested and the realised ankle position.
        """
        r = torch.sqrt(x * x + z * z).clamp_min(1e-6)
        rc = r.clamp(self.r_min, self.r_max)
        scale = rc / r
        xc, zc = x * scale, z * scale
        cos_k = (rc * rc - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        knee = torch.acos(cos_k.clamp(-1.0, 1.0))
        gamma = torch.atan2(xc, -zc)
        beta = torch.atan2(self.l2 * torch.sin(knee), self.l1 + self.l2 * torch.cos(knee))
        hip = -(gamma + beta)
        return hip, knee, (r - rc).abs()

    def fk(self, hip: torch.Tensor, knee: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        phi1 = -hip
        phi2 = phi1 - knee
        x = self.l1 * torch.sin(phi1) + self.l2 * torch.sin(phi2)
        z = -(self.l1 * torch.cos(phi1) + self.l2 * torch.cos(phi2))
        return x, z
