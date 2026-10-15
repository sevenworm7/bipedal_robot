"""Left-right mirror (y -> -y) of observations and actions (control/sim2real, against the step-to gait of 2026-10-06).

Robot-agnostic: joint pairs come from ``LEG_JOINT_LAYOUT`` (left / right swap) and signs from the robot module's
``MIRROR_SIGN``; non-leg joints (tail) mirror onto themselves. Canonical base frame (x forward, y left, z up):

* vectors (lin vel, gravity): (x, y, z) -> (x, -y, z); angular velocity (pseudovector): (x, y, z) -> (-x, y, -z)
* heading error -> -heading error; speed / frequency / clearance commands unchanged
* gait clock: the left leg's phase is the clock and the right leg's is clock + 0.5, so swapping the legs shifts the clock
  by half a cycle: (sin, cos) -> (-sin, -cos)
* privileged scalars unchanged; the two foot-contact flags swap

``Mirror.obs_v1`` / ``teacher`` / ``student`` / ``actions`` follow ``core/interface.py`` and ``core/interface_v2.py``.
Mirroring twice is the identity (checked by ``Mirror.self_test``).
"""

from __future__ import annotations

import torch

from . import interface_v2 as itf2


class Mirror:
    def __init__(self, rb, device):
        names = list(rb.ACTION_JOINT_NAMES)
        leg_names, layout = list(rb.LEG_JOINT_NAMES), list(rb.LEG_JOINT_LAYOUT)
        side_role = {leg_names[i]: sr for i, sr in enumerate(layout)}
        perm, sign = [], []
        for n in names:
            if n in side_role:
                side, role = side_role[n]
                other = leg_names[layout.index(("right" if side == "left" else "left", role))]
                perm.append(names.index(other))
                sign.append(rb.MIRROR_SIGN[role])
            else:
                perm.append(names.index(n))
                sign.append(rb.MIRROR_SIGN[n])
        self.perm = torch.tensor(perm, device=device)
        self.sign = torch.tensor(sign, device=device)
        self.lay = itf2.LayoutV2(len(names))

    def joints(self, x: torch.Tensor) -> torch.Tensor:
        return x[..., self.perm] * self.sign

    actions = joints

    def obs_v1(self, o: torch.Tensor) -> torch.Tensor:
        s = self.lay.v1.slices()
        m = o.clone()
        m[..., s["base_lin_vel"]] = o[..., s["base_lin_vel"]] * o.new_tensor([1.0, -1.0, 1.0])
        m[..., s["base_ang_vel"]] = o[..., s["base_ang_vel"]] * o.new_tensor([-1.0, 1.0, -1.0])
        m[..., s["projected_gravity"]] = o[..., s["projected_gravity"]] * o.new_tensor([1.0, -1.0, 1.0])
        m[..., s["heading_error"]] = -o[..., s["heading_error"]]
        for k in ("joint_pos_rel", "joint_vel", "last_action"):
            m[..., s[k]] = self.joints(o[..., s[k]])
        m[..., s["gait_phase"]] = -o[..., s["gait_phase"]]
        return m

    def teacher(self, o: torch.Tensor) -> torch.Tensor:
        v1 = self.lay.v1.dim
        m = o.clone()
        m[..., :v1] = self.obs_v1(o[..., :v1])
        fc = self.lay.priv_slices()["foot_contact"]
        m[..., fc] = o[..., fc].flip(-1)
        return m

    def student(self, o: torch.Tensor) -> torch.Tensor:
        """Student history [..., H * frame]: each frame = v1 obs without base_lin_vel + clearance command."""
        h, f = self.lay.history, self.lay.frame_dim
        x = o.reshape(*o.shape[:-1], h, f)
        pad = torch.zeros(*x.shape[:-1], 3, device=o.device, dtype=o.dtype)  # base_lin_vel placeholder
        v1 = torch.cat((pad, x[..., :-1]), dim=-1)
        out = torch.cat((self.obs_v1(v1)[..., 3:], x[..., -1:]), dim=-1)
        return out.reshape(o.shape)

    def self_test(self, n: int = 8) -> float:
        """max |mirror(mirror(x)) - x| over random teacher / student / action tensors (should be 0)."""
        d = self.perm.device
        errs = []
        for fn, dim in ((self.teacher, self.lay.teacher_dim), (self.student, self.lay.student_dim),
                        (self.actions, self.lay.num_joints)):
            x = torch.randn(n, dim, device=d)
            errs.append(float((fn(fn(x)) - x).abs().max()))
        return max(errs)
