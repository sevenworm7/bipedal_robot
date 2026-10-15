"""Agents for interface-v2 policies (``core/interface_v2.py``).

The observation group an actor needs follows from its input size: 44 = v1 (03 models), 55 = privileged teacher
(``policy`` group of a v2 env), 5 x 42 = 210 = deployable student (``student`` group). ``rollout`` helpers pass the
``policy`` tensor to ``compute_action``; :class:`StudentAgent` ignores it and reads ``env.obs_v2["student"]``.
"""

from __future__ import annotations

from pathlib import Path

import torch

from flat_pipeline.policy import PolicyAgent, actor_mean, load_actor


def checkpoint_input_dim(path: str | Path) -> int:
    sd = torch.load(path, map_location="cpu", weights_only=False)["actor_state_dict"]
    return int(sd["mlp.0.weight"].shape[1])


def obs_version_for(spec: str) -> int:
    """Interface version an agent spec needs: ``policy:<ckpt>`` with 44 inputs -> 1, otherwise 2."""
    kind, _, arg = spec.partition(":")
    if kind == "policy" and checkpoint_input_dim(arg) == 44:
        return 1
    return 2 if kind in ("policy", "student") else 1


class StudentAgent:
    """Deployable student: acts on the env's latest ``student`` observation."""

    def __init__(self, actor, env):
        self.actor, self.env = actor, env
        self.last_mean: torch.Tensor | None = None

    def reset(self, env_ids=None) -> None:
        pass

    @torch.no_grad()
    def compute_action(self, obs: torch.Tensor) -> torch.Tensor:
        self.last_mean = actor_mean(self.actor, self.env.obs_v2["student"])
        return self.last_mean


def load_agent_v2(spec: str, env, actor_cfg, root: Path):
    """``policy:<ckpt>`` (44 or 55 inputs, acts on the ``policy`` group), ``student:<ckpt>``, or a 03 ``teacher:`` spec."""
    from flat_pipeline.policy import load_agent

    kind, _, arg = spec.partition(":")
    if kind == "student":
        return StudentAgent(load_actor(arg, actor_cfg, env.device), env)
    if kind == "policy":
        return PolicyAgent(load_actor(arg, actor_cfg, env.device))
    return load_agent(spec, env, actor_cfg, root)
