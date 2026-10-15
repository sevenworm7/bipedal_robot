"""Policy networks and agents shared by every stage.

The BC policy *is* an rsl_rl ``MLPModel`` built with the same arguments as the PPO actor, so a BC
checkpoint loads into PPO with ``load_state_dict`` (keys: ``obs_normalizer._mean/_var/_std/count``,
``mlp.<i>.weight/bias``, ``distribution.std_param``).

Agents expose the interface used by ``simulation/rollout.py``: ``reset(env_ids)`` and
``compute_action(obs) -> action`` (normalised actions, see ``core/interface.py``).
"""

from __future__ import annotations

from pathlib import Path

import torch
from tensordict import TensorDict

from .config import ActorCfg

OBS_GROUP = "policy"


def make_actor(cfg: ActorCfg, obs_dim: int, act_dim: int, device, hidden_dims=None, activation=None):
    """rsl_rl ``MLPModel`` with a Gaussian head, identical to the PPO actor built from ``ppo.py``."""
    from rsl_rl.models import MLPModel

    obs = TensorDict({OBS_GROUP: torch.zeros(1, obs_dim)}, batch_size=[1])
    return MLPModel(
        obs, {"actor": [OBS_GROUP]}, "actor", act_dim,
        hidden_dims=list(hidden_dims or cfg.hidden_dims), activation=activation or cfg.activation,
        obs_normalization=cfg.obs_normalization,
        distribution_cfg={"class_name": "GaussianDistribution", "init_std": cfg.init_std, "std_type": "scalar"},
    ).to(device)


def set_normalizer(actor, mean: torch.Tensor, var: torch.Tensor, count: int) -> None:
    """Load dataset statistics into the actor's ``EmpiricalNormalization``."""
    norm = actor.obs_normalizer
    norm._mean.copy_(mean.reshape(1, -1))
    norm._var.copy_(var.reshape(1, -1))
    norm._std.copy_(var.clamp_min(0).sqrt().reshape(1, -1))
    norm.count.fill_(int(count))


def freeze_normalizer(actor) -> None:
    """Stop running-statistics updates (PPO keeps the BC input mapping fixed)."""
    if actor.obs_normalization:
        actor.obs_normalizer.until = int(actor.obs_normalizer.count)


def actor_mean(actor, obs: torch.Tensor) -> torch.Tensor:
    """Deterministic action for a plain observation tensor ``[N, obs_dim]``."""
    return actor.mlp(actor.obs_normalizer(obs))


def save_actor(path: Path, actor, meta: dict) -> None:
    """Checkpoint readable by ``OnPolicyRunner.load(..., load_cfg={"actor": True})``."""
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save({"actor_state_dict": actor.state_dict(), "iter": 0, "infos": {"meta": meta}}, path)


def _dims_from_state(sd: dict) -> tuple[int, int, list[int]]:
    weights = sorted((k for k in sd if k.startswith("mlp.") and k.endswith(".weight")), key=lambda k: int(k.split(".")[1]))
    shapes = [tuple(sd[k].shape) for k in weights]
    return shapes[0][1], shapes[-1][0], [s[0] for s in shapes[:-1]]


def load_actor(path: str | Path, cfg: ActorCfg, device):
    """Load the actor from a BC checkpoint or an rsl_rl PPO checkpoint (``model_<it>.pt``)."""
    ckpt = torch.load(path, map_location=device, weights_only=False)
    sd = ckpt["actor_state_dict"]
    obs_dim, act_dim, hidden = _dims_from_state(sd)
    actor = make_actor(cfg, obs_dim, act_dim, device, hidden_dims=hidden)
    actor.load_state_dict(sd)
    actor.eval()
    return actor


class PolicyAgent:
    """Learned policy as an agent. ``noise_std`` > 0 adds Gaussian exploration noise with this absolute std
    (normalised action units) to the executed action; ``last_mean`` keeps the clean action (distillation
    label). Experiment 1 scaled the noise by the policy std instead; see ``FilterCfg.rollout_noise_std``."""

    def __init__(self, actor, noise_std: float = 0.0):
        self.actor = actor
        self.noise_std = noise_std
        self.last_mean: torch.Tensor | None = None

    def reset(self, env_ids=None) -> None:
        pass

    @torch.no_grad()
    def compute_action(self, obs: torch.Tensor) -> torch.Tensor:
        mean = actor_mean(self.actor, obs)
        self.last_mean = mean
        if self.noise_std > 0:
            return mean + self.noise_std * torch.randn_like(mean)
        return mean


class TeacherAgent:
    """Rule-based controller with its gait parameters loaded from JSON."""

    def __init__(self, rb, num_envs: int, device, params_path: str | Path | None):
        from rule_based import RuleBasedBipedController
        from rule_based.config import load_cfg

        cfg = load_cfg(rb, str(params_path) if params_path else None)
        self.cfg = cfg
        self.ctrl = RuleBasedBipedController(rb, num_envs, device, cfg)
        self.last_mean: torch.Tensor | None = None

    def reset(self, env_ids=None) -> None:
        self.ctrl.reset(env_ids)

    def compute_action(self, obs: torch.Tensor) -> torch.Tensor:
        a = self.ctrl.compute_action(obs)
        self.last_mean = a
        return a


def load_agent(spec: str, env, actor_cfg: ActorCfg, root: Path):
    """``teacher[:<params.json>]`` or ``policy:<checkpoint.pt>`` -> agent for ``env``. Relative teacher
    parameter paths are resolved against ``root`` (the project root) when they do not exist as given."""
    kind, _, arg = spec.partition(":")
    if kind == "teacher":
        params = Path(arg) if arg else None
        if params is not None and not params.is_absolute() and not params.exists():
            params = root / params
        return TeacherAgent(env.rb, env.num_envs, env.device, params)
    if kind == "policy":
        return PolicyAgent(load_actor(arg, actor_cfg, env.device))
    raise ValueError(f"unknown agent spec {spec!r} (use teacher[:params.json] or policy:<ckpt>)")
