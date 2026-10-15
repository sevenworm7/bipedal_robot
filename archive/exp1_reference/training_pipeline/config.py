"""Hyper-parameters of the training pipeline (collect -> BC -> PPO -> filter -> distill), robot-agnostic.

Every stage script saves the config it used (``config.yaml``) into its run folder. The robot is
selected by ``PipelineCfg.robot`` and only reaches the code through ``shared/robots``.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field


@dataclass
class ActorCfg:
    """MLP policy shared by BC and PPO (must match exactly so BC weights load into the rsl_rl actor)."""

    hidden_dims: tuple[int, ...] = (512, 256, 128)
    activation: str = "elu"
    obs_normalization: bool = True
    """rsl_rl ``EmpiricalNormalization``; BC sets it from the dataset statistics and PPO keeps it frozen."""
    init_std: float = 0.2
    """Action std written into the BC checkpoint = initial PPO exploration std [normalised action units;
    x ACTION_SCALE = 0.05 rad]. Small so that PPO does not destroy the cloned gait."""


@dataclass
class CollectCfg:
    """Teacher / policy rollouts saved as HDF5."""

    num_envs: int = 4096
    """One complete episode per env (about 4 M steps with 20 s episodes)."""
    episode_s: float = 20.0
    exec_noise_std: float = 0.05
    """Gaussian noise added to the *executed* action (normalised units) while the label stays the clean
    action (DART-style): the dataset then contains the teacher's corrections from slightly perturbed states.
    Measured on 256 episodes: noise 0 / 0.05 / 0.1 -> success 96 / 86 / 55 %, no falls in any case (the noise
    adds short foot bounces, i.e. extra steps, which raise the distance bound)."""
    teacher_params: str = "rule_based/params/cem2_best.json"
    """Gait parameters of the rule-based teacher, relative to ``archive/exp1_reference``."""
    seed: int = 1


@dataclass
class BCCfg:
    epochs: int = 60
    batch_size: int = 8192
    lr: float = 1e-3
    lr_min: float = 1e-5
    """Cosine decay target."""
    weight_decay: float = 0.0
    val_frac: float = 0.1
    """Fraction of *episodes* held out for validation (split by episode, never by step)."""
    loss: str = "mse"
    eval_episodes: int = 100
    """Rollout check in simulation after training (same fixed evaluation set as every stage)."""
    seed: int = 2


@dataclass
class PPOCfg:
    """rsl_rl OnPolicyRunner settings (converted to ``RslRlOnPolicyRunnerCfg`` in ``ppo.py``)."""

    num_envs: int = 4096
    max_iterations: int = 1500
    num_steps_per_env: int = 24
    save_interval: int = 100
    critic_hidden_dims: tuple[int, ...] = (512, 256, 128)
    critic_warmup_iters: int = 50
    """Iterations with the actor frozen so the fresh critic can fit the returns of the cloned policy
    before the policy gradient starts (otherwise random advantages destroy the BC initialisation)."""
    learning_rate: float = 3e-4
    schedule: str = "adaptive"
    desired_kl: float = 0.01
    entropy_coef: float = 0.002
    clip_param: float = 0.2
    gamma: float = 0.99
    lam: float = 0.95
    num_learning_epochs: int = 5
    num_mini_batches: int = 4
    value_loss_coef: float = 1.0
    max_grad_norm: float = 1.0
    push_interval_s: tuple[float, float] = (4.0, 8.0)
    push_speed: float = 0.2
    """Light pushes during training (user decision 2026-09-22); evaluation has no pushes."""
    init_at_random_ep_len: bool = True
    seed: int = 3


@dataclass
class FilterCfg:
    """Self-distillation filter (tentative thresholds, archive/exp1_reference/CLAUDE.md). Per iteration values are
    indexed by iteration number - 1."""

    rollout_envs: int = 4096
    rollout_noise_scale: float = 0.3
    """Executed action = mean + noise * scale * policy std; the label is the policy mean."""
    max_lateral_m: tuple[float, ...] = (0.15, 0.15, 0.15)
    """Per iteration. Originally (0.15, 0.115, 0.08); kept at 0.15 m for all rounds (user decision 2026-09-23):
    the rocking gait itself sways ~0.1 m, < 0.08 m occurred in only 0-2 % of the PPO rollouts."""
    max_heading_deg: float = 10.0
    max_speed_err_frac: float = 0.25
    max_torque_sat_frac: float = 0.05
    min_steps: int = 10
    distance_coef: float = 0.8
    top_frac_per_bin: float = 0.5
    """Keep the best fraction (by episode return) inside each command bin."""
    cmd_bins: int = 3
    prev_mix_frac: float = 0.2
    """Share of the previous round's data mixed into the distillation set."""
    stop_if_kept_below: float = 0.30
    """Stop and discuss with the user if fewer episodes survive (user rule)."""


@dataclass
class EvalCfg:
    """Fixed evaluation set used by every stage (same as the rule-based evaluation)."""

    episodes: int = 100
    episode_s: float = 20.0
    seed: int = 42
    push_speed: float = 0.3
    push_times_s: tuple[float, ...] = (5.0, 10.0, 15.0)


@dataclass
class PipelineCfg:
    robot: str = "ref_biped"
    exp: str = "exp1"
    actor: ActorCfg = field(default_factory=ActorCfg)
    collect: CollectCfg = field(default_factory=CollectCfg)
    bc: BCCfg = field(default_factory=BCCfg)
    ppo: PPOCfg = field(default_factory=PPOCfg)
    filter: FilterCfg = field(default_factory=FilterCfg)
    eval: EvalCfg = field(default_factory=EvalCfg)

    def to_dict(self) -> dict:
        return asdict(self)
