"""Hyper-parameters of the training pipeline (collect -> BC -> PPO -> filter -> distill), robot-agnostic.

Experiment 2 copy of ``archive/exp1_reference/training_pipeline/config.py``. Changes: robot ``custom_biped`` / ``exp2``,
teacher parameters of robot v1, absolute exploration noise for distillation rollouts (experiment 1 lesson),
lighter training pushes (0.1 m/s, user decision 2026-10-01), ``env_overrides`` for env settings shared by every
stage (gait-clock window measured on this teacher), and provisional filter thresholds (decided after the
iteration 1 statistics, user decision 2026-10-01).

Every stage script saves the config it used (``config.yaml``) into its run folder. The robot is
selected by ``PipelineCfg.robot`` and only reaches the code through ``core/robots``.
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
    teacher_params: str = "control/rule_based/params/robot_v1_cem1_best.json"
    """Gait parameters of the rule-based teacher, relative to the project root."""
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
    push_speed: float = 0.1
    """Light pushes during training (user decision 2026-10-01: 0.1 m/s for the 0.7 kg robot; experiment 1
    used 0.2 m/s on 28 kg); evaluation has no pushes."""
    init_at_random_ep_len: bool = True
    seed: int = 3


@dataclass
class FilterCfg:
    """Self-distillation filter (thresholds decided with the user after the iteration 1 statistics). Per iteration values are
    indexed by iteration number - 1."""

    rollout_envs: int = 4096
    rollout_noise_std: float = 0.05
    """Executed action = mean + N(0, rollout_noise_std^2) in normalised units (absolute, same as the teacher's
    DART noise); the label is the policy mean. Experiment 1 scaled the noise by the policy std, which had
    shrunk to ~0.015 at the end of PPO, so its distillation data had almost no corrective examples."""
    max_lateral_m: tuple[float, ...] = (0.05, 0.05, 0.04)
    """Per iteration; the user decides each round's value after its ``--stats_only`` report (2026-10-01).
    Iteration 1: 0.05 m (45 % of the PPO rollouts pass; teacher 0.019 m on average, experiment 1 used 0.15 m).
    Iteration 2: 0.05 m (44 % pass). Iteration 3: 0.04 m (36 % pass; user 2026-10-01)."""
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
    push_speed: float = 0.15
    """Same as the rule-based push test of robot v1."""
    push_times_s: tuple[float, ...] = (5.0, 10.0, 15.0)
    video_speeds: tuple[float, ...] = (0.06, 0.10, 0.14)
    """Slow / medium / fast commands of the evaluation videos [m/s] (same as the rule-based videos)."""
    video_freq: float = 1.6


@dataclass
class PipelineCfg:
    robot: str = "custom_biped"
    exp: str = "exp2"
    actor: ActorCfg = field(default_factory=ActorCfg)
    collect: CollectCfg = field(default_factory=CollectCfg)
    bc: BCCfg = field(default_factory=BCCfg)
    ppo: PPOCfg = field(default_factory=PPOCfg)
    filter: FilterCfg = field(default_factory=FilterCfg)
    eval: EvalCfg = field(default_factory=EvalCfg)
    env_overrides: dict = field(default_factory=lambda: {
        # gait-clock reward window, measured on the robot v1 teacher (measure_gait.py, see README)
        "gait_swing_start": 0.05,  # iter0_gait_20261001_093020: P(air) > 0.5 for leg phase 0.05-0.50
        "gait_swing_frac": 0.45,
    })
    """``BipedWalkEnvCfg`` fields set for every stage (collect, BC eval, PPO, evaluation) so that rewards and
    episode statistics are identical everywhere."""
    reward_overrides: dict = field(default_factory=lambda: {
        # torque [N^2 m^2]: experiment 1 used -2e-4 for 10-80 N m joints; the v1 servos give ~100x smaller torques,
        # so -2e-4 cost the teacher only -0.0008 per episode. -0.7 restores the experiment 1 magnitude (teacher
        # about -2.8 per 20 s episode, experiment 1 teacher -2.7).
        "torque": -0.7,
        # lateral_deviation stays at -40: -400 (with sigma_heading 0.05) was tried on 2026-10-01 (iter1_ppo_lat) and made
        # things worse; the policy cannot observe its lateral offset, so a larger weight only adds return noise.
    })
    """``RewardWeightsCfg`` weights changed for this robot (logic unchanged; see the README table)."""

    def to_dict(self) -> dict:
        return asdict(self)
