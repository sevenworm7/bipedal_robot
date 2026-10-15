"""Hyper-parameters of the sim-to-real stage. Saved into every run's config.yaml.

Network, BC and evaluation settings are those of ``control/flat_pipeline/config.py`` (imported, not
copied); only what differs is defined here.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field


@dataclass
class TeacherCfg:
    """Privileged teacher: PPO on interface v2 (``policy`` = ``critic`` = 55-dim teacher observation)."""

    scenario: str = "dr_rough_gait_train_v1"
    init_checkpoint: str = "outputs/robot_v1/control/flat_pipeline/iter3_distill_20261001_132024/model.pt"
    """03 final model (44 inputs); its first layer is zero-padded to the 55 teacher inputs."""
    norm_stat_steps: int = 50
    """Control steps of the zero-action rollout used to estimate the normaliser statistics of the 11 new inputs."""
    num_envs: int = 4096
    max_iterations: int = 3000
    num_steps_per_env: int = 24
    save_interval: int = 200
    critic_hidden_dims: tuple[int, ...] = (512, 256, 128)
    critic_warmup_iters: int = 50
    init_std: float = 0.2
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
    push_speed: float = 0.25
    """User decision 2026-10-06: pushes up to 0.25 m/s."""
    reward_overrides: dict = field(default_factory=lambda: {
        # relaxed gait clock (user decision 2026-10-06): 2.0 in 03 locked the gait to one rhythm
        "gait_contact": 1.0,
        # swing apex vs the clearance command, per touchdown; ~3 touchdowns/s x 20 s x 0.02 s -> up to ~6 per episode
        "clearance": 5.0,
    })
    env_overrides: dict = field(default_factory=lambda: {
        # round 2 (user decision 2026-10-06): heading is observable through the IMU, 0.2 rad (11 deg) let bumps turn the
        # robot by > 10 deg; 0.1 m/s on the speed reward is as wide as the whole command range (14 % overspeed on flat)
        "sigma_heading": 0.1,
        "sigma_vx": 0.05,
    })
    """Env reward settings for training (teacher v1 used the 03 values: sigma_heading 0.2, sigma_vx 0.1)."""
    reward_presets: dict = field(default_factory=lambda: {
        # round 5 (user decision 2026-10-06): drop the unobservable, accumulating lateral position penalty (the largest
        # term, -26.7 per episode in teacher v4) and penalise the steady sideways drift the teacher can observe instead;
        # -1000 cost ~-19 per episode in the smoke test (training pushes make large filtered sideways velocities) and would
        # dominate like lateral_deviation did; -300 -> about -5.6 per episode in that state, -1.4 for a steady 0.015 m/s drift
        "obs_lateral": {"lateral_deviation": 0.0, "lateral_vel_avg": -300.0},
    })
    """Extra reward overrides selected with ``--reward_preset`` (teachers v1-v4 used none)."""
    symmetry: bool = False
    """Left-right mirror augmentation + mirror loss in PPO (``core/symmetry.py``; user decision 2026-10-06 against the
    step-to gait of teacher v3). Off for teachers v1-v3 (recorded runs), on from round 4 via ``--symmetry``."""
    mirror_loss_coeff: float = 1.0
    seed: int = 11


@dataclass
class StudentCfg:
    """Deployable student distilled from the teacher with DAgger (interface v2 ``student`` group, 5 x 42 inputs)."""

    scenario: str = "dr_rough_gait_train_v1"
    num_envs: int = 4096
    rounds: int = 120
    """Round 2 (user decision 2026-10-06): 40 -> 120."""
    steps_per_round: int = 100
    """Control steps of every rollout round (2 s); the env keeps running across rounds (episodes continue)."""
    beta_rounds: int = 6
    """The teacher acts with probability beta = 1 - round / beta_rounds (per env and round), then only the student."""
    buffer_steps: int = 4_000_000
    """Aggregated dataset size (oldest samples dropped first)."""
    epochs_per_round: int = 2
    batch_size: int = 16384
    lr: float = 1e-3
    lr_min: float = 1e-4
    val_frac: float = 0.05
    push_interval_s: tuple[float, float] = (4.0, 8.0)
    push_speed: float = 0.25
    mirror_augment: bool = False
    """Also store the left-right mirror of every (student observation, teacher action) pair (round 4, ``--symmetry``)."""
    seed: int = 21


@dataclass
class Sim2RealCfg:
    teacher: TeacherCfg = field(default_factory=TeacherCfg)
    student: StudentCfg = field(default_factory=StudentCfg)

    def to_dict(self) -> dict:
        return asdict(self)
