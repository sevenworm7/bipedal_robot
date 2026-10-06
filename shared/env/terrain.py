"""Uneven ground for the sim-to-real stage: random bumps whose height grows with the curriculum level.

Isaac Lab's ``random_uniform_terrain`` ignores the difficulty and uses a 5 mm height step, which is too coarse for
the +-8 mm bumps of robot v1 (18 mm foot lift). ``graded_bumps`` samples heights on a coarse grid
(``bump_spacing``), interpolates them to the height-field resolution, and scales the amplitude by the difficulty.
Used only when ``DomainRandCfg.rough_terrain`` is set (``_configure_startup_randomisation`` in ``biped_env.py``).
"""

from __future__ import annotations

import numpy as np
from scipy import interpolate

from isaaclab.terrains.height_field.hf_terrains_cfg import HfTerrainBaseCfg
from isaaclab.terrains.height_field.utils import height_field_to_mesh
from isaaclab.utils import configclass


@height_field_to_mesh
def graded_bumps(difficulty: float, cfg: "GradedBumpsCfg") -> np.ndarray:
    """Height field [pixels] with heights uniform in +-(min + difficulty * (max - min)) amplitude."""
    rng = np.random.default_rng(cfg.seed_offset + int(difficulty * 1e6))
    w = round(cfg.size[0] / cfg.horizontal_scale)
    h = round(cfg.size[1] / cfg.horizontal_scale)
    amp = cfg.amplitude_range[0] + difficulty * (cfg.amplitude_range[1] - cfg.amplitude_range[0])
    nx = max(2, int(cfg.size[0] / cfg.bump_spacing) + 1)
    ny = max(2, int(cfg.size[1] / cfg.bump_spacing) + 1)
    coarse = rng.uniform(-amp, amp, size=(nx, ny))
    xs = np.linspace(0, cfg.size[0], nx)
    ys = np.linspace(0, cfg.size[1], ny)
    f = interpolate.RectBivariateSpline(xs, ys, coarse, kx=1, ky=1)
    hf = f(np.linspace(0, cfg.size[0], w), np.linspace(0, cfg.size[1], h))
    return np.rint(hf / cfg.vertical_scale).astype(np.int16)


@configclass
class GradedBumpsCfg(HfTerrainBaseCfg):
    function = graded_bumps
    amplitude_range: tuple[float, float] = (0.0, 0.008)
    """Bump amplitude [m] at difficulty 0 and 1 (heights uniform in +-amplitude)."""
    bump_spacing: float = 0.1
    """Distance between independent height samples [m] (bumps of about one foot length)."""
    seed_offset: int = 0
