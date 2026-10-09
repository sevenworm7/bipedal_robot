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


@height_field_to_mesh
def x_profile(difficulty: float, cfg: "XProfileCfg") -> np.ndarray:
    """Height field [pixels] that varies only along x (the walking direction): flat for ``flat_until`` metres past the
    tile centre (the robots start at the centre), then a ramp or steps. The start area is at height 0 (world z 0).
    ``difficulty`` is ignored (stress tests use fixed values)."""
    w = round(cfg.size[0] / cfg.horizontal_scale)
    h = round(cfg.size[1] / cfg.horizontal_scale)
    x = np.linspace(0, cfg.size[0], w) - cfg.size[0] / 2 - cfg.flat_until  # distance past the flat zone
    past = np.clip(x, 0.0, None)
    if cfg.kind in ("ramp_up", "ramp_down"):
        z = np.tan(np.radians(cfg.slope_deg)) * past
    elif cfg.kind in ("steps_up", "steps_down"):
        z = np.where(x > 0, cfg.step_height * (np.floor(past / cfg.step_length) + 1), 0.0)
    else:
        raise ValueError(cfg.kind)
    if cfg.kind.endswith("_down"):
        z = -z
    return np.rint(np.repeat(z[:, None], h, axis=1) / cfg.vertical_scale).astype(np.int16)


@configclass
class XProfileCfg(HfTerrainBaseCfg):
    function = x_profile
    kind: str = "ramp_up"
    """ramp_up, ramp_down, steps_up or steps_down."""
    slope_deg: float = 5.0
    step_height: float = 0.005
    """[m]"""
    step_length: float = 0.25
    """Tread length [m]."""
    flat_until: float = 1.0
    """Flat distance past the tile centre before the profile starts [m]. Stress tests use 0.3 m (slow robots walk only
    1 m in 20 s) and spawn at world height 0 (``spawn_z_abs``), since the tile origin height is no longer 0."""
