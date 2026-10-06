"""rsl_rl symmetry hook (``RslRlSymmetryCfg.data_augmentation_func = "sim2real.symmetry:augment"``).

Returns the batch followed by its left-right mirror (``shared/symmetry.py``) for the observation groups (``policy`` /
``critic`` = teacher observation, ``student`` = history) and the actions.
"""

from __future__ import annotations

import torch

from shared.symmetry import Mirror

_MIRRORS: dict = {}


def _mirror(env, device) -> Mirror:
    key = str(device)
    if key not in _MIRRORS:
        _MIRRORS[key] = Mirror(env.unwrapped.rb, device)
    return _MIRRORS[key]


def augment(env, obs=None, actions=None):
    obs_aug = act_aug = None
    if obs is not None:
        m = _mirror(env, next(iter(obs.values())).device)
        mirrored = obs.clone()
        for k in mirrored.keys():
            if k in ("policy", "critic"):
                mirrored[k] = m.teacher(obs[k])
            elif k == "student":
                mirrored[k] = m.student(obs[k])
        obs_aug = torch.cat([obs, mirrored], dim=0)
    if actions is not None:
        m = _mirror(env, actions.device)
        act_aug = torch.cat([actions, m.actions(actions)], dim=0)
    return obs_aug, act_aug
