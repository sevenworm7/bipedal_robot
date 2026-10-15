"""Rule-based walking controller (teacher policy) for experiment 1."""

from .config import GaitParams, RuleBasedCfg
from .controller import RuleBasedBipedController

__all__ = ["GaitParams", "RuleBasedCfg", "RuleBasedBipedController"]
