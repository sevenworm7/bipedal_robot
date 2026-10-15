"""Rule-based walking controller (teacher policy) for experiment 2 (ported from archive/exp1_reference/rule_based)."""

from .config import GaitParams, RuleBasedCfg
from .controller import RuleBasedBipedController

__all__ = ["GaitParams", "RuleBasedCfg", "RuleBasedBipedController"]
