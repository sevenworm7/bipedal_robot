"""Rule-based walking controller (teacher policy) for experiment 2 (ported from 00_test/rule_based)."""

from .config import GaitParams, RuleBasedCfg
from .controller import RuleBasedBipedController

__all__ = ["GaitParams", "RuleBasedCfg", "RuleBasedBipedController"]
