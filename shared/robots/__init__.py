"""Robot registry: pipeline code selects the robot by name (config field ``robot``)."""

from importlib import import_module
from types import ModuleType

ROBOTS = ("custom_biped",)


def get_robot(name: str) -> ModuleType:
    """Return the robot module ``shared.robots.<name>``."""
    if name not in ROBOTS:
        raise KeyError(f"Unknown robot '{name}', expected one of {ROBOTS}")
    return import_module(f"{__name__}.{name}")
