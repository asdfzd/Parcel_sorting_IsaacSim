"""Shared M0609 palletizing package.

Only configuration is exported here so importing this package has no Isaac Sim side
effects. Import ``worker`` or ``bootstrap`` only after creating ``SimulationApp``.
"""

from .config import (
    A_DUAL_CONFIG,
    A_STANDALONE_CONFIG,
    B_DUAL_CONFIG,
    B_STANDALONE_CONFIG,
    CONFIGS,
    RobotCellConfig,
)

__all__ = [
    "RobotCellConfig",
    "A_STANDALONE_CONFIG",
    "B_STANDALONE_CONFIG",
    "A_DUAL_CONFIG",
    "B_DUAL_CONFIG",
    "CONFIGS",
]
