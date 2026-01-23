"""
Nimbus Simulation Package.

Provides a grid-based 2D virtual world for testing behaviors
without hardware.

Usage:
    nimbus run --sim
    nimbus run --sim --map path/to/map.txt
    nimbus run --sim --behavior wander
"""

from .world import SimulatedWorld
from .node import SimulatorNode

__all__ = ["SimulatedWorld", "SimulatorNode"]
