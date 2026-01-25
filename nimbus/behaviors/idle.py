"""
Idle behavior - robot stays stationary.

The simplest behavior: do nothing, wait for commands.
"""

from typing import Optional
from nimbus.core.state import RobotContext, RobotState, Velocity
from .base import Behavior


class IdleBehavior(Behavior):
    """
    Idle behavior: robot stays stationary.

    Used as:
    - Default behavior when no action needed
    - Safe state between operations
    - Manual control mode (waiting for teleop)
    """

    name = "idle"
    description = "Stay stationary, waiting for commands"
    priority = 0  # Lowest priority

    def compute(self, context: RobotContext) -> Optional[Velocity]:
        """Return zero velocity and set state to IDLE."""
        # Ensure state reflects idle (unless in emergency stop)
        if context.state != RobotState.EMERGENCY_STOP:
            context.set_state(RobotState.IDLE)
        return Velocity.stop()
