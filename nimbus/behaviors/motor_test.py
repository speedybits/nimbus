"""
Motor test behavior - direct motor control ignoring sensors.

This mode bypasses obstacle avoidance and safety filtering for
testing whether motors actually work. Use with caution.
"""

from typing import Optional
from nimbus.core.state import RobotContext, Velocity, RobotState
from .base import Behavior


class MotorTestBehavior(Behavior):
    """
    Direct motor control for testing.

    Ignores all sensor data and outputs configurable velocity.
    Dashboard keys control motion:
    - Arrow keys or WASD for direction
    - +/- for speed adjustment
    """

    name = "motor_test"
    description = "Direct motor control (bypasses safety)"
    priority = 100  # High priority

    # Flag for runner to check - bypass safety filtering
    bypass_safety = True

    def __init__(
        self,
        default_linear: float = 0.0,
        default_angular: float = 0.0,
        linear_step: float = 0.05,
        angular_step: float = 0.2,
        max_linear: float = 0.3,
        max_angular: float = 1.0,
    ):
        super().__init__()
        self.linear = default_linear
        self.angular = default_angular
        self.linear_step = linear_step
        self.angular_step = angular_step
        self.max_linear = max_linear
        self.max_angular = max_angular

    def activate(self) -> None:
        """Start with zero velocity for safety."""
        super().activate()
        self.linear = 0.0
        self.angular = 0.0

    def deactivate(self) -> None:
        """Stop motors on deactivation."""
        super().deactivate()
        self.linear = 0.0
        self.angular = 0.0

    def compute(self, context: RobotContext) -> Optional[Velocity]:
        """
        Return current velocity setting, ignoring all sensors.
        """
        # Set state to indicate we're in test mode
        context.set_state(RobotState.NAVIGATING)
        return Velocity(linear=self.linear, angular=self.angular)

    # --- Control methods for dashboard ---

    def forward(self) -> tuple[float, float]:
        """Increase forward speed."""
        self.linear = min(self.max_linear, self.linear + self.linear_step)
        self.angular = 0.0  # Stop rotation when changing direction
        return (self.linear, self.angular)

    def backward(self) -> tuple[float, float]:
        """Increase backward speed."""
        self.linear = max(-self.max_linear, self.linear - self.linear_step)
        self.angular = 0.0
        return (self.linear, self.angular)

    def left(self) -> tuple[float, float]:
        """Turn left (counterclockwise)."""
        self.angular = min(self.max_angular, self.angular + self.angular_step)
        return (self.linear, self.angular)

    def right(self) -> tuple[float, float]:
        """Turn right (clockwise)."""
        self.angular = max(-self.max_angular, self.angular - self.angular_step)
        return (self.linear, self.angular)

    def stop_motion(self) -> tuple[float, float]:
        """Stop all motion."""
        self.linear = 0.0
        self.angular = 0.0
        return (self.linear, self.angular)

    def set_velocity(self, linear: float, angular: float) -> None:
        """Set velocity directly."""
        self.linear = max(-self.max_linear, min(self.max_linear, linear))
        self.angular = max(-self.max_angular, min(self.max_angular, angular))

    def get_velocity(self) -> tuple[float, float]:
        """Get current velocity setting."""
        return (self.linear, self.angular)
