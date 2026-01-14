"""
Safety controller for Nimbus.

This is the ALWAYS-ON safety layer that:
- Triggers emergency stop when obstacles are too close
- Limits velocity based on proximity
- Cannot be bypassed by behaviors or API calls

The safety controller sits between the behavior layer and motor commands,
filtering all velocity requests through safety checks.
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Tuple
import time


class SafetyLevel(Enum):
    """Current safety status."""

    NORMAL = auto()     # Full speed allowed
    CAUTION = auto()    # Speed limited based on proximity
    EMERGENCY = auto()  # Full stop


@dataclass(frozen=True)
class SafetyConfig:
    """Safety controller configuration."""

    emergency_distance: float = 0.15  # Full stop threshold (meters)
    caution_distance: float = 0.40    # Speed reduction threshold (meters)
    emergency_timeout: float = 5.0    # Seconds before auto-recovery attempt
    max_linear_speed: float = 0.30    # Maximum forward speed (m/s)
    max_angular_speed: float = 1.0    # Maximum rotation speed (rad/s)
    reverse_speed: float = 0.1        # Speed for backing away from obstacles


class SafetyController:
    """
    Hardware safety layer.

    ALWAYS runs. Cannot be disabled. Filters all velocity commands
    through safety checks before they reach the motors.

    Usage:
        safety = SafetyController()

        # Before sending any velocity command:
        linear, angular = safety.limit_velocity(
            desired_linear, desired_angular, closest_obstacle
        )

        # Check safety status:
        level = safety.evaluate(closest_obstacle)
        if level == SafetyLevel.EMERGENCY:
            # Handle emergency
    """

    def __init__(self, config: SafetyConfig | None = None):
        self.config = config or SafetyConfig()
        self._emergency_start: float | None = None
        self._last_level = SafetyLevel.NORMAL
        self._consecutive_emergency_count = 0

    def evaluate(self, closest_distance: float) -> SafetyLevel:
        """
        Evaluate current safety level based on obstacle proximity.

        Args:
            closest_distance: Distance to nearest obstacle (meters)

        Returns:
            Current SafetyLevel
        """
        if closest_distance < self.config.emergency_distance:
            level = SafetyLevel.EMERGENCY
        elif closest_distance < self.config.caution_distance:
            level = SafetyLevel.CAUTION
        else:
            level = SafetyLevel.NORMAL

        self._last_level = level
        return level

    def limit_velocity(
        self,
        linear: float,
        angular: float,
        closest_distance: float
    ) -> Tuple[float, float]:
        """
        Apply safety limits to velocity commands.

        This is the main filtering function. ALL velocity commands
        should pass through this before being sent to motors.

        Args:
            linear: Desired linear velocity (m/s)
            angular: Desired angular velocity (rad/s)
            closest_distance: Distance to nearest obstacle (meters)

        Returns:
            Tuple of (safe_linear, safe_angular) velocities
        """
        level = self.evaluate(closest_distance)

        if level == SafetyLevel.EMERGENCY:
            # Track emergency state
            if self._emergency_start is None:
                self._emergency_start = time.time()
                self._consecutive_emergency_count += 1

            # Full stop - no forward motion allowed
            # Allow rotation to find clear path
            # Allow slow reverse to back away
            if linear > 0:
                linear = 0.0

            # After timeout, allow slow reverse to escape
            elapsed = time.time() - self._emergency_start
            if elapsed > self.config.emergency_timeout and linear <= 0:
                linear = max(linear, -self.config.reverse_speed)

            return self._clamp_velocity(linear, angular)

        # Clear emergency state
        self._emergency_start = None
        self._consecutive_emergency_count = 0

        if level == SafetyLevel.CAUTION:
            # Scale speed based on distance
            # At caution_distance: full speed
            # At emergency_distance: nearly stopped
            scale = (closest_distance - self.config.emergency_distance) / (
                self.config.caution_distance - self.config.emergency_distance
            )
            scale = max(0.1, min(1.0, scale))  # Clamp to 0.1-1.0

            if linear > 0:  # Only limit forward motion
                linear *= scale

        return self._clamp_velocity(linear, angular)

    def _clamp_velocity(self, linear: float, angular: float) -> Tuple[float, float]:
        """Clamp velocities to configured maximums."""
        linear = max(-self.config.max_linear_speed,
                    min(self.config.max_linear_speed, linear))
        angular = max(-self.config.max_angular_speed,
                     min(self.config.max_angular_speed, angular))
        return (linear, angular)

    @property
    def is_emergency(self) -> bool:
        """Check if currently in emergency state."""
        return self._last_level == SafetyLevel.EMERGENCY

    @property
    def can_move_forward(self) -> bool:
        """Check if forward motion is allowed."""
        return self._last_level != SafetyLevel.EMERGENCY

    @property
    def emergency_duration(self) -> float:
        """Seconds in current emergency state (0 if not in emergency)."""
        if self._emergency_start is None:
            return 0.0
        return time.time() - self._emergency_start

    def can_recover(self) -> bool:
        """
        Check if enough time has passed for recovery attempt.

        Returns True if:
        - Not in emergency, or
        - Emergency timeout has passed
        """
        if self._emergency_start is None:
            return True
        return self.emergency_duration > self.config.emergency_timeout

    def force_stop(self) -> Tuple[float, float]:
        """Force an immediate stop (returns zero velocities)."""
        return (0.0, 0.0)

    @property
    def status(self) -> dict:
        """Get current safety status as dictionary."""
        return {
            "level": self._last_level.name,
            "is_emergency": self.is_emergency,
            "can_move_forward": self.can_move_forward,
            "emergency_duration": self.emergency_duration,
            "can_recover": self.can_recover(),
            "consecutive_emergencies": self._consecutive_emergency_count,
        }


class VelocitySmoother:
    """
    Smooth velocity commands to prevent jerky motion.

    Limits acceleration/deceleration to prevent wheel slip
    and provide smoother motion.
    """

    def __init__(
        self,
        max_linear_accel: float = 0.5,   # m/s^2
        max_angular_accel: float = 2.0,   # rad/s^2
        dt: float = 0.1                   # Control loop period
    ):
        self.max_linear_accel = max_linear_accel
        self.max_angular_accel = max_angular_accel
        self.dt = dt
        self._last_linear = 0.0
        self._last_angular = 0.0

    def smooth(self, linear: float, angular: float) -> Tuple[float, float]:
        """
        Apply acceleration limits to velocity command.

        Args:
            linear: Desired linear velocity
            angular: Desired angular velocity

        Returns:
            Smoothed (linear, angular) velocities
        """
        # Compute maximum change per timestep
        max_linear_delta = self.max_linear_accel * self.dt
        max_angular_delta = self.max_angular_accel * self.dt

        # Apply limits
        linear_delta = linear - self._last_linear
        linear_delta = max(-max_linear_delta, min(max_linear_delta, linear_delta))
        new_linear = self._last_linear + linear_delta

        angular_delta = angular - self._last_angular
        angular_delta = max(-max_angular_delta, min(max_angular_delta, angular_delta))
        new_angular = self._last_angular + angular_delta

        # Update state
        self._last_linear = new_linear
        self._last_angular = new_angular

        return (new_linear, new_angular)

    def reset(self) -> None:
        """Reset smoother state (e.g., after emergency stop)."""
        self._last_linear = 0.0
        self._last_angular = 0.0
