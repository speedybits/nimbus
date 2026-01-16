"""
Wander behavior - random exploration with obstacle avoidance.

The robot moves forward, turning when obstacles are encountered.
Good for exploring unknown spaces or demonstrating basic autonomy.
"""

from typing import Optional
from enum import Enum, auto
import random
import time
import math
from nimbus.core.state import RobotContext, Velocity, RobotState
from nimbus.navigation.vfh import VFHNavigator, VFHConfig
from nimbus.sensors.lidar import LidarProcessor, LidarConfig
from .base import Behavior


class RecoveryState(Enum):
    """State machine for unstuck recovery."""
    NONE = auto()        # Normal operation
    BACKING = auto()     # Backing away from obstacle
    TURNING = auto()     # Turning to find clear direction


class WanderBehavior(Behavior):
    """
    Random exploration with VFH obstacle avoidance.

    The robot:
    1. Picks a random direction bias
    2. Moves forward while avoiding obstacles
    3. Periodically changes direction bias
    4. Intelligently recovers when stuck near obstacles
    """

    name = "wander"
    description = "Random exploration with obstacle avoidance"
    priority = 10

    def __init__(
        self,
        forward_speed: float = 0.2,
        turn_speed: float = 0.5,
        direction_change_interval: float = 10.0,  # seconds
        # Recovery parameters
        backup_speed: float = 0.1,
        backup_duration: float = 1.5,  # seconds to back up
        turn_duration: float = 1.0,    # seconds to turn after backup
        stuck_threshold: float = 1.0,  # seconds before triggering recovery
        min_safe_distance: float = 0.6,  # ~2 feet - start slowing down at this distance
    ):
        super().__init__()
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.direction_change_interval = direction_change_interval

        # Recovery parameters
        self.backup_speed = backup_speed
        self.backup_duration = backup_duration
        self.turn_duration = turn_duration
        self.stuck_threshold = stuck_threshold
        self.min_safe_distance = min_safe_distance

        # Initialize processors with conservative settings to keep 2ft clearance
        # safety_radius=0.5 -> obstacle_threshold=1.25m for histogram
        # threshold_high=0.5 -> obstacles at 0.6m will register as blocked
        self._lidar = LidarProcessor(LidarConfig(safety_radius=0.5))
        self._vfh = VFHNavigator(VFHConfig(threshold_high=0.5, threshold_low=0.2))

        # State
        self._goal_direction = 0.0  # Current preferred direction (radians)
        self._last_direction_change = 0.0

        # Recovery state
        self._recovery_state = RecoveryState.NONE
        self._blocked_start_time: Optional[float] = None
        self._recovery_start_time: Optional[float] = None
        self._recovery_turn_direction = 1  # 1 = left, -1 = right

    def activate(self) -> None:
        """Initialize wander state."""
        super().activate()
        self._pick_random_direction()
        self._last_direction_change = time.time()
        self._reset_recovery()

    def reset(self) -> None:
        """Reset wander state."""
        self._goal_direction = 0.0
        self._last_direction_change = time.time()
        self._reset_recovery()

    def _reset_recovery(self) -> None:
        """Reset recovery state machine."""
        self._recovery_state = RecoveryState.NONE
        self._blocked_start_time = None
        self._recovery_start_time = None

    def compute(self, context: RobotContext) -> Optional[Velocity]:
        """
        Compute wandering velocity with obstacle avoidance and recovery.
        """
        sensors = context.sensors
        if sensors is None:
            return Velocity.stop()

        # Handle recovery state machine first
        if self._recovery_state != RecoveryState.NONE:
            return self._handle_recovery(context)

        # Check if we should enter recovery mode (stuck too long)
        if context.state == RobotState.EMERGENCY_STOP:
            if self._blocked_start_time is None:
                self._blocked_start_time = time.time()
            elif time.time() - self._blocked_start_time > self.stuck_threshold:
                return self._start_recovery(context, sensors.obstacle_direction)
        else:
            self._blocked_start_time = None

        # Periodically change direction
        if time.time() - self._last_direction_change > self.direction_change_interval:
            self._pick_random_direction()
            self._last_direction_change = time.time()

        # Process LIDAR data
        if len(sensors.lidar_ranges) < 360:
            return Velocity.stop()

        import numpy as np
        histogram = self._lidar.process(np.array(sensors.lidar_ranges))

        # Use VFH to compute steering
        steering, blocked = self._vfh.compute_steering(histogram, self._goal_direction)

        if blocked:
            # Track blocked time
            if self._blocked_start_time is None:
                self._blocked_start_time = time.time()
            elif time.time() - self._blocked_start_time > self.stuck_threshold:
                # Stuck too long - enter recovery
                return self._start_recovery(context, sensors.obstacle_direction)

            # No clear path - rotate in place (turn away from obstacle)
            context.set_state(RobotState.AVOIDING)
            turn_dir = -1 if sensors.obstacle_direction > 0 else 1
            return Velocity(linear=0.0, angular=self.turn_speed * turn_dir)

        # Clear - reset blocked timer
        self._blocked_start_time = None

        # Move forward while steering
        context.set_state(RobotState.NAVIGATING)

        # Proactive obstacle handling - slow down and steer away
        closest = sensors.closest_obstacle
        obstacle_angle = sensors.obstacle_direction

        if closest < self.min_safe_distance:
            # Scale speed: full at min_safe_distance, 20% at emergency distance (0.15m)
            dist_scale = max(0.2, (closest - 0.15) / (self.min_safe_distance - 0.15))

            # Add steering bias away from obstacle (even if VFH found a path)
            # Stronger bias when closer
            avoidance_strength = 1.0 - dist_scale  # 0 at safe dist, 0.8 at emergency
            avoidance_bias = -math.copysign(avoidance_strength * 0.5, obstacle_angle)
            steering = steering + avoidance_bias
        else:
            dist_scale = 1.0

        # Scale forward speed based on steering (slow down when turning)
        steering_scale = 1.0 - min(abs(steering) / 1.5, 0.7)
        linear = self.forward_speed * steering_scale * dist_scale

        # Convert steering angle to angular velocity
        angular = steering * 1.5  # Gain factor

        return Velocity(linear=linear, angular=angular)

    def _start_recovery(self, context: RobotContext, obstacle_angle: float) -> Velocity:
        """Start the recovery state machine to get unstuck."""
        self._recovery_state = RecoveryState.BACKING
        self._recovery_start_time = time.time()
        # Turn away from obstacle after backing up
        self._recovery_turn_direction = -1 if obstacle_angle > 0 else 1
        context.set_state(RobotState.AVOIDING)
        # Start backing up
        return Velocity(linear=-self.backup_speed, angular=0.0)

    def _handle_recovery(self, context: RobotContext) -> Velocity:
        """Execute the recovery state machine."""
        elapsed = time.time() - self._recovery_start_time
        context.set_state(RobotState.AVOIDING)

        if self._recovery_state == RecoveryState.BACKING:
            if elapsed < self.backup_duration:
                # Still backing up
                return Velocity(linear=-self.backup_speed, angular=0.0)
            else:
                # Done backing, start turning
                self._recovery_state = RecoveryState.TURNING
                self._recovery_start_time = time.time()
                return Velocity(linear=0.0, angular=self.turn_speed * self._recovery_turn_direction)

        elif self._recovery_state == RecoveryState.TURNING:
            if elapsed < self.turn_duration:
                # Still turning
                return Velocity(linear=0.0, angular=self.turn_speed * self._recovery_turn_direction)
            else:
                # Done with recovery - pick new direction and resume
                self._reset_recovery()
                self._pick_random_direction()
                return Velocity(linear=0.0, angular=0.0)

        # Shouldn't get here, but reset if we do
        self._reset_recovery()
        return Velocity.stop()

    def _pick_random_direction(self) -> None:
        """Pick a new random preferred direction."""
        import math
        # Bias toward forward, but allow some variation
        self._goal_direction = random.gauss(0.0, math.pi / 4)
        # Clamp to reasonable range
        self._goal_direction = max(-math.pi / 2, min(math.pi / 2, self._goal_direction))


class SimpleWanderBehavior(Behavior):
    """
    Simple wander without VFH - just reactive obstacle avoidance.

    Lighter weight alternative when VFH is overkill.
    """

    name = "simple_wander"
    description = "Simple wandering with reactive avoidance"
    priority = 5

    def __init__(
        self,
        forward_speed: float = 0.15,
        turn_speed: float = 0.4,
        obstacle_threshold: float = 0.6,  # ~2 feet
    ):
        super().__init__()
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.obstacle_threshold = obstacle_threshold
        self._turn_direction = 1  # 1 = left, -1 = right

    def compute(self, context: RobotContext) -> Optional[Velocity]:
        """Simple reactive wandering."""
        sensors = context.sensors
        if sensors is None:
            return Velocity.stop()

        closest = sensors.closest_obstacle
        obstacle_angle = sensors.obstacle_direction

        if closest < self.obstacle_threshold:
            # Obstacle ahead - turn away
            context.set_state(RobotState.AVOIDING)

            # Turn away from obstacle
            if obstacle_angle > 0:  # Obstacle on left
                angular = -self.turn_speed
            else:  # Obstacle on right
                angular = self.turn_speed

            return Velocity(linear=0.0, angular=angular)

        # Clear ahead - move forward
        context.set_state(RobotState.NAVIGATING)
        return Velocity(linear=self.forward_speed, angular=0.0)
