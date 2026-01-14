"""
Wander behavior - random exploration with obstacle avoidance.

The robot moves forward, turning when obstacles are encountered.
Good for exploring unknown spaces or demonstrating basic autonomy.
"""

from typing import Optional
import random
import time
from nimbus.core.state import RobotContext, Velocity, RobotState
from nimbus.navigation.vfh import VFHNavigator, VFHConfig
from nimbus.sensors.lidar import LidarProcessor, LidarConfig
from .base import Behavior


class WanderBehavior(Behavior):
    """
    Random exploration with VFH obstacle avoidance.

    The robot:
    1. Picks a random direction bias
    2. Moves forward while avoiding obstacles
    3. Periodically changes direction bias
    """

    name = "wander"
    description = "Random exploration with obstacle avoidance"
    priority = 10

    def __init__(
        self,
        forward_speed: float = 0.2,
        turn_speed: float = 0.5,
        direction_change_interval: float = 10.0,  # seconds
    ):
        super().__init__()
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.direction_change_interval = direction_change_interval

        # Initialize processors
        self._lidar = LidarProcessor(LidarConfig())
        self._vfh = VFHNavigator(VFHConfig())

        # State
        self._goal_direction = 0.0  # Current preferred direction (radians)
        self._last_direction_change = 0.0

    def activate(self) -> None:
        """Initialize wander state."""
        super().activate()
        self._pick_random_direction()
        self._last_direction_change = time.time()

    def reset(self) -> None:
        """Reset wander state."""
        self._goal_direction = 0.0
        self._last_direction_change = time.time()

    def compute(self, context: RobotContext) -> Optional[Velocity]:
        """
        Compute wandering velocity with obstacle avoidance.
        """
        sensors = context.sensors
        if sensors is None:
            return Velocity.stop()

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
            # No clear path - rotate in place
            context.set_state(RobotState.AVOIDING)
            return Velocity(linear=0.0, angular=self.turn_speed)

        # Move forward while steering
        context.set_state(RobotState.NAVIGATING)

        # Scale forward speed based on steering (slow down when turning)
        speed_scale = 1.0 - min(abs(steering) / 1.5, 0.7)
        linear = self.forward_speed * speed_scale

        # Convert steering angle to angular velocity
        # Proportional control toward desired direction
        angular = steering * 1.5  # Gain factor

        return Velocity(linear=linear, angular=angular)

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
        obstacle_threshold: float = 0.5,
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
