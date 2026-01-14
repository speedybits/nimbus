"""
GoTo behavior - navigate to a specific coordinate.

Combines goal-seeking with VFH obstacle avoidance.
"""

from typing import Optional
import math
from nimbus.core.state import RobotContext, Velocity, Pose2D, RobotState
from nimbus.navigation.vfh import VFHNavigator, VFHConfig
from nimbus.sensors.lidar import LidarProcessor, LidarConfig
from .base import Behavior


class GoToBehavior(Behavior):
    """
    Navigate to a target position with obstacle avoidance.

    Uses VFH to find clear paths while biasing toward the goal.
    """

    name = "goto"
    description = "Navigate to target coordinates"
    priority = 20

    def __init__(
        self,
        forward_speed: float = 0.25,
        turn_speed: float = 0.6,
        goal_tolerance: float = 0.15,      # meters
        angle_tolerance: float = 0.1,      # radians (~6 degrees)
    ):
        super().__init__()
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.goal_tolerance = goal_tolerance
        self.angle_tolerance = angle_tolerance

        # Processors
        self._lidar = LidarProcessor(LidarConfig())
        self._vfh = VFHNavigator(VFHConfig())

        # State
        self._target: Optional[Pose2D] = None
        self._reached_goal = False

    def set_target(self, x: float, y: float, theta: Optional[float] = None) -> None:
        """
        Set navigation target.

        Args:
            x: Target X coordinate (meters)
            y: Target Y coordinate (meters)
            theta: Optional final orientation (radians)
        """
        self._target = Pose2D(x=x, y=y, theta=theta or 0.0)
        self._reached_goal = False

    def clear_target(self) -> None:
        """Clear the current target."""
        self._target = None
        self._reached_goal = False

    @property
    def has_target(self) -> bool:
        """Check if a target is set."""
        return self._target is not None

    @property
    def reached_goal(self) -> bool:
        """Check if goal has been reached."""
        return self._reached_goal

    def reset(self) -> None:
        """Reset behavior state."""
        self._reached_goal = False

    def compute(self, context: RobotContext) -> Optional[Velocity]:
        """
        Compute velocity toward target with obstacle avoidance.
        """
        sensors = context.sensors
        if sensors is None or self._target is None:
            return Velocity.stop()

        current_pose = sensors.pose

        # Check if we've reached the goal
        distance = current_pose.distance_to(self._target)
        if distance < self.goal_tolerance:
            self._reached_goal = True
            context.set_state(RobotState.IDLE)
            context.clear_target()
            return Velocity.stop()

        # Compute angle to target
        angle_to_goal = current_pose.angle_to(self._target)

        # Convert to robot-relative angle
        heading_error = self._normalize_angle(angle_to_goal - current_pose.theta)

        # Process LIDAR for obstacle avoidance
        import numpy as np
        histogram = self._lidar.process(np.array(sensors.lidar_ranges))

        # Use VFH with goal direction
        steering, blocked = self._vfh.compute_steering(histogram, heading_error)

        if blocked:
            # No clear path toward goal - try to find any clear direction
            context.set_state(RobotState.AVOIDING)
            return Velocity(linear=0.0, angular=self.turn_speed)

        context.set_state(RobotState.NAVIGATING)

        # Blend goal-seeking with obstacle avoidance
        # When far from goal, prioritize VFH steering
        # When close, prioritize direct heading to goal

        if distance > 1.0:
            # Far from goal - trust VFH more
            angular = steering * 1.5
        else:
            # Close to goal - blend VFH with direct heading
            blend = distance  # 0 at goal, 1 at 1m
            angular = steering * blend + heading_error * (1 - blend)

        # Scale speed based on steering and distance
        speed_scale = 1.0 - min(abs(angular) / 1.5, 0.5)

        # Slow down when approaching goal
        if distance < 0.5:
            speed_scale *= distance / 0.5

        linear = self.forward_speed * speed_scale

        return Velocity(linear=linear, angular=angular)

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class PatrolBehavior(Behavior):
    """
    Patrol between a list of waypoints.

    Cycles through waypoints using GoToBehavior for each leg.
    """

    name = "patrol"
    description = "Patrol between waypoints"
    priority = 15

    def __init__(
        self,
        waypoints: Optional[list[tuple[float, float]]] = None,
        loop: bool = True,
    ):
        super().__init__()
        self._waypoints = waypoints or []
        self._loop = loop
        self._current_index = 0
        self._goto = GoToBehavior()

    def set_waypoints(self, waypoints: list[tuple[float, float]]) -> None:
        """Set the list of waypoints to patrol."""
        self._waypoints = waypoints
        self._current_index = 0

    def add_waypoint(self, x: float, y: float) -> None:
        """Add a waypoint to the patrol route."""
        self._waypoints.append((x, y))

    def clear_waypoints(self) -> None:
        """Clear all waypoints."""
        self._waypoints.clear()
        self._current_index = 0

    def activate(self) -> None:
        """Start patrol from first waypoint."""
        super().activate()
        self._current_index = 0
        self._goto.activate()
        self._set_current_target()

    def deactivate(self) -> None:
        """Stop patrol."""
        super().deactivate()
        self._goto.deactivate()

    def reset(self) -> None:
        """Reset to first waypoint."""
        self._current_index = 0
        self._goto.reset()

    def compute(self, context: RobotContext) -> Optional[Velocity]:
        """Navigate to current waypoint, advance when reached."""
        if not self._waypoints:
            return Velocity.stop()

        # Check if current waypoint reached
        if self._goto.reached_goal:
            self._advance_waypoint()
            self._set_current_target()

        return self._goto.compute(context)

    def _advance_waypoint(self) -> None:
        """Move to next waypoint."""
        self._current_index += 1
        if self._current_index >= len(self._waypoints):
            if self._loop:
                self._current_index = 0
            else:
                self._current_index = len(self._waypoints) - 1

    def _set_current_target(self) -> None:
        """Set GoTo target to current waypoint."""
        if self._waypoints and self._current_index < len(self._waypoints):
            x, y = self._waypoints[self._current_index]
            self._goto.set_target(x, y)

    @property
    def current_waypoint_index(self) -> int:
        """Index of current target waypoint."""
        return self._current_index

    @property
    def total_waypoints(self) -> int:
        """Total number of waypoints."""
        return len(self._waypoints)
