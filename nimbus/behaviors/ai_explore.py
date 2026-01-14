"""
AI Explore behavior - Ollama-driven exploration with obstacle avoidance.

Uses Ollama to decide waypoints based on LIDAR data and exploration memory.
Navigation to waypoints uses VFH for safe obstacle avoidance.
"""

import math
import time
from typing import Optional
import numpy as np

from nimbus.core.state import RobotContext, Velocity, Pose2D, RobotState
from nimbus.navigation.vfh import VFHNavigator, VFHConfig
from nimbus.sensors.lidar import LidarProcessor, LidarConfig
from nimbus.ai.ollama import AsyncOllamaClient, OllamaConfig, WaypointDecision
from nimbus.ai.describer import LidarDescriber, DescriptionConfig
from nimbus.ai.memory import ExplorationMemory
from .base import Behavior


class AIExploreBehavior(Behavior):
    """
    AI-driven exploration using Ollama for waypoint decisions.

    The robot:
    1. Describes its LIDAR environment to Ollama
    2. Receives a waypoint decision (x, y offset + reasoning)
    3. Navigates to the waypoint using VFH obstacle avoidance
    4. Requests a new waypoint when:
       - Current waypoint is reached
       - Timeout occurs (stuck or slow progress)
       - Robot detects it's stuck

    Memory persists exploration progress across sessions.

    Usage:
        behavior = AIExploreBehavior(memory_name="kitchen")
        # ... later ...
        behavior.memory.save()  # Persist exploration
    """

    name = "ai_explore"
    description = "AI-driven exploration using Ollama"
    priority = 25  # Higher than wander (10), goto (20)

    def __init__(
        self,
        memory_name: str = "default",
        forward_speed: float = 0.2,
        turn_speed: float = 0.5,
        goal_tolerance: float = 0.3,      # meters - when waypoint is "reached"
        decision_timeout: float = 30.0,   # seconds - max time per waypoint
        stuck_threshold: int = 10,        # cycles with no progress
        stuck_distance: float = 0.05,     # meters - minimum progress per check
        ollama_config: Optional[OllamaConfig] = None,
    ):
        super().__init__()

        # Navigation parameters
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.goal_tolerance = goal_tolerance
        self.decision_timeout = decision_timeout
        self.stuck_threshold = stuck_threshold
        self.stuck_distance = stuck_distance

        # Initialize components
        self._lidar = LidarProcessor(LidarConfig())
        self._vfh = VFHNavigator(VFHConfig())
        self._describer = LidarDescriber(DescriptionConfig())
        self._ollama = AsyncOllamaClient(ollama_config)

        # Load/create memory
        self.memory = ExplorationMemory.load_or_create(memory_name)

        # State
        self._current_target: Optional[Pose2D] = None
        self._last_decision_time: float = 0.0
        self._last_decision_reason: str = ""
        self._stuck_counter: int = 0
        self._last_position: Optional[tuple] = None
        self._waiting_for_ollama: bool = False

        # Event callbacks (for API/WebSocket events)
        self._on_decision_callback = None
        self._on_waypoint_reached_callback = None
        self._on_stuck_callback = None

    @property
    def current_target(self) -> Optional[Pose2D]:
        """Current navigation target."""
        return self._current_target

    @property
    def last_decision_reason(self) -> str:
        """Reason for the last waypoint decision."""
        return self._last_decision_reason

    @property
    def is_waiting_for_ollama(self) -> bool:
        """Check if waiting for Ollama response."""
        return self._waiting_for_ollama or self._ollama.is_pending()

    def set_memory(self, memory_name: str) -> None:
        """Switch to a different memory (saves current first)."""
        self.memory.save()
        self.memory = ExplorationMemory.load_or_create(memory_name)
        self._current_target = None
        self._last_decision_reason = ""

    def on_decision(self, callback) -> None:
        """Register callback for waypoint decisions."""
        self._on_decision_callback = callback

    def on_waypoint_reached(self, callback) -> None:
        """Register callback for waypoint reached events."""
        self._on_waypoint_reached_callback = callback

    def on_stuck(self, callback) -> None:
        """Register callback for stuck events."""
        self._on_stuck_callback = callback

    def activate(self) -> None:
        """Start AI exploration."""
        super().activate()
        self._last_decision_time = time.time()
        self._stuck_counter = 0
        self._last_position = None
        self._waiting_for_ollama = False

        # Check Ollama availability
        if not self._ollama.is_available():
            print("[AI Explore] Warning: Ollama not available. Will retry...")

    def deactivate(self) -> None:
        """Stop AI exploration and save memory."""
        super().deactivate()
        self.memory.save()
        self._ollama.cancel()

    def reset(self) -> None:
        """Reset exploration state (but keep memory)."""
        self._current_target = None
        self._last_decision_time = time.time()
        self._stuck_counter = 0
        self._last_position = None
        self._waiting_for_ollama = False
        self._ollama.cancel()

    def compute(self, context: RobotContext) -> Optional[Velocity]:
        """
        Compute velocity for AI exploration.

        Decision flow:
        1. Check if we need a new waypoint (reached, timeout, stuck)
        2. If needed, request from Ollama (async)
        3. Check for Ollama response
        4. Navigate toward current target (or wander if none)
        """
        sensors = context.sensors
        if sensors is None:
            return Velocity.stop()

        # Update exploration memory with current position
        self.memory.mark_visited(sensors.pose)

        # Check for Ollama response
        if self._ollama.has_result():
            self._handle_ollama_response(sensors.pose)
            self._waiting_for_ollama = False

        # Determine if we need a new waypoint
        need_new_waypoint = self._should_request_waypoint(context)

        if need_new_waypoint and not self._ollama.is_pending():
            self._request_waypoint(sensors)

        # Navigate toward current target or wander
        if self._current_target is not None:
            return self._navigate_to_target(context)
        else:
            return self._wander_while_waiting(context)

    def _should_request_waypoint(self, context: RobotContext) -> bool:
        """Determine if we need a new waypoint from Ollama."""
        sensors = context.sensors

        # No target yet
        if self._current_target is None:
            return True

        # Already waiting
        if self._ollama.is_pending():
            return False

        # Check if reached current target
        if sensors:
            distance = sensors.pose.distance_to(self._current_target)
            if distance < self.goal_tolerance:
                self._on_waypoint_reached(sensors.pose)
                return True

        # Check timeout
        elapsed = time.time() - self._last_decision_time
        if elapsed > self.decision_timeout:
            return True

        # Check if stuck
        if self._is_stuck(sensors):
            if self._on_stuck_callback:
                self._on_stuck_callback(sensors.pose if sensors else None)
            return True

        return False

    def _is_stuck(self, sensors) -> bool:
        """Check if robot is stuck (not making progress)."""
        if sensors is None:
            return False

        current_pos = (sensors.pose.x, sensors.pose.y)

        if self._last_position is None:
            self._last_position = current_pos
            return False

        # Check distance moved since last check
        dx = current_pos[0] - self._last_position[0]
        dy = current_pos[1] - self._last_position[1]
        distance_moved = math.sqrt(dx * dx + dy * dy)

        if distance_moved < self.stuck_distance:
            self._stuck_counter += 1
        else:
            self._stuck_counter = 0
            self._last_position = current_pos

        return self._stuck_counter >= self.stuck_threshold

    def _request_waypoint(self, sensors) -> None:
        """Request a new waypoint from Ollama."""
        # Build description
        description = self._describer.describe(sensors, self.memory)

        # Get memory context
        explored_summary = self.memory.get_explored_summary()
        recent_path = self.memory.get_recent_path()
        recent_decisions = self.memory.get_recent_decisions()

        # Request async
        self._waiting_for_ollama = True
        self._ollama.request_waypoint(
            description=description,
            explored_summary=explored_summary,
            recent_path=recent_path,
            recent_decisions=recent_decisions,
        )

    def _handle_ollama_response(self, current_pose: Pose2D) -> None:
        """Handle Ollama's waypoint decision."""
        result = self._ollama.get_result()
        if result is None:
            return

        if not result.success:
            # Ollama failed - will retry on next cycle
            self._last_decision_reason = f"Ollama error: {result.error}"
            return

        # Convert relative coordinates to absolute
        # Result is in robot frame: x = forward, y = left
        # Need to rotate by robot's heading
        theta = current_pose.theta
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        # Rotate from robot frame to world frame
        world_dx = result.x * cos_theta - result.y * sin_theta
        world_dy = result.x * sin_theta + result.y * cos_theta

        target_x = current_pose.x + world_dx
        target_y = current_pose.y + world_dy

        # Set new target
        self._current_target = Pose2D(x=target_x, y=target_y)
        self._last_decision_time = time.time()
        self._last_decision_reason = result.reason
        self._stuck_counter = 0
        self._last_position = (current_pose.x, current_pose.y)

        # Record in memory
        self.memory.add_waypoint(target_x, target_y, result.reason)

        # Notify callback
        if self._on_decision_callback:
            self._on_decision_callback(
                target_x, target_y, result.reason
            )

    def _on_waypoint_reached(self, pose: Pose2D) -> None:
        """Handle reaching a waypoint."""
        if self._current_target:
            self.memory.mark_waypoint_reached(
                self._current_target.x,
                self._current_target.y,
            )
            if self._on_waypoint_reached_callback:
                self._on_waypoint_reached_callback(pose)

        self._current_target = None
        self._last_decision_time = time.time()

    def _navigate_to_target(self, context: RobotContext) -> Velocity:
        """Navigate toward current target using VFH."""
        sensors = context.sensors
        if sensors is None:
            return Velocity.stop()

        current_pose = sensors.pose

        # Compute angle to target
        angle_to_goal = current_pose.angle_to(self._current_target)
        heading_error = self._normalize_angle(angle_to_goal - current_pose.theta)

        # Process LIDAR for obstacle avoidance
        histogram = self._lidar.process(np.array(sensors.lidar_ranges))

        # Use VFH with goal direction
        steering, blocked = self._vfh.compute_steering(histogram, heading_error)

        if blocked:
            # No clear path - rotate to find one
            context.set_state(RobotState.AVOIDING)
            return Velocity(linear=0.0, angular=self.turn_speed)

        context.set_state(RobotState.NAVIGATING)

        # Scale speed based on steering
        speed_scale = 1.0 - min(abs(steering) / 1.5, 0.5)

        # Slow down near target
        distance = current_pose.distance_to(self._current_target)
        if distance < 0.5:
            speed_scale *= distance / 0.5

        linear = self.forward_speed * speed_scale
        angular = steering * 1.5

        return Velocity(linear=linear, angular=angular)

    def _wander_while_waiting(self, context: RobotContext) -> Velocity:
        """Simple wandering while waiting for Ollama."""
        sensors = context.sensors
        if sensors is None:
            return Velocity.stop()

        # Simple reactive avoidance
        closest = sensors.closest_obstacle

        if closest < 0.4:
            # Too close - rotate
            context.set_state(RobotState.AVOIDING)
            return Velocity(linear=0.0, angular=self.turn_speed * 0.5)

        # Move forward slowly
        context.set_state(RobotState.NAVIGATING)
        return Velocity(linear=self.forward_speed * 0.5, angular=0.0)

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_status(self) -> dict:
        """Get current exploration status for API."""
        return {
            "active": self.is_active,
            "memory": self.memory.name,
            "current_target": {
                "x": self._current_target.x,
                "y": self._current_target.y,
            } if self._current_target else None,
            "cells_explored": len(self.memory.visited_cells),
            "last_decision": self._last_decision_reason,
            "waiting_for_ollama": self.is_waiting_for_ollama,
        }
