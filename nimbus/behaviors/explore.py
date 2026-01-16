"""
Explore behavior - systematic exploration with persistent mapping.

The robot moves toward unexplored areas, building a persistent map
of visited cells. Unlike wander (random) or ai_explore (LLM-driven),
this uses simple heuristics to maximize coverage.
"""

from typing import Optional
from enum import Enum, auto
import random
import time
import math
import numpy as np

from nimbus.core.state import RobotContext, Velocity, RobotState, Pose2D
from nimbus.navigation.vfh import VFHNavigator, VFHConfig
from nimbus.sensors.lidar import LidarProcessor, LidarConfig
from nimbus.ai.memory import ExplorationMemory
from .base import Behavior


# Direction name to angle (robot frame: 0=forward, +left, -right)
DIRECTION_ANGLES = {
    "north": 0.0,
    "northeast": -math.pi / 4,
    "east": -math.pi / 2,
    "northwest": math.pi / 4,
    "west": math.pi / 2,
    "southeast": -3 * math.pi / 4,
    "southwest": 3 * math.pi / 4,
    "south": math.pi,
}


class RecoveryState(Enum):
    """State machine for unstuck recovery."""
    NONE = auto()
    BACKING = auto()
    TURNING = auto()


class DrivingState(Enum):
    """State machine for plan-then-drive navigation."""
    PLANNING = auto()   # Analyze LIDAR, pick heading
    TURNING = auto()    # Rotate to target heading
    DRIVING = auto()    # Drive forward
    BACKING = auto()    # Back up to create space


class ExploreBehavior(Behavior):
    """
    Systematic exploration with persistent mapping.

    Uses ExplorationMemory to track visited areas and
    strongly prefers directions with unexplored cells.

    The robot:
    1. Marks its current position as visited each cycle
    2. Picks directions toward unexplored areas
    3. Uses VFH for obstacle avoidance
    4. Recovers intelligently when stuck
    5. Persists memory across sessions
    """

    name = "explore"
    description = "Systematic exploration of new territory"
    priority = 15

    def __init__(
        self,
        memory_name: str = "explore",
        forward_speed: float = 0.2,
        turn_speed: float = 0.5,
        direction_change_interval: float = 15.0,  # Longer commitment to reach distant areas
        # Recovery parameters
        backup_speed: float = 0.1,
        backup_duration: float = 1.5,
        turn_duration: float = 1.0,
        stuck_threshold: float = 1.0,
        min_safe_distance: float = 0.6,
        # Exploration parameters
        exploration_range: float = 5.0,  # How far to look for unexplored areas
    ):
        super().__init__()
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.direction_change_interval = direction_change_interval
        self.exploration_range = exploration_range

        # Recovery parameters
        self.backup_speed = backup_speed
        self.backup_duration = backup_duration
        self.turn_duration = turn_duration
        self.stuck_threshold = stuck_threshold
        self.min_safe_distance = min_safe_distance

        # Initialize VFH with conservative settings (2ft clearance)
        self._lidar = LidarProcessor(LidarConfig(safety_radius=0.5))
        self._vfh = VFHNavigator(VFHConfig(threshold_high=0.5, threshold_low=0.2))

        # Memory (loaded on activate)
        self._memory_name = memory_name
        self._memory: Optional[ExplorationMemory] = None

        # State
        self._goal_direction = 0.0
        self._last_direction_change = 0.0
        self._current_pose: Optional[Pose2D] = None

        # Recovery state
        self._recovery_state = RecoveryState.NONE
        self._blocked_start_time: Optional[float] = None
        self._recovery_start_time: Optional[float] = None
        self._recovery_turn_direction = 1

        # Driving state machine (plan-then-drive)
        self._driving_state = DrivingState.PLANNING
        self._target_heading: float = 0.0           # World heading to drive toward
        self._heading_tolerance: float = 0.17       # ~10 degrees
        self._max_steering_correction: float = 0.26 # ~15 degrees during DRIVING
        self._drive_distance: float = 2.0           # Meters to drive before replanning
        self._drive_start_pose: Optional[Pose2D] = None
        self._backing_start_time: Optional[float] = None
        self._backing_duration: float = 1.0         # Seconds to back up

    def set_memory(self, memory_name: str) -> None:
        """Switch to a different memory (saves current first)."""
        if self._memory:
            self._memory.save()
        self._memory_name = memory_name
        self._memory = ExplorationMemory.load_or_create(memory_name)

    def activate(self) -> None:
        """Start exploration."""
        super().activate()
        try:
            self._memory = ExplorationMemory.load_or_create(self._memory_name)
        except Exception as e:
            print(f"[Explore] Failed to load memory: {e}")
            self._memory = None
        self._last_direction_change = 0.0
        self._reset_recovery()
        self._reset_driving()

    def deactivate(self) -> None:
        """Stop exploration and save memory."""
        if self._memory:
            self._memory.save()
        super().deactivate()

    def reset(self) -> None:
        """Reset exploration state (but keep memory)."""
        self._goal_direction = 0.0
        self._last_direction_change = time.time()
        self._reset_recovery()
        self._reset_driving()

    def _reset_recovery(self) -> None:
        """Reset recovery state machine."""
        self._recovery_state = RecoveryState.NONE
        self._blocked_start_time = None
        self._recovery_start_time = None

    def _reset_driving(self) -> None:
        """Reset driving state machine."""
        self._driving_state = DrivingState.PLANNING
        self._target_heading = 0.0
        self._drive_start_pose = None
        self._backing_start_time = None

    def compute(self, context: RobotContext) -> Optional[Velocity]:
        """Compute exploration velocity."""
        try:
            return self._compute_internal(context)
        except Exception as e:
            import traceback
            traceback.print_exc()
            return Velocity.stop()

    def _compute_internal(self, context: RobotContext) -> Optional[Velocity]:
        """Internal compute using plan-then-drive state machine."""
        sensors = context.sensors
        if sensors is None:
            return Velocity.stop()

        # Track current pose
        self._current_pose = sensors.pose

        # Mark current position as visited
        if self._memory and self._current_pose:
            self._memory.mark_visited(self._current_pose)

        # Handle recovery state machine first (from getting stuck)
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

        # Driving state machine: PLANNING -> TURNING -> DRIVING -> (BACKING if blocked) -> PLANNING
        if self._driving_state == DrivingState.PLANNING:
            return self._handle_planning(context)
        elif self._driving_state == DrivingState.TURNING:
            return self._handle_turning(context)
        elif self._driving_state == DrivingState.DRIVING:
            return self._handle_driving(context)
        elif self._driving_state == DrivingState.BACKING:
            return self._handle_backing(context)

        return Velocity.stop()

    def _handle_planning(self, context: RobotContext) -> Velocity:
        """Analyze LIDAR and pick best heading toward unexplored area."""
        sensors = context.sensors
        context.set_state(RobotState.NAVIGATING)

        # Process LIDAR
        if len(sensors.lidar_ranges) < 10:
            return Velocity.stop()

        # Pick direction toward unexplored area
        self._pick_exploration_direction()

        # Convert goal direction (robot frame) to world heading
        self._target_heading = self._current_pose.theta + self._goal_direction

        # Normalize to [-pi, pi]
        while self._target_heading > math.pi:
            self._target_heading -= 2 * math.pi
        while self._target_heading < -math.pi:
            self._target_heading += 2 * math.pi

        # Record start position for distance tracking
        self._drive_start_pose = self._current_pose

        # Transition to turning
        self._driving_state = DrivingState.TURNING

        return Velocity(linear=0.0, angular=0.0)

    def _handle_turning(self, context: RobotContext) -> Velocity:
        """Rotate in place to face target heading."""
        context.set_state(RobotState.NAVIGATING)

        # Calculate heading error
        heading_error = self._target_heading - self._current_pose.theta

        # Normalize to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Check if aligned
        if abs(heading_error) < self._heading_tolerance:
            self._driving_state = DrivingState.DRIVING
            self._drive_start_pose = self._current_pose
            return Velocity(linear=0.0, angular=0.0)

        # Turn toward target heading (slow down as we approach)
        turn_speed = min(self.turn_speed, abs(heading_error) * 2)
        turn_dir = 1 if heading_error > 0 else -1

        return Velocity(linear=0.0, angular=turn_speed * turn_dir)

    def _handle_driving(self, context: RobotContext) -> Velocity:
        """Drive forward with minimal steering corrections."""
        sensors = context.sensors
        context.set_state(RobotState.NAVIGATING)

        # Check if we've driven far enough - time to replan
        if self._drive_start_pose:
            distance_traveled = self._current_pose.distance_to(self._drive_start_pose)
            if distance_traveled >= self._drive_distance:
                self._driving_state = DrivingState.PLANNING
                return Velocity(linear=0.0, angular=0.0)

        # Process LIDAR
        if len(sensors.lidar_ranges) < 10:
            return Velocity.stop()

        # Check if too close to obstacle - back up instead of spinning
        closest = sensors.closest_obstacle
        if closest < 0.4:  # Too close - need to back up
            self._driving_state = DrivingState.BACKING
            self._backing_start_time = time.time()
            context.set_state(RobotState.AVOIDING)
            return Velocity(linear=-self.backup_speed, angular=0.0)

        histogram = self._lidar.process(np.array(sensors.lidar_ranges))

        # VFH for obstacle avoidance
        goal_in_robot_frame = self._target_heading - self._current_pose.theta
        while goal_in_robot_frame > math.pi:
            goal_in_robot_frame -= 2 * math.pi
        while goal_in_robot_frame < -math.pi:
            goal_in_robot_frame += 2 * math.pi

        steering, blocked = self._vfh.compute_steering(histogram, goal_in_robot_frame)

        if blocked:
            # Path is blocked - back up to create space, then replan
            self._driving_state = DrivingState.BACKING
            self._backing_start_time = time.time()
            context.set_state(RobotState.AVOIDING)
            return Velocity(linear=-self.backup_speed, angular=0.0)

        # Limit steering corrections during driving (stay committed to heading)
        steering = max(-self._max_steering_correction,
                       min(self._max_steering_correction, steering))

        # Proactive obstacle slowdown
        if closest < self.min_safe_distance:
            dist_scale = max(0.2, (closest - 0.15) / (self.min_safe_distance - 0.15))
        else:
            dist_scale = 1.0

        # Drive forward at constant speed (no steering-based slowdown!)
        linear = self.forward_speed * dist_scale
        angular = steering * 1.0  # Gentler steering gain

        return Velocity(linear=linear, angular=angular)

    def _handle_backing(self, context: RobotContext) -> Velocity:
        """Back up to create space before replanning."""
        context.set_state(RobotState.AVOIDING)

        # Check if we've backed up long enough
        if self._backing_start_time:
            elapsed = time.time() - self._backing_start_time
            if elapsed >= self._backing_duration:
                # Done backing, replan
                self._driving_state = DrivingState.PLANNING
                self._backing_start_time = None
                return Velocity(linear=0.0, angular=0.0)

        # Continue backing up
        return Velocity(linear=-self.backup_speed, angular=0.0)

    def _pick_exploration_direction(self) -> None:
        """Pick direction toward unexplored areas, preferring directions close to current heading."""
        if self._memory and self._current_pose:
            unexplored = self._memory.get_unexplored_directions(
                self._current_pose,
                check_distance=self.exploration_range  # Look further for unexplored territory
            )
            if unexplored:
                # Score each unexplored direction by how close it is to current heading
                # Lower angle difference = better score
                scored_directions = []
                for direction in unexplored:
                    if direction in DIRECTION_ANGLES:
                        target_angle = DIRECTION_ANGLES[direction]
                        # Angle difference from current goal (to maintain momentum)
                        angle_diff = abs(target_angle - self._goal_direction)
                        # Normalize to [-pi, pi]
                        if angle_diff > math.pi:
                            angle_diff = 2 * math.pi - angle_diff
                        scored_directions.append((direction, angle_diff))

                if scored_directions:
                    # Sort by angle difference (prefer directions close to current heading)
                    scored_directions.sort(key=lambda x: x[1])

                    # Pick from the best candidates (top 3) with some randomness
                    best_candidates = scored_directions[:min(3, len(scored_directions))]
                    direction, _ = random.choice(best_candidates)
                    self._goal_direction = DIRECTION_ANGLES[direction]
                    return

        # Fallback: random direction (biased forward)
        self._goal_direction = random.gauss(0.0, math.pi / 4)
        self._goal_direction = max(-math.pi / 2, min(math.pi / 2, self._goal_direction))

    def _start_recovery(self, context: RobotContext, obstacle_angle: float) -> Velocity:
        """Start the recovery state machine to get unstuck."""
        self._recovery_state = RecoveryState.BACKING
        self._recovery_start_time = time.time()
        self._recovery_turn_direction = -1 if obstacle_angle > 0 else 1
        context.set_state(RobotState.AVOIDING)
        return Velocity(linear=-self.backup_speed, angular=0.0)

    def _handle_recovery(self, context: RobotContext) -> Velocity:
        """Execute the recovery state machine."""
        elapsed = time.time() - self._recovery_start_time
        context.set_state(RobotState.AVOIDING)

        if self._recovery_state == RecoveryState.BACKING:
            if elapsed < self.backup_duration:
                return Velocity(linear=-self.backup_speed, angular=0.0)
            else:
                self._recovery_state = RecoveryState.TURNING
                self._recovery_start_time = time.time()
                return Velocity(linear=0.0, angular=self.turn_speed * self._recovery_turn_direction)

        elif self._recovery_state == RecoveryState.TURNING:
            if elapsed < self.turn_duration:
                return Velocity(linear=0.0, angular=self.turn_speed * self._recovery_turn_direction)
            else:
                self._reset_recovery()
                self._reset_driving()  # Go back to PLANNING after recovery
                return Velocity(linear=0.0, angular=0.0)

        self._reset_recovery()
        self._reset_driving()
        return Velocity.stop()

    @property
    def memory(self) -> Optional[ExplorationMemory]:
        """Get the exploration memory."""
        return self._memory

    def get_status(self) -> dict:
        """Get current exploration status."""
        status = {
            "goal_direction": self._goal_direction,
            "recovery_state": self._recovery_state.name,
            "driving_state": self._driving_state.name,
            "target_heading": self._target_heading,
        }
        if self._memory:
            status["cells_explored"] = len(self._memory.visited_cells)
            status["memory_name"] = self._memory_name
            status["coverage_percent"] = self._memory.get_coverage_percentage()
            if self._current_pose:
                status["new_area_confidence"] = self._memory.get_new_area_confidence(self._current_pose)
        return status
