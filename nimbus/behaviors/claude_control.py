"""
Claude Control behavior - API-driven movement with distance/rotation targets.

Enables Claude (or any API client) to issue blocking movement commands:
- Move X meters forward/backward
- Turn Y degrees left/right

Safety is enabled - obstacles will stop movement.
"""

import math
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Literal, Optional

from nimbus.core.state import RobotContext, RobotState, Pose2D, Velocity
from .base import Behavior


class CommandState(Enum):
    """State of the current command."""
    IDLE = auto()
    MOVING = auto()
    TURNING = auto()
    COMPLETE = auto()
    ERROR = auto()


@dataclass
class ClaudeCommand:
    """A movement command from Claude."""
    command_type: Literal["move", "turn"]
    target: float           # meters for move, degrees for turn
    speed: float            # m/s for move, rad/s for turn
    timeout: float = 30.0
    start_pose: Optional[Pose2D] = None
    start_time: Optional[float] = None
    result: Optional[str] = None  # success/timeout/cancelled/obstacle
    actual: Optional[float] = None  # actual distance/rotation achieved
    accumulated_rotation: float = 0.0  # for tracking rotation across wraparound
    last_distance: float = 0.0  # track last computed distance for move commands
    open_loop: bool = False  # True when using time-based fallback (no odom feedback)


def _normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class ClaudeControlBehavior(Behavior):
    """
    API-driven movement with distance/rotation targets.

    Commands are blocking - the API waits until movement completes.
    New commands cancel any in-progress movement.
    """

    name = "claude_control"
    description = "API-driven movement with distance/rotation targets"
    priority = 50  # Above wander, below motor_test

    # Safety enabled - obstacles will stop movement
    bypass_safety = False

    # Tolerances
    LINEAR_TOLERANCE = 0.02    # 2cm
    ANGULAR_TOLERANCE = 2.0    # 2 degrees
    POSE_SANITY_LIMIT = 1000.0  # Max plausible pose value (meters)

    # Default speeds
    DEFAULT_LINEAR_SPEED = 0.15   # m/s
    DEFAULT_ANGULAR_SPEED = 0.5   # rad/s (~28°/s)

    # Open-loop fallback: number of compute cycles with no odom change before switching
    ODOM_STALE_CYCLES = 5  # ~0.5s at 10Hz control loop

    def __init__(self):
        super().__init__()
        self._lock = threading.Lock()
        self._state = CommandState.IDLE
        self._command: Optional[ClaudeCommand] = None
        self._completion_event = threading.Event()
        self._last_theta: Optional[float] = None
        self._odom_check_cycles: int = 0  # cycles since command started
        self._odom_ever_changed: bool = False  # whether odom has changed from initial

    def activate(self) -> None:
        """Called when behavior becomes active."""
        super().activate()
        with self._lock:
            self._state = CommandState.IDLE
            self._command = None
            self._completion_event.clear()
            self._last_theta = None
            self._odom_check_cycles = 0
            self._odom_ever_changed = False

    def deactivate(self) -> None:
        """Called when behavior is deactivated."""
        super().deactivate()
        with self._lock:
            # Cancel any pending command
            if self._command and self._command.result is None:
                self._command.result = "cancelled"
                self._command.actual = self._calculate_progress()
            self._state = CommandState.IDLE
            self._completion_event.set()

    def reset(self) -> None:
        """Reset behavior state."""
        with self._lock:
            self._state = CommandState.IDLE
            self._command = None
            self._completion_event.clear()
            self._last_theta = None
            self._odom_check_cycles = 0
            self._odom_ever_changed = False

    def set_command(self, command: ClaudeCommand) -> None:
        """
        Set a new movement command.

        Cancels any in-progress command and starts the new one.
        """
        with self._lock:
            # Cancel existing command
            if self._command and self._command.result is None:
                self._command.result = "cancelled"
                self._command.actual = self._calculate_progress()
                self._completion_event.set()

            # Set new command
            self._command = command
            self._command.start_time = time.time()
            self._completion_event.clear()
            self._last_theta = None
            self._odom_check_cycles = 0
            self._odom_ever_changed = False

            if command.command_type == "move":
                self._state = CommandState.MOVING
            else:
                self._state = CommandState.TURNING

    def stop_command(self) -> Optional[ClaudeCommand]:
        """Stop the current command and return its result."""
        with self._lock:
            if self._command:
                self._command.result = "cancelled"
                self._command.actual = self._calculate_progress()
                self._state = CommandState.IDLE
                self._completion_event.set()
                return self._command
            return None

    def wait_for_completion(self, timeout: Optional[float] = None) -> bool:
        """
        Wait for the current command to complete.

        Returns True if completed, False if timeout.
        """
        return self._completion_event.wait(timeout=timeout)

    def get_result(self) -> Optional[ClaudeCommand]:
        """Get the current/last command with its result."""
        with self._lock:
            return self._command

    def get_status(self) -> dict:
        """Get current status for API."""
        with self._lock:
            return {
                "state": self._state.name.lower(),
                "command": {
                    "type": self._command.command_type,
                    "target": self._command.target,
                    "speed": self._command.speed,
                    "progress": self._calculate_progress(),
                    "elapsed": time.time() - self._command.start_time if self._command.start_time else 0,
                } if self._command else None,
            }

    def compute(self, context: RobotContext) -> Optional[Velocity]:
        """Compute velocity based on current command."""
        with self._lock:
            if self._state == CommandState.IDLE:
                return Velocity.stop()

            if self._state in (CommandState.COMPLETE, CommandState.ERROR):
                return Velocity.stop()

            if not self._command:
                return Velocity.stop()

            # Initialize start pose on first compute
            sensors = context.sensors
            if not sensors:
                return Velocity.stop()

            if self._command.start_pose is None:
                self._command.start_pose = sensors.pose
                self._last_theta = sensors.pose.theta
                self._command.accumulated_rotation = 0.0

            # Check timeout
            elapsed = time.time() - self._command.start_time
            if elapsed > self._command.timeout:
                self._complete_command("timeout")
                return Velocity.stop()

            # Check for obstacle in forward arc only
            if self._state == CommandState.MOVING and self._command.target > 0:
                # Only stop for obstacles within ±45° of front
                angle = math.atan2(
                    math.sin(sensors.obstacle_direction),
                    math.cos(sensors.obstacle_direction),
                )
                if abs(angle) <= math.radians(45) and sensors.closest_obstacle < 0.15:
                    self._complete_command("obstacle")
                    return Velocity.stop()

            # Execute command
            if self._state == CommandState.MOVING:
                return self._compute_move(context)
            else:
                return self._compute_turn(context)

    def _compute_move(self, context: RobotContext) -> Velocity:
        """Compute velocity for move command."""
        sensors = context.sensors
        current_pose = sensors.pose
        start_pose = self._command.start_pose
        target_distance = abs(self._command.target)

        # Track whether odometry is providing feedback
        self._odom_check_cycles += 1

        # Detect garbage odom (CDR parsing artifacts produce extreme values)
        pose_sane = (abs(current_pose.x) < self.POSE_SANITY_LIMIT
                     and abs(current_pose.y) < self.POSE_SANITY_LIMIT
                     and abs(start_pose.x) < self.POSE_SANITY_LIMIT
                     and abs(start_pose.y) < self.POSE_SANITY_LIMIT)

        if pose_sane:
            distance_traveled = current_pose.distance_to(start_pose)
            if distance_traveled > self.LINEAR_TOLERANCE:
                self._odom_ever_changed = True
        else:
            distance_traveled = 0.0

        # Open-loop fallback: if odom hasn't changed after enough cycles, use time-based
        if (not self._odom_ever_changed
                and not self._command.open_loop
                and self._odom_check_cycles >= self.ODOM_STALE_CYCLES):
            self._command.open_loop = True

        if self._command.open_loop:
            # Time-based completion
            elapsed = time.time() - self._command.start_time
            target_time = target_distance / self._command.speed
            if elapsed >= target_time:
                self._complete_command("completed_open_loop")
                return Velocity.stop()
        else:
            # Odometry-based completion
            self._command.last_distance = distance_traveled
            if distance_traveled >= target_distance - self.LINEAR_TOLERANCE:
                self._complete_command("completed")
                return Velocity.stop()

        # Calculate velocity (direction based on sign of target)
        direction = 1.0 if self._command.target > 0 else -1.0
        linear = direction * self._command.speed

        # Slow down as we approach target (only useful with odom feedback)
        if not self._command.open_loop:
            remaining = target_distance - distance_traveled
            if remaining < 0.1:
                linear *= max(0.3, remaining / 0.1)

        context.set_state(RobotState.NAVIGATING)
        return Velocity(linear=linear, angular=0.0)

    def _compute_turn(self, context: RobotContext) -> Velocity:
        """Compute velocity for turn command."""
        sensors = context.sensors
        current_theta = sensors.pose.theta

        # Detect garbage theta (valid range is [-pi, pi])
        theta_sane = abs(current_theta) <= math.pi + 0.01

        # Track rotation with wraparound handling (only with sane values)
        if self._last_theta is not None and theta_sane:
            delta = _normalize_angle(current_theta - self._last_theta)
            self._command.accumulated_rotation += delta
        if theta_sane:
            self._last_theta = current_theta

        # Convert target to radians
        target_radians = math.radians(self._command.target)

        # Track whether odometry is providing feedback
        self._odom_check_cycles += 1
        if theta_sane and abs(self._command.accumulated_rotation) > math.radians(self.ANGULAR_TOLERANCE):
            self._odom_ever_changed = True

        # Open-loop fallback: if odom hasn't changed after enough cycles, use time-based
        if (not self._odom_ever_changed
                and not self._command.open_loop
                and self._odom_check_cycles >= self.ODOM_STALE_CYCLES):
            self._command.open_loop = True

        if self._command.open_loop:
            # Time-based completion
            elapsed = time.time() - self._command.start_time
            target_time = abs(target_radians) / self._command.speed
            if elapsed >= target_time:
                self._complete_command("completed_open_loop")
                return Velocity.stop()
        else:
            # Odometry-based completion
            rotation_achieved = self._command.accumulated_rotation
            angular_tolerance_rad = math.radians(self.ANGULAR_TOLERANCE)
            if abs(rotation_achieved) >= abs(target_radians) - angular_tolerance_rad:
                self._complete_command("completed")
                return Velocity.stop()

        # Calculate angular velocity (direction based on sign of target)
        direction = 1.0 if target_radians > 0 else -1.0
        angular = direction * self._command.speed

        # Slow down as we approach target (only useful with odom feedback)
        if not self._command.open_loop:
            rotation_achieved = self._command.accumulated_rotation
            remaining = abs(target_radians) - abs(rotation_achieved)
            if remaining < 0.2:
                angular *= max(0.3, remaining / 0.2)

        context.set_state(RobotState.NAVIGATING)
        return Velocity(linear=0.0, angular=angular)

    def _complete_command(self, result: str) -> None:
        """Mark current command as complete."""
        if self._command:
            self._command.result = result
            if self._command.open_loop and result == "completed_open_loop":
                # In open-loop mode, report target as actual (best estimate)
                self._command.actual = abs(self._command.target)
            else:
                self._command.actual = self._calculate_progress()
        is_success = result in ("completed", "completed_open_loop")
        self._state = CommandState.COMPLETE if is_success else CommandState.ERROR
        self._completion_event.set()

    def _calculate_progress(self) -> float:
        """Calculate progress of current command."""
        if not self._command:
            return 0.0

        if self._command.command_type == "move":
            # Return distance in meters (tracked incrementally)
            return self._command.last_distance
        else:
            # Return rotation in degrees
            return math.degrees(self._command.accumulated_rotation)


def create_move_command(
    distance: float,
    speed: float = ClaudeControlBehavior.DEFAULT_LINEAR_SPEED,
    timeout: float = 30.0,
) -> ClaudeCommand:
    """Create a move command."""
    return ClaudeCommand(
        command_type="move",
        target=distance,
        speed=abs(speed),
        timeout=timeout,
    )


def create_turn_command(
    degrees: float,
    speed: float = ClaudeControlBehavior.DEFAULT_ANGULAR_SPEED,
    timeout: float = 30.0,
) -> ClaudeCommand:
    """Create a turn command."""
    return ClaudeCommand(
        command_type="turn",
        target=degrees,
        speed=abs(speed),
        timeout=timeout,
    )
