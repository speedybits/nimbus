"""
State machine and data structures for Nimbus.

The robot context provides thread-safe access to all robot state,
sensor data, and configuration. The state machine ensures clean
transitions between operating modes.
"""

from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum, auto
from typing import Callable, Optional
import threading


class RobotState(Enum):
    """Robot operating states."""

    IDLE = auto()           # Stationary, awaiting commands
    NAVIGATING = auto()     # Moving toward a goal
    AVOIDING = auto()       # Actively avoiding an obstacle
    EMERGENCY_STOP = auto() # Stopped due to safety trigger
    CALIBRATING = auto()    # Running calibration routine


@dataclass(frozen=True)
class Pose2D:
    """Immutable 2D pose (position + orientation)."""

    x: float = 0.0      # meters
    y: float = 0.0      # meters
    theta: float = 0.0  # radians, 0 = facing +X axis

    def distance_to(self, other: "Pose2D") -> float:
        """Euclidean distance to another pose."""
        import math
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def angle_to(self, other: "Pose2D") -> float:
        """Angle from this pose to another (radians)."""
        import math
        return math.atan2(other.y - self.y, other.x - self.x)


@dataclass(frozen=True)
class Velocity:
    """Immutable velocity command."""

    linear: float = 0.0   # m/s, positive = forward
    angular: float = 0.0  # rad/s, positive = counter-clockwise

    @classmethod
    def stop(cls) -> "Velocity":
        """Create a zero velocity (stop) command."""
        return cls(0.0, 0.0)

    def is_stopped(self) -> bool:
        """Check if this represents a stop command."""
        return abs(self.linear) < 0.001 and abs(self.angular) < 0.001


@dataclass(frozen=True)
class SensorSnapshot:
    """
    Immutable snapshot of all sensor data at a point in time.

    This is the primary data structure passed between components.
    Being immutable ensures thread-safety and predictable behavior.
    """

    timestamp: datetime
    pose: Pose2D
    velocity: Velocity
    lidar_ranges: tuple[float, ...]  # 360 floats, one per degree
    closest_obstacle: float          # meters to nearest obstacle
    obstacle_direction: float        # radians, angle to nearest obstacle

    @classmethod
    def empty(cls) -> "SensorSnapshot":
        """Create an empty snapshot (no sensor data yet)."""
        return cls(
            timestamp=datetime.now(),
            pose=Pose2D(),
            velocity=Velocity(),
            lidar_ranges=tuple([float('inf')] * 360),
            closest_obstacle=float('inf'),
            obstacle_direction=0.0
        )


class RobotContext:
    """
    Central state container for the robot.

    Thread-safe access to robot state, sensor data, and goals.
    This is the single source of truth for all components.

    Usage:
        ctx = RobotContext()
        ctx.set_state(RobotState.NAVIGATING)
        ctx.update_sensors(snapshot)

        # Read current state
        if ctx.state == RobotState.IDLE:
            ...
    """

    def __init__(self):
        self._lock = threading.RLock()
        self._state = RobotState.IDLE
        self._target: Optional[Pose2D] = None
        self._velocity_cmd = Velocity()
        self._latest_sensors: Optional[SensorSnapshot] = None
        self._state_listeners: list[Callable[[RobotState, RobotState], None]] = []
        self._current_behavior: Optional[str] = None

    # --- State Management ---

    @property
    def state(self) -> RobotState:
        """Current robot state."""
        with self._lock:
            return self._state

    def set_state(self, new_state: RobotState) -> None:
        """
        Transition to a new state.

        Notifies all registered listeners of the transition.
        """
        with self._lock:
            old_state = self._state
            if old_state == new_state:
                return
            self._state = new_state
            listeners = self._state_listeners.copy()

        # Notify listeners outside the lock
        for listener in listeners:
            try:
                listener(old_state, new_state)
            except Exception:
                pass  # Don't let listener errors crash state machine

    def on_state_change(self, callback: Callable[[RobotState, RobotState], None]) -> None:
        """Register a callback for state transitions."""
        with self._lock:
            self._state_listeners.append(callback)

    # --- Sensor Data ---

    @property
    def sensors(self) -> Optional[SensorSnapshot]:
        """Latest sensor snapshot."""
        with self._lock:
            return self._latest_sensors

    def update_sensors(self, snapshot: SensorSnapshot) -> None:
        """Update with new sensor data."""
        with self._lock:
            self._latest_sensors = snapshot

    # --- Navigation Target ---

    @property
    def target(self) -> Optional[Pose2D]:
        """Current navigation target."""
        with self._lock:
            return self._target

    def set_target(self, target: Optional[Pose2D]) -> None:
        """Set navigation target."""
        with self._lock:
            self._target = target

    def clear_target(self) -> None:
        """Clear navigation target."""
        with self._lock:
            self._target = None

    # --- Velocity Command ---

    @property
    def velocity_cmd(self) -> Velocity:
        """Current velocity command."""
        with self._lock:
            return self._velocity_cmd

    def set_velocity_cmd(self, velocity: Velocity) -> None:
        """Set velocity command."""
        with self._lock:
            self._velocity_cmd = velocity

    # --- Behavior ---

    @property
    def current_behavior(self) -> Optional[str]:
        """Name of currently active behavior."""
        with self._lock:
            return self._current_behavior

    def set_behavior(self, behavior_name: Optional[str]) -> None:
        """Set active behavior name."""
        with self._lock:
            self._current_behavior = behavior_name

    # --- Convenience Methods ---

    def is_safe(self) -> bool:
        """Check if robot is in a safe state (not emergency stopped)."""
        return self.state != RobotState.EMERGENCY_STOP

    def distance_to_target(self) -> Optional[float]:
        """Distance to current target, or None if no target/sensors."""
        with self._lock:
            if self._target is None or self._latest_sensors is None:
                return None
            return self._latest_sensors.pose.distance_to(self._target)

    def to_dict(self) -> dict:
        """Export state as dictionary (for API responses)."""
        with self._lock:
            sensors = self._latest_sensors
            return {
                "state": self._state.name,
                "pose": {
                    "x": sensors.pose.x if sensors else 0,
                    "y": sensors.pose.y if sensors else 0,
                    "theta": sensors.pose.theta if sensors else 0,
                },
                "velocity": {
                    "linear": sensors.velocity.linear if sensors else 0,
                    "angular": sensors.velocity.angular if sensors else 0,
                },
                "closest_obstacle": sensors.closest_obstacle if sensors else float('inf'),
                "target": {
                    "x": self._target.x,
                    "y": self._target.y,
                    "theta": self._target.theta,
                } if self._target else None,
                "current_behavior": self._current_behavior,
            }
