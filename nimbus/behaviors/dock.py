"""
Dock behavior — find and dock with the charging station.

Uses LIDAR intensity spikes from the charger's IR emitter to locate
the dock, navigate toward it, align with back facing the dock, and
reverse onto the charging contacts.

State machine:
    SEARCHING → APPROACHING → ALIGNING → REVERSING → DOCKED / FAILED
"""

from enum import Enum, auto
from typing import Optional
import json
import logging
import math
import time
from pathlib import Path

from nimbus.core.state import RobotContext, Velocity, Pose2D, RobotState
from nimbus.sensors.dock_detector import DockDetector, DockDetectorConfig, DockDetection
from .base import Behavior

logger = logging.getLogger(__name__)

DOCK_LOCATION_FILE = Path.home() / ".nimbus" / "dock_location.json"


class DockState(Enum):
    IDLE = auto()
    SEARCHING = auto()
    APPROACHING = auto()
    ALIGNING = auto()
    REVERSING = auto()
    DOCKED = auto()
    FAILED = auto()


class DockBehavior(Behavior):
    """Find and dock with the charging station via IR intensity detection.

    The Neato's LIDAR captures intensity data alongside distance. The
    charger's IR emitter creates intensity spikes that reveal its bearing.
    The robot rotates to find the signal, approaches, turns so its back
    faces the dock, then reverses onto the charging contacts.
    """

    name = "dock"
    description = "Find and dock with charging station"
    priority = 30

    def __init__(
        self,
        search_angular_speed: float = 0.3,      # rad/s while scanning
        approach_speed: float = 0.15,            # m/s toward dock
        approach_distance: float = 0.5,          # m — stop and align at this distance
        align_angular_speed: float = 0.25,       # rad/s for fine alignment
        align_tolerance_deg: float = 8.0,        # degrees — close enough to reverse
        reverse_speed: float = 0.05,             # m/s backward onto dock
        reverse_timeout: float = 12.0,           # seconds before giving up
        search_timeout: float = 30.0,            # seconds for full search rotation
        detector_config: Optional[DockDetectorConfig] = None,
    ):
        super().__init__()
        self._detector = DockDetector(detector_config)

        # Speeds and thresholds
        self._search_angular_speed = search_angular_speed
        self._approach_speed = approach_speed
        self._approach_distance = approach_distance
        self._align_angular_speed = align_angular_speed
        self._align_tolerance_deg = align_tolerance_deg
        self._reverse_speed = reverse_speed
        self._reverse_timeout = reverse_timeout
        self._search_timeout = search_timeout

        # State
        self._state = DockState.IDLE
        self._state_start_time = 0.0
        self._last_detection: Optional[DockDetection] = None
        self._search_start_theta = 0.0
        self._search_rotated = 0.0
        self._consecutive_detections = 0
        self._required_detections = 3  # need N consecutive before acting

        # Smoothed angle (exponential moving average to prevent oscillation)
        self._smoothed_angle_sin = 0.0
        self._smoothed_angle_cos = 1.0  # default: 0°
        self._angle_smooth_alpha = 0.3  # EMA weight (0=all history, 1=latest only)

    @property
    def state(self) -> DockState:
        return self._state

    @property
    def suppress_bumper_recovery(self) -> bool:
        """Suppress bumper recovery during reverse docking."""
        return self._state == DockState.REVERSING

    def activate(self) -> None:
        super().activate()
        self._state = DockState.SEARCHING
        self._state_start_time = time.time()
        self._last_detection = None
        self._consecutive_detections = 0
        self._search_rotated = 0.0
        logger.info("Dock behavior activated — searching for charger IR")

    def deactivate(self) -> None:
        super().deactivate()
        self._state = DockState.IDLE
        logger.info("Dock behavior deactivated")

    def reset(self) -> None:
        self._state = DockState.SEARCHING
        self._state_start_time = time.time()
        self._last_detection = None
        self._consecutive_detections = 0
        self._search_rotated = 0.0

    def compute(self, context: RobotContext) -> Optional[Velocity]:
        sensors = context.sensors
        if sensors is None:
            return Velocity.stop()

        # Run IR detection on every tick
        detection = self._detector.detect(
            sensors.lidar_ranges, sensors.lidar_intensities
        )

        if detection and detection.detected:
            self._consecutive_detections += 1
            self._last_detection = detection
            # Update smoothed angle (EMA in sin/cos space for wraparound)
            a = self._angle_smooth_alpha
            rad = math.radians(detection.angle_deg)
            self._smoothed_angle_sin = (1 - a) * self._smoothed_angle_sin + a * math.sin(rad)
            self._smoothed_angle_cos = (1 - a) * self._smoothed_angle_cos + a * math.cos(rad)
        else:
            self._consecutive_detections = 0

        # Dispatch to current state
        if self._state == DockState.SEARCHING:
            return self._do_searching(context, detection)
        elif self._state == DockState.APPROACHING:
            return self._do_approaching(context, detection)
        elif self._state == DockState.ALIGNING:
            return self._do_aligning(context, detection)
        elif self._state == DockState.REVERSING:
            return self._do_reversing(context)
        elif self._state == DockState.DOCKED:
            return Velocity.stop()
        elif self._state == DockState.FAILED:
            return Velocity.stop()
        else:
            return Velocity.stop()

    @property
    def _smoothed_angle_deg(self) -> int:
        """Get the smoothed IR bearing in degrees."""
        rad = math.atan2(self._smoothed_angle_sin, self._smoothed_angle_cos)
        return int(round(math.degrees(rad))) % 360

    def _do_searching(
        self, context: RobotContext, detection: Optional[DockDetection]
    ) -> Velocity:
        """Rotate in place looking for IR intensity spike."""
        # Check timeout
        if time.time() - self._state_start_time > self._search_timeout:
            logger.warning("Dock search timed out — no IR signal found")
            self._transition(DockState.FAILED)
            return Velocity.stop()

        # If we have consistent detection, transition
        if self._consecutive_detections >= self._required_detections:
            logger.info(
                "Dock IR found at %d° dist=%.2fm — approaching",
                self._last_detection.angle_deg,
                self._last_detection.distance_m,
            )
            self._transition(DockState.APPROACHING)
            return self._do_approaching(context, detection)

        # Keep rotating
        context.set_state(RobotState.NAVIGATING)
        return Velocity(linear=0.0, angular=self._search_angular_speed)

    def _do_approaching(
        self, context: RobotContext, detection: Optional[DockDetection]
    ) -> Velocity:
        """Drive toward the detected dock IR signal."""
        if self._last_detection is None:
            # Lost the signal — go back to searching
            logger.info("Lost dock IR during approach — re-searching")
            self._transition(DockState.SEARCHING)
            return Velocity(linear=0.0, angular=self._search_angular_speed)

        det = self._last_detection
        context.set_state(RobotState.NAVIGATING)

        # If close enough, switch to alignment
        if det.distance_m <= self._approach_distance:
            logger.info("Close to dock (%.2fm) — aligning", det.distance_m)
            self._transition(DockState.ALIGNING)
            return self._do_aligning(context, detection)

        # Steer toward the smoothed IR bearing (prevents oscillation
        # when charger has dual IR emitters)
        angle_error = self._smoothed_angle_deg
        if angle_error > 180:
            angle_error -= 360

        angle_error_rad = math.radians(angle_error)

        # Proportional steering
        angular = angle_error_rad * 1.2
        angular = max(-self._search_angular_speed * 2,
                     min(self._search_angular_speed * 2, angular))

        # Slow down when turning hard
        speed_scale = 1.0 - min(abs(angular) / 1.5, 0.7)
        linear = self._approach_speed * speed_scale

        return Velocity(linear=linear, angular=angular)

    def _do_aligning(
        self, context: RobotContext, detection: Optional[DockDetection]
    ) -> Velocity:
        """Rotate so the robot's back faces the dock (IR at 180°)."""
        if self._last_detection is None:
            logger.info("Lost dock IR during alignment — re-searching")
            self._transition(DockState.SEARCHING)
            return Velocity(linear=0.0, angular=self._search_angular_speed)

        context.set_state(RobotState.NAVIGATING)

        # Use smoothed angle to prevent oscillation from dual IR beams
        # We want the IR signal at 180° (directly behind us)
        smoothed = self._smoothed_angle_deg
        angle_to_180 = smoothed - 180
        if angle_to_180 > 180:
            angle_to_180 -= 360
        elif angle_to_180 < -180:
            angle_to_180 += 360

        if abs(angle_to_180) <= self._align_tolerance_deg:
            logger.info("Aligned with dock — reversing onto charger")
            self._transition(DockState.REVERSING)
            return self._do_reversing(context)

        # Rotate toward alignment
        # Positive angle_to_180 means IR is CW from 180° → rotate CW (negative angular)
        angular = -math.radians(angle_to_180) * 0.8
        angular = max(-self._align_angular_speed,
                     min(self._align_angular_speed, angular))

        return Velocity(linear=0.0, angular=angular)

    def _do_reversing(self, context: RobotContext) -> Velocity:
        """Drive backward onto the dock, watching for charging confirmation."""
        # Check if charging started
        sensors = context.sensors
        if sensors and sensors.charging:
            logger.info("Charging detected — docking successful!")
            self._transition(DockState.DOCKED)
            self._save_dock_location(sensors.pose)
            context.set_state(RobotState.IDLE)
            return Velocity.stop()

        # Check timeout
        if time.time() - self._state_start_time > self._reverse_timeout:
            logger.warning("Reverse docking timed out — no charging signal")
            self._transition(DockState.FAILED)
            return Velocity.stop()

        context.set_state(RobotState.NAVIGATING)
        # Drive backward slowly
        return Velocity(linear=-self._reverse_speed, angular=0.0)

    def _transition(self, new_state: DockState) -> None:
        """Transition to a new dock state."""
        logger.debug("Dock state: %s → %s", self._state.name, new_state.name)
        self._state = new_state
        self._state_start_time = time.time()
        if new_state == DockState.SEARCHING:
            self._consecutive_detections = 0

    def _save_dock_location(self, pose: Pose2D) -> None:
        """Persist dock location for future docking attempts."""
        try:
            DOCK_LOCATION_FILE.parent.mkdir(parents=True, exist_ok=True)
            data = {"x": pose.x, "y": pose.y, "theta": pose.theta}
            DOCK_LOCATION_FILE.write_text(json.dumps(data))
            logger.info("Saved dock location: (%.2f, %.2f)", pose.x, pose.y)
        except Exception as e:
            logger.warning("Failed to save dock location: %s", e)

    @staticmethod
    def load_dock_location() -> Optional[Pose2D]:
        """Load previously saved dock location."""
        try:
            if DOCK_LOCATION_FILE.exists():
                data = json.loads(DOCK_LOCATION_FILE.read_text())
                return Pose2D(x=data["x"], y=data["y"], theta=data["theta"])
        except Exception:
            pass
        return None

    def get_status(self) -> dict:
        """Get current docking status for API/dashboard."""
        return {
            "state": self._state.name,
            "last_detection": {
                "angle_deg": self._last_detection.angle_deg,
                "distance_m": self._last_detection.distance_m,
                "confidence": self._last_detection.confidence,
                "intensity_peak": self._last_detection.intensity_peak,
            } if self._last_detection else None,
            "consecutive_detections": self._consecutive_detections,
            "elapsed_s": time.time() - self._state_start_time,
        }
