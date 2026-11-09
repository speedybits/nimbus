"""Tests for dock behavior state machine."""

import pytest
from datetime import datetime
from unittest.mock import patch

from nimbus.behaviors.dock import DockBehavior, DockState
from nimbus.sensors.dock_detector import DockDetectorConfig
from nimbus.core.state import RobotContext, SensorSnapshot, Pose2D, Velocity


def _make_context(
    lidar_intensities=None,
    lidar_ranges=None,
    charging=False,
    battery_percent=50,
    pose=None,
):
    """Create a RobotContext with the given sensor data."""
    ctx = RobotContext()
    snapshot = SensorSnapshot(
        timestamp=datetime.now(),
        pose=pose or Pose2D(),
        velocity=Velocity(),
        lidar_ranges=lidar_ranges or tuple([2.0] * 360),
        closest_obstacle=2.0,
        obstacle_direction=0.0,
        bumper_triggered=False,
        lidar_intensities=lidar_intensities or tuple([600.0] * 360),
        charging=charging,
        battery_percent=battery_percent,
    )
    ctx.update_sensors(snapshot)
    return ctx


def _make_ir_intensities(peak_angle=0, peak_intensity=4500.0, peak_width=8, base=600.0):
    """Create intensity array with an IR peak at the given angle."""
    intensities = [base] * 360
    half = peak_width // 2
    for offset in range(-half, half + 1):
        angle = (peak_angle + offset) % 360
        taper = 1.0 - abs(offset) / (half + 1)
        intensities[angle] = base + (peak_intensity - base) * taper
    return tuple(intensities)


class TestDockBehavior:
    """Test dock behavior state machine transitions."""

    def setup_method(self):
        self.behavior = DockBehavior(
            search_timeout=5.0,
            reverse_timeout=5.0,
            detector_config=DockDetectorConfig(
                intensity_threshold=400.0,
                intensity_ratio=3.5,
            ),
        )
        self.behavior.activate()

    def test_initial_state_is_searching(self):
        assert self.behavior.state == DockState.SEARCHING

    def test_searching_rotates_in_place(self):
        ctx = _make_context()
        vel = self.behavior.compute(ctx)
        assert vel.linear == 0.0
        assert vel.angular > 0.0  # rotating

    def test_searching_transitions_to_approaching(self):
        """After N consecutive detections, should transition to APPROACHING."""
        intensities = _make_ir_intensities(peak_angle=45)
        ctx = _make_context(lidar_intensities=intensities)

        # Need multiple consecutive detections
        for _ in range(5):
            self.behavior.compute(ctx)

        assert self.behavior.state in (DockState.APPROACHING, DockState.ALIGNING)

    def test_searching_timeout_fails(self):
        """Search should fail after timeout with no IR signal."""
        ctx = _make_context()  # no IR peak

        # Simulate timeout
        self.behavior._state_start_time -= 10.0
        vel = self.behavior.compute(ctx)

        assert self.behavior.state == DockState.FAILED
        assert vel.linear == 0.0
        assert vel.angular == 0.0

    def test_approaching_drives_toward_ir(self):
        """During approach, robot should drive toward the IR signal."""
        # Set up state as if we've already detected and transitioned
        self.behavior._state = DockState.APPROACHING
        self.behavior._consecutive_detections = 5

        # IR peak at 30° (slightly left)
        intensities = _make_ir_intensities(peak_angle=30)
        ranges = tuple([2.0] * 360)  # dock is 2m away
        ctx = _make_context(lidar_intensities=intensities, lidar_ranges=ranges)

        vel = self.behavior.compute(ctx)
        assert vel.linear > 0.0   # driving forward
        assert vel.angular > 0.0  # turning left toward IR

    def test_approaching_transitions_to_aligning_when_close(self):
        """Should switch to aligning when close to dock."""
        self.behavior._state = DockState.APPROACHING
        self.behavior._consecutive_detections = 5

        # IR peak at close range
        intensities = _make_ir_intensities(peak_angle=10)
        ranges = list([2.0] * 360)
        # Set range at peak angles to 0.3m (below approach_distance)
        for a in range(6, 15):
            ranges[a] = 0.3
        ctx = _make_context(lidar_intensities=intensities, lidar_ranges=tuple(ranges))

        self.behavior.compute(ctx)

        assert self.behavior.state in (DockState.ALIGNING, DockState.REVERSING)

    def test_aligning_rotates_ir_to_180(self):
        """Aligning should rotate until IR is at 180° (behind robot)."""
        self.behavior._state = DockState.ALIGNING
        self.behavior._consecutive_detections = 5

        # IR peak at 90° — needs to rotate to get it to 180°
        intensities = _make_ir_intensities(peak_angle=90)
        ctx = _make_context(lidar_intensities=intensities)

        vel = self.behavior.compute(ctx)
        assert vel.linear == 0.0  # rotating in place
        assert vel.angular != 0.0

    def test_aligning_transitions_to_reversing(self):
        """When IR is near 180°, should transition to reversing."""
        self.behavior._state = DockState.ALIGNING
        self.behavior._consecutive_detections = 5

        # IR peak at 180° (already aligned)
        intensities = _make_ir_intensities(peak_angle=180)
        ctx = _make_context(lidar_intensities=intensities)

        # Run several ticks to let the smoothed angle EMA converge to 180°
        for _ in range(15):
            self.behavior.compute(ctx)

        assert self.behavior.state == DockState.REVERSING

    def test_reversing_drives_backward(self):
        """During reversing, robot should drive backward."""
        self.behavior._state = DockState.REVERSING
        self.behavior._state_start_time = __import__('time').time()

        ctx = _make_context(charging=False)
        vel = self.behavior.compute(ctx)

        assert vel.linear < 0.0   # backward
        assert vel.angular == 0.0

    def test_reversing_detects_charging(self):
        """Charging confirmation should transition to DOCKED."""
        self.behavior._state = DockState.REVERSING
        self.behavior._state_start_time = __import__('time').time()

        ctx = _make_context(charging=True)
        vel = self.behavior.compute(ctx)

        assert self.behavior.state == DockState.DOCKED
        assert vel.linear == 0.0
        assert vel.angular == 0.0

    def test_reversing_timeout_fails(self):
        """Reversing should fail after timeout."""
        self.behavior._state = DockState.REVERSING
        self.behavior._state_start_time -= 20.0  # simulate timeout

        ctx = _make_context(charging=False)
        vel = self.behavior.compute(ctx)

        assert self.behavior.state == DockState.FAILED

    def test_docked_state_stops(self):
        """DOCKED state should output zero velocity."""
        self.behavior._state = DockState.DOCKED
        ctx = _make_context()
        vel = self.behavior.compute(ctx)
        assert vel.is_stopped()

    def test_failed_state_stops(self):
        """FAILED state should output zero velocity."""
        self.behavior._state = DockState.FAILED
        ctx = _make_context()
        vel = self.behavior.compute(ctx)
        assert vel.is_stopped()

    def test_suppress_bumper_recovery_only_during_reversing(self):
        """suppress_bumper_recovery should only be True during REVERSING."""
        self.behavior._state = DockState.SEARCHING
        assert self.behavior.suppress_bumper_recovery is False

        self.behavior._state = DockState.APPROACHING
        assert self.behavior.suppress_bumper_recovery is False

        self.behavior._state = DockState.REVERSING
        assert self.behavior.suppress_bumper_recovery is True

        self.behavior._state = DockState.DOCKED
        assert self.behavior.suppress_bumper_recovery is False

    def test_no_sensors_returns_stop(self):
        """Should return stop if no sensor data available."""
        ctx = RobotContext()  # no sensors
        vel = self.behavior.compute(ctx)
        assert vel.is_stopped()

    def test_get_status(self):
        """Status should include state and detection info."""
        status = self.behavior.get_status()
        assert status["state"] == "SEARCHING"
        assert status["last_detection"] is None
        assert "consecutive_detections" in status
        assert "elapsed_s" in status
