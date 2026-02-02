"""
Unit tests for Claude control behavior.
"""

import math
import pytest
import threading
import time
from datetime import datetime

from nimbus.core.state import RobotContext, RobotState, Pose2D, Velocity, SensorSnapshot
from nimbus.behaviors.claude_control import (
    ClaudeControlBehavior,
    ClaudeCommand,
    CommandState,
    create_move_command,
    create_turn_command,
    _normalize_angle,
)


class TestNormalizeAngle:
    """Tests for angle normalization."""

    def test_already_normalized(self):
        """Angles in range should stay unchanged."""
        assert abs(_normalize_angle(0.0)) < 0.001
        assert abs(_normalize_angle(1.0) - 1.0) < 0.001
        assert abs(_normalize_angle(-1.0) + 1.0) < 0.001

    def test_positive_overflow(self):
        """Angles > pi should wrap to negative."""
        result = _normalize_angle(math.pi + 0.1)
        assert result < 0
        assert abs(result - (-math.pi + 0.1)) < 0.001

    def test_negative_overflow(self):
        """Angles < -pi should wrap to positive."""
        result = _normalize_angle(-math.pi - 0.1)
        assert result > 0
        assert abs(result - (math.pi - 0.1)) < 0.001

    def test_full_rotation(self):
        """Full rotations should normalize to zero."""
        assert abs(_normalize_angle(2 * math.pi)) < 0.001
        assert abs(_normalize_angle(-2 * math.pi)) < 0.001


class TestClaudeControlBehavior:
    """Tests for ClaudeControlBehavior."""

    @pytest.fixture
    def behavior(self):
        """Create a fresh behavior instance."""
        b = ClaudeControlBehavior()
        b.activate()
        return b

    @pytest.fixture
    def mock_context(self):
        """Create a mock robot context with sensor data."""
        ctx = RobotContext()
        snapshot = SensorSnapshot(
            timestamp=datetime.now(),
            pose=Pose2D(x=0.0, y=0.0, theta=0.0),
            velocity=Velocity(linear=0.0, angular=0.0),
            lidar_ranges=tuple([2.0] * 360),
            closest_obstacle=2.0,
            obstacle_direction=0.0,
        )
        ctx.update_sensors(snapshot)
        return ctx

    def _update_pose(self, ctx: RobotContext, x: float, y: float, theta: float):
        """Helper to update the pose in a context."""
        snapshot = SensorSnapshot(
            timestamp=datetime.now(),
            pose=Pose2D(x=x, y=y, theta=theta),
            velocity=Velocity(linear=0.1, angular=0.0),
            lidar_ranges=tuple([2.0] * 360),
            closest_obstacle=2.0,
            obstacle_direction=0.0,
        )
        ctx.update_sensors(snapshot)

    def test_initial_state_is_idle(self, behavior):
        """Behavior should start in idle state."""
        status = behavior.get_status()
        assert status["state"] == "idle"
        assert status["command"] is None

    def test_idle_returns_stop(self, behavior, mock_context):
        """Idle state should return zero velocity."""
        velocity = behavior.compute(mock_context)
        assert velocity.is_stopped()

    def test_move_command_starts_moving(self, behavior, mock_context):
        """Setting move command should transition to moving state."""
        cmd = create_move_command(1.0)
        behavior.set_command(cmd)

        status = behavior.get_status()
        assert status["state"] == "moving"

        velocity = behavior.compute(mock_context)
        assert velocity.linear > 0

    def test_move_forward_positive_velocity(self, behavior, mock_context):
        """Positive distance should produce positive linear velocity."""
        cmd = create_move_command(1.0, speed=0.15)
        behavior.set_command(cmd)

        velocity = behavior.compute(mock_context)
        assert velocity.linear > 0
        assert velocity.angular == 0

    def test_move_backward_negative_velocity(self, behavior, mock_context):
        """Negative distance should produce negative linear velocity."""
        cmd = create_move_command(-1.0, speed=0.15)
        behavior.set_command(cmd)

        velocity = behavior.compute(mock_context)
        assert velocity.linear < 0
        assert velocity.angular == 0

    def test_move_completes_at_distance(self, behavior, mock_context):
        """Move should complete when target distance is reached."""
        cmd = create_move_command(0.5, speed=0.15)
        behavior.set_command(cmd)

        # First compute initializes start pose
        behavior.compute(mock_context)

        # Simulate robot moved 0.5m
        self._update_pose(mock_context, 0.5, 0.0, 0.0)
        velocity = behavior.compute(mock_context)

        # Should have completed
        result = behavior.get_result()
        assert result.result == "completed"
        assert velocity.is_stopped()

    def test_turn_command_starts_turning(self, behavior, mock_context):
        """Setting turn command should transition to turning state."""
        cmd = create_turn_command(90.0)
        behavior.set_command(cmd)

        status = behavior.get_status()
        assert status["state"] == "turning"

        velocity = behavior.compute(mock_context)
        assert velocity.angular > 0

    def test_turn_left_positive_angular(self, behavior, mock_context):
        """Positive degrees should produce positive angular velocity (CCW)."""
        cmd = create_turn_command(90.0, speed=0.5)
        behavior.set_command(cmd)

        velocity = behavior.compute(mock_context)
        assert velocity.angular > 0
        assert velocity.linear == 0

    def test_turn_right_negative_angular(self, behavior, mock_context):
        """Negative degrees should produce negative angular velocity (CW)."""
        cmd = create_turn_command(-90.0, speed=0.5)
        behavior.set_command(cmd)

        velocity = behavior.compute(mock_context)
        assert velocity.angular < 0
        assert velocity.linear == 0

    def test_turn_completes_at_angle(self, behavior, mock_context):
        """Turn should complete when target rotation is reached."""
        cmd = create_turn_command(90.0, speed=0.5)
        behavior.set_command(cmd)

        # First compute initializes start pose
        behavior.compute(mock_context)

        # Simulate robot turned 90 degrees
        self._update_pose(mock_context, 0.0, 0.0, math.radians(90))
        velocity = behavior.compute(mock_context)

        # Should have completed
        result = behavior.get_result()
        assert result.result == "completed"
        assert velocity.is_stopped()

    def test_timeout_triggers_error(self, behavior, mock_context):
        """Command should fail on timeout."""
        cmd = create_move_command(10.0, timeout=0.01)  # Very short timeout
        behavior.set_command(cmd)

        # First compute initializes
        behavior.compute(mock_context)

        # Wait for timeout
        time.sleep(0.02)

        # Next compute should detect timeout
        velocity = behavior.compute(mock_context)

        result = behavior.get_result()
        assert result.result == "timeout"
        assert velocity.is_stopped()

    def test_new_command_cancels_existing(self, behavior, mock_context):
        """New command should cancel in-progress command."""
        cmd1 = create_move_command(10.0)
        behavior.set_command(cmd1)
        behavior.compute(mock_context)

        cmd2 = create_turn_command(90.0)
        behavior.set_command(cmd2)

        # First command should be cancelled
        # (Result is from second command now)
        status = behavior.get_status()
        assert status["state"] == "turning"

    def test_stop_command_cancels(self, behavior, mock_context):
        """Stop should cancel current command."""
        cmd = create_move_command(10.0)
        behavior.set_command(cmd)
        behavior.compute(mock_context)

        result = behavior.stop_command()

        assert result.result == "cancelled"
        assert behavior.get_status()["state"] == "idle"

    def test_obstacle_stops_forward_movement(self, behavior, mock_context):
        """Obstacle should stop forward movement."""
        cmd = create_move_command(1.0)
        behavior.set_command(cmd)
        behavior.compute(mock_context)

        # Simulate obstacle detected close
        snapshot = SensorSnapshot(
            timestamp=datetime.now(),
            pose=Pose2D(x=0.1, y=0.0, theta=0.0),
            velocity=Velocity(linear=0.1, angular=0.0),
            lidar_ranges=tuple([0.1] * 360),
            closest_obstacle=0.1,  # Very close obstacle
            obstacle_direction=0.0,
        )
        mock_context.update_sensors(snapshot)

        velocity = behavior.compute(mock_context)

        result = behavior.get_result()
        assert result.result == "obstacle"
        assert velocity.is_stopped()

    def test_obstacle_ignored_for_backward_movement(self, behavior, mock_context):
        """Obstacle detection should not stop backward movement."""
        cmd = create_move_command(-1.0)  # Backward
        behavior.set_command(cmd)
        behavior.compute(mock_context)

        # Simulate obstacle detected close (but we're moving backward)
        snapshot = SensorSnapshot(
            timestamp=datetime.now(),
            pose=Pose2D(x=-0.1, y=0.0, theta=0.0),
            velocity=Velocity(linear=-0.1, angular=0.0),
            lidar_ranges=tuple([0.1] * 360),
            closest_obstacle=0.1,
            obstacle_direction=0.0,
        )
        mock_context.update_sensors(snapshot)

        velocity = behavior.compute(mock_context)

        # Should still be moving backward
        result = behavior.get_result()
        assert result.result is None  # Not complete yet
        assert velocity.linear < 0

    def test_deactivate_cancels_command(self, behavior, mock_context):
        """Deactivating behavior should cancel pending command."""
        cmd = create_move_command(10.0)
        behavior.set_command(cmd)
        behavior.compute(mock_context)

        behavior.deactivate()

        result = behavior.get_result()
        assert result.result == "cancelled"

    def test_wait_for_completion_blocks(self, behavior, mock_context):
        """wait_for_completion should block until command completes."""
        cmd = create_move_command(0.1, timeout=0.1)
        behavior.set_command(cmd)

        completed = []

        def wait_thread():
            behavior.wait_for_completion(timeout=1.0)
            completed.append(True)

        thread = threading.Thread(target=wait_thread)
        thread.start()

        # Compute until timeout
        behavior.compute(mock_context)
        time.sleep(0.15)
        behavior.compute(mock_context)

        thread.join(timeout=1.0)
        assert completed == [True]


class TestCommandCreation:
    """Tests for command factory functions."""

    def test_create_move_command(self):
        """Should create move command with correct parameters."""
        cmd = create_move_command(1.5, speed=0.2, timeout=15.0)

        assert cmd.command_type == "move"
        assert cmd.target == 1.5
        assert cmd.speed == 0.2
        assert cmd.timeout == 15.0

    def test_create_turn_command(self):
        """Should create turn command with correct parameters."""
        cmd = create_turn_command(-45.0, speed=0.8, timeout=20.0)

        assert cmd.command_type == "turn"
        assert cmd.target == -45.0
        assert cmd.speed == 0.8
        assert cmd.timeout == 20.0

    def test_speed_always_positive(self):
        """Speed should be stored as absolute value."""
        cmd = create_move_command(1.0, speed=-0.2)
        assert cmd.speed == 0.2

        cmd = create_turn_command(90.0, speed=-0.5)
        assert cmd.speed == 0.5


class TestOpenLoopFallback:
    """Tests for open-loop (time-based) fallback when odometry is stale."""

    @pytest.fixture
    def behavior(self):
        """Create a fresh behavior instance."""
        b = ClaudeControlBehavior()
        b.activate()
        return b

    @pytest.fixture
    def stale_context(self):
        """Create a context where odometry never changes (always 0,0,0)."""
        ctx = RobotContext()
        snapshot = SensorSnapshot(
            timestamp=datetime.now(),
            pose=Pose2D(x=0.0, y=0.0, theta=0.0),
            velocity=Velocity(linear=0.0, angular=0.0),
            lidar_ranges=tuple([2.0] * 360),
            closest_obstacle=2.0,
            obstacle_direction=0.0,
        )
        ctx.update_sensors(snapshot)
        return ctx

    def test_move_falls_back_to_open_loop(self, behavior, stale_context):
        """Move should switch to open-loop when odom doesn't change."""
        cmd = create_move_command(0.3, speed=0.15)
        behavior.set_command(cmd)

        # Run enough compute cycles with unchanging odom to trigger fallback
        for _ in range(ClaudeControlBehavior.ODOM_STALE_CYCLES + 1):
            velocity = behavior.compute(stale_context)

        # Should have switched to open-loop
        result = behavior.get_result()
        assert result.open_loop is True
        # Should still be moving (not yet timed out)
        assert velocity.linear > 0

    def test_move_completes_open_loop_by_time(self, behavior, stale_context):
        """Move should complete via time-based fallback."""
        speed = 0.15
        distance = 0.3  # target_time = 2.0s
        cmd = create_move_command(distance, speed=speed, timeout=30.0)
        behavior.set_command(cmd)
        # Set start_time in the past so time-based completion triggers immediately
        cmd.start_time = time.time() - 5.0  # well past target_time of 2.0s

        # Trigger open-loop mode
        for _ in range(ClaudeControlBehavior.ODOM_STALE_CYCLES + 1):
            velocity = behavior.compute(stale_context)

        result = behavior.get_result()
        assert result.result == "completed_open_loop"
        assert result.open_loop is True
        assert result.actual == distance  # reports target as actual
        assert velocity.is_stopped()

    def test_turn_falls_back_to_open_loop(self, behavior, stale_context):
        """Turn should switch to open-loop when odom doesn't change."""
        cmd = create_turn_command(90.0, speed=0.5)
        behavior.set_command(cmd)

        for _ in range(ClaudeControlBehavior.ODOM_STALE_CYCLES + 1):
            velocity = behavior.compute(stale_context)

        result = behavior.get_result()
        assert result.open_loop is True
        assert velocity.angular > 0

    def test_turn_completes_open_loop_by_time(self, behavior, stale_context):
        """Turn should complete via time-based fallback."""
        degrees = 45.0
        speed = 0.5  # rad/s
        # target_time = radians(45) / 0.5 ≈ 1.57s
        cmd = create_turn_command(degrees, speed=speed, timeout=10.0)
        behavior.set_command(cmd)
        cmd.start_time = time.time() - 5.0  # well past target_time

        for _ in range(ClaudeControlBehavior.ODOM_STALE_CYCLES + 1):
            velocity = behavior.compute(stale_context)

        result = behavior.get_result()
        assert result.result == "completed_open_loop"
        assert result.open_loop is True
        assert result.actual == degrees

    def test_odom_working_skips_open_loop(self, behavior, stale_context):
        """When odom is working, open-loop should not activate."""
        cmd = create_move_command(0.5, speed=0.15)
        behavior.set_command(cmd)

        # First compute initializes
        behavior.compute(stale_context)

        # Simulate odom updating (robot moved)
        snapshot = SensorSnapshot(
            timestamp=datetime.now(),
            pose=Pose2D(x=0.1, y=0.0, theta=0.0),
            velocity=Velocity(linear=0.15, angular=0.0),
            lidar_ranges=tuple([2.0] * 360),
            closest_obstacle=2.0,
            obstacle_direction=0.0,
        )
        stale_context.update_sensors(snapshot)

        # Run more cycles
        for _ in range(ClaudeControlBehavior.ODOM_STALE_CYCLES + 2):
            behavior.compute(stale_context)

        result = behavior.get_result()
        assert result.open_loop is False

    def test_open_loop_command_field_default(self):
        """ClaudeCommand.open_loop should default to False."""
        cmd = create_move_command(1.0)
        assert cmd.open_loop is False
