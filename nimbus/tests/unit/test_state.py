"""
Unit tests for state machine.
"""

import pytest
from nimbus.core.state import RobotState, RobotContext, Pose2D, Velocity, SensorSnapshot
from datetime import datetime


class TestPose2D:
    """Tests for Pose2D."""

    def test_distance_to(self):
        """Should calculate correct distance."""
        p1 = Pose2D(x=0, y=0, theta=0)
        p2 = Pose2D(x=3, y=4, theta=0)

        assert p1.distance_to(p2) == 5.0

    def test_angle_to(self):
        """Should calculate correct angle."""
        import math
        p1 = Pose2D(x=0, y=0, theta=0)
        p2 = Pose2D(x=1, y=1, theta=0)

        angle = p1.angle_to(p2)
        assert abs(angle - math.pi / 4) < 0.01


class TestVelocity:
    """Tests for Velocity."""

    def test_stop(self):
        """Stop should be zero velocity."""
        v = Velocity.stop()
        assert v.linear == 0.0
        assert v.angular == 0.0

    def test_is_stopped(self):
        """Should correctly identify stopped state."""
        v1 = Velocity(0.0, 0.0)
        v2 = Velocity(0.1, 0.0)

        assert v1.is_stopped()
        assert not v2.is_stopped()


class TestRobotContext:
    """Tests for RobotContext."""

    def test_initial_state_is_idle(self, robot_context):
        """Context should start in IDLE state."""
        assert robot_context.state == RobotState.IDLE

    def test_state_transition(self, robot_context):
        """Should transition states correctly."""
        robot_context.set_state(RobotState.NAVIGATING)
        assert robot_context.state == RobotState.NAVIGATING

    def test_state_listener(self, robot_context):
        """Should notify listeners on state change."""
        transitions = []

        def listener(old, new):
            transitions.append((old, new))

        robot_context.on_state_change(listener)
        robot_context.set_state(RobotState.NAVIGATING)

        assert len(transitions) == 1
        assert transitions[0] == (RobotState.IDLE, RobotState.NAVIGATING)

    def test_no_duplicate_state_notification(self, robot_context):
        """Should not notify if state unchanged."""
        transitions = []

        robot_context.on_state_change(lambda o, n: transitions.append((o, n)))
        robot_context.set_state(RobotState.IDLE)  # Same as initial

        assert len(transitions) == 0

    def test_target_management(self, robot_context):
        """Should manage target correctly."""
        target = Pose2D(x=1.0, y=2.0, theta=0.5)

        robot_context.set_target(target)
        assert robot_context.target == target

        robot_context.clear_target()
        assert robot_context.target is None

    def test_sensor_update(self, robot_context, sample_sensor_snapshot):
        """Should update sensors correctly."""
        robot_context.update_sensors(sample_sensor_snapshot)

        assert robot_context.sensors == sample_sensor_snapshot

    def test_distance_to_target(self, robot_context, sample_sensor_snapshot):
        """Should calculate distance to target."""
        robot_context.update_sensors(sample_sensor_snapshot)
        robot_context.set_target(Pose2D(x=4.0, y=6.0, theta=0))

        # From (1, 2) to (4, 6) = sqrt(9 + 16) = 5
        dist = robot_context.distance_to_target()
        assert dist == 5.0

    def test_to_dict(self, robot_context, sample_sensor_snapshot):
        """Should export to dict correctly."""
        robot_context.update_sensors(sample_sensor_snapshot)
        robot_context.set_behavior("wander")

        d = robot_context.to_dict()

        assert d["state"] == "IDLE"
        assert d["current_behavior"] == "wander"
        assert "pose" in d
        assert "velocity" in d
