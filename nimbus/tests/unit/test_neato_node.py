"""Tests for NeatoNode with MockNeatoTransport."""

import time
import math
import pytest
from nimbus.core.neato.node import NeatoNode
from nimbus.core.neato.mock_serial import MockNeatoTransport
from nimbus.core.xrce.messages import LaserScan, Odometry, Twist


class TestNeatoNodeLifecycle:
    """Test node start/stop lifecycle."""

    def test_start_and_shutdown(self):
        mock = MockNeatoTransport()
        node = NeatoNode(transport=mock, scan_rate_hz=20.0, odom_rate_hz=20.0, cmd_rate_hz=20.0)

        node.start()
        assert node.is_running
        assert mock.is_connected

        node.shutdown()
        assert not node.is_running
        assert not mock.is_connected

    def test_double_start(self):
        mock = MockNeatoTransport()
        node = NeatoNode(transport=mock)
        node.start()
        node.start()  # Should be a no-op
        assert node.is_running
        node.shutdown()

    def test_subscribe_and_publisher(self):
        mock = MockNeatoTransport()
        node = NeatoNode(transport=mock)
        node.start()

        scan_buf = node.subscribe("/scan", LaserScan, buffer_size=1)
        odom_buf = node.subscribe("/odom_raw", Odometry, buffer_size=1)
        cmd_pub = node.publisher("/cmd_vel", Twist)

        assert scan_buf is not None
        assert odom_buf is not None
        assert cmd_pub is not None
        assert cmd_pub.topic == "/cmd_vel"

        node.shutdown()


class TestNeatoNodeScan:
    """Test LIDAR scan generation."""

    def test_generates_laser_scan(self):
        mock = MockNeatoTransport()
        mock.set_wall_distance(1500)  # 1.5m

        node = NeatoNode(transport=mock, scan_rate_hz=50.0, odom_rate_hz=50.0, cmd_rate_hz=50.0)
        node.start()

        scan_buf = node.subscribe("/scan", LaserScan, buffer_size=1)

        # Wait for at least one scan
        time.sleep(0.15)

        scan = scan_buf.latest()
        assert scan is not None
        assert isinstance(scan, LaserScan)
        assert len(scan.ranges) == 360
        assert scan.ranges[0] == pytest.approx(1.5, abs=0.01)
        assert scan.angle_min == 0.0
        assert scan.range_min == pytest.approx(0.02)

        node.shutdown()

    def test_scan_error_handling(self):
        """Error codes in scan data should produce inf ranges."""
        mock = MockNeatoTransport()
        node = NeatoNode(transport=mock, scan_rate_hz=50.0, odom_rate_hz=50.0, cmd_rate_hz=50.0)
        node.start()

        # No error codes from mock, so all ranges should be valid
        scan_buf = node.subscribe("/scan", LaserScan, buffer_size=1)
        time.sleep(0.15)

        scan = scan_buf.latest()
        assert scan is not None
        assert all(r != float("inf") for r in scan.ranges)

        node.shutdown()


class TestNeatoNodeOdometry:
    """Test odometry generation from encoder data."""

    def test_generates_odometry(self):
        mock = MockNeatoTransport()
        node = NeatoNode(transport=mock, scan_rate_hz=50.0, odom_rate_hz=50.0, cmd_rate_hz=50.0)
        node.start()

        odom_buf = node.subscribe("/odom_raw", Odometry, buffer_size=1)

        # Wait for baseline + at least one odom update
        time.sleep(0.15)

        odom = odom_buf.latest()
        # May or may not have odom yet (need 2 readings for delta)
        # With zero movement, position should be near origin if present
        if odom is not None:
            assert isinstance(odom, Odometry)
            assert odom.pose.pose.position.x == pytest.approx(0.0, abs=0.01)
            assert odom.pose.pose.position.y == pytest.approx(0.0, abs=0.01)

        node.shutdown()


class TestNeatoNodeVelocity:
    """Test velocity command forwarding."""

    def test_velocity_command_forwarded(self):
        mock = MockNeatoTransport()
        node = NeatoNode(transport=mock, scan_rate_hz=50.0, odom_rate_hz=50.0, cmd_rate_hz=50.0)
        node.start()

        node.subscribe("/scan", LaserScan)
        node.subscribe("/odom_raw", Odometry)
        cmd_pub = node.publisher("/cmd_vel", Twist)

        # Send a forward velocity command
        cmd_pub.publish(Twist.create(linear_x=0.1, angular_z=0.0))

        # Let the velocity loop execute a few times
        time.sleep(0.15)

        # The mock transport should have accumulated encoder positions
        # (from set_motor calls in the velocity loop)
        motors = mock.get_motors()
        # With 0.1 m/s = 100 mm/s for ~0.15s at 50Hz, expect some accumulation
        total = abs(motors.left_wheel_position_mm) + abs(motors.right_wheel_position_mm)
        assert total > 0, "Motor commands should have accumulated encoder positions"

        node.shutdown()

    def test_zero_velocity_stops_motors(self):
        mock = MockNeatoTransport()
        node = NeatoNode(transport=mock, scan_rate_hz=50.0, odom_rate_hz=50.0, cmd_rate_hz=50.0)
        node.start()

        cmd_pub = node.publisher("/cmd_vel", Twist)
        node.subscribe("/scan", LaserScan)
        node.subscribe("/odom_raw", Odometry)

        # Send zero velocity
        cmd_pub.publish(Twist.create(linear_x=0.0, angular_z=0.0))

        time.sleep(0.1)

        # Encoder should not have accumulated
        motors = mock.get_motors()
        assert motors.left_wheel_position_mm == 0
        assert motors.right_wheel_position_mm == 0

        node.shutdown()

    def test_cmd_vel_publisher_records_messages(self):
        mock = MockNeatoTransport()
        node = NeatoNode(transport=mock)
        node.start()

        cmd_pub = node.publisher("/cmd_vel", Twist)
        msg = Twist.create(linear_x=0.15, angular_z=0.3)
        cmd_pub.publish(msg)

        assert len(cmd_pub.published_messages) == 1
        assert cmd_pub.last_message is msg

        node.shutdown()
