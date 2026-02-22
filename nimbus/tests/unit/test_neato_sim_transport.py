"""Tests for SimNeatoTransport — Neato protocol over SimulatedWorld physics."""

import math
import time
import pytest

from nimbus.core.neato.sim_transport import SimNeatoTransport
from nimbus.core.neato.transport import NeatoScanData, NeatoMotorData
from nimbus.core.neato.kinematics import WHEEL_BASE_M, WHEEL_BASE_MM
from nimbus.sim.world import SimulatedWorld


def _make_world(template: str = "empty_room") -> SimulatedWorld:
    """Create a SimulatedWorld for testing."""
    return SimulatedWorld.from_template(template)


# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

class TestLifecycle:
    def test_connect_disconnect(self):
        world = _make_world()
        t = SimNeatoTransport(world)
        assert not t.is_connected

        t.connect()
        assert t.is_connected

        t.disconnect()
        assert not t.is_connected

    def test_world_accessible(self):
        world = _make_world()
        t = SimNeatoTransport(world)
        assert t.world is world


# ---------------------------------------------------------------------------
# Scan conversion
# ---------------------------------------------------------------------------

class TestScanConversion:
    def test_scan_returns_360_rays(self):
        world = _make_world()
        t = SimNeatoTransport(world)
        t.connect()

        scan = t.get_lds_scan()
        assert len(scan.angles) == 360
        assert len(scan.distances_mm) == 360
        assert len(scan.intensities) == 360
        assert len(scan.error_codes) == 360

    def test_scan_angles_sequential(self):
        world = _make_world()
        t = SimNeatoTransport(world)
        t.connect()

        scan = t.get_lds_scan()
        assert scan.angles == list(range(360))

    def test_scan_distances_positive(self):
        """All distances should be positive (robot is inside the room)."""
        world = _make_world()
        t = SimNeatoTransport(world)
        t.connect()

        scan = t.get_lds_scan()
        assert all(d > 0 for d in scan.distances_mm)

    def test_scan_no_errors(self):
        """Simulated scans should have no error codes."""
        world = _make_world()
        t = SimNeatoTransport(world)
        t.connect()

        scan = t.get_lds_scan()
        assert all(e == 0 for e in scan.error_codes)

    def test_scan_distances_in_mm(self):
        """Distances should be in millimetres (matching real Neato)."""
        world = _make_world()
        t = SimNeatoTransport(world)
        t.connect()

        scan = t.get_lds_scan()
        # In a 5m empty room from center, expect ~2.5m max ≈ 2500mm
        # Minimum should be > 100mm (robot not right against a wall)
        assert all(100 < d < 5000 for d in scan.distances_mm)


# ---------------------------------------------------------------------------
# Motor commands & encoder accumulation
# ---------------------------------------------------------------------------

class TestMotorCommands:
    def test_encoder_accumulates(self):
        world = _make_world()
        t = SimNeatoTransport(world)
        t.connect()

        t.set_motor(100, 100, 200)  # 100mm forward at 200mm/s
        motors = t.get_motors()
        assert motors.left_wheel_position_mm == 100
        assert motors.right_wheel_position_mm == 100

    def test_encoder_accumulates_multiple_calls(self):
        world = _make_world()
        t = SimNeatoTransport(world)
        t.connect()

        t.set_motor(50, 50, 100)
        t.set_motor(30, 30, 100)
        motors = t.get_motors()
        assert motors.left_wheel_position_mm == 80
        assert motors.right_wheel_position_mm == 80

    def test_zero_speed_is_noop(self):
        world = _make_world()
        t = SimNeatoTransport(world)
        t.connect()

        x0, y0, _ = world.get_robot_pose()
        t.set_motor(100, 100, 0)  # speed=0 should be ignored
        x1, y1, _ = world.get_robot_pose()

        assert x0 == x1
        assert y0 == y1

    def test_stop_motors_is_noop(self):
        world = _make_world()
        t = SimNeatoTransport(world)
        t.connect()

        x0, y0, _ = world.get_robot_pose()
        t.stop_motors()
        x1, y1, _ = world.get_robot_pose()

        assert x0 == x1
        assert y0 == y1


# ---------------------------------------------------------------------------
# Physics integration — motor commands move the world robot
# ---------------------------------------------------------------------------

class TestPhysics:
    def test_forward_motion(self):
        """Straight-line motor command should move robot forward."""
        world = _make_world()
        world.set_robot_pose(0.0, 0.0, 0.0)  # facing +x
        t = SimNeatoTransport(world)
        t.connect()

        # Drive 200mm forward at 200mm/s → 1s of travel
        t.set_motor(200, 200, 200)

        x, y, theta = world.get_robot_pose()
        assert x == pytest.approx(0.2, abs=0.02)
        assert y == pytest.approx(0.0, abs=0.02)

    def test_pure_rotation(self):
        """Opposite wheel directions should rotate without translation."""
        world = _make_world()
        world.set_robot_pose(0.0, 0.0, 0.0)
        t = SimNeatoTransport(world)
        t.connect()

        # Left backward, right forward → counter-clockwise rotation
        # delta_theta = (right - left) / wheel_base
        dist_mm = 50
        t.set_motor(-dist_mm, dist_mm, 200)

        x, y, theta = world.get_robot_pose()
        # Translation should be near-zero
        assert abs(x) < 0.02
        assert abs(y) < 0.02
        # Should have rotated
        expected_theta = (2 * dist_mm / 1000.0) / WHEEL_BASE_M
        assert abs(theta) == pytest.approx(expected_theta, abs=0.05)

    def test_scan_changes_after_rotation(self):
        """After rotating, scan readings should change."""
        world = SimulatedWorld.from_template("simple_maze")
        world.set_robot_pose(0.0, 0.0, 0.0)
        t = SimNeatoTransport(world)
        t.connect()

        scan1 = t.get_lds_scan()

        # Rotate ~90 degrees
        # theta = (right - left) / wheel_base; for 90deg = pi/2
        half_arc_mm = int(WHEEL_BASE_MM * math.pi / 4)
        t.set_motor(-half_arc_mm, half_arc_mm, 200)

        scan2 = t.get_lds_scan()

        # At least some readings should differ
        diffs = sum(1 for a, b in zip(scan1.distances_mm, scan2.distances_mm) if a != b)
        assert diffs > 10


# ---------------------------------------------------------------------------
# Sensor stubs
# ---------------------------------------------------------------------------

class TestSensorStubs:
    def test_bumpers_all_clear(self):
        world = _make_world()
        t = SimNeatoTransport(world)
        t.connect()

        bumpers = t.get_digital_sensors()
        assert not bumpers.left_bumper
        assert not bumpers.right_bumper
        assert not bumpers.left_wheel_drop
        assert not bumpers.right_wheel_drop

    def test_battery_full(self):
        world = _make_world()
        t = SimNeatoTransport(world)
        t.connect()

        battery = t.get_charger()
        assert battery.battery_percent == 100
        assert not battery.charging


# ---------------------------------------------------------------------------
# Integration with NeatoNode
# ---------------------------------------------------------------------------

class TestNeatoNodeIntegration:
    """SimNeatoTransport plugged into NeatoNode — the full Neato sim path."""

    def test_node_starts_and_produces_scans(self):
        from nimbus.core.neato.node import NeatoNode
        from nimbus.core.xrce.messages import LaserScan

        world = _make_world()
        transport = SimNeatoTransport(world)
        node = NeatoNode(
            transport=transport,
            scan_rate_hz=50.0,
            odom_rate_hz=50.0,
            cmd_rate_hz=50.0,
        )
        node.start()

        scan_buf = node.subscribe("/scan", LaserScan, buffer_size=1)
        time.sleep(0.15)

        scan = scan_buf.latest()
        assert scan is not None
        assert len(scan.ranges) == 360
        # All ranges should be finite (robot inside room)
        assert all(r < float("inf") for r in scan.ranges)

        node.shutdown()

    def test_node_produces_odometry(self):
        from nimbus.core.neato.node import NeatoNode
        from nimbus.core.xrce.messages import LaserScan, Odometry, Twist

        world = _make_world()
        transport = SimNeatoTransport(world)
        node = NeatoNode(
            transport=transport,
            scan_rate_hz=50.0,
            odom_rate_hz=50.0,
            cmd_rate_hz=50.0,
        )
        node.start()

        node.subscribe("/scan", LaserScan)
        odom_buf = node.subscribe("/odom_raw", Odometry, buffer_size=1)
        cmd_pub = node.publisher("/cmd_vel", Twist)

        # Drive forward
        cmd_pub.publish(Twist.create(linear_x=0.1, angular_z=0.0))
        time.sleep(0.3)

        odom = odom_buf.latest()
        assert odom is not None
        # Robot should have moved in +x
        assert odom.pose.pose.position.x > 0.0

        node.shutdown()

    def test_node_shutdown_clean(self):
        from nimbus.core.neato.node import NeatoNode

        world = _make_world()
        transport = SimNeatoTransport(world)
        node = NeatoNode(transport=transport)

        node.start()
        assert node.is_running

        node.shutdown()
        assert not node.is_running
