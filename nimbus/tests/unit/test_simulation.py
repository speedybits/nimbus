"""
Unit tests for Nimbus simulation system.

Tests:
- SimulatedWorld grid loading and ray-casting
- SimulatorNode sensor data generation
- Kinematics calculations
"""

import pytest
import math
import time
from pathlib import Path
from tempfile import NamedTemporaryFile

from nimbus.sim.world import SimulatedWorld
from nimbus.sim.node import SimulatorNode


class TestSimulatedWorld:
    """Tests for SimulatedWorld."""

    def test_from_template_empty_room(self):
        """Load empty_room template."""
        world = SimulatedWorld.from_template("empty_room")
        assert world.width > 0
        assert world.height > 0
        assert world.resolution == 0.1

    def test_from_template_simple_maze(self):
        """Load simple_maze template."""
        world = SimulatedWorld.from_template("simple_maze")
        assert world.width > 0
        assert world.height > 0

    def test_from_template_unknown_raises(self):
        """Unknown template raises ValueError."""
        with pytest.raises(ValueError, match="Unknown template"):
            SimulatedWorld.from_template("nonexistent")

    def test_from_file(self):
        """Load world from file."""
        map_content = """##########
#........#
#........#
#...R....#
##########
resolution: 0.1
"""
        with NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
            f.write(map_content)
            f.flush()

            world = SimulatedWorld.from_file(Path(f.name))
            assert world.width == 10
            assert world.height == 5
            assert world.resolution == 0.1

    def test_spawn_position(self):
        """Robot spawns at R position."""
        map_content = """##########
#........#
#........#
#...R....#
##########
resolution: 0.1
"""
        with NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
            f.write(map_content)
            f.flush()

            world = SimulatedWorld.from_file(Path(f.name))
            # R is at column 4, row 1 from bottom (row 3 from top, but reversed)
            # Check that robot is somewhere in the map (not at edges)
            assert -world._origin_x < world.robot_x < world._origin_x
            assert -world._origin_y < world.robot_y < world._origin_y

    def test_set_robot_pose(self):
        """Set and get robot pose."""
        world = SimulatedWorld.from_template("empty_room")
        world.set_robot_pose(1.0, 2.0, 1.5)

        x, y, theta = world.get_robot_pose()
        assert x == 1.0
        assert y == 2.0
        assert theta == 1.5

    def test_apply_velocity_forward(self):
        """Apply forward velocity."""
        world = SimulatedWorld.from_template("empty_room")
        world.set_robot_pose(0.0, 0.0, 0.0)  # Facing +X

        world.apply_velocity(linear=1.0, angular=0.0, dt=0.1)

        x, y, theta = world.get_robot_pose()
        assert abs(x - 0.1) < 0.01
        assert abs(y) < 0.01
        assert abs(theta) < 0.01

    def test_apply_velocity_rotation(self):
        """Apply rotational velocity."""
        world = SimulatedWorld.from_template("empty_room")
        world.set_robot_pose(0.0, 0.0, 0.0)

        world.apply_velocity(linear=0.0, angular=math.pi, dt=0.5)

        x, y, theta = world.get_robot_pose()
        assert abs(x) < 0.01
        assert abs(y) < 0.01
        assert abs(theta - math.pi / 2) < 0.01

    def test_apply_velocity_arc(self):
        """Apply combined velocity (arc motion)."""
        world = SimulatedWorld.from_template("empty_room")
        world.set_robot_pose(0.0, 0.0, 0.0)

        # Move forward while turning
        for _ in range(20):
            world.apply_velocity(linear=0.1, angular=0.2, dt=0.1)

        x, y, theta = world.get_robot_pose()
        # Should have moved forward and turned
        assert x > 0.05
        assert y > 0.001  # Slight curve (small due to arc geometry)
        assert theta > 0.1

    def test_raycast_empty_room(self):
        """Ray-cast in empty room returns max range for most directions."""
        world = SimulatedWorld.from_template("empty_room")
        world.set_robot_pose(0.0, 0.0, 0.0)

        ranges = world.raycast(num_rays=360, max_range=3.5)

        assert len(ranges) == 360
        # Most rays should reach max range or walls
        assert all(0 < r <= 3.5 for r in ranges)

    def test_raycast_wall_ahead(self):
        """Ray-cast detects wall ahead."""
        # Create a small room where the robot is close to a wall
        world = SimulatedWorld._create_empty_room(size=1.0, resolution=0.1)
        world.set_robot_pose(0.0, 0.0, 0.0)  # Facing +X, near center

        ranges = world.raycast(num_rays=360, max_range=3.5)

        # Ray 0 is forward (+X direction), should hit wall
        forward_range = ranges[0]
        assert forward_range < 1.0  # Should hit wall within 1m

    def test_raycast_360_coverage(self):
        """Raycast covers all angles."""
        world = SimulatedWorld.from_template("empty_room")
        world.set_robot_pose(0.0, 0.0, 0.0)

        ranges = world.raycast(num_rays=360)

        # All directions should have valid readings
        assert len(ranges) == 360
        assert all(r > 0 for r in ranges)

    def test_collision_prevention(self):
        """Robot cannot move through walls."""
        world = SimulatedWorld._create_empty_room(size=1.0, resolution=0.1)
        world.set_robot_pose(0.0, 0.0, 0.0)  # Near center

        # Try to move into wall
        initial_x = world.robot_x
        for _ in range(100):
            world.apply_velocity(linear=0.5, angular=0.0, dt=0.1)

        # Robot should be stopped by wall
        final_x = world.robot_x
        assert final_x < 0.5  # Shouldn't have passed the wall

    def test_theta_normalization(self):
        """Theta stays in [-pi, pi] range."""
        world = SimulatedWorld.from_template("empty_room")
        world.set_robot_pose(0.0, 0.0, 0.0)

        # Spin multiple rotations
        for _ in range(100):
            world.apply_velocity(linear=0.0, angular=1.0, dt=0.1)

        _, _, theta = world.get_robot_pose()
        assert -math.pi <= theta <= math.pi


    def test_from_template_four_rooms(self):
        """Load four_rooms template with correct dimensions."""
        world = SimulatedWorld.from_template("four_rooms")
        assert world.width == 80
        assert world.height == 80
        assert world.resolution == 0.1

    def test_four_rooms_doors_passable(self):
        """All four door gaps are free space."""
        world = SimulatedWorld.from_template("four_rooms")
        mid = 40
        door_width = 8

        # NW<->NE door: x=mid, y=60
        door_y = mid + mid // 2  # 60
        for dy in range(door_width):
            assert not world.grid[door_y + dy][mid], f"NW-NE door blocked at y={door_y + dy}"

        # SW<->SE door: x=mid, y=10
        door_y = mid // 4  # 10
        for dy in range(door_width):
            assert not world.grid[door_y + dy][mid], f"SW-SE door blocked at y={door_y + dy}"

        # NW<->SW door: y=mid, x=10
        door_x = mid // 4  # 10
        for dx in range(door_width):
            assert not world.grid[mid][door_x + dx], f"NW-SW door blocked at x={door_x + dx}"

        # NE<->SE door: y=mid, x=60
        door_x = mid + mid // 2  # 60
        for dx in range(door_width):
            assert not world.grid[mid][door_x + dx], f"NE-SE door blocked at x={door_x + dx}"

    def test_four_rooms_walls_present(self):
        """Cross-walls are solid outside door gaps."""
        world = SimulatedWorld.from_template("four_rooms")
        mid = 40

        # Vertical wall at x=mid should be mostly solid
        wall_cells = sum(1 for y in range(1, 79) if world.grid[y][mid])
        # Total interior cells on vertical wall: 78, minus 16 door cells (2 doors × 8)
        assert wall_cells >= 62, f"Vertical wall too sparse: {wall_cells} solid cells"

        # Horizontal wall at y=mid should be mostly solid
        wall_cells = sum(1 for x in range(1, 79) if world.grid[mid][x])
        assert wall_cells >= 62, f"Horizontal wall too sparse: {wall_cells} solid cells"

    def test_four_rooms_spawn_in_sw(self):
        """Robot spawns in SW quadrant (negative x, negative y)."""
        world = SimulatedWorld.from_template("four_rooms")
        assert world.robot_x < 0, f"Spawn x={world.robot_x} should be negative"
        assert world.robot_y < 0, f"Spawn y={world.robot_y} should be negative"

    def test_four_rooms_reproducible(self):
        """Same seed produces identical grids."""
        w1 = SimulatedWorld._create_four_rooms(seed=42)
        w2 = SimulatedWorld._create_four_rooms(seed=42)
        assert w1.grid == w2.grid

    def test_four_rooms_different_seeds(self):
        """Different seeds produce different grids."""
        w1 = SimulatedWorld._create_four_rooms(seed=42)
        w2 = SimulatedWorld._create_four_rooms(seed=99)
        assert w1.grid != w2.grid

    def test_four_rooms_raycast(self):
        """Raycast from spawn produces 360 valid readings."""
        world = SimulatedWorld.from_template("four_rooms")
        ranges = world.raycast(num_rays=360, max_range=3.5)
        assert len(ranges) == 360
        assert all(0 < r <= 3.5 for r in ranges)


class TestSimulatorNode:
    """Tests for SimulatorNode."""

    def test_create_node(self):
        """Create a simulator node."""
        world = SimulatedWorld.from_template("empty_room")
        node = SimulatorNode(world=world, tick_rate=10.0)
        assert not node.is_running

    def test_start_stop(self):
        """Start and stop the node."""
        world = SimulatedWorld.from_template("empty_room")
        node = SimulatorNode(world=world, tick_rate=10.0)

        node.start()
        assert node.is_running

        node.shutdown()
        assert not node.is_running

    def test_subscribe(self):
        """Subscribe to topics."""
        world = SimulatedWorld.from_template("empty_room")
        node = SimulatorNode(world=world, tick_rate=10.0)
        node.start()

        try:
            from nimbus.core.xrce.messages import LaserScan, Odometry

            scan_buffer = node.subscribe("/scan", LaserScan, buffer_size=1)
            odom_buffer = node.subscribe("/odom_raw", Odometry, buffer_size=1)

            assert scan_buffer is not None
            assert odom_buffer is not None
        finally:
            node.shutdown()

    def test_generates_lidar(self):
        """Node generates LIDAR data."""
        world = SimulatedWorld.from_template("empty_room")
        node = SimulatorNode(world=world, tick_rate=10.0)
        node.start()

        try:
            from nimbus.core.xrce.messages import LaserScan

            scan_buffer = node.subscribe("/scan", LaserScan, buffer_size=1)

            # Wait for data
            time.sleep(0.3)

            scan = scan_buffer.latest()
            assert scan is not None
            assert len(scan.ranges) == 360
        finally:
            node.shutdown()

    def test_generates_odometry(self):
        """Node generates odometry data."""
        world = SimulatedWorld.from_template("empty_room")
        node = SimulatorNode(world=world, tick_rate=10.0)
        node.start()

        try:
            from nimbus.core.xrce.messages import Odometry

            odom_buffer = node.subscribe("/odom_raw", Odometry, buffer_size=1)

            # Wait for data
            time.sleep(0.3)

            odom = odom_buffer.latest()
            assert odom is not None
            assert odom.pose is not None
        finally:
            node.shutdown()

    def test_velocity_commands(self):
        """Velocity commands move the robot."""
        world = SimulatedWorld.from_template("empty_room")
        world.set_robot_pose(0.0, 0.0, 0.0)
        node = SimulatorNode(world=world, tick_rate=10.0)
        node.start()

        try:
            from nimbus.core.xrce.messages import Twist, Odometry

            odom_buffer = node.subscribe("/odom_raw", Odometry, buffer_size=1)
            cmd_pub = node.publisher("/cmd_vel", Twist)

            # Send forward velocity
            cmd = Twist.create(linear_x=0.5, angular_z=0.0)
            cmd_pub.publish(cmd)

            # Wait for movement
            time.sleep(0.5)

            odom = odom_buffer.latest()
            assert odom is not None
            # Robot should have moved forward
            assert odom.pose.pose.position.x > 0.05
        finally:
            node.shutdown()

    def test_world_property(self):
        """Node exposes the world."""
        world = SimulatedWorld.from_template("empty_room")
        node = SimulatorNode(world=world)

        assert node.world is world


class TestSimulationIntegration:
    """Integration tests for simulation with runner."""

    def test_runner_with_sim(self):
        """Runner can use simulation mode."""
        from nimbus.core.runner import NimbusRunner
        from nimbus.core.config import NimbusConfig
        import threading

        config = NimbusConfig()
        config.simulation.enabled = True
        config.simulation.map_template = "empty_room"

        runner = NimbusRunner(config=config, sim=True)

        try:
            runner.start()
            assert runner.is_running

            # Set wander behavior
            runner.set_behavior("wander")

            # Run one control step manually to populate sensors
            runner._control_step()

            # Check that sensors are populated after control step
            assert runner.context.sensors is not None
        finally:
            runner.stop()
