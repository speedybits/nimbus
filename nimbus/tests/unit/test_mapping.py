"""Unit tests for obstacle mapping."""

import math
import pytest

from nimbus.sensors.mapping import ObstacleMap, MapConfig
from nimbus.core.state import Pose2D


class TestObstacleMapBasics:
    """Basic obstacle map functionality."""

    def test_empty_map_bounds(self):
        """Empty map returns zero bounds."""
        obs_map = ObstacleMap()
        bounds = obs_map.get_bounds()
        assert bounds == (0.0, 0.0, 0.0, 0.0)

    def test_empty_map_counts(self):
        """Empty map has zero obstacles and visited cells."""
        obs_map = ObstacleMap()
        assert obs_map.num_obstacles == 0
        assert obs_map.num_visited == 0

    def test_clear_map(self):
        """Clear removes all data."""
        config = MapConfig(cell_size=0.15)
        obs_map = ObstacleMap(config)

        # Add some data
        pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        ranges = tuple([1.0] * 360)
        obs_map.update(pose, ranges)

        assert obs_map.num_obstacles > 0

        # Clear
        obs_map.clear()
        assert obs_map.num_obstacles == 0
        assert obs_map.num_visited == 0
        assert len(obs_map.get_robot_trail()) == 0


class TestObstacleDetection:
    """Test obstacle detection from LIDAR."""

    def test_single_obstacle(self):
        """LIDAR point creates obstacle cell."""
        config = MapConfig(cell_size=0.15, max_range=3.0)
        obs_map = ObstacleMap(config)

        # Robot at origin, facing +X (theta=0)
        pose = Pose2D(x=0.0, y=0.0, theta=0.0)

        # Create LIDAR reading with obstacle directly ahead at 1m
        # LIDAR convention: index 180 = forward, index 0 = backward
        ranges = [float('inf')] * 360
        ranges[180] = 1.0  # Obstacle 1m ahead (index 180 = forward)
        ranges = tuple(ranges)

        obs_map.update(pose, ranges)

        # Should have created an obstacle cell
        assert obs_map.num_obstacles >= 1

        # Check bounds include the obstacle position (~1m in +X direction)
        min_x, min_y, max_x, max_y = obs_map.get_bounds()
        assert max_x >= 0.9  # Obstacle should be near x=1.0

    def test_obstacle_at_angle(self):
        """Obstacle at 90 degrees (left) from robot."""
        config = MapConfig(cell_size=0.15, max_range=3.0)
        obs_map = ObstacleMap(config)

        # Robot at origin, facing +X
        pose = Pose2D(x=0.0, y=0.0, theta=0.0)

        # LIDAR convention: index 180 = forward, so index 270 = left (90° CCW from forward)
        ranges = [float('inf')] * 360
        ranges[270] = 1.0  # Obstacle 1m to the left
        ranges = tuple(ranges)

        obs_map.update(pose, ranges)

        assert obs_map.num_obstacles >= 1

        # Obstacle should be in +Y direction (robot's left when facing +X)
        min_x, min_y, max_x, max_y = obs_map.get_bounds()
        assert max_y >= 0.8  # Obstacle near y=1.0


class TestRayCasting:
    """Test that rays mark free space correctly."""

    def test_ray_casting_marks_free_space(self):
        """Cells along ray are marked as visited (free)."""
        config = MapConfig(cell_size=0.15, max_range=3.0)
        obs_map = ObstacleMap(config)

        # Robot at origin, facing +X
        pose = Pose2D(x=0.0, y=0.0, theta=0.0)

        # Obstacle 2m ahead (index 180 = forward)
        ranges = [float('inf')] * 360
        ranges[180] = 2.0
        ranges = tuple(ranges)

        obs_map.update(pose, ranges)

        # Should have visited cells between robot and obstacle
        visited = obs_map.get_visited()
        assert len(visited) > 1  # More than just the robot cell

        # Check that visited cells are along the +X axis
        # At least some cells should be in the range 0 < x < 2
        has_intermediate_cells = False
        for cell_x, cell_y in visited:
            world_x = (cell_x + 0.5) * config.cell_size
            world_y = (cell_y + 0.5) * config.cell_size
            if 0.2 < world_x < 1.8 and abs(world_y) < 0.3:
                has_intermediate_cells = True
                break

        assert has_intermediate_cells, "Should have free cells between robot and obstacle"


class TestBoundsExpansion:
    """Test that bounds grow with exploration."""

    def test_bounds_expand_with_exploration(self):
        """Bounds grow as robot moves and discovers obstacles."""
        config = MapConfig(cell_size=0.15, max_range=3.0)
        obs_map = ObstacleMap(config)

        # First update at origin
        pose1 = Pose2D(x=0.0, y=0.0, theta=0.0)
        ranges1 = [float('inf')] * 360
        ranges1[180] = 1.0  # Obstacle 1m ahead (index 180 = forward)
        obs_map.update(pose1, tuple(ranges1))

        bounds1 = obs_map.get_bounds()

        # Move robot 2m to the right and update
        pose2 = Pose2D(x=2.0, y=0.0, theta=0.0)
        ranges2 = [float('inf')] * 360
        ranges2[180] = 1.0  # Another obstacle 1m ahead
        obs_map.update(pose2, tuple(ranges2))

        bounds2 = obs_map.get_bounds()

        # Bounds should have expanded
        min_x1, min_y1, max_x1, max_y1 = bounds1
        min_x2, min_y2, max_x2, max_y2 = bounds2

        # max_x should be larger after moving right
        assert max_x2 > max_x1


class TestRobotTrail:
    """Test robot trail tracking."""

    def test_trail_accumulates(self):
        """Robot trail grows as robot moves."""
        config = MapConfig(cell_size=0.15, max_trail_points=50)
        obs_map = ObstacleMap(config)

        ranges = tuple([float('inf')] * 360)

        # Move robot along a path
        for i in range(5):
            pose = Pose2D(x=i * 0.2, y=0.0, theta=0.0)
            obs_map.update(pose, ranges)

        trail = obs_map.get_robot_trail()
        # Should have multiple trail points (depends on movement threshold)
        assert len(trail) >= 2

    def test_trail_persists(self):
        """Trail keeps full history (no truncation) once active."""
        config = MapConfig(cell_size=0.15)
        obs_map = ObstacleMap(config)

        ranges = tuple([float('inf')] * 360)

        # Move robot many times (0.1m steps, trail activates after 0.15m)
        for i in range(20):
            pose = Pose2D(x=i * 0.1, y=0.0, theta=0.0)
            obs_map.update(pose, ranges)

        trail = obs_map.get_robot_trail()
        # First point is origin (skipped), trail activates at ~0.2m,
        # then records every 0.1m step (>5cm threshold)
        assert len(trail) >= 17

    def test_trail_deferred_at_startup(self):
        """Trail doesn't record until robot moves 15cm from start."""
        config = MapConfig(cell_size=0.15)
        obs_map = ObstacleMap(config)

        ranges = tuple([float('inf')] * 360)

        # Small movements under 15cm from origin
        for i in range(10):
            pose = Pose2D(x=i * 0.01, y=0.0, theta=0.0)
            obs_map.update(pose, ranges)

        trail = obs_map.get_robot_trail()
        assert len(trail) == 0


class TestConfidence:
    """Test obstacle confidence updates."""

    def test_repeated_hits_increase_confidence(self):
        """Multiple hits on same cell increase confidence."""
        config = MapConfig(
            cell_size=0.15,
            obstacle_confidence_inc=0.3,
            max_confidence=1.0
        )
        obs_map = ObstacleMap(config)

        pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        ranges = [float('inf')] * 360
        ranges[0] = 1.0
        ranges = tuple(ranges)

        # Update multiple times
        obs_map.update(pose, ranges)
        obstacles1 = obs_map.get_obstacles()

        obs_map.update(pose, ranges)
        obstacles2 = obs_map.get_obstacles()

        # Find the obstacle cell
        # Confidence should have increased (or maxed out)
        assert len(obstacles2) >= 1

        # Get any obstacle cell's confidence
        cell, conf1 = next(iter(obstacles1.items()))
        conf2 = obstacles2.get(cell, 0)

        # Second confidence should be >= first (could be maxed)
        assert conf2 >= conf1


class TestInvalidReadings:
    """Test handling of invalid LIDAR readings."""

    def test_ignores_inf_readings(self):
        """Infinite readings are ignored."""
        obs_map = ObstacleMap()

        pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        # All readings are inf
        ranges = tuple([float('inf')] * 360)

        obs_map.update(pose, ranges)

        # Should have marked robot position as visited, but no obstacles
        assert obs_map.num_obstacles == 0
        assert obs_map.num_visited >= 1  # At least robot cell

    def test_ignores_readings_beyond_max_range(self):
        """Readings beyond max_range are ignored."""
        config = MapConfig(max_range=3.0)
        obs_map = ObstacleMap(config)

        pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        ranges = [5.0] * 360  # All beyond max_range
        ranges = tuple(ranges)

        obs_map.update(pose, ranges)

        # Should have visited cell but no obstacles
        assert obs_map.num_obstacles == 0

    def test_ignores_readings_below_min_range(self):
        """Readings below min_range are ignored (noise)."""
        config = MapConfig(min_range=0.1)
        obs_map = ObstacleMap(config)

        pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        ranges = [0.05] * 360  # All below min_range
        ranges = tuple(ranges)

        obs_map.update(pose, ranges)

        # Should have no obstacles (readings are noise)
        assert obs_map.num_obstacles == 0
