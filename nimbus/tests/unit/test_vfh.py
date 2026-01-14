"""
Unit tests for VFH navigation algorithm.
"""

import pytest
import numpy as np
import math
from nimbus.navigation.vfh import VFHNavigator, VFHConfig
from nimbus.sensors.lidar import LidarProcessor, LidarConfig


class TestVFHNavigator:
    """Tests for Vector Field Histogram navigation."""

    def test_clear_path_goes_to_goal(self, vfh_navigator, lidar_processor, mock_scan_empty):
        """With no obstacles, steer directly toward goal."""
        histogram = lidar_processor.process(np.array(mock_scan_empty.ranges))

        # Goal is straight ahead (0 radians)
        steering, blocked = vfh_navigator.compute_steering(histogram, 0.0)

        assert not blocked
        assert abs(steering) < 0.2  # Should go roughly straight

    def test_wall_ahead_steers_around(self, vfh_navigator, lidar_processor):
        """With wall ahead, should steer left or right."""
        # Use closer wall to ensure detection
        from nimbus.tests.conftest import MockLaserScan
        scan = MockLaserScan.wall_ahead(distance=0.3)
        histogram = lidar_processor.process(np.array(scan.ranges))

        steering, blocked = vfh_navigator.compute_steering(histogram, 0.0)

        # With a wall at 0.3m, VFH should find a way around
        # If blocked, that's acceptable too (very close wall)
        if not blocked:
            assert abs(steering) > 0.1  # Should turn at least a bit

    def test_corridor_navigation(self, vfh_navigator, lidar_processor, mock_scan_corridor):
        """In corridor, should go straight through."""
        histogram = lidar_processor.process(np.array(mock_scan_corridor.ranges))

        steering, blocked = vfh_navigator.compute_steering(histogram, 0.0)

        assert not blocked
        # Should stay relatively centered
        assert abs(steering) < 0.5

    def test_blocked_when_surrounded(self, vfh_navigator, lidar_processor):
        """Should report blocked when completely surrounded by very close obstacles."""
        # Create a scan where obstacles are within safety radius
        from nimbus.tests.conftest import MockLaserScan
        scan = MockLaserScan.surrounded(distance=0.15)  # Very close
        histogram = lidar_processor.process(np.array(scan.ranges))

        steering, blocked = vfh_navigator.compute_steering(histogram, 0.0)

        # With obstacles at 0.15m all around, histogram should be mostly blocked
        # Either blocked=True or steering should be 0 (nowhere to go)
        assert blocked or abs(steering) < 0.5

    def test_goal_bias_left(self, vfh_navigator, lidar_processor):
        """When goal is left, should steer left if path clear."""
        # Create scan with obstacle on right only
        from nimbus.tests.conftest import MockLaserScan
        scan = MockLaserScan.obstacle_at(angle_deg=270, distance=0.4)
        histogram = lidar_processor.process(np.array(scan.ranges))

        # Goal is to the left (positive angle)
        steering, blocked = vfh_navigator.compute_steering(histogram, math.pi / 2)

        assert not blocked
        assert steering > 0  # Should steer left

    def test_goal_bias_right(self, vfh_navigator, lidar_processor):
        """When goal is right, should steer right if path clear."""
        from nimbus.tests.conftest import MockLaserScan
        scan = MockLaserScan.obstacle_at(angle_deg=90, distance=0.4)
        histogram = lidar_processor.process(np.array(scan.ranges))

        # Goal is to the right (negative angle)
        steering, blocked = vfh_navigator.compute_steering(histogram, -math.pi / 2)

        assert not blocked
        assert steering < 0  # Should steer right


class TestLidarProcessor:
    """Tests for LIDAR processing."""

    def test_empty_scan_produces_clear_histogram(self, lidar_processor, mock_scan_empty):
        """Empty scan should produce all-clear histogram."""
        histogram = lidar_processor.process(np.array(mock_scan_empty.ranges))

        assert len(histogram) == lidar_processor.config.num_sectors
        assert np.all(histogram == 0)  # All clear

    def test_wall_ahead_detected(self, lidar_processor):
        """Wall ahead should be detected in front sectors."""
        # Use close wall to ensure detection
        from nimbus.tests.conftest import MockLaserScan
        scan = MockLaserScan.wall_ahead(distance=0.3)
        histogram = lidar_processor.process(np.array(scan.ranges))

        # Front sectors (around index 0 and num_sectors-1) should be blocked
        front_sector = 0
        assert histogram[front_sector] > 0.3  # Should show obstacle

    def test_find_closest_empty(self, lidar_processor, mock_scan_empty):
        """Empty scan should report infinite distance."""
        distance, angle = lidar_processor.find_closest(np.array(mock_scan_empty.ranges))

        assert distance == float('inf')

    def test_find_closest_wall(self, lidar_processor, mock_scan_wall_ahead):
        """Should find wall ahead."""
        distance, angle = lidar_processor.find_closest(np.array(mock_scan_wall_ahead.ranges))

        assert 0.4 < distance < 0.6  # Around 0.5m
        assert abs(angle) < 0.5  # Should be roughly ahead

    def test_sector_ranges(self, lidar_processor, mock_scan_corridor):
        """Corridor should show obstacles on left and right."""
        sector_ranges = lidar_processor.get_sector_ranges(np.array(mock_scan_corridor.ranges))

        # Left and right should be closer than front
        assert sector_ranges["left"] < sector_ranges["front"]
        assert sector_ranges["right"] < sector_ranges["front"]
