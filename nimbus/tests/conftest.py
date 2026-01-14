"""
Test fixtures for Nimbus.

Provides mock objects and data generators for testing
without requiring ROS2 or hardware.
"""

import pytest
import numpy as np
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

from nimbus.core.state import Pose2D, Velocity, SensorSnapshot, RobotContext
from nimbus.sensors.lidar import LidarProcessor, LidarConfig, create_mock_scan
from nimbus.navigation.vfh import VFHNavigator, VFHConfig
from nimbus.navigation.safety import SafetyController, SafetyConfig


# --- Mock Message Classes ---

@dataclass
class MockLaserScan:
    """Mock ROS2 LaserScan message."""
    ranges: list
    angle_min: float = -3.14159
    angle_max: float = 3.14159
    angle_increment: float = 0.0174533  # ~1 degree
    time_increment: float = 0.0
    scan_time: float = 0.1
    range_min: float = 0.05
    range_max: float = 10.0

    @classmethod
    def empty(cls) -> "MockLaserScan":
        """Scan with no obstacles."""
        return cls(ranges=[float('inf')] * 360)

    @classmethod
    def wall_ahead(cls, distance: float = 0.5) -> "MockLaserScan":
        """Wall directly in front."""
        ranges = [float('inf')] * 360
        for i in list(range(0, 31)) + list(range(330, 360)):
            ranges[i] = distance
        return cls(ranges=ranges)

    @classmethod
    def corridor(cls, width: float = 0.8) -> "MockLaserScan":
        """Corridor with walls on left and right."""
        ranges = [float('inf')] * 360
        half_width = width / 2
        for i in range(60, 120):
            ranges[i] = half_width
        for i in range(240, 300):
            ranges[i] = half_width
        return cls(ranges=ranges)

    @classmethod
    def surrounded(cls, distance: float = 0.3) -> "MockLaserScan":
        """Obstacles all around."""
        return cls(ranges=[distance] * 360)

    @classmethod
    def obstacle_at(cls, angle_deg: float, distance: float, width_deg: float = 20) -> "MockLaserScan":
        """Single obstacle at specified angle."""
        ranges = [float('inf')] * 360
        center = int(angle_deg) % 360
        half_width = int(width_deg / 2)
        for offset in range(-half_width, half_width + 1):
            idx = (center + offset) % 360
            ranges[idx] = distance
        return cls(ranges=ranges)


@dataclass
class MockOdometry:
    """Mock ROS2 Odometry message."""

    def __init__(
        self,
        x: float = 0.0,
        y: float = 0.0,
        theta: float = 0.0,
        linear_vel: float = 0.0,
        angular_vel: float = 0.0
    ):
        # Position
        self.pose = type('obj', (object,), {
            'pose': type('obj', (object,), {
                'position': type('obj', (object,), {'x': x, 'y': y, 'z': 0.0})(),
                'orientation': self._euler_to_quaternion(0, 0, theta)
            })()
        })()

        # Velocity
        self.twist = type('obj', (object,), {
            'twist': type('obj', (object,), {
                'linear': type('obj', (object,), {'x': linear_vel, 'y': 0.0, 'z': 0.0})(),
                'angular': type('obj', (object,), {'x': 0.0, 'y': 0.0, 'z': angular_vel})()
            })()
        })()

    def _euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        import math
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return type('obj', (object,), {
            'w': cr * cp * cy + sr * sp * sy,
            'x': sr * cp * cy - cr * sp * sy,
            'y': cr * sp * cy + sr * cp * sy,
            'z': cr * cp * sy - sr * sp * cy
        })()


# --- Fixtures ---

@pytest.fixture
def lidar_config():
    """Default LIDAR configuration."""
    return LidarConfig()


@pytest.fixture
def lidar_processor(lidar_config):
    """LIDAR processor instance."""
    return LidarProcessor(lidar_config)


@pytest.fixture
def vfh_config():
    """Default VFH configuration."""
    return VFHConfig()


@pytest.fixture
def vfh_navigator(vfh_config):
    """VFH navigator instance."""
    return VFHNavigator(vfh_config)


@pytest.fixture
def safety_config():
    """Default safety configuration."""
    return SafetyConfig()


@pytest.fixture
def safety_controller(safety_config):
    """Safety controller instance."""
    return SafetyController(safety_config)


@pytest.fixture
def robot_context():
    """Fresh robot context."""
    return RobotContext()


@pytest.fixture
def mock_scan_empty():
    """Empty scan (no obstacles)."""
    return MockLaserScan.empty()


@pytest.fixture
def mock_scan_wall_ahead():
    """Wall ahead at 0.5m."""
    return MockLaserScan.wall_ahead(0.5)


@pytest.fixture
def mock_scan_corridor():
    """Corridor 0.8m wide."""
    return MockLaserScan.corridor(0.8)


@pytest.fixture
def mock_scan_surrounded():
    """Surrounded by obstacles at 0.3m."""
    return MockLaserScan.surrounded(0.3)


@pytest.fixture
def sample_sensor_snapshot():
    """Sample sensor snapshot for testing."""
    return SensorSnapshot(
        timestamp=datetime.now(),
        pose=Pose2D(x=1.0, y=2.0, theta=0.5),
        velocity=Velocity(linear=0.2, angular=0.1),
        lidar_ranges=tuple([2.0] * 360),
        closest_obstacle=2.0,
        obstacle_direction=0.0
    )


# --- Helper Functions ---

def create_scan_with_obstacles(obstacles: list[tuple[float, float]]) -> MockLaserScan:
    """
    Create scan with multiple obstacles.

    Args:
        obstacles: List of (angle_deg, distance) tuples
    """
    ranges = [float('inf')] * 360
    for angle_deg, distance in obstacles:
        idx = int(angle_deg) % 360
        for offset in range(-10, 11):
            i = (idx + offset) % 360
            ranges[i] = min(ranges[i], distance)
    return MockLaserScan(ranges=ranges)
