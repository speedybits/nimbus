"""
LIDAR sensor processing for Nimbus.

Converts raw LaserScan messages into polar histograms
suitable for VFH obstacle avoidance.
"""

from dataclasses import dataclass
from typing import Tuple, Optional
import numpy as np


@dataclass(frozen=True)
class LidarConfig:
    """LIDAR processing configuration."""

    num_sectors: int = 72          # 5-degree sectors (360/72 = 5)
    safety_radius: float = 0.30    # Robot radius + safety margin (m)
    max_range: float = 3.5         # Maximum reliable range (m)
    min_range: float = 0.05        # Minimum reliable range (m)


class LidarProcessor:
    """
    Efficient LIDAR data processing.

    Converts 360 raw range values into polar histogram for VFH.
    Uses numpy vectorization for speed.

    Usage:
        processor = LidarProcessor()
        histogram = processor.process(scan_msg)
        closest_dist, closest_angle = processor.find_closest(scan_msg)
    """

    def __init__(self, config: Optional[LidarConfig] = None):
        self.config = config or LidarConfig()
        self._sector_size = 360.0 / self.config.num_sectors

        # Precompute angle arrays for vectorized operations
        self._angles_deg = np.arange(360)
        self._angles_rad = np.deg2rad(self._angles_deg)

        # Sector lookup for each LIDAR angle
        self._sector_lookup = (self._angles_deg / self._sector_size).astype(int)

    def process(self, ranges: np.ndarray) -> np.ndarray:
        """
        Convert raw LIDAR ranges to polar obstacle histogram.

        Args:
            ranges: Array of 360 distance measurements (meters)

        Returns:
            Histogram array of shape (num_sectors,) with values [0, 1]
            where 0 = clear, 1 = blocked
        """
        if len(ranges) != 360:
            # Handle non-360 scans by interpolating
            ranges = self._interpolate_ranges(ranges)

        ranges = np.array(ranges, dtype=np.float32)

        # Filter invalid readings
        valid_mask = (
            (ranges > self.config.min_range) &
            (ranges < self.config.max_range) &
            np.isfinite(ranges)
        )

        # Create histogram
        histogram = np.zeros(self.config.num_sectors, dtype=np.float32)

        # Obstacle detection threshold (2x safety radius)
        obstacle_threshold = self.config.safety_radius * 2.5

        for i in range(360):
            if valid_mask[i] and ranges[i] < obstacle_threshold:
                sector = self._sector_lookup[i]
                # Closer = higher certainty
                certainty = 1.0 - (ranges[i] / obstacle_threshold)
                histogram[sector] = max(histogram[sector], certainty)

        return histogram

    def process_from_msg(self, scan_msg) -> np.ndarray:
        """
        Process a ROS2 LaserScan message.

        Args:
            scan_msg: sensor_msgs/LaserScan message

        Returns:
            Polar histogram array
        """
        return self.process(np.array(scan_msg.ranges))

    def find_closest(self, ranges: np.ndarray) -> Tuple[float, float]:
        """
        Find the closest obstacle.

        Args:
            ranges: Array of distance measurements

        Returns:
            Tuple of (distance_meters, angle_radians)
            Returns (inf, 0) if no valid obstacles found
        """
        ranges = np.array(ranges, dtype=np.float32)

        # Create valid mask
        valid_mask = (
            (ranges > self.config.min_range) &
            (ranges < self.config.max_range) &
            np.isfinite(ranges)
        )

        if not np.any(valid_mask):
            return (float('inf'), 0.0)

        # Replace invalid with infinity for argmin
        ranges_filtered = np.where(valid_mask, ranges, float('inf'))
        min_idx = np.argmin(ranges_filtered)

        distance = float(ranges_filtered[min_idx])

        # Convert index to angle
        # LIDAR typically: index 0 = front (0 deg), increases CCW
        # We want: 0 = front, positive = left, negative = right
        angle_deg = min_idx if min_idx <= 180 else min_idx - 360
        angle_rad = np.deg2rad(angle_deg)

        return (distance, angle_rad)

    def find_closest_from_msg(self, scan_msg) -> Tuple[float, float]:
        """Find closest obstacle from a ROS2 LaserScan message."""
        return self.find_closest(np.array(scan_msg.ranges))

    def get_sector_ranges(self, ranges: np.ndarray) -> dict:
        """
        Get minimum range in each named sector.

        Useful for debugging and simple obstacle avoidance.

        Returns:
            Dict with keys: front, front_left, left, rear_left,
                           rear, rear_right, right, front_right
        """
        ranges = np.array(ranges, dtype=np.float32)
        valid_ranges = np.where(
            (ranges > self.config.min_range) & (ranges < self.config.max_range),
            ranges,
            float('inf')
        )

        # Sector definitions (angle ranges, LIDAR convention)
        sectors = {
            "front": (337.5, 22.5),      # -22.5 to 22.5 deg
            "front_left": (22.5, 67.5),
            "left": (67.5, 112.5),
            "rear_left": (112.5, 157.5),
            "rear": (157.5, 202.5),
            "rear_right": (202.5, 247.5),
            "right": (247.5, 292.5),
            "front_right": (292.5, 337.5),
        }

        result = {}
        for name, (start, end) in sectors.items():
            if start > end:  # Wraps around 0
                mask = (self._angles_deg >= start) | (self._angles_deg < end)
            else:
                mask = (self._angles_deg >= start) & (self._angles_deg < end)

            sector_ranges = valid_ranges[mask]
            result[name] = float(np.min(sector_ranges)) if len(sector_ranges) > 0 else float('inf')

        return result

    def _interpolate_ranges(self, ranges: np.ndarray) -> np.ndarray:
        """Interpolate non-360 scans to 360 points."""
        num_points = len(ranges)
        if num_points == 360:
            return ranges

        # Linear interpolation to 360 points
        old_indices = np.linspace(0, 359, num_points)
        new_indices = np.arange(360)
        return np.interp(new_indices, old_indices, ranges)


# Convenience function for creating mock LIDAR data for testing
def create_mock_scan(
    pattern: str = "empty",
    obstacle_distance: float = 0.5,
    obstacle_angle: float = 0.0,
    corridor_width: float = 0.8
) -> np.ndarray:
    """
    Create mock LIDAR scan data for testing.

    Args:
        pattern: One of "empty", "wall_ahead", "corridor", "surrounded"
        obstacle_distance: Distance to obstacles (meters)
        obstacle_angle: Angle for single obstacles (degrees)
        corridor_width: Width for corridor pattern (meters)

    Returns:
        Array of 360 range values
    """
    ranges = np.full(360, 10.0, dtype=np.float32)  # Default: far away

    if pattern == "empty":
        pass  # All clear

    elif pattern == "wall_ahead":
        # Wall directly ahead (-30 to +30 degrees)
        for i in list(range(0, 31)) + list(range(330, 360)):
            ranges[i] = obstacle_distance

    elif pattern == "corridor":
        # Walls on left and right
        half_width = corridor_width / 2
        for i in range(60, 120):  # Left side
            ranges[i] = half_width
        for i in range(240, 300):  # Right side
            ranges[i] = half_width

    elif pattern == "surrounded":
        # Obstacles all around
        ranges[:] = obstacle_distance

    elif pattern == "obstacle":
        # Single obstacle at specified angle
        angle_idx = int(obstacle_angle) % 360
        for offset in range(-10, 11):
            idx = (angle_idx + offset) % 360
            ranges[idx] = obstacle_distance

    return ranges
