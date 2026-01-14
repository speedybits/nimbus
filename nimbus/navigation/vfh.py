"""
Vector Field Histogram (VFH) obstacle avoidance for Nimbus.

VFH is a real-time obstacle avoidance algorithm that:
1. Builds a polar histogram of obstacle density
2. Identifies "valleys" (safe directions)
3. Chooses the best valley based on goal direction

This implementation is optimized for 360-degree LIDAR data.
"""

from dataclasses import dataclass
from typing import Tuple, List, Optional
import numpy as np


@dataclass(frozen=True)
class VFHConfig:
    """VFH algorithm configuration."""

    num_sectors: int = 72              # Histogram sectors
    threshold_high: float = 0.7        # Blocked if certainty > this
    threshold_low: float = 0.3         # Clear if certainty < this
    wide_valley_min: int = 8           # Sectors for "wide" valley
    goal_weight: float = 5.0           # Weight for goal direction
    smoothing_passes: int = 3          # Histogram smoothing iterations
    min_valley_width: int = 3          # Minimum usable valley


@dataclass
class Valley:
    """A contiguous clear region in the polar histogram."""

    start_sector: int
    end_sector: int
    width: int
    center_sector: int
    center_angle: float  # radians

    @property
    def is_wide(self) -> bool:
        """Check if this is a wide valley."""
        return self.width >= 8


class VFHNavigator:
    """
    Vector Field Histogram obstacle avoidance.

    Usage:
        vfh = VFHNavigator()
        steering, blocked = vfh.compute_steering(histogram, goal_direction)
    """

    def __init__(self, config: Optional[VFHConfig] = None):
        self.config = config or VFHConfig()
        self._sector_angle = 2 * np.pi / self.config.num_sectors

    def compute_steering(
        self,
        histogram: np.ndarray,
        goal_direction: float  # radians, 0 = forward
    ) -> Tuple[float, bool]:
        """
        Compute steering direction to avoid obstacles while heading toward goal.

        Args:
            histogram: Polar histogram from LidarProcessor
            goal_direction: Desired direction (radians, 0=forward, +ve=left)

        Returns:
            Tuple of (steering_angle_radians, is_blocked)
            - steering_angle: Direction to steer (-pi to pi)
            - is_blocked: True if no valid path found
        """
        # Ensure histogram is correct size
        if len(histogram) != self.config.num_sectors:
            histogram = self._resize_histogram(histogram)

        # Smooth histogram to reduce noise
        smoothed = self._smooth_histogram(histogram)

        # Apply thresholds to create binary blocked/clear map
        blocked_sectors = smoothed > self.config.threshold_high

        # Find valleys (contiguous clear sectors)
        valleys = self._find_valleys(blocked_sectors)

        if not valleys:
            # No clear path - robot is stuck
            return (0.0, True)

        # Find best valley based on goal direction
        best_valley = self._choose_best_valley(valleys, goal_direction)

        # Compute steering angle
        steering = self._compute_steering_from_valley(best_valley, goal_direction)

        return (steering, False)

    def _smooth_histogram(self, histogram: np.ndarray) -> np.ndarray:
        """Apply smoothing to reduce noise."""
        result = histogram.copy()
        kernel = np.array([0.25, 0.5, 0.25])

        for _ in range(self.config.smoothing_passes):
            # Wrap-around convolution for circular histogram
            padded = np.concatenate([result[-1:], result, result[:1]])
            convolved = np.convolve(padded, kernel, mode='valid')
            result = convolved

        return result

    def _find_valleys(self, blocked: np.ndarray) -> List[Valley]:
        """
        Find contiguous unblocked sectors (valleys).

        Returns list of Valley objects.
        """
        num_sectors = len(blocked)
        valleys = []

        # Handle wraparound by processing doubled array
        in_valley = False
        start_idx = 0

        # Find first blocked sector to start search
        first_blocked = np.argmax(blocked) if np.any(blocked) else 0

        for i in range(num_sectors):
            idx = (first_blocked + i) % num_sectors

            if not blocked[idx]:  # Clear sector
                if not in_valley:
                    start_idx = idx
                    in_valley = True
            else:  # Blocked sector
                if in_valley:
                    end_idx = (idx - 1) % num_sectors
                    valley = self._create_valley(start_idx, end_idx, num_sectors)
                    if valley.width >= self.config.min_valley_width:
                        valleys.append(valley)
                    in_valley = False

        # Handle valley that wraps around
        if in_valley:
            end_idx = (first_blocked - 1) % num_sectors
            valley = self._create_valley(start_idx, end_idx, num_sectors)
            if valley.width >= self.config.min_valley_width:
                valleys.append(valley)

        return valleys

    def _create_valley(self, start: int, end: int, num_sectors: int) -> Valley:
        """Create a Valley object from sector indices."""
        # Calculate width accounting for wraparound
        if end >= start:
            width = end - start + 1
        else:
            width = (num_sectors - start) + end + 1

        # Calculate center
        center = (start + width // 2) % num_sectors

        # Convert center to angle (-pi to pi, 0 = forward)
        center_angle = self._sector_to_angle(center)

        return Valley(
            start_sector=start,
            end_sector=end,
            width=width,
            center_sector=center,
            center_angle=center_angle
        )

    def _sector_to_angle(self, sector: int) -> float:
        """Convert sector index to angle (radians)."""
        # Sector 0 = 0 degrees = forward
        # Positive angles = counter-clockwise (left)
        angle = sector * self._sector_angle
        # Normalize to -pi to pi
        if angle > np.pi:
            angle -= 2 * np.pi
        return float(angle)

    def _angle_to_sector(self, angle: float) -> int:
        """Convert angle (radians) to sector index."""
        # Normalize angle to 0 to 2*pi
        while angle < 0:
            angle += 2 * np.pi
        while angle >= 2 * np.pi:
            angle -= 2 * np.pi
        return int(angle / self._sector_angle) % self.config.num_sectors

    def _choose_best_valley(self, valleys: List[Valley], goal_direction: float) -> Valley:
        """
        Choose the best valley based on goal direction and valley properties.

        Prefers wide valleys close to goal direction.
        """
        goal_sector = self._angle_to_sector(goal_direction)

        best_score = float('-inf')
        best_valley = valleys[0]

        for valley in valleys:
            # Distance to goal (accounting for wraparound)
            dist_to_goal = self._sector_distance(valley.center_sector, goal_sector)

            # Score: prefer wide valleys close to goal
            width_bonus = valley.width * (0.5 if valley.is_wide else 0.2)
            goal_penalty = dist_to_goal * self.config.goal_weight / self.config.num_sectors

            score = width_bonus - goal_penalty

            if score > best_score:
                best_score = score
                best_valley = valley

        return best_valley

    def _sector_distance(self, sector1: int, sector2: int) -> int:
        """Calculate minimum distance between sectors (accounting for wraparound)."""
        diff = abs(sector1 - sector2)
        return min(diff, self.config.num_sectors - diff)

    def _compute_steering_from_valley(self, valley: Valley, goal_direction: float) -> float:
        """
        Compute steering angle toward the chosen valley.

        For wide valleys, steer toward the edge closest to goal.
        For narrow valleys, steer toward center.
        """
        if valley.is_wide:
            # Wide valley: bias toward goal edge
            goal_sector = self._angle_to_sector(goal_direction)

            start_dist = self._sector_distance(valley.start_sector, goal_sector)
            end_dist = self._sector_distance(valley.end_sector, goal_sector)

            if start_dist < end_dist:
                # Goal is closer to start edge
                target_sector = (valley.start_sector + 2) % self.config.num_sectors
            else:
                # Goal is closer to end edge
                target_sector = (valley.end_sector - 2) % self.config.num_sectors

            return self._sector_to_angle(target_sector)
        else:
            # Narrow valley: steer toward center
            return valley.center_angle

    def _resize_histogram(self, histogram: np.ndarray) -> np.ndarray:
        """Resize histogram to expected number of sectors."""
        old_sectors = len(histogram)
        new_sectors = self.config.num_sectors

        if old_sectors == new_sectors:
            return histogram

        # Interpolate to new size
        old_indices = np.linspace(0, new_sectors - 1, old_sectors)
        new_indices = np.arange(new_sectors)
        return np.interp(new_indices, old_indices, histogram)


class SimpleAvoidance:
    """
    Simple reactive obstacle avoidance.

    Fallback when VFH is too complex or for simple scenarios.
    Uses three-zone detection (left, front, right).
    """

    def __init__(
        self,
        threshold: float = 0.5,
        turn_speed: float = 0.5,
        forward_speed: float = 0.2
    ):
        self.threshold = threshold
        self.turn_speed = turn_speed
        self.forward_speed = forward_speed

    def compute(self, sector_ranges: dict) -> Tuple[float, float]:
        """
        Compute velocity from sector ranges.

        Args:
            sector_ranges: Dict with 'front', 'left', 'right' minimum ranges

        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        front = sector_ranges.get("front", float('inf'))
        left = sector_ranges.get("front_left", float('inf'))
        right = sector_ranges.get("front_right", float('inf'))

        # Check each zone
        front_blocked = front < self.threshold
        left_blocked = left < self.threshold
        right_blocked = right < self.threshold

        if not front_blocked and not left_blocked and not right_blocked:
            # All clear - go forward
            return (self.forward_speed, 0.0)

        elif front_blocked and not left_blocked and not right_blocked:
            # Front blocked - turn toward more open side
            if left > right:
                return (0.0, self.turn_speed)
            else:
                return (0.0, -self.turn_speed)

        elif front_blocked and left_blocked and not right_blocked:
            # Front and left blocked - turn right
            return (0.0, -self.turn_speed)

        elif front_blocked and not left_blocked and right_blocked:
            # Front and right blocked - turn left
            return (0.0, self.turn_speed)

        elif not front_blocked and left_blocked:
            # Left blocked - bias right
            return (self.forward_speed * 0.5, -self.turn_speed * 0.3)

        elif not front_blocked and right_blocked:
            # Right blocked - bias left
            return (self.forward_speed * 0.5, self.turn_speed * 0.3)

        else:
            # Everything blocked - stop and think
            return (0.0, 0.0)
