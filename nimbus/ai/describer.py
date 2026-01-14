"""
LIDAR to natural language describer for Nimbus AI exploration.

Converts sensor data into human-readable descriptions that
Ollama can understand and reason about.
"""

import math
from dataclasses import dataclass
from typing import Optional, TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from nimbus.core.state import SensorSnapshot
    from nimbus.ai.memory import ExplorationMemory


@dataclass(frozen=True)
class DescriptionConfig:
    """Configuration for LIDAR description."""

    # Distance thresholds (meters)
    blocked_threshold: float = 0.3
    nearby_threshold: float = 1.0
    clear_threshold: float = 2.0
    # Above clear_threshold = "open"

    # Include memory context
    include_memory: bool = True
    max_recent_positions: int = 5


class LidarDescriber:
    """
    Convert LIDAR sensor data to natural language.

    Produces descriptions like:
    "Position: (1.5, 2.0)m, heading northeast (45°).
     Environment: Wall 0.3m on left, clear corridor ahead (2.5m),
     open space to the right (3.0m+)."

    Usage:
        describer = LidarDescriber()
        description = describer.describe(sensors, memory)
    """

    def __init__(self, config: Optional[DescriptionConfig] = None):
        self.config = config or DescriptionConfig()

        # Sector definitions for description
        # Each sector: (name, start_angle, end_angle) in degrees
        # Angles: 0 = front, positive = left (CCW), negative = right (CW)
        self._sectors = [
            ("front", -22.5, 22.5),
            ("front-left", 22.5, 67.5),
            ("left", 67.5, 112.5),
            ("back-left", 112.5, 157.5),
            ("back", 157.5, 180.0),  # Split back
            ("back", -180.0, -157.5),
            ("back-right", -157.5, -112.5),
            ("right", -112.5, -67.5),
            ("front-right", -67.5, -22.5),
        ]

    def describe(
        self,
        sensors: "SensorSnapshot",
        memory: Optional["ExplorationMemory"] = None,
    ) -> str:
        """
        Generate natural language description of the environment.

        Args:
            sensors: Current sensor snapshot
            memory: Optional exploration memory for context

        Returns:
            Human-readable description string
        """
        lines = []

        # Position and heading
        pose_desc = self._describe_pose(sensors)
        lines.append(pose_desc)

        # Environment description
        env_desc = self._describe_environment(sensors)
        lines.append(f"Environment: {env_desc}")

        # Closest obstacle
        closest_desc = self._describe_closest(sensors)
        lines.append(closest_desc)

        # Memory context if available
        if memory and self.config.include_memory:
            memory_desc = self._describe_memory_context(sensors, memory)
            if memory_desc:
                lines.append(memory_desc)

        return "\n".join(lines)

    def _describe_pose(self, sensors: "SensorSnapshot") -> str:
        """Describe current position and heading."""
        pose = sensors.pose
        x, y = pose.x, pose.y
        theta_deg = math.degrees(pose.theta)

        # Convert heading to compass direction
        direction = self._heading_to_compass(theta_deg)

        return f"Position: ({x:.1f}, {y:.1f})m, heading {direction} ({theta_deg:.0f}°)"

    def _heading_to_compass(self, degrees: float) -> str:
        """Convert heading angle to compass direction."""
        # Normalize to 0-360
        degrees = degrees % 360
        if degrees < 0:
            degrees += 360

        directions = [
            (0, "east"),
            (45, "northeast"),
            (90, "north"),
            (135, "northwest"),
            (180, "west"),
            (225, "southwest"),
            (270, "south"),
            (315, "southeast"),
            (360, "east"),
        ]

        for threshold, name in directions:
            if degrees < threshold + 22.5:
                return name
        return "east"

    def _describe_environment(self, sensors: "SensorSnapshot") -> str:
        """Describe what's in each direction."""
        ranges = np.array(sensors.lidar_ranges)

        # Get min range in each sector
        sector_ranges = {}
        for name, start, end in self._sectors:
            sector_range = self._get_sector_min(ranges, start, end)
            if name in sector_ranges:
                # Merge split sectors (like "back")
                sector_ranges[name] = min(sector_ranges[name], sector_range)
            else:
                sector_ranges[name] = sector_range

        # Build description
        descriptions = []
        for name in ["front", "front-left", "left", "back", "right", "front-right"]:
            if name not in sector_ranges:
                continue

            dist = sector_ranges[name]
            desc = self._distance_to_description(dist)
            descriptions.append(f"{desc} {name} ({dist:.1f}m)")

        return ", ".join(descriptions)

    def _get_sector_min(
        self, ranges: np.ndarray, start_deg: float, end_deg: float
    ) -> float:
        """Get minimum range in an angular sector."""
        # Convert to LIDAR indices (0 = front, increases CCW)
        # Handle wrap-around
        if start_deg < 0:
            start_deg += 360
        if end_deg < 0:
            end_deg += 360

        start_idx = int(start_deg) % 360
        end_idx = int(end_deg) % 360

        if start_idx <= end_idx:
            sector = ranges[start_idx:end_idx]
        else:
            # Wrap around
            sector = np.concatenate([ranges[start_idx:], ranges[:end_idx]])

        # Filter invalid
        valid = sector[(sector > 0.05) & (sector < 10.0) & np.isfinite(sector)]
        if len(valid) == 0:
            return float("inf")

        return float(np.min(valid))

    def _distance_to_description(self, distance: float) -> str:
        """Convert distance to descriptive word."""
        if distance < self.config.blocked_threshold:
            return "BLOCKED"
        elif distance < self.config.nearby_threshold:
            return "obstacle"
        elif distance < self.config.clear_threshold:
            return "clear"
        else:
            return "open"

    def _describe_closest(self, sensors: "SensorSnapshot") -> str:
        """Describe the closest obstacle."""
        dist = sensors.closest_obstacle
        angle = sensors.obstacle_direction

        if dist > 10.0 or not math.isfinite(dist):
            return "Closest obstacle: None detected within range"

        # Convert angle to direction word
        angle_deg = math.degrees(angle)
        if abs(angle_deg) < 22.5:
            direction = "directly ahead"
        elif angle_deg > 0:
            if angle_deg < 67.5:
                direction = "ahead-left"
            elif angle_deg < 112.5:
                direction = "to the left"
            else:
                direction = "behind-left"
        else:
            if angle_deg > -67.5:
                direction = "ahead-right"
            elif angle_deg > -112.5:
                direction = "to the right"
            else:
                direction = "behind-right"

        urgency = ""
        if dist < self.config.blocked_threshold:
            urgency = " [DANGER - too close!]"
        elif dist < self.config.nearby_threshold:
            urgency = " [caution]"

        return f"Closest obstacle: {dist:.2f}m {direction}{urgency}"

    def _describe_memory_context(
        self,
        sensors: "SensorSnapshot",
        memory: "ExplorationMemory",
    ) -> str:
        """Add context from exploration memory."""
        pose = sensors.pose

        # Get unexplored directions
        unexplored = memory.get_unexplored_directions(pose)
        explored_count = len(memory.visited_cells)

        parts = []

        if explored_count > 0:
            parts.append(f"Cells explored: {explored_count}")

        if unexplored:
            parts.append(f"Unexplored directions: {', '.join(unexplored)}")
        else:
            parts.append("All nearby areas explored")

        return "Memory: " + "; ".join(parts) if parts else ""


def describe_for_debug(sensors: "SensorSnapshot") -> str:
    """
    Quick description for debugging/logging.

    Returns a one-line summary.
    """
    pose = sensors.pose
    closest = sensors.closest_obstacle

    return (
        f"pos=({pose.x:.2f},{pose.y:.2f}) "
        f"heading={math.degrees(pose.theta):.0f}° "
        f"closest={closest:.2f}m"
    )
