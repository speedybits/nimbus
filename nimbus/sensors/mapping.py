"""
Obstacle mapping for Nimbus dashboard visualization.

Accumulates obstacles from LIDAR scans into a persistent grid map,
distinguishing between free space (traversed by LIDAR rays) and
obstacles (where rays terminate).
"""

import math
from dataclasses import dataclass, field
from typing import Optional

from nimbus.core.state import Pose2D


@dataclass
class MapConfig:
    """Configuration for obstacle mapping."""

    cell_size: float = 0.15  # 15cm resolution
    max_range: float = 3.0   # Ignore LIDAR readings beyond this
    min_range: float = 0.1   # Ignore readings closer than this (noise)
    obstacle_confidence_inc: float = 0.3  # Confidence increase per obstacle hit
    obstacle_confidence_dec: float = 0.1  # Confidence decrease when ray passes through
    max_confidence: float = 1.0
    min_confidence: float = 0.0
    max_trail_points: int = 50  # Robot position history length


class ObstacleMap:
    """
    Accumulates obstacles from LIDAR scans.

    Uses a sparse grid representation:
    - _obstacles: dict of (cell_x, cell_y) -> confidence (0-1)
    - _visited: set of cells the robot has passed through or LIDAR rays have cleared

    The map is centered on the explored space, not the robot, and
    bounds expand as exploration continues.
    """

    def __init__(self, config: Optional[MapConfig] = None):
        self.config = config or MapConfig()
        self._obstacles: dict[tuple[int, int], float] = {}
        self._visited: set[tuple[int, int]] = set()
        self._robot_trail: list[tuple[float, float]] = []
        self._trail_origin: Optional[tuple[float, float]] = None
        self._trail_active: bool = False
        self._last_pose: Optional[Pose2D] = None

    def _world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        """Convert world coordinates to grid cell indices."""
        cell_x = int(math.floor(x / self.config.cell_size))
        cell_y = int(math.floor(y / self.config.cell_size))
        return (cell_x, cell_y)

    def _cell_to_world(self, cell_x: int, cell_y: int) -> tuple[float, float]:
        """Convert cell indices to world coordinates (cell center)."""
        x = (cell_x + 0.5) * self.config.cell_size
        y = (cell_y + 0.5) * self.config.cell_size
        return (x, y)

    def update(self, pose: Pose2D, lidar_ranges: tuple[float, ...]) -> None:
        """
        Update map with new LIDAR scan.

        Args:
            pose: Current robot pose in world frame
            lidar_ranges: Tuple of range readings (one per degree, 360 total)
        """
        if not lidar_ranges or len(lidar_ranges) == 0:
            return

        # Update robot trail
        self._update_trail(pose)

        # Mark robot position as visited
        robot_cell = self._world_to_cell(pose.x, pose.y)
        self._visited.add(robot_cell)

        # Process each LIDAR ray
        num_readings = len(lidar_ranges)
        degrees_per_reading = 360.0 / num_readings

        for i, distance in enumerate(lidar_ranges):
            # Skip invalid readings
            if distance < self.config.min_range or distance > self.config.max_range:
                continue
            if math.isinf(distance) or math.isnan(distance):
                continue

            # Calculate world angle
            # Real LIDAR convention: index 0 = backward, index 180 = forward
            # Add pi to convert to world frame correctly
            lidar_angle_deg = i * degrees_per_reading
            lidar_angle_rad = math.radians(lidar_angle_deg)
            world_angle = pose.theta + math.pi + lidar_angle_rad

            # Calculate obstacle endpoint in world frame
            obs_x = pose.x + distance * math.cos(world_angle)
            obs_y = pose.y + distance * math.sin(world_angle)

            # Mark cells along the ray as free (using Bresenham-style stepping)
            self._trace_ray_free(pose.x, pose.y, obs_x, obs_y)

            # Mark obstacle cell
            obs_cell = self._world_to_cell(obs_x, obs_y)
            self._mark_obstacle(obs_cell)

        self._last_pose = pose

    def _update_trail(self, pose: Pose2D) -> None:
        """Update robot trail with new position.

        Trail recording is deferred until the robot has moved at least 0.15m
        from its initial position, avoiding startup drift artifacts.
        """
        # Remember where the robot started
        if self._trail_origin is None:
            self._trail_origin = (pose.x, pose.y)
            return

        # Don't record trail until robot has moved meaningfully from start
        if not self._trail_active:
            ox, oy = self._trail_origin
            dist_from_origin = math.sqrt((pose.x - ox) ** 2 + (pose.y - oy) ** 2)
            if dist_from_origin < 0.15:  # 15cm threshold
                return
            self._trail_active = True

        # Only add if moved significantly from last trail point
        if self._robot_trail:
            last_x, last_y = self._robot_trail[-1]
            dist = math.sqrt((pose.x - last_x) ** 2 + (pose.y - last_y) ** 2)
            if dist < 0.05:  # Less than 5cm movement
                return

        self._robot_trail.append((pose.x, pose.y))

    def _trace_ray_free(self, x0: float, y0: float, x1: float, y1: float) -> None:
        """
        Mark cells along a ray as free space (visited).

        Uses a simple stepping algorithm along the ray.
        Stops just before the endpoint (obstacle cell).
        """
        dx = x1 - x0
        dy = y1 - y0
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.01:
            return

        # Step size slightly smaller than cell size for accuracy
        step_size = self.config.cell_size * 0.5
        num_steps = int(dist / step_size)

        for i in range(num_steps):
            # Don't mark the final step (that's the obstacle)
            t = i / max(num_steps, 1)
            x = x0 + dx * t
            y = y0 + dy * t
            cell = self._world_to_cell(x, y)
            self._visited.add(cell)

            # Slightly decrease obstacle confidence if ray passes through
            if cell in self._obstacles:
                self._obstacles[cell] = max(
                    self.config.min_confidence,
                    self._obstacles[cell] - self.config.obstacle_confidence_dec
                )
                # Remove if confidence drops to zero
                if self._obstacles[cell] <= self.config.min_confidence:
                    del self._obstacles[cell]

    def _mark_obstacle(self, cell: tuple[int, int]) -> None:
        """Mark a cell as containing an obstacle."""
        current = self._obstacles.get(cell, 0.0)
        new_confidence = min(
            self.config.max_confidence,
            current + self.config.obstacle_confidence_inc
        )
        self._obstacles[cell] = new_confidence

    def get_bounds(self) -> tuple[float, float, float, float]:
        """
        Get world-coordinate bounds of all known space.

        Returns:
            (min_x, min_y, max_x, max_y) in meters.
            Returns (0, 0, 0, 0) if map is empty.
        """
        # Collect all known cells (both obstacles and visited)
        all_cells = set(self._obstacles.keys()) | self._visited

        if not all_cells:
            return (0.0, 0.0, 0.0, 0.0)

        min_cell_x = min(c[0] for c in all_cells)
        max_cell_x = max(c[0] for c in all_cells)
        min_cell_y = min(c[1] for c in all_cells)
        max_cell_y = max(c[1] for c in all_cells)

        # Convert to world coordinates
        min_x = min_cell_x * self.config.cell_size
        min_y = min_cell_y * self.config.cell_size
        max_x = (max_cell_x + 1) * self.config.cell_size
        max_y = (max_cell_y + 1) * self.config.cell_size

        return (min_x, min_y, max_x, max_y)

    def get_robot_trail(self, max_points: int = 50) -> list[tuple[float, float]]:
        """Get recent robot positions for rendering trail."""
        if max_points >= len(self._robot_trail):
            return list(self._robot_trail)
        return self._robot_trail[-max_points:]

    def get_obstacles(self) -> dict[tuple[int, int], float]:
        """Get obstacle cells with their confidence values."""
        return dict(self._obstacles)

    def get_visited(self) -> set[tuple[int, int]]:
        """Get all visited (free space) cells."""
        return set(self._visited)

    def clear(self) -> None:
        """Clear all map data."""
        self._obstacles.clear()
        self._visited.clear()
        self._robot_trail.clear()
        self._trail_origin = None
        self._trail_active = False
        self._last_pose = None

    @property
    def num_obstacles(self) -> int:
        """Number of obstacle cells."""
        return len(self._obstacles)

    @property
    def num_visited(self) -> int:
        """Number of visited (free space) cells."""
        return len(self._visited)
