"""
Simulated 2D grid world for Nimbus.

Provides:
- Grid-based map loading from ASCII files
- Robot pose tracking
- Ray-casting for synthetic LIDAR
- Differential drive kinematics
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple, List
import math


@dataclass
class SimulatedWorld:
    """
    2D grid-based simulated world.

    The world is represented as a grid where:
    - '#' = wall (obstacle)
    - '.' = free space
    - 'R' = robot spawn point (treated as free space)

    Coordinate system:
    - Origin (0, 0) is at the center of the map
    - X increases to the right
    - Y increases upward
    - Theta is counter-clockwise from X axis
    """

    grid: List[List[bool]] = field(default_factory=list)  # True = obstacle
    resolution: float = 0.1  # meters per cell
    width: int = 0  # cells
    height: int = 0  # cells

    # Robot state
    robot_x: float = 0.0
    robot_y: float = 0.0
    robot_theta: float = 0.0

    # Cached origin offset (center of map in world coords)
    _origin_x: float = 0.0
    _origin_y: float = 0.0

    @classmethod
    def from_file(cls, path: Path) -> "SimulatedWorld":
        """
        Load a world from an ASCII map file.

        File format:
            # Comment lines start with #
            ##########
            #........#
            #...##...#
            #...R....#
            ##########
            resolution: 0.1

        Args:
            path: Path to the map file

        Returns:
            SimulatedWorld instance
        """
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"Map file not found: {path}")

        lines = []
        resolution = 0.1
        spawn_x, spawn_y = None, None

        with open(path) as f:
            for line in f:
                line = line.rstrip('\n')

                # Parse metadata (must start with keyword, not be a wall line)
                if line.startswith("resolution:"):
                    resolution = float(line.split(":")[1].strip())
                    continue

                # Skip empty lines
                if not line:
                    continue

                # Check if this is a map line (contains map characters)
                map_chars = set("#.R")
                if any(c in map_chars for c in line):
                    lines.append(line)

        if not lines:
            raise ValueError(f"No map data found in {path}")

        # Parse grid (reverse so Y increases upward)
        width = max(len(line) for line in lines)
        height = len(lines)
        grid = []

        for row_idx, line in enumerate(reversed(lines)):
            row = []
            for col_idx, char in enumerate(line):
                if char == 'R':
                    spawn_x = col_idx
                    spawn_y = row_idx
                    row.append(False)  # Spawn point is free space
                elif char == '#':
                    row.append(True)  # Wall
                else:
                    row.append(False)  # Free space
            # Pad row to width
            row.extend([False] * (width - len(row)))
            grid.append(row)

        # Create world
        world = cls(
            grid=grid,
            resolution=resolution,
            width=width,
            height=height,
        )

        # Calculate origin (center of map)
        world._origin_x = (width * resolution) / 2
        world._origin_y = (height * resolution) / 2

        # Set spawn position
        if spawn_x is not None and spawn_y is not None:
            world.robot_x = spawn_x * resolution - world._origin_x + resolution / 2
            world.robot_y = spawn_y * resolution - world._origin_y + resolution / 2
        else:
            world.robot_x = 0.0
            world.robot_y = 0.0
        world.robot_theta = 0.0

        return world

    @classmethod
    def from_template(cls, name: str) -> "SimulatedWorld":
        """
        Load a built-in map template.

        Available templates:
        - empty_room: Simple 5x5m room
        - simple_maze: Room with obstacles

        Args:
            name: Template name

        Returns:
            SimulatedWorld instance
        """
        # Find the maps directory (relative to this file)
        maps_dir = Path(__file__).parent / "maps"
        template_path = maps_dir / f"{name}.txt"

        if template_path.exists():
            return cls.from_file(template_path)

        # Built-in fallback templates
        if name == "empty_room":
            return cls._create_empty_room()
        elif name == "simple_maze":
            return cls._create_simple_maze()
        else:
            raise ValueError(f"Unknown template: {name}. Available: empty_room, simple_maze")

    @classmethod
    def _create_empty_room(cls, size: float = 5.0, resolution: float = 0.1) -> "SimulatedWorld":
        """Create an empty rectangular room."""
        cells = int(size / resolution)
        grid = []

        for y in range(cells):
            row = []
            for x in range(cells):
                # Walls on edges
                is_wall = (x == 0 or x == cells - 1 or y == 0 or y == cells - 1)
                row.append(is_wall)
            grid.append(row)

        world = cls(
            grid=grid,
            resolution=resolution,
            width=cells,
            height=cells,
        )
        world._origin_x = size / 2
        world._origin_y = size / 2

        return world

    @classmethod
    def _create_simple_maze(cls, resolution: float = 0.1) -> "SimulatedWorld":
        """Create a simple maze with obstacles."""
        # 5x5 meter room with internal obstacles
        cells = int(5.0 / resolution)
        grid = []

        for y in range(cells):
            row = []
            for x in range(cells):
                # Outer walls
                if x == 0 or x == cells - 1 or y == 0 or y == cells - 1:
                    row.append(True)
                # Internal obstacle (vertical wall)
                elif 15 <= x <= 17 and 10 <= y <= 30:
                    row.append(True)
                # Internal obstacle (horizontal wall)
                elif 30 <= x <= 40 and 23 <= y <= 25:
                    row.append(True)
                else:
                    row.append(False)
            grid.append(row)

        world = cls(
            grid=grid,
            resolution=resolution,
            width=cells,
            height=cells,
        )
        world._origin_x = 2.5
        world._origin_y = 2.5

        return world

    def set_robot_pose(self, x: float, y: float, theta: float) -> None:
        """Set the robot's position and orientation."""
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta

    def get_robot_pose(self) -> Tuple[float, float, float]:
        """Get the robot's current pose."""
        return (self.robot_x, self.robot_y, self.robot_theta)

    def apply_velocity(self, linear: float, angular: float, dt: float) -> None:
        """
        Apply velocity command using differential drive kinematics.

        Updates robot pose based on:
            x += linear * cos(theta) * dt
            y += linear * sin(theta) * dt
            theta += angular * dt

        Args:
            linear: Forward velocity (m/s)
            angular: Rotational velocity (rad/s)
            dt: Time step (seconds)
        """
        # Simple kinematics (no collision for now)
        new_x = self.robot_x + linear * math.cos(self.robot_theta) * dt
        new_y = self.robot_y + linear * math.sin(self.robot_theta) * dt
        new_theta = self.robot_theta + angular * dt

        # Normalize theta to [-pi, pi]
        while new_theta > math.pi:
            new_theta -= 2 * math.pi
        while new_theta < -math.pi:
            new_theta += 2 * math.pi

        # Check for collision before moving
        if not self._is_collision(new_x, new_y):
            self.robot_x = new_x
            self.robot_y = new_y
        self.robot_theta = new_theta

    def _is_collision(self, x: float, y: float, radius: float = 0.15) -> bool:
        """Check if a position collides with obstacles."""
        # Check a few points around the robot
        for dx in [-radius, 0, radius]:
            for dy in [-radius, 0, radius]:
                if self._is_obstacle(x + dx, y + dy):
                    return True
        return False

    def _is_obstacle(self, x: float, y: float) -> bool:
        """Check if a world coordinate is an obstacle."""
        # Convert world coords to grid coords
        grid_x = int((x + self._origin_x) / self.resolution)
        grid_y = int((y + self._origin_y) / self.resolution)

        # Out of bounds is treated as obstacle
        if grid_x < 0 or grid_x >= self.width:
            return True
        if grid_y < 0 or grid_y >= self.height:
            return True

        return self.grid[grid_y][grid_x]

    def raycast(self, num_rays: int = 360, max_range: float = 3.5) -> List[float]:
        """
        Cast rays from the robot to generate synthetic LIDAR data.

        Args:
            num_rays: Number of rays (default 360 for 1-degree resolution)
            max_range: Maximum range in meters

        Returns:
            List of distances for each ray (index 0 = directly backward,
            index 180 = directly forward, matching real LIDAR convention)
        """
        ranges = []

        for i in range(num_rays):
            # Calculate ray angle
            # Real LIDAR convention: index 0 = backward, index 180 = forward
            # Add pi to offset so index 0 points backward
            angle = self.robot_theta + math.pi + (i * 2 * math.pi / num_rays)

            # Cast the ray
            distance = self._cast_ray(
                self.robot_x, self.robot_y,
                angle, max_range
            )
            ranges.append(distance)

        return ranges

    def _cast_ray(
        self,
        start_x: float,
        start_y: float,
        angle: float,
        max_range: float,
        step: float = 0.02  # 2cm steps
    ) -> float:
        """
        Cast a single ray and return distance to obstacle.

        Uses simple ray marching (not DDA, but sufficient for simulation).
        """
        dx = math.cos(angle) * step
        dy = math.sin(angle) * step

        x, y = start_x, start_y
        distance = 0.0

        while distance < max_range:
            x += dx
            y += dy
            distance += step

            if self._is_obstacle(x, y):
                return distance

        return max_range

    def __repr__(self) -> str:
        return (
            f"SimulatedWorld({self.width}x{self.height} cells, "
            f"resolution={self.resolution}m, "
            f"robot=({self.robot_x:.2f}, {self.robot_y:.2f}, {self.robot_theta:.2f}))"
        )
