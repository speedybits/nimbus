"""
Exploration memory for Nimbus AI exploration.

Tracks visited locations, saves/loads exploration state,
and provides context for Ollama decisions.
"""

import json
import math
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from nimbus.core.state import Pose2D


@dataclass
class WaypointRecord:
    """Record of a single waypoint decision."""

    x: float
    y: float
    reason: str
    timestamp: str
    reached: bool = False

    def to_dict(self) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "reason": self.reason,
            "timestamp": self.timestamp,
            "reached": self.reached,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "WaypointRecord":
        return cls(
            x=data["x"],
            y=data["y"],
            reason=data["reason"],
            timestamp=data["timestamp"],
            reached=data.get("reached", False),
        )


@dataclass
class ExplorationMemory:
    """
    Persistent memory of explored areas.

    Tracks visited grid cells, path history, and Ollama decisions.
    Can be saved/loaded to continue exploration across sessions.

    Grid uses 0.5m resolution by default - each cell represents
    a 0.5m x 0.5m area.

    Usage:
        memory = ExplorationMemory.load_or_create("kitchen")
        memory.mark_visited(current_pose)
        memory.save()
    """

    name: str
    visited_cells: set = field(default_factory=set)  # set of (grid_x, grid_y)
    path_history: list = field(default_factory=list)  # list of (x, y) tuples
    waypoint_history: list = field(default_factory=list)  # list of WaypointRecord
    created_at: str = field(default_factory=lambda: datetime.now().isoformat())
    updated_at: str = field(default_factory=lambda: datetime.now().isoformat())

    # Configuration
    cell_size: float = 0.5  # meters per grid cell
    max_path_history: int = 100
    max_waypoint_history: int = 20

    # Default storage location
    _storage_dir: Path = field(
        default_factory=lambda: Path.home() / ".nimbus" / "memories"
    )

    def __post_init__(self):
        # Convert lists to proper types if loaded from JSON
        if isinstance(self.visited_cells, list):
            self.visited_cells = {tuple(cell) for cell in self.visited_cells}
        if self.waypoint_history and isinstance(self.waypoint_history[0], dict):
            self.waypoint_history = [
                WaypointRecord.from_dict(w) for w in self.waypoint_history
            ]

    def _pose_to_cell(self, x: float, y: float) -> tuple:
        """Convert world coordinates to grid cell."""
        cell_x = int(math.floor(x / self.cell_size))
        cell_y = int(math.floor(y / self.cell_size))
        return (cell_x, cell_y)

    def mark_visited(self, pose: "Pose2D") -> None:
        """Mark current location as visited."""
        cell = self._pose_to_cell(pose.x, pose.y)
        self.visited_cells.add(cell)

        # Add to path history
        self.path_history.append((pose.x, pose.y))
        if len(self.path_history) > self.max_path_history:
            self.path_history = self.path_history[-self.max_path_history:]

        self.updated_at = datetime.now().isoformat()

    def add_waypoint(self, x: float, y: float, reason: str) -> None:
        """Record a waypoint decision."""
        record = WaypointRecord(
            x=x,
            y=y,
            reason=reason,
            timestamp=datetime.now().isoformat(),
        )
        self.waypoint_history.append(record)

        if len(self.waypoint_history) > self.max_waypoint_history:
            self.waypoint_history = self.waypoint_history[-self.max_waypoint_history:]

        self.updated_at = datetime.now().isoformat()

    def mark_waypoint_reached(self, x: float, y: float) -> None:
        """Mark the most recent matching waypoint as reached."""
        for wp in reversed(self.waypoint_history):
            if abs(wp.x - x) < 0.5 and abs(wp.y - y) < 0.5:
                wp.reached = True
                break

    def is_visited(self, x: float, y: float) -> bool:
        """Check if a location has been visited."""
        cell = self._pose_to_cell(x, y)
        return cell in self.visited_cells

    def get_unexplored_directions(
        self, pose: "Pose2D", check_distance: float = 2.0
    ) -> list:
        """
        Get directions with unexplored areas.

        Checks cells in 8 directions from current position.

        Returns:
            List of direction names ("north", "northeast", etc.)
        """
        directions = [
            ("north", 0, 1),
            ("northeast", 1, 1),
            ("east", 1, 0),
            ("southeast", 1, -1),
            ("south", 0, -1),
            ("southwest", -1, -1),
            ("west", -1, 0),
            ("northwest", -1, 1),
        ]

        unexplored = []
        check_cells = int(check_distance / self.cell_size)

        for name, dx, dy in directions:
            # Check cells in this direction
            has_unexplored = False
            for i in range(1, check_cells + 1):
                check_x = pose.x + dx * i * self.cell_size
                check_y = pose.y + dy * i * self.cell_size
                if not self.is_visited(check_x, check_y):
                    has_unexplored = True
                    break

            if has_unexplored:
                unexplored.append(name)

        return unexplored

    def get_coverage_percentage(self) -> float:
        """
        Calculate percentage of bounding box that has been explored.

        Returns 0-100 float. Can decrease as bounding box expands
        when the robot discovers the environment is larger than
        initially thought.
        """
        if len(self.visited_cells) < 2:
            return 100.0  # Single cell = 100% of known area

        cells = list(self.visited_cells)
        min_x = min(c[0] for c in cells)
        max_x = max(c[0] for c in cells)
        min_y = min(c[1] for c in cells)
        max_y = max(c[1] for c in cells)

        bounding_box_cells = (max_x - min_x + 1) * (max_y - min_y + 1)
        return (len(self.visited_cells) / bounding_box_cells) * 100

    def get_new_area_confidence(self, pose: "Pose2D", search_radius: float = 2.0) -> float:
        """
        Calculate confidence that robot is in new/unexplored territory.

        Measures density of unvisited cells in a radius around current position.
        High percentage = surrounded by unexplored area (new territory).
        Low percentage = in familiar/revisited area.

        Args:
            pose: Current robot position
            search_radius: Radius in meters to check (default 2.0m)

        Returns:
            0-100 float percentage
        """
        robot_cell = self._pose_to_cell(pose.x, pose.y)
        search_cells = int(search_radius / self.cell_size)

        unvisited = 0
        total = 0

        for dx in range(-search_cells, search_cells + 1):
            for dy in range(-search_cells, search_cells + 1):
                cell = (robot_cell[0] + dx, robot_cell[1] + dy)
                total += 1
                if cell not in self.visited_cells:
                    unvisited += 1

        return (unvisited / total) * 100 if total > 0 else 0.0

    def get_explored_summary(self) -> str:
        """Get a summary of explored areas for Ollama context."""
        if not self.visited_cells:
            return "None yet"

        # Find bounds
        cells = list(self.visited_cells)
        min_x = min(c[0] for c in cells) * self.cell_size
        max_x = max(c[0] for c in cells) * self.cell_size
        min_y = min(c[1] for c in cells) * self.cell_size
        max_y = max(c[1] for c in cells) * self.cell_size

        return (
            f"{len(self.visited_cells)} cells, "
            f"area ({min_x:.1f},{min_y:.1f}) to ({max_x:.1f},{max_y:.1f})m"
        )

    def get_recent_path(self, count: int = 5) -> str:
        """Get recent path history for Ollama context."""
        if not self.path_history:
            return "Just started"

        recent = self.path_history[-count:]
        points = [f"({x:.1f},{y:.1f})" for x, y in recent]
        return " -> ".join(points)

    def get_recent_decisions(self, count: int = 3) -> str:
        """Get recent Ollama decisions for context."""
        if not self.waypoint_history:
            return "None yet"

        recent = self.waypoint_history[-count:]
        decisions = [
            f"({w.x:.1f},{w.y:.1f}): {w.reason[:30]}"
            for w in recent
        ]
        return "; ".join(decisions)

    def save(self, path: Optional[Path] = None) -> Path:
        """Save memory to JSON file."""
        if path is None:
            self._storage_dir.mkdir(parents=True, exist_ok=True)
            path = self._storage_dir / f"{self.name}.json"

        self.updated_at = datetime.now().isoformat()

        data = {
            "name": self.name,
            "visited_cells": list(self.visited_cells),
            "path_history": self.path_history,
            "waypoint_history": [w.to_dict() for w in self.waypoint_history],
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "cell_size": self.cell_size,
        }

        with open(path, "w") as f:
            json.dump(data, f, indent=2)

        return path

    @classmethod
    def load(cls, path: Path) -> "ExplorationMemory":
        """Load memory from JSON file."""
        with open(path) as f:
            data = json.load(f)

        return cls(
            name=data["name"],
            visited_cells={tuple(c) for c in data.get("visited_cells", [])},
            path_history=data.get("path_history", []),
            waypoint_history=[
                WaypointRecord.from_dict(w)
                for w in data.get("waypoint_history", [])
            ],
            created_at=data.get("created_at", datetime.now().isoformat()),
            updated_at=data.get("updated_at", datetime.now().isoformat()),
            cell_size=data.get("cell_size", 0.5),
        )

    @classmethod
    def load_or_create(
        cls,
        name: str,
        storage_dir: Optional[Path] = None,
    ) -> "ExplorationMemory":
        """Load existing memory or create new one."""
        if storage_dir is None:
            storage_dir = Path.home() / ".nimbus" / "memories"

        path = storage_dir / f"{name}.json"
        if path.exists():
            return cls.load(path)

        memory = cls(name=name)
        memory._storage_dir = storage_dir
        return memory

    @classmethod
    def list_memories(
        cls,
        storage_dir: Optional[Path] = None,
    ) -> list:
        """List all saved memory files."""
        if storage_dir is None:
            storage_dir = Path.home() / ".nimbus" / "memories"

        if not storage_dir.exists():
            return []

        memories = []
        for path in storage_dir.glob("*.json"):
            try:
                with open(path) as f:
                    data = json.load(f)
                memories.append({
                    "name": data.get("name", path.stem),
                    "cells": len(data.get("visited_cells", [])),
                    "created_at": data.get("created_at", "unknown"),
                    "updated_at": data.get("updated_at", "unknown"),
                    "path": str(path),
                })
            except Exception:
                continue

        return sorted(memories, key=lambda m: m["updated_at"], reverse=True)

    @classmethod
    def delete_memory(
        cls,
        name: str,
        storage_dir: Optional[Path] = None,
    ) -> bool:
        """Delete a saved memory file."""
        if storage_dir is None:
            storage_dir = Path.home() / ".nimbus" / "memories"

        path = storage_dir / f"{name}.json"
        if path.exists():
            path.unlink()
            return True
        return False

    def clear(self) -> None:
        """Clear all exploration data."""
        self.visited_cells.clear()
        self.path_history.clear()
        self.waypoint_history.clear()
        self.updated_at = datetime.now().isoformat()

    def to_dict(self) -> dict:
        """Export as dictionary for API responses."""
        return {
            "name": self.name,
            "cells_explored": len(self.visited_cells),
            "path_length": len(self.path_history),
            "waypoints_decided": len(self.waypoint_history),
            "created_at": self.created_at,
            "updated_at": self.updated_at,
        }
