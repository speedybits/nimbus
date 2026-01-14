"""
Unit tests for AI exploration module.

Tests the Ollama client, LIDAR describer, and exploration memory.
"""

import json
import math
import pytest
import tempfile
from pathlib import Path
from datetime import datetime
from unittest.mock import Mock, patch, MagicMock
import numpy as np

from nimbus.ai.ollama import (
    OllamaClient,
    OllamaConfig,
    AsyncOllamaClient,
    WaypointDecision,
)
from nimbus.ai.describer import LidarDescriber, DescriptionConfig
from nimbus.ai.memory import ExplorationMemory, WaypointRecord
from nimbus.core.state import SensorSnapshot, Pose2D, Velocity


# --- Fixtures ---

@pytest.fixture
def mock_sensor_snapshot():
    """Create a mock sensor snapshot."""
    return SensorSnapshot(
        timestamp=datetime.now(),
        pose=Pose2D(x=1.5, y=2.0, theta=math.pi / 4),  # 45 degrees
        velocity=Velocity(linear=0.2, angular=0.1),
        lidar_ranges=tuple([2.0] * 360),  # All clear at 2m
        closest_obstacle=2.0,
        obstacle_direction=0.0,
    )


@pytest.fixture
def mock_sensor_with_wall():
    """Create sensor snapshot with wall ahead."""
    ranges = [2.0] * 360
    # Wall directly ahead (indices 0-30 and 330-359)
    for i in list(range(0, 31)) + list(range(330, 360)):
        ranges[i] = 0.5

    return SensorSnapshot(
        timestamp=datetime.now(),
        pose=Pose2D(x=1.0, y=1.0, theta=0.0),
        velocity=Velocity(linear=0.0, angular=0.0),
        lidar_ranges=tuple(ranges),
        closest_obstacle=0.5,
        obstacle_direction=0.0,
    )


@pytest.fixture
def temp_memory_dir():
    """Create a temporary directory for memory storage."""
    with tempfile.TemporaryDirectory() as tmpdir:
        yield Path(tmpdir)


# --- OllamaClient Tests ---

class TestOllamaClient:
    """Tests for OllamaClient."""

    def test_config_defaults(self):
        """Test default configuration."""
        config = OllamaConfig()
        assert config.model == "llama3.1:8b"
        assert "localhost:11434" in config.url
        assert config.temperature == 0.3

    def test_parse_valid_json(self):
        """Test parsing valid JSON response."""
        client = OllamaClient()

        response = '{"x": 1.5, "y": -0.5, "reason": "Open space detected"}'
        result = client._parse_response(response)

        assert result.success
        assert result.x == 1.5
        assert result.y == -0.5
        assert "Open space" in result.reason

    def test_parse_json_with_extra_text(self):
        """Test parsing JSON embedded in text."""
        client = OllamaClient()

        response = 'Here is my decision: {"x": 2.0, "y": 0.0, "reason": "Clear ahead"} Thank you.'
        result = client._parse_response(response)

        assert result.success
        assert result.x == 2.0
        assert result.y == 0.0

    def test_parse_invalid_json(self):
        """Test handling invalid JSON."""
        client = OllamaClient()

        response = "I don't understand the question."
        result = client._parse_response(response)

        assert not result.success
        assert "No JSON" in result.error

    def test_parse_waypoint_too_far(self):
        """Test rejection of waypoints too far away."""
        client = OllamaClient()

        response = '{"x": 10.0, "y": 10.0, "reason": "Far away"}'
        result = client._parse_response(response)

        assert not result.success
        assert "too far" in result.error.lower()

    def test_failed_decision(self):
        """Test creating a failed decision."""
        decision = WaypointDecision.failed("Connection error")

        assert not decision.success
        assert "Connection error" in decision.error
        assert decision.x == 0.0
        assert decision.y == 0.0

    @patch('requests.get')
    def test_is_available_true(self, mock_get):
        """Test availability check when Ollama is running."""
        mock_get.return_value = Mock(status_code=200)
        client = OllamaClient()

        assert client.is_available() is True

    @patch('requests.get')
    def test_is_available_false(self, mock_get):
        """Test availability check when Ollama is not running."""
        import requests
        mock_get.side_effect = requests.RequestException()
        client = OllamaClient()

        assert client.is_available() is False

    @patch('requests.post')
    def test_decide_waypoint_success(self, mock_post):
        """Test successful waypoint decision."""
        mock_response = Mock()
        mock_response.status_code = 200
        mock_response.json.return_value = {
            "response": '{"x": 1.0, "y": 0.5, "reason": "Moving forward"}'
        }
        mock_post.return_value = mock_response

        client = OllamaClient()
        result = client.decide_waypoint("Test description")

        assert result.success
        assert result.x == 1.0
        assert result.y == 0.5

    @patch('requests.post')
    def test_decide_waypoint_timeout(self, mock_post):
        """Test handling of request timeout."""
        import requests
        mock_post.side_effect = requests.Timeout()

        client = OllamaClient()
        result = client.decide_waypoint("Test description")

        assert not result.success
        assert "timed out" in result.error.lower()


class TestAsyncOllamaClient:
    """Tests for AsyncOllamaClient."""

    def test_initial_state(self):
        """Test initial state of async client."""
        client = AsyncOllamaClient()

        assert not client.is_pending()
        assert not client.has_result()

    @patch.object(OllamaClient, 'decide_waypoint')
    def test_request_and_result(self, mock_decide):
        """Test async request and result retrieval."""
        mock_decide.return_value = WaypointDecision(x=1.0, y=0.5, reason="Test")

        client = AsyncOllamaClient()
        started = client.request_waypoint("Description")

        assert started

        # Wait for thread to complete
        import time
        time.sleep(0.1)

        assert client.has_result()
        result = client.get_result()
        assert result.x == 1.0

        # Result should be cleared
        assert not client.has_result()

    def test_duplicate_request_rejected(self):
        """Test that duplicate requests are rejected."""
        client = AsyncOllamaClient()

        # Mock to make the first request block
        with patch.object(OllamaClient, 'decide_waypoint') as mock_decide:
            import time
            mock_decide.side_effect = lambda *args, **kwargs: time.sleep(1)

            first = client.request_waypoint("First")
            second = client.request_waypoint("Second")

            assert first is True
            assert second is False  # Rejected - already pending


# --- LidarDescriber Tests ---

class TestLidarDescriber:
    """Tests for LidarDescriber."""

    def test_describe_clear_environment(self, mock_sensor_snapshot):
        """Test description of clear environment."""
        describer = LidarDescriber()
        description = describer.describe(mock_sensor_snapshot)

        assert "Position:" in description
        assert "1.5" in description
        assert "2.0" in description
        assert "northeast" in description.lower()  # 45 degrees

    def test_describe_wall_ahead(self, mock_sensor_with_wall):
        """Test description with wall ahead."""
        describer = LidarDescriber()
        description = describer.describe(mock_sensor_with_wall)

        # Should mention obstacle ahead
        assert "front" in description.lower()
        assert "0.5m" in description or "obstacle" in description.lower()

    def test_heading_to_compass(self):
        """Test heading to compass direction conversion."""
        describer = LidarDescriber()

        assert "east" == describer._heading_to_compass(0)
        assert "north" == describer._heading_to_compass(90)
        assert "west" == describer._heading_to_compass(180)
        assert "south" == describer._heading_to_compass(270)
        assert "northeast" == describer._heading_to_compass(45)

    def test_distance_to_description(self):
        """Test distance threshold descriptions."""
        describer = LidarDescriber()

        assert "BLOCKED" == describer._distance_to_description(0.2)
        assert "obstacle" == describer._distance_to_description(0.5)
        assert "clear" == describer._distance_to_description(1.5)
        assert "open" == describer._distance_to_description(3.0)

    def test_describe_with_memory(self, mock_sensor_snapshot, temp_memory_dir):
        """Test description including memory context."""
        describer = LidarDescriber(DescriptionConfig(include_memory=True))
        memory = ExplorationMemory(name="test", _storage_dir=temp_memory_dir)

        # Add some visited cells
        memory.mark_visited(Pose2D(0, 0, 0))
        memory.mark_visited(Pose2D(1, 1, 0))

        description = describer.describe(mock_sensor_snapshot, memory)

        assert "Memory:" in description
        assert "Cells explored: 2" in description


# --- ExplorationMemory Tests ---

class TestExplorationMemory:
    """Tests for ExplorationMemory."""

    def test_create_empty_memory(self, temp_memory_dir):
        """Test creating empty exploration memory."""
        memory = ExplorationMemory(name="test", _storage_dir=temp_memory_dir)

        assert memory.name == "test"
        assert len(memory.visited_cells) == 0
        assert len(memory.path_history) == 0

    def test_mark_visited(self, temp_memory_dir):
        """Test marking cells as visited."""
        memory = ExplorationMemory(name="test", _storage_dir=temp_memory_dir)
        pose = Pose2D(x=1.0, y=1.0, theta=0.0)

        memory.mark_visited(pose)

        assert len(memory.visited_cells) == 1
        assert memory.is_visited(1.0, 1.0)
        assert not memory.is_visited(5.0, 5.0)

    def test_cell_resolution(self, temp_memory_dir):
        """Test grid cell resolution (0.5m default)."""
        memory = ExplorationMemory(name="test", _storage_dir=temp_memory_dir)

        # These should be in the same cell (0.5m resolution)
        memory.mark_visited(Pose2D(x=0.1, y=0.1, theta=0))
        memory.mark_visited(Pose2D(x=0.4, y=0.4, theta=0))

        assert len(memory.visited_cells) == 1

        # This should be a different cell
        memory.mark_visited(Pose2D(x=0.6, y=0.6, theta=0))

        assert len(memory.visited_cells) == 2

    def test_path_history(self, temp_memory_dir):
        """Test path history tracking."""
        memory = ExplorationMemory(name="test", _storage_dir=temp_memory_dir)

        for i in range(5):
            memory.mark_visited(Pose2D(x=float(i), y=0.0, theta=0.0))

        assert len(memory.path_history) == 5
        assert memory.path_history[0] == (0.0, 0.0)
        assert memory.path_history[-1] == (4.0, 0.0)

    def test_path_history_limit(self, temp_memory_dir):
        """Test path history is limited."""
        memory = ExplorationMemory(name="test", _storage_dir=temp_memory_dir)
        memory.max_path_history = 10

        for i in range(20):
            memory.mark_visited(Pose2D(x=float(i), y=0.0, theta=0.0))

        assert len(memory.path_history) == 10

    def test_add_waypoint(self, temp_memory_dir):
        """Test adding waypoint decisions."""
        memory = ExplorationMemory(name="test", _storage_dir=temp_memory_dir)

        memory.add_waypoint(2.0, 1.5, "Open space ahead")

        assert len(memory.waypoint_history) == 1
        assert memory.waypoint_history[0].x == 2.0
        assert memory.waypoint_history[0].y == 1.5
        assert "Open space" in memory.waypoint_history[0].reason

    def test_get_unexplored_directions(self, temp_memory_dir):
        """Test getting unexplored directions."""
        memory = ExplorationMemory(name="test", _storage_dir=temp_memory_dir)
        pose = Pose2D(x=0.0, y=0.0, theta=0.0)

        # All directions unexplored initially
        unexplored = memory.get_unexplored_directions(pose)
        assert len(unexplored) == 8  # All 8 directions

        # Mark ALL cells in the east direction as visited (check_distance=2.0m, cell_size=0.5m = 4 cells)
        # Need to visit all cells to mark direction as explored
        memory.mark_visited(Pose2D(x=0.5, y=0.0, theta=0.0))  # East cell 1
        memory.mark_visited(Pose2D(x=1.0, y=0.0, theta=0.0))  # East cell 2
        memory.mark_visited(Pose2D(x=1.5, y=0.0, theta=0.0))  # East cell 3
        memory.mark_visited(Pose2D(x=2.0, y=0.0, theta=0.0))  # East cell 4

        unexplored = memory.get_unexplored_directions(pose)
        # East should no longer be unexplored (all cells visited)
        assert "east" not in unexplored
        # Other directions should still be unexplored
        assert "north" in unexplored
        assert "west" in unexplored

    def test_save_and_load(self, temp_memory_dir):
        """Test saving and loading memory."""
        memory = ExplorationMemory(name="test_save", _storage_dir=temp_memory_dir)
        memory.mark_visited(Pose2D(x=1.0, y=2.0, theta=0.0))
        memory.add_waypoint(3.0, 4.0, "Test waypoint")

        path = memory.save()

        # Load and verify
        loaded = ExplorationMemory.load(path)

        assert loaded.name == "test_save"
        assert len(loaded.visited_cells) == 1
        assert loaded.is_visited(1.0, 2.0)
        assert len(loaded.waypoint_history) == 1
        assert loaded.waypoint_history[0].reason == "Test waypoint"

    def test_load_or_create_new(self, temp_memory_dir):
        """Test load_or_create for new memory."""
        memory = ExplorationMemory.load_or_create("new_mem", storage_dir=temp_memory_dir)

        assert memory.name == "new_mem"
        assert len(memory.visited_cells) == 0

    def test_load_or_create_existing(self, temp_memory_dir):
        """Test load_or_create for existing memory."""
        # Create and save
        original = ExplorationMemory(name="existing", _storage_dir=temp_memory_dir)
        original.mark_visited(Pose2D(x=5.0, y=5.0, theta=0.0))
        original.save(temp_memory_dir / "existing.json")

        # Load
        loaded = ExplorationMemory.load_or_create("existing", storage_dir=temp_memory_dir)

        assert loaded.name == "existing"
        assert len(loaded.visited_cells) == 1

    def test_list_memories(self, temp_memory_dir):
        """Test listing saved memories."""
        # Create some memories
        for name in ["mem1", "mem2", "mem3"]:
            mem = ExplorationMemory(name=name, _storage_dir=temp_memory_dir)
            mem.save(temp_memory_dir / f"{name}.json")

        memories = ExplorationMemory.list_memories(storage_dir=temp_memory_dir)

        assert len(memories) == 3
        names = [m["name"] for m in memories]
        assert "mem1" in names
        assert "mem2" in names
        assert "mem3" in names

    def test_delete_memory(self, temp_memory_dir):
        """Test deleting a memory file."""
        mem = ExplorationMemory(name="to_delete", _storage_dir=temp_memory_dir)
        mem.save(temp_memory_dir / "to_delete.json")

        assert (temp_memory_dir / "to_delete.json").exists()

        result = ExplorationMemory.delete_memory("to_delete", storage_dir=temp_memory_dir)

        assert result is True
        assert not (temp_memory_dir / "to_delete.json").exists()

    def test_delete_nonexistent(self, temp_memory_dir):
        """Test deleting non-existent memory."""
        result = ExplorationMemory.delete_memory("nonexistent", storage_dir=temp_memory_dir)
        assert result is False

    def test_clear(self, temp_memory_dir):
        """Test clearing memory."""
        memory = ExplorationMemory(name="test", _storage_dir=temp_memory_dir)
        memory.mark_visited(Pose2D(x=1.0, y=1.0, theta=0.0))
        memory.add_waypoint(2.0, 2.0, "Test")

        memory.clear()

        assert len(memory.visited_cells) == 0
        assert len(memory.path_history) == 0
        assert len(memory.waypoint_history) == 0

    def test_get_explored_summary(self, temp_memory_dir):
        """Test explored summary generation."""
        memory = ExplorationMemory(name="test", _storage_dir=temp_memory_dir)

        # Empty
        assert "None yet" in memory.get_explored_summary()

        # With cells
        memory.mark_visited(Pose2D(x=0.0, y=0.0, theta=0.0))
        memory.mark_visited(Pose2D(x=1.0, y=1.0, theta=0.0))

        summary = memory.get_explored_summary()
        assert "2 cells" in summary

    def test_get_recent_path(self, temp_memory_dir):
        """Test recent path formatting."""
        memory = ExplorationMemory(name="test", _storage_dir=temp_memory_dir)

        # Empty
        assert "Just started" in memory.get_recent_path()

        # With points
        memory.mark_visited(Pose2D(x=0.0, y=0.0, theta=0.0))
        memory.mark_visited(Pose2D(x=1.0, y=0.0, theta=0.0))

        path = memory.get_recent_path(count=5)
        assert "(0.0,0.0)" in path
        assert "(1.0,0.0)" in path

    def test_to_dict(self, temp_memory_dir):
        """Test dictionary export."""
        memory = ExplorationMemory(name="test", _storage_dir=temp_memory_dir)
        memory.mark_visited(Pose2D(x=1.0, y=1.0, theta=0.0))
        memory.add_waypoint(2.0, 2.0, "Test")

        data = memory.to_dict()

        assert data["name"] == "test"
        assert data["cells_explored"] == 1
        assert data["waypoints_decided"] == 1


class TestWaypointRecord:
    """Tests for WaypointRecord."""

    def test_to_dict(self):
        """Test converting record to dict."""
        record = WaypointRecord(
            x=1.0,
            y=2.0,
            reason="Test reason",
            timestamp="2024-01-01T00:00:00",
            reached=True,
        )

        data = record.to_dict()

        assert data["x"] == 1.0
        assert data["y"] == 2.0
        assert data["reason"] == "Test reason"
        assert data["reached"] is True

    def test_from_dict(self):
        """Test creating record from dict."""
        data = {
            "x": 3.0,
            "y": 4.0,
            "reason": "Another reason",
            "timestamp": "2024-01-01T12:00:00",
            "reached": False,
        }

        record = WaypointRecord.from_dict(data)

        assert record.x == 3.0
        assert record.y == 4.0
        assert record.reason == "Another reason"
        assert record.reached is False
