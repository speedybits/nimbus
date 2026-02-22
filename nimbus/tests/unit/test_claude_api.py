"""
Unit tests for Claude high-level API endpoints: scan, goto, map.
"""

import math
import numpy as np
import pytest
from datetime import datetime
from unittest.mock import MagicMock, patch, PropertyMock

from nimbus.api.server import (
    create_app,
    safe_float,
    _math_to_compass,
    _heading_arrow,
    _render_map_plain,
)
from nimbus.api.schemas import ClaudeScanOpening
from nimbus.core.state import RobotContext, Pose2D, Velocity, SensorSnapshot
from nimbus.sensors.mapping import ObstacleMap, MapConfig


# --- Helper function tests ---


class TestSafeFloat:
    """Tests for safe_float JSON sanitizer."""

    def test_normal_float(self):
        assert safe_float(1.5) == 1.5

    def test_zero(self):
        assert safe_float(0.0) == 0.0

    def test_inf(self):
        assert safe_float(float('inf')) is None

    def test_neg_inf(self):
        assert safe_float(float('-inf')) is None

    def test_nan(self):
        assert safe_float(float('nan')) is None

    def test_none(self):
        assert safe_float(None) is None


class TestMathToCompass:
    """Tests for math angle to compass bearing conversion."""

    def test_east_is_90(self):
        """Math 0 (east) → compass 90°."""
        assert _math_to_compass(0) == 90

    def test_north_is_0(self):
        """Math pi/2 (north) → compass 0°."""
        assert _math_to_compass(math.pi / 2) == pytest.approx(0, abs=0.1)

    def test_west_is_270(self):
        """Math pi (west) → compass 270°."""
        assert _math_to_compass(math.pi) == pytest.approx(270, abs=0.1)

    def test_south_is_180(self):
        """Math -pi/2 (south) → compass 180°."""
        assert _math_to_compass(-math.pi / 2) == pytest.approx(180, abs=0.1)

    def test_always_positive(self):
        """Result should always be in [0, 360)."""
        for angle in [0, 0.5, -0.5, math.pi, -math.pi, 3.0, -3.0]:
            result = _math_to_compass(angle)
            assert 0 <= result < 360, f"angle={angle} gave {result}"


class TestHeadingArrow:
    """Tests for heading arrow character selection."""

    def test_east(self):
        assert _heading_arrow(0) == '→'

    def test_north(self):
        assert _heading_arrow(math.pi / 2) == '↑'

    def test_west(self):
        assert _heading_arrow(math.pi) == '←'

    def test_south(self):
        assert _heading_arrow(-math.pi / 2) == '↓'


class TestRenderMapPlain:
    """Tests for plain-text map renderer."""

    def test_empty_map(self):
        """Empty map should still render without error."""
        obstacle_map = ObstacleMap()
        result = _render_map_plain(obstacle_map, None, width=20, height=10)
        assert isinstance(result, str)
        lines = result.split('\n')
        assert len(lines) == 10

    def test_with_robot_pose(self):
        """Map should contain robot arrow when pose given."""
        obstacle_map = ObstacleMap()
        # Add a visited cell so bounds are non-trivial
        obstacle_map._visited.add((0, 0))
        pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        result = _render_map_plain(obstacle_map, pose, width=20, height=10)
        assert '→' in result  # Robot arrow for theta=0 (east)

    def test_with_obstacles(self):
        """Map should render obstacle characters."""
        obstacle_map = ObstacleMap()
        obstacle_map._obstacles[(0, 0)] = 0.8  # High confidence
        obstacle_map._obstacles[(1, 0)] = 0.3  # Low confidence
        obstacle_map._visited.add((0, 0))
        result = _render_map_plain(obstacle_map, None, width=30, height=15)
        assert '#' in result  # High confidence obstacle
        assert '+' in result  # Low confidence obstacle

    def test_compass_rose(self):
        """Map should include compass rose."""
        obstacle_map = ObstacleMap()
        obstacle_map._visited.add((0, 0))
        result = _render_map_plain(obstacle_map, None, width=30, height=15)
        assert 'N' in result
        assert 'S' in result
        assert 'E' in result
        assert 'W' in result

    def test_dimensions(self):
        """Map should respect requested dimensions."""
        obstacle_map = ObstacleMap()
        result = _render_map_plain(obstacle_map, None, width=40, height=20)
        lines = result.split('\n')
        assert len(lines) == 20
        for line in lines:
            assert len(line) == 40


# --- API endpoint tests using FastAPI TestClient ---


@pytest.fixture
def mock_runner():
    """Create a mock runner with sensor data."""
    runner = MagicMock()
    ctx = RobotContext()
    snapshot = SensorSnapshot(
        timestamp=datetime.now(),
        pose=Pose2D(x=1.0, y=-0.5, theta=math.pi / 4),  # 45° (NE)
        velocity=Velocity(linear=0.0, angular=0.0),
        lidar_ranges=tuple([2.0] * 360),
        closest_obstacle=2.0,
        obstacle_direction=0.0,
    )
    ctx.update_sensors(snapshot)
    runner.context = ctx

    # Mock obstacle map
    obstacle_map = ObstacleMap()
    obstacle_map._visited.add((0, 0))
    obstacle_map._visited.add((1, 0))
    obstacle_map._obstacles[(5, 0)] = 0.8
    obstacle_map._robot_trail = [(0.0, 0.0), (0.15, 0.0), (0.3, 0.0)]
    obstacle_map._trail_active = True
    runner.obstacle_map = obstacle_map

    return runner


@pytest.fixture
def client(mock_runner):
    """Create a test client with mocked runner."""
    import nimbus.api.server as server_module
    from fastapi.testclient import TestClient

    original_runner = server_module._runner
    server_module._runner = mock_runner
    app = create_app()
    client = TestClient(app)
    yield client
    server_module._runner = original_runner


class TestClaudeScanEndpoint:
    """Tests for GET /api/claude/scan."""

    def test_scan_returns_200(self, client):
        response = client.get("/api/claude/scan")
        assert response.status_code == 200

    def test_scan_has_pose(self, client):
        data = client.get("/api/claude/scan").json()
        assert "pose" in data
        assert "x" in data["pose"]
        assert "y" in data["pose"]
        assert "heading" in data["pose"]

    def test_scan_heading_is_compass(self, client):
        """Heading should be compass degrees, not radians."""
        data = client.get("/api/claude/scan").json()
        heading = data["pose"]["heading"]
        # Robot is at pi/4 (NE in math), which is 45° compass
        assert heading == pytest.approx(45, abs=1)

    def test_scan_has_walls(self, client):
        """Should have 8 cardinal/intercardinal wall readings."""
        data = client.get("/api/claude/scan").json()
        assert "walls" in data
        expected_keys = {"front", "front_left", "left", "rear_left",
                         "rear", "rear_right", "right", "front_right"}
        assert set(data["walls"].keys()) == expected_keys

    def test_scan_has_openings(self, client):
        """Should detect openings (all clear = one big opening)."""
        data = client.get("/api/claude/scan").json()
        assert "openings" in data
        assert isinstance(data["openings"], list)

    def test_scan_has_timestamp(self, client):
        data = client.get("/api/claude/scan").json()
        assert "timestamp" in data

    def test_scan_503_without_runner(self):
        """Should return 503 when runner not initialized."""
        import nimbus.api.server as server_module
        from fastapi.testclient import TestClient

        original = server_module._runner
        server_module._runner = None
        app = create_app()
        c = TestClient(app)
        response = c.get("/api/claude/scan")
        assert response.status_code == 503
        server_module._runner = original

    def test_scan_with_obstacles(self):
        """Scan with obstacles should detect closest and narrow openings."""
        import nimbus.api.server as server_module
        from fastapi.testclient import TestClient

        runner = MagicMock()
        ctx = RobotContext()

        # Wall ahead at 0.5m, clear sides
        ranges = list([3.0] * 360)
        for i in list(range(0, 30)) + list(range(330, 360)):
            ranges[i] = 0.5

        snapshot = SensorSnapshot(
            timestamp=datetime.now(),
            pose=Pose2D(x=0.0, y=0.0, theta=0.0),
            velocity=Velocity(linear=0.0, angular=0.0),
            lidar_ranges=tuple(ranges),
            closest_obstacle=0.5,
            obstacle_direction=0.0,
        )
        ctx.update_sensors(snapshot)
        runner.context = ctx
        runner.obstacle_map = ObstacleMap()

        original = server_module._runner
        server_module._runner = runner
        app = create_app()
        c = TestClient(app)

        data = c.get("/api/claude/scan").json()

        # Should detect a close obstacle
        assert data["closest_obstacle"] is not None
        assert data["closest_obstacle"]["distance"] < 1.0

        # Front wall should be close
        assert data["walls"]["front"] is not None
        assert data["walls"]["front"] < 1.0

        server_module._runner = original


class TestClaudeMapEndpoint:
    """Tests for GET /api/claude/map."""

    def test_map_returns_200(self, client):
        response = client.get("/api/claude/map")
        assert response.status_code == 200

    def test_map_has_ascii(self, client):
        data = client.get("/api/claude/map").json()
        assert "ascii_map" in data
        assert isinstance(data["ascii_map"], str)
        assert len(data["ascii_map"]) > 0

    def test_map_has_bounds(self, client):
        data = client.get("/api/claude/map").json()
        assert "bounds" in data
        for key in ["min_x", "min_y", "max_x", "max_y"]:
            assert key in data["bounds"]

    def test_map_has_stats(self, client):
        data = client.get("/api/claude/map").json()
        assert "stats" in data
        assert "num_obstacles" in data["stats"]
        assert "num_visited" in data["stats"]
        assert "cell_size_m" in data["stats"]

    def test_map_has_trail(self, client):
        data = client.get("/api/claude/map").json()
        assert "robot_trail" in data
        assert isinstance(data["robot_trail"], list)
        # We set 3 trail points in the fixture
        assert len(data["robot_trail"]) == 3
        assert "x" in data["robot_trail"][0]
        assert "y" in data["robot_trail"][0]

    def test_map_has_pose(self, client):
        data = client.get("/api/claude/map").json()
        assert "pose" in data
        assert "heading" in data["pose"]

    def test_map_has_legend(self, client):
        data = client.get("/api/claude/map").json()
        assert "legend" in data

    def test_map_503_without_runner(self):
        import nimbus.api.server as server_module
        from fastapi.testclient import TestClient

        original = server_module._runner
        server_module._runner = None
        app = create_app()
        c = TestClient(app)
        assert c.get("/api/claude/map").status_code == 503
        server_module._runner = original


class TestClaudeGotoEndpoint:
    """Tests for POST /api/claude/goto."""

    def test_goto_503_without_runner(self):
        import nimbus.api.server as server_module
        from fastapi.testclient import TestClient

        original = server_module._runner
        server_module._runner = None
        app = create_app()
        c = TestClient(app)
        response = c.post("/api/claude/goto?x=1.0&y=1.0")
        assert response.status_code == 503
        server_module._runner = original

    def test_goto_timeout(self):
        """Goto with very short timeout should return timeout result."""
        import nimbus.api.server as server_module
        from fastapi.testclient import TestClient
        from nimbus.behaviors.goto import GoToBehavior

        runner = MagicMock()
        ctx = RobotContext()
        snapshot = SensorSnapshot(
            timestamp=datetime.now(),
            pose=Pose2D(x=0.0, y=0.0, theta=0.0),
            velocity=Velocity(linear=0.0, angular=0.0),
            lidar_ranges=tuple([2.0] * 360),
            closest_obstacle=2.0,
            obstacle_direction=0.0,
        )
        ctx.update_sensors(snapshot)
        runner.context = ctx

        goto = GoToBehavior()
        runner._behavior_manager.get_behavior.return_value = goto
        runner.set_behavior.return_value = True

        original = server_module._runner
        server_module._runner = runner
        app = create_app()
        c = TestClient(app)

        # Very short timeout — will timeout immediately
        response = c.post("/api/claude/goto?x=5.0&y=5.0&timeout=0.3")
        assert response.status_code == 200
        data = response.json()
        assert data["success"] is False
        assert data["result"] == "timeout"
        assert data["target"] == {"x": 5.0, "y": 5.0}
        assert "final_pose" in data
        assert "distance_remaining" in data
        assert data["distance_remaining"] > 0

        server_module._runner = original

    def test_goto_response_has_compass_heading(self):
        """Final pose heading should be compass degrees."""
        import nimbus.api.server as server_module
        from fastapi.testclient import TestClient
        from nimbus.behaviors.goto import GoToBehavior

        runner = MagicMock()
        ctx = RobotContext()
        snapshot = SensorSnapshot(
            timestamp=datetime.now(),
            pose=Pose2D(x=0.0, y=0.0, theta=math.pi / 2),  # Facing north
            velocity=Velocity(linear=0.0, angular=0.0),
            lidar_ranges=tuple([2.0] * 360),
            closest_obstacle=2.0,
            obstacle_direction=0.0,
        )
        ctx.update_sensors(snapshot)
        runner.context = ctx

        goto = GoToBehavior()
        runner._behavior_manager.get_behavior.return_value = goto
        runner.set_behavior.return_value = True

        original = server_module._runner
        server_module._runner = runner
        app = create_app()
        c = TestClient(app)

        response = c.post("/api/claude/goto?x=1.0&y=0.0&timeout=0.3")
        data = response.json()

        # pi/2 in math = 0° compass (north)
        assert data["final_pose"]["heading"] == pytest.approx(0, abs=1)

        server_module._runner = original
