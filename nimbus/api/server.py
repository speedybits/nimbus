"""
REST API server for Nimbus.

Provides HTTP endpoints for external control and monitoring.
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from typing import Optional
import asyncio
import threading
import time
import uvicorn

from .schemas import (
    StatusResponse,
    NavigateRequest,
    BehaviorResponse,
    HealthResponse,
    SensorResponse,
    ClaudeCommandResponse,
    ClaudeStatusResponse,
    ClaudeScanOpening,
    ClaudeScanResponse,
    ClaudeGotoResponse,
    ClaudeMapResponse,
)

import math

# Global reference to runner (set by start_api_background)
_runner = None
_api_thread = None


def safe_float(v: float) -> float | None:
    """Sanitize float for JSON (inf/nan -> None)."""
    if v is None or math.isinf(v) or math.isnan(v):
        return None
    return v


def _math_to_compass(theta_rad: float) -> float:
    """Convert math-convention angle (0=east, CCW+) to compass bearing (0=N, CW+)."""
    return (90 - math.degrees(theta_rad)) % 360


def _heading_arrow(theta_rad: float) -> str:
    """Map heading angle to directional arrow character."""
    arrows = ['→', '↗', '↑', '↖', '←', '↙', '↓', '↘']
    index = round(theta_rad / (math.pi / 4)) % 8
    return arrows[index]


def _render_map_plain(obstacle_map, robot_pose, width: int = 60, height: int = 25) -> str:
    """
    Render obstacle map as plain ASCII text (no Rich formatting).

    Adapted from dashboard._render_map_ascii() for API consumption.
    """
    canvas = [[' '] * width for _ in range(height)]

    # Get map bounds with padding
    min_x, min_y, max_x, max_y = obstacle_map.get_bounds()

    padding = 0.5
    min_x -= padding
    min_y -= padding
    max_x += padding
    max_y += padding

    world_width = max(max_x - min_x, 2.0)
    world_height = max(max_y - min_y, 2.0)

    # Scale factors (divide by 2 for width to account for character aspect ratio)
    scale_x = (width - 2) / world_width / 2.0
    scale_y = (height - 2) / world_height
    scale = min(scale_x, scale_y)

    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2

    def world_to_canvas(wx: float, wy: float) -> tuple[int, int]:
        cx = int((wx - center_x) * scale * 2 + width / 2)
        cy = int(height / 2 - (wy - center_y) * scale)  # Y inverted
        return (cx, cy)

    # Draw obstacles
    for cell, confidence in obstacle_map.get_obstacles().items():
        cell_x, cell_y = cell
        wx = (cell_x + 0.5) * obstacle_map.config.cell_size
        wy = (cell_y + 0.5) * obstacle_map.config.cell_size
        cx, cy = world_to_canvas(wx, wy)
        if 0 <= cx < width and 0 <= cy < height:
            canvas[cy][cx] = '#' if confidence >= 0.6 else '+'

    # Draw robot trail with interpolation
    trail = obstacle_map.get_robot_trail()
    trail_canvas = [world_to_canvas(wx, wy) for wx, wy in trail]
    for i in range(len(trail_canvas) - 1):
        x0, y0 = trail_canvas[i]
        x1, y1 = trail_canvas[i + 1]
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        steps = max(dx, dy, 1)
        for s in range(steps + 1):
            t = s / steps
            cx = int(x0 + t * (x1 - x0) + 0.5)
            cy = int(y0 + t * (y1 - y0) + 0.5)
            if 0 <= cx < width and 0 <= cy < height:
                if canvas[cy][cx] == ' ':
                    canvas[cy][cx] = '~'

    # Draw robot position
    robot_arrow = '→'
    if robot_pose:
        robot_arrow = _heading_arrow(robot_pose.theta)
        cx, cy = world_to_canvas(robot_pose.x, robot_pose.y)
        if 0 <= cx < width and 0 <= cy < height:
            canvas[cy][cx] = robot_arrow

    # Draw compass rose in top-right corner
    compass_x = width - 5
    compass_chars = [
        (compass_x + 2, 0, 'N'),
        (compass_x, 1, 'W'),
        (compass_x + 2, 1, robot_arrow),
        (compass_x + 4, 1, 'E'),
        (compass_x + 2, 2, 'S'),
    ]
    for cx_c, cy_c, char in compass_chars:
        if 0 <= cx_c < width and 0 <= cy_c < height:
            canvas[cy_c][cx_c] = char

    return '\n'.join(''.join(row) for row in canvas)


def create_app() -> FastAPI:
    """Create FastAPI application."""

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        # Startup
        yield
        # Shutdown

    app = FastAPI(
        title="Nimbus Robot API",
        description="Control and monitor your Nimbus robot",
        version="1.0.0",
        lifespan=lifespan,
    )

    # CORS for web dashboard
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # --- Routes ---

    @app.get("/api/health", response_model=HealthResponse)
    async def health():
        """Health check endpoint."""
        return HealthResponse(
            status="ok",
            running=_runner.is_running if _runner else False,
            version="1.0.0"
        )

    @app.get("/api/status", response_model=StatusResponse)
    async def get_status():
        """Get current robot status."""
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        ctx = _runner.context
        sensors = ctx.sensors

        # Sanitize obstacle distance for JSON (inf -> None)
        obstacle = sensors.closest_obstacle if sensors else None
        if obstacle is not None and (math.isinf(obstacle) or math.isnan(obstacle)):
            obstacle = None

        return StatusResponse(
            state=ctx.state.name,
            pose={
                "x": sensors.pose.x if sensors else 0,
                "y": sensors.pose.y if sensors else 0,
                "theta": sensors.pose.theta if sensors else 0,
            },
            velocity={
                "linear": sensors.velocity.linear if sensors else 0,
                "angular": sensors.velocity.angular if sensors else 0,
            },
            closest_obstacle=obstacle,
            current_behavior=ctx.current_behavior,
            target={
                "x": ctx.target.x,
                "y": ctx.target.y,
                "theta": ctx.target.theta,
            } if ctx.target else None,
        )

    @app.get("/api/sensors", response_model=SensorResponse)
    async def get_sensors():
        """Get raw sensor readings."""
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        ctx = _runner.context
        sensors = ctx.sensors

        if not sensors:
            raise HTTPException(503, detail="No sensor data available")

        # Odom buffer diagnostics
        odom_age = None
        odom_stale = None
        if hasattr(_runner, '_odom_buffer') and _runner._odom_buffer:
            odom_age = safe_float(_runner._odom_buffer.age)
            odom_stale = _runner._odom_buffer.is_stale

        return SensorResponse(
            timestamp=sensors.timestamp.isoformat(),
            pose={
                "x": sensors.pose.x,
                "y": sensors.pose.y,
                "theta": sensors.pose.theta,
            },
            velocity={
                "linear": sensors.velocity.linear,
                "angular": sensors.velocity.angular,
            },
            closest_obstacle=safe_float(sensors.closest_obstacle),
            obstacle_direction=sensors.obstacle_direction,
            lidar_histogram=[safe_float(r) for r in sensors.lidar_ranges[:72]],
            odom_age=odom_age,
            odom_stale=odom_stale,
        )

    @app.post("/api/navigate")
    async def navigate(request: NavigateRequest):
        """Navigate to specified coordinates."""
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        success = _runner.goto(request.x, request.y, request.theta)

        if not success:
            raise HTTPException(400, detail="Failed to start navigation")

        return {
            "status": "navigating",
            "target": {"x": request.x, "y": request.y, "theta": request.theta}
        }

    @app.post("/api/stop")
    async def emergency_stop():
        """Emergency stop - halt all motion."""
        if _runner:
            _runner.emergency_stop()
        return {"status": "stopped"}

    @app.get("/api/behaviors", response_model=BehaviorResponse)
    async def list_behaviors():
        """List available behaviors."""
        if not _runner:
            return BehaviorResponse(behaviors=[], current=None)

        return BehaviorResponse(
            behaviors=_runner.available_behaviors,
            current=_runner.current_behavior
        )

    @app.post("/api/behavior/{name}")
    async def set_behavior(name: str):
        """Set active behavior."""
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        if name not in _runner.available_behaviors:
            raise HTTPException(404, detail=f"Unknown behavior: {name}")

        success = _runner.set_behavior(name)
        if not success:
            raise HTTPException(400, detail=f"Failed to activate behavior: {name}")

        return {"status": "ok", "behavior": name}

    @app.get("/api/safety")
    async def get_safety():
        """Get safety controller status."""
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        return _runner.safety_status

    # --- Motor Test Endpoints ---

    @app.post("/api/motor_test/velocity")
    async def set_motor_test_velocity(linear: float = 0.0, angular: float = 0.0):
        """Set motor test velocity (only works in motor_test mode)."""
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        if _runner.current_behavior != "motor_test":
            raise HTTPException(400, detail="Not in motor_test mode. Set behavior to motor_test first.")

        from nimbus.behaviors.motor_test import MotorTestBehavior
        motor_test = _runner._behavior_manager.get_behavior("motor_test")
        if not motor_test or not isinstance(motor_test, MotorTestBehavior):
            raise HTTPException(500, detail="Motor test behavior not found")

        motor_test.set_velocity(linear, angular)
        return {
            "status": "ok",
            "linear": motor_test.linear,
            "angular": motor_test.angular
        }

    @app.get("/api/motor_test/velocity")
    async def get_motor_test_velocity():
        """Get current motor test velocity setting."""
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        from nimbus.behaviors.motor_test import MotorTestBehavior
        motor_test = _runner._behavior_manager.get_behavior("motor_test")
        if not motor_test or not isinstance(motor_test, MotorTestBehavior):
            raise HTTPException(500, detail="Motor test behavior not found")

        lin, ang = motor_test.get_velocity()
        return {
            "linear": lin,
            "angular": ang,
            "active": _runner.current_behavior == "motor_test"
        }

    @app.get("/api/debug/xrce")
    async def get_xrce_debug():
        """Get XRCE agent debug info."""
        if not _runner or not _runner._node:
            raise HTTPException(503, detail="Nimbus not initialized")

        node = _runner._node
        if not hasattr(node, '_agent') or node._agent is None:
            return {"error": "Mock mode - no XRCE agent"}

        agent = node._agent
        entities = agent._entities

        # Publisher stats from node
        pub_stats = {}
        for topic, pub in node._publishers.items():
            pub_stats[topic] = len(pub.published_messages)

        # Agent-level publisher stats
        agent_pub_stats = {}
        for topic, info in agent._publishers.items():
            agent_pub_stats[topic] = info.message_count

        # Velocity command from context
        vel_cmd = None
        try:
            vc = _runner.context.velocity_cmd
            vel_cmd = {"linear": vc.linear, "angular": vc.angular}
        except Exception:
            pass

        # Session details
        session_info = {
            "client_key": agent._session.client_key.hex() if agent._session.client_key else None,
            "session_connected": agent._session.is_connected,
        }

        # READ_DATA requests from ESP32 (shows actual object IDs)
        pending_reads = {str(k): f"{v} (0x{v:04x})" for k, v in agent._pending_read_requests.items()}

        # Full entity diagnostics
        entity_diag = entities.get_diagnostic_info()

        # Also show what publish() would use for cmd_vel
        cmd_vel_dr = entities.get_datareader_for_topic("/cmd_vel")

        # Find unmapped pending reads (not associated with known datareaders)
        known_read_ids = set()
        for known_dr_id in [dr.object_id for dr in entities._datareaders.values()]:
            known_read_ids.add(known_dr_id)
            known_read_ids.add(known_dr_id + 256)
        unmapped_reads = {str(k): v for k, v in agent._pending_read_requests.items()
                         if k not in known_read_ids}

        publish_info = {
            "dr_id_from_entities": cmd_vel_dr,
            "using_fallback": cmd_vel_dr is None,
            "unmapped_read_data": unmapped_reads,
            "will_use": unmapped_reads if cmd_vel_dr is None and unmapped_reads else None,
        }

        return {
            "connected": agent.is_connected,
            "client_addr": agent._client_addr,
            "session": session_info,
            "entities": entity_diag,
            "node_publishers": pub_stats,
            "agent_publishers": agent_pub_stats,
            "velocity_cmd": vel_cmd,
            "pending_read_requests": pending_reads,
            "cmd_vel_publish": publish_info,
        }

    # --- Exploration Endpoints ---

    @app.post("/api/explore/start")
    async def start_explore(memory: str = "default"):
        """Start AI exploration with Ollama."""
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        success = _runner.explore(memory)
        if not success:
            raise HTTPException(400, detail="Failed to start exploration")

        return {"status": "exploring", "memory": memory}

    @app.post("/api/explore/stop")
    async def stop_explore():
        """Stop AI exploration."""
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        # Save memory before stopping
        from nimbus.behaviors.ai_explore import AIExploreBehavior
        ai_behavior = _runner._behavior_manager.get_behavior("ai_explore")
        if ai_behavior and isinstance(ai_behavior, AIExploreBehavior):
            ai_behavior.memory.save()

        _runner.set_behavior("idle")
        return {"status": "stopped"}

    @app.get("/api/explore/status")
    async def get_explore_status():
        """Get AI exploration status."""
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        status = _runner.get_exploration_status()
        if status is None:
            return {"active": False}

        return status

    @app.get("/api/explore/memories")
    async def list_explore_memories():
        """List saved exploration memories."""
        from nimbus.ai.memory import ExplorationMemory
        memories = ExplorationMemory.list_memories()
        return {"memories": [m["name"] for m in memories]}

    @app.post("/api/explore/memory/save")
    async def save_explore_memory(name: Optional[str] = None):
        """Save current exploration memory."""
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        from nimbus.behaviors.ai_explore import AIExploreBehavior
        ai_behavior = _runner._behavior_manager.get_behavior("ai_explore")
        if not ai_behavior or not isinstance(ai_behavior, AIExploreBehavior):
            raise HTTPException(400, detail="AI exploration not active")

        if name:
            # Rename and save
            ai_behavior.memory.name = name

        path = ai_behavior.memory.save()
        return {
            "status": "saved",
            "name": ai_behavior.memory.name,
            "cells": len(ai_behavior.memory.visited_cells),
            "path": str(path),
        }

    @app.delete("/api/explore/memory/{name}")
    async def delete_explore_memory(name: str):
        """Delete a saved exploration memory."""
        from nimbus.ai.memory import ExplorationMemory

        if ExplorationMemory.delete_memory(name):
            return {"status": "deleted", "name": name}
        else:
            raise HTTPException(404, detail=f"Memory not found: {name}")

    # --- Claude Control Endpoints ---

    @app.post("/api/claude/move", response_model=ClaudeCommandResponse)
    async def claude_move(
        distance: float,
        speed: float = 0.15,
        timeout: float = 30.0
    ):
        """
        Move the robot forward/backward by a specified distance.

        Args:
            distance: Distance in meters (positive=forward, negative=backward)
            speed: Speed in m/s (default 0.15)
            timeout: Timeout in seconds (default 30)

        Returns:
            ClaudeCommandResponse with result and actual distance traveled.
        """
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        from nimbus.behaviors.claude_control import (
            ClaudeControlBehavior,
            create_move_command,
        )

        # Get or activate claude_control behavior
        behavior = _runner._behavior_manager.get_behavior("claude_control")
        if not behavior or not isinstance(behavior, ClaudeControlBehavior):
            raise HTTPException(500, detail="Claude control behavior not found")

        # Auto-activate if not active
        if _runner.current_behavior != "claude_control":
            _runner.set_behavior("claude_control")

        # Create and set command
        command = create_move_command(distance, speed, timeout)
        behavior.set_command(command)

        # Wait for completion in executor to not block event loop
        loop = asyncio.get_event_loop()
        completed = await loop.run_in_executor(
            None,
            lambda: behavior.wait_for_completion(timeout=timeout + 1.0)
        )

        result = behavior.get_result()
        if not result:
            raise HTTPException(500, detail="No result from command")

        duration = time.time() - result.start_time if result.start_time else 0

        return ClaudeCommandResponse(
            success=result.result in ("completed", "completed_open_loop"),
            result=result.result or "unknown",
            target=result.target,
            actual=result.actual or 0.0,
            duration=duration,
        )

    @app.post("/api/claude/turn", response_model=ClaudeCommandResponse)
    async def claude_turn(
        degrees: float,
        speed: float = 0.5,
        timeout: float = 30.0
    ):
        """
        Turn the robot by a specified angle.

        Args:
            degrees: Rotation in degrees (positive=left/CCW, negative=right/CW)
            speed: Speed in rad/s (default 0.5)
            timeout: Timeout in seconds (default 30)

        Returns:
            ClaudeCommandResponse with result and actual rotation achieved.
        """
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        from nimbus.behaviors.claude_control import (
            ClaudeControlBehavior,
            create_turn_command,
        )

        # Get or activate claude_control behavior
        behavior = _runner._behavior_manager.get_behavior("claude_control")
        if not behavior or not isinstance(behavior, ClaudeControlBehavior):
            raise HTTPException(500, detail="Claude control behavior not found")

        # Auto-activate if not active
        if _runner.current_behavior != "claude_control":
            _runner.set_behavior("claude_control")

        # Create and set command
        command = create_turn_command(degrees, speed, timeout)
        behavior.set_command(command)

        # Wait for completion in executor to not block event loop
        loop = asyncio.get_event_loop()
        completed = await loop.run_in_executor(
            None,
            lambda: behavior.wait_for_completion(timeout=timeout + 1.0)
        )

        result = behavior.get_result()
        if not result:
            raise HTTPException(500, detail="No result from command")

        duration = time.time() - result.start_time if result.start_time else 0

        return ClaudeCommandResponse(
            success=result.result in ("completed", "completed_open_loop"),
            result=result.result or "unknown",
            target=result.target,
            actual=result.actual or 0.0,
            duration=duration,
        )

    @app.post("/api/claude/stop")
    async def claude_stop():
        """Stop any in-progress Claude control command."""
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        from nimbus.behaviors.claude_control import ClaudeControlBehavior

        behavior = _runner._behavior_manager.get_behavior("claude_control")
        if behavior and isinstance(behavior, ClaudeControlBehavior):
            result = behavior.stop_command()
            if result:
                return {
                    "status": "stopped",
                    "command_type": result.command_type,
                    "actual": result.actual,
                }

        return {"status": "no_command"}

    @app.get("/api/claude/status", response_model=ClaudeStatusResponse)
    async def claude_status():
        """Get current Claude control status."""
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        from nimbus.behaviors.claude_control import ClaudeControlBehavior

        behavior = _runner._behavior_manager.get_behavior("claude_control")
        if not behavior or not isinstance(behavior, ClaudeControlBehavior):
            return ClaudeStatusResponse(state="unavailable", command=None)

        status = behavior.get_status()
        return ClaudeStatusResponse(
            state=status["state"],
            command=status["command"],
        )

    # --- Claude High-Level Endpoints ---

    @app.get("/api/claude/scan", response_model=ClaudeScanResponse)
    async def claude_scan():
        """
        Get a processed, world-relative summary of surroundings.

        Returns compass-bearing openings, wall distances, and closest obstacle
        so the caller can reason spatially without doing trig.
        """
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        ctx = _runner.context
        sensors = ctx.sensors
        if not sensors or not sensors.lidar_ranges:
            raise HTTPException(503, detail="No sensor data available")

        import numpy as np
        from nimbus.sensors.lidar import LidarProcessor, LidarConfig
        from nimbus.navigation.vfh import VFHNavigator, VFHConfig

        ranges = np.array(sensors.lidar_ranges, dtype=np.float32)
        robot_theta = sensors.pose.theta

        # Fresh processors (no shared state, thread-safe)
        lidar = LidarProcessor(LidarConfig())
        vfh = VFHNavigator(VFHConfig())

        # Wall distances in 8 cardinal directions
        sector_ranges = lidar.get_sector_ranges(ranges)
        walls = {k: safe_float(v) for k, v in sector_ranges.items()}

        # Closest obstacle
        closest_dist, closest_angle_rad = lidar.find_closest(ranges)
        closest_obstacle = None
        if closest_dist != float('inf'):
            # Convert robot-relative angle to world compass bearing
            world_angle = robot_theta + closest_angle_rad
            closest_obstacle = {
                "distance": round(closest_dist, 2),
                "bearing": round(_math_to_compass(world_angle)),
            }

        # Find openings via VFH
        histogram = lidar.process(ranges)
        smoothed = vfh._smooth_histogram(histogram)
        blocked_sectors = smoothed > vfh.config.threshold_high
        valleys = vfh._find_valleys(blocked_sectors)

        openings = []
        for valley in valleys:
            # Convert robot-relative valley center to world compass bearing
            world_angle = robot_theta + valley.center_angle
            bearing = _math_to_compass(world_angle)
            width_deg = valley.width * (360.0 / vfh.config.num_sectors)

            # Sample LIDAR range at valley center direction
            # Valley center_angle is robot-relative, map to LIDAR index
            lidar_deg = math.degrees(valley.center_angle) % 360
            lidar_idx = int(lidar_deg) % len(ranges)
            distance = float(ranges[lidar_idx])
            distance = safe_float(distance)

            openings.append(ClaudeScanOpening(
                bearing=round(bearing),
                width_degrees=round(width_deg),
                distance=round(distance, 2) if distance else None,
                is_wide=width_deg >= 40,
            ))

        heading = _math_to_compass(robot_theta)

        return ClaudeScanResponse(
            pose={
                "x": round(sensors.pose.x, 2),
                "y": round(sensors.pose.y, 2),
                "heading": round(heading),
            },
            closest_obstacle=closest_obstacle,
            walls=walls,
            openings=openings,
            timestamp=sensors.timestamp.isoformat(),
        )

    @app.post("/api/claude/goto", response_model=ClaudeGotoResponse)
    async def claude_goto(
        x: float,
        y: float,
        timeout: float = 60.0,
    ):
        """
        Navigate to a coordinate using VFH obstacle avoidance (blocking).

        Args:
            x: Target X coordinate in meters
            y: Target Y coordinate in meters
            timeout: Maximum time in seconds (default 60)
        """
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        from nimbus.behaviors.goto import GoToBehavior

        goto_behavior = _runner._behavior_manager.get_behavior("goto")
        if not goto_behavior or not isinstance(goto_behavior, GoToBehavior):
            raise HTTPException(500, detail="GoTo behavior not found")

        # Set target and activate
        goto_behavior.set_target(x, y)
        _runner.set_behavior("goto")

        start_time = time.time()
        last_pose = None
        last_move_time = start_time

        def _wait_for_goto():
            nonlocal last_pose, last_move_time

            while True:
                elapsed = time.time() - start_time
                if elapsed > timeout:
                    return "timeout"

                if goto_behavior.reached_goal:
                    return "reached"

                # Stuck detection: hasn't moved >2cm in 5 seconds
                sensors = _runner.context.sensors
                if sensors:
                    current = (sensors.pose.x, sensors.pose.y)
                    if last_pose is not None:
                        dx = current[0] - last_pose[0]
                        dy = current[1] - last_pose[1]
                        if math.sqrt(dx * dx + dy * dy) > 0.02:
                            last_pose = current
                            last_move_time = time.time()
                    else:
                        last_pose = current
                        last_move_time = time.time()

                    if time.time() - last_move_time > 5.0:
                        return "stuck"

                time.sleep(0.2)

        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(None, _wait_for_goto)

        # Switch back to claude_control so robot stops
        _runner.set_behavior("claude_control")

        duration = time.time() - start_time
        sensors = _runner.context.sensors
        final_x = sensors.pose.x if sensors else 0
        final_y = sensors.pose.y if sensors else 0
        final_theta = sensors.pose.theta if sensors else 0
        dist_remaining = math.sqrt((x - final_x) ** 2 + (y - final_y) ** 2)

        return ClaudeGotoResponse(
            success=(result == "reached"),
            result=result,
            target={"x": x, "y": y},
            final_pose={
                "x": round(final_x, 2),
                "y": round(final_y, 2),
                "heading": round(_math_to_compass(final_theta)),
            },
            distance_remaining=round(dist_remaining, 2),
            duration=round(duration, 1),
        )

    @app.get("/api/claude/map", response_model=ClaudeMapResponse)
    async def claude_map():
        """
        Return the accumulated obstacle map as ASCII art with metadata.
        """
        if not _runner:
            raise HTTPException(503, detail="Nimbus not initialized")

        obstacle_map = _runner.obstacle_map
        sensors = _runner.context.sensors
        robot_pose = sensors.pose if sensors else None

        ascii_map = _render_map_plain(obstacle_map, robot_pose)

        min_x, min_y, max_x, max_y = obstacle_map.get_bounds()

        trail = obstacle_map.get_robot_trail()
        trail_dicts = [{"x": round(tx, 2), "y": round(ty, 2)} for tx, ty in trail]

        heading = _math_to_compass(robot_pose.theta) if robot_pose else 0

        return ClaudeMapResponse(
            ascii_map=ascii_map,
            bounds={
                "min_x": round(min_x, 2),
                "min_y": round(min_y, 2),
                "max_x": round(max_x, 2),
                "max_y": round(max_y, 2),
            },
            stats={
                "num_obstacles": obstacle_map.num_obstacles,
                "num_visited": obstacle_map.num_visited,
                "cell_size_m": obstacle_map.config.cell_size,
            },
            robot_trail=trail_dicts,
            pose={
                "x": round(robot_pose.x, 2) if robot_pose else 0,
                "y": round(robot_pose.y, 2) if robot_pose else 0,
                "heading": round(heading),
            },
        )

    return app


def start_api_background(runner, host: str = "0.0.0.0", port: int = 8080) -> threading.Thread:
    """
    Start API server in a background thread.

    Args:
        runner: NimbusRunner instance
        host: Host to bind to
        port: Port to listen on

    Returns:
        Thread running the server
    """
    global _runner, _api_thread

    _runner = runner

    app = create_app()

    def run_server():
        config = uvicorn.Config(
            app,
            host=host,
            port=port,
            log_level="warning",  # Reduce noise
        )
        server = uvicorn.Server(config)
        server.run()

    _api_thread = threading.Thread(target=run_server, daemon=True)
    _api_thread.start()

    return _api_thread


def stop_api():
    """Stop the API server."""
    global _api_thread
    # Uvicorn doesn't have a clean stop mechanism for threaded mode
    # The daemon thread will stop when main process exits
    _api_thread = None
