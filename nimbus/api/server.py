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
)

# Global reference to runner (set by start_api_background)
_runner = None
_api_thread = None


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
        import math
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

        # Sanitize float values for JSON (inf -> None)
        import math
        def safe_float(v: float) -> float | None:
            return None if math.isinf(v) or math.isnan(v) else v

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
