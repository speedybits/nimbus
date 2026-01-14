"""
WebSocket handlers for real-time telemetry.

Provides live data streams to connected clients.
"""

from fastapi import WebSocket, WebSocketDisconnect
from typing import List
import asyncio
import json


class ConnectionManager:
    """Manages WebSocket connections."""

    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket):
        """Accept and register a new connection."""
        await websocket.accept()
        async with self._lock:
            self.active_connections.append(websocket)

    async def disconnect(self, websocket: WebSocket):
        """Remove a connection."""
        async with self._lock:
            if websocket in self.active_connections:
                self.active_connections.remove(websocket)

    async def broadcast(self, data: dict):
        """Send data to all connected clients."""
        async with self._lock:
            dead_connections = []
            for connection in self.active_connections:
                try:
                    await connection.send_json(data)
                except Exception:
                    dead_connections.append(connection)

            # Clean up dead connections
            for conn in dead_connections:
                self.active_connections.remove(conn)

    @property
    def client_count(self) -> int:
        """Number of connected clients."""
        return len(self.active_connections)


# Global connection managers for different streams
telemetry_manager = ConnectionManager()
lidar_manager = ConnectionManager()
events_manager = ConnectionManager()


async def telemetry_endpoint(websocket: WebSocket):
    """
    WebSocket endpoint for telemetry stream.

    Sends sensor data at ~10Hz to connected clients.
    """
    await telemetry_manager.connect(websocket)
    try:
        while True:
            # Keep connection alive
            data = await websocket.receive_text()
            # Client can send "ping" to keep alive
    except WebSocketDisconnect:
        await telemetry_manager.disconnect(websocket)


async def lidar_endpoint(websocket: WebSocket):
    """
    WebSocket endpoint for LIDAR data stream.

    Sends full LIDAR scans for visualization.
    """
    await lidar_manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        await lidar_manager.disconnect(websocket)


async def events_endpoint(websocket: WebSocket):
    """
    WebSocket endpoint for event stream.

    Sends state changes, alerts, and navigation events.
    """
    await events_manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        await events_manager.disconnect(websocket)


async def broadcast_telemetry(runner) -> None:
    """
    Background task to broadcast telemetry to connected clients.

    Call this in a loop from the main runner.
    """
    if telemetry_manager.client_count == 0:
        return

    ctx = runner.context
    sensors = ctx.sensors

    if sensors:
        data = {
            "type": "telemetry",
            "timestamp": sensors.timestamp.isoformat(),
            "state": ctx.state.name,
            "pose": {
                "x": sensors.pose.x,
                "y": sensors.pose.y,
                "theta": sensors.pose.theta,
            },
            "velocity": {
                "linear": sensors.velocity.linear,
                "angular": sensors.velocity.angular,
            },
            "closest_obstacle": sensors.closest_obstacle,
        }
        await telemetry_manager.broadcast(data)


async def broadcast_lidar(runner) -> None:
    """Broadcast LIDAR data to connected clients."""
    if lidar_manager.client_count == 0:
        return

    ctx = runner.context
    sensors = ctx.sensors

    if sensors and sensors.lidar_ranges:
        # Downsample for bandwidth
        downsampled = sensors.lidar_ranges[::5]  # Every 5th point
        data = {
            "type": "lidar",
            "timestamp": sensors.timestamp.isoformat(),
            "ranges": list(downsampled),
            "closest": sensors.closest_obstacle,
            "closest_angle": sensors.obstacle_direction,
        }
        await lidar_manager.broadcast(data)


async def broadcast_event(event_type: str, data: dict) -> None:
    """Broadcast an event to connected clients."""
    if events_manager.client_count == 0:
        return

    event = {
        "type": "event",
        "event_type": event_type,
        **data
    }
    await events_manager.broadcast(event)
