"""
Pydantic schemas for Nimbus API.

Defines request/response models for type safety and validation.
"""

from pydantic import BaseModel, Field
from typing import Optional


class Pose(BaseModel):
    """2D pose (position + orientation)."""
    x: float = Field(default=0.0, description="X position in meters")
    y: float = Field(default=0.0, description="Y position in meters")
    theta: float = Field(default=0.0, description="Orientation in radians")


class VelocityModel(BaseModel):
    """Velocity vector."""
    linear: float = Field(default=0.0, description="Linear velocity in m/s")
    angular: float = Field(default=0.0, description="Angular velocity in rad/s")


class StatusResponse(BaseModel):
    """Robot status response."""
    state: str = Field(..., description="Current state: IDLE, NAVIGATING, etc.")
    pose: Pose = Field(default_factory=Pose, description="Current position")
    velocity: VelocityModel = Field(default_factory=VelocityModel, description="Current velocity")
    closest_obstacle: Optional[float] = Field(None, description="Distance to nearest obstacle in meters")
    current_behavior: Optional[str] = Field(None, description="Active behavior name")
    target: Optional[Pose] = Field(None, description="Navigation target if set")


class SensorResponse(BaseModel):
    """Sensor data response."""
    timestamp: str = Field(..., description="ISO timestamp of reading")
    pose: Pose = Field(..., description="Robot pose")
    velocity: VelocityModel = Field(..., description="Robot velocity")
    closest_obstacle: Optional[float] = Field(None, description="Nearest obstacle distance")
    obstacle_direction: Optional[float] = Field(None, description="Angle to nearest obstacle (radians)")
    lidar_histogram: list[Optional[float]] = Field(default_factory=list, description="Polar histogram (72 sectors)")
    odom_age: Optional[float] = Field(None, description="Seconds since last odometry message")
    odom_stale: Optional[bool] = Field(None, description="Whether odometry data is stale (>1s old)")


class NavigateRequest(BaseModel):
    """Navigation goal request."""
    x: float = Field(..., description="Target X coordinate in meters")
    y: float = Field(..., description="Target Y coordinate in meters")
    theta: Optional[float] = Field(None, description="Optional target orientation in radians")


class BehaviorResponse(BaseModel):
    """Behavior list response."""
    behaviors: list[str] = Field(..., description="Available behavior names")
    current: Optional[str] = Field(None, description="Currently active behavior")


class HealthResponse(BaseModel):
    """Health check response."""
    status: str = Field(..., description="Health status: ok, error")
    running: bool = Field(..., description="Whether runner is active")
    version: str = Field(..., description="API version")


class TelemetryData(BaseModel):
    """Real-time telemetry for WebSocket."""
    timestamp: str
    state: str
    pose: Pose
    velocity: VelocityModel
    closest_obstacle: float
    lidar_histogram: list[float]


class ClaudeCommandResponse(BaseModel):
    """Response for Claude control commands."""
    success: bool = Field(..., description="Whether command completed successfully")
    result: str = Field(..., description="Result: completed, timeout, cancelled, obstacle")
    target: float = Field(..., description="Requested distance (m) or rotation (degrees)")
    actual: float = Field(..., description="Achieved distance (m) or rotation (degrees)")
    duration: float = Field(..., description="Time elapsed in seconds")


class ClaudeStatusResponse(BaseModel):
    """Status of Claude control behavior."""
    state: str = Field(..., description="Current state: idle, moving, turning, complete, error")
    command: Optional[dict] = Field(None, description="Current command details if active")


class ClaudeScanOpening(BaseModel):
    """A navigable opening detected in the LIDAR scan."""
    bearing: float = Field(..., description="Compass bearing in degrees (0=N, 90=E)")
    width_degrees: float = Field(..., description="Angular width of opening in degrees")
    distance: Optional[float] = Field(None, description="Range at center of opening (meters)")
    is_wide: bool = Field(..., description="True if opening >= 40 degrees wide")


class ClaudeScanResponse(BaseModel):
    """Processed LIDAR scan with world-relative spatial summary."""
    pose: dict = Field(..., description="Robot pose: x, y (meters), heading (compass degrees)")
    closest_obstacle: Optional[dict] = Field(None, description="Nearest obstacle: distance (m), bearing (compass deg)")
    walls: dict = Field(..., description="Min range in 8 cardinal/intercardinal directions (meters, null if clear)")
    openings: list[ClaudeScanOpening] = Field(default_factory=list, description="Navigable openings")
    timestamp: str = Field(..., description="ISO timestamp")


class ClaudeGotoResponse(BaseModel):
    """Response from blocking goto navigation command."""
    success: bool = Field(..., description="Whether robot reached the target")
    result: str = Field(..., description="Outcome: reached, timeout, stuck, cancelled")
    target: dict = Field(..., description="Requested target: x, y")
    final_pose: dict = Field(..., description="Final pose: x, y, heading (compass degrees)")
    distance_remaining: float = Field(..., description="Distance to target at completion (meters)")
    duration: float = Field(..., description="Elapsed time in seconds")


class ClaudeMapResponse(BaseModel):
    """Obstacle map as ASCII art with metadata."""
    ascii_map: str = Field(..., description="Plain-text map with obstacles, trail, and robot")
    bounds: dict = Field(..., description="World coordinate bounds: min_x, min_y, max_x, max_y")
    stats: dict = Field(..., description="Map statistics: num_obstacles, num_visited, cell_size_m")
    robot_trail: list[dict] = Field(default_factory=list, description="Recent robot positions: x, y")
    pose: dict = Field(..., description="Current robot pose: x, y, heading (compass degrees)")
    legend: str = Field(default="# = wall  + = uncertain  ~ = trail  arrow = robot")
