"""
Odometry processing for Nimbus.

Tracks robot pose from XRCE-DDS odometry messages.
"""

from dataclasses import dataclass
from typing import Optional
from datetime import datetime
import math

from nimbus.core.state import Pose2D, Velocity, SensorSnapshot


@dataclass
class OdometryConfig:
    """Odometry processing configuration."""

    max_velocity: float = 1.0      # Maximum expected velocity (m/s)
    position_timeout: float = 1.0  # Seconds before position is stale


class OdometryProcessor:
    """
    Process odometry messages into Pose2D and Velocity.

    Usage:
        processor = OdometryProcessor()
        pose = processor.process(odom_msg)
    """

    def __init__(self, config: Optional[OdometryConfig] = None):
        self.config = config or OdometryConfig()
        self._last_pose: Optional[Pose2D] = None
        self._last_velocity: Optional[Velocity] = None
        self._last_update: Optional[datetime] = None

    def process(self, odom_msg) -> tuple[Pose2D, Velocity]:
        """
        Extract pose and velocity from ROS2 Odometry message.

        Args:
            odom_msg: nav_msgs/Odometry message

        Returns:
            Tuple of (Pose2D, Velocity)
        """
        # Extract position
        pos = odom_msg.pose.pose.position
        x = float(pos.x)
        y = float(pos.y)

        # Extract orientation (quaternion to yaw)
        orient = odom_msg.pose.pose.orientation
        theta = self._quaternion_to_yaw(orient.x, orient.y, orient.z, orient.w)

        # Extract velocity
        twist = odom_msg.twist.twist
        linear = float(twist.linear.x)
        angular = float(twist.angular.z)

        # Create immutable objects
        pose = Pose2D(x=x, y=y, theta=theta)
        velocity = Velocity(linear=linear, angular=angular)

        # Cache values
        self._last_pose = pose
        self._last_velocity = velocity
        self._last_update = datetime.now()

        return pose, velocity

    def _quaternion_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        """Convert quaternion to yaw angle (radians)."""
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @property
    def last_pose(self) -> Optional[Pose2D]:
        """Get the last processed pose."""
        return self._last_pose

    @property
    def last_velocity(self) -> Optional[Velocity]:
        """Get the last processed velocity."""
        return self._last_velocity

    @property
    def is_stale(self) -> bool:
        """Check if odometry data is stale."""
        if self._last_update is None:
            return True
        age = (datetime.now() - self._last_update).total_seconds()
        return age > self.config.position_timeout


class SensorFusion:
    """
    Combine LIDAR and odometry into unified sensor snapshots.

    This is a simple fusion that just combines the latest data.
    For more sophisticated fusion, consider an EKF.
    """

    def __init__(self):
        self._last_snapshot: Optional[SensorSnapshot] = None

    def update(
        self,
        pose: Pose2D,
        velocity: Velocity,
        lidar_ranges: tuple[float, ...],
        closest_obstacle: float,
        obstacle_direction: float,
        bumper_triggered: bool = False,
    ) -> SensorSnapshot:
        """
        Create a new sensor snapshot from all sensor data.

        Args:
            pose: Current robot pose
            velocity: Current velocity
            lidar_ranges: 360 LIDAR range values
            closest_obstacle: Distance to nearest obstacle
            obstacle_direction: Angle to nearest obstacle
            bumper_triggered: True if any physical bumper is pressed

        Returns:
            Immutable SensorSnapshot
        """
        snapshot = SensorSnapshot(
            timestamp=datetime.now(),
            pose=pose,
            velocity=velocity,
            lidar_ranges=lidar_ranges,
            closest_obstacle=closest_obstacle,
            obstacle_direction=obstacle_direction,
            bumper_triggered=bumper_triggered,
        )
        self._last_snapshot = snapshot
        return snapshot

    @property
    def latest(self) -> Optional[SensorSnapshot]:
        """Get the most recent snapshot."""
        return self._last_snapshot


# Helper function to create velocity command messages
def create_twist_msg(linear: float, angular: float):
    """
    Create an XRCE Twist message.

    Args:
        linear: Forward velocity (m/s)
        angular: Rotational velocity (rad/s)

    Returns:
        XRCE Twist message
    """
    from nimbus.core.xrce.messages import Twist
    return Twist.create(linear_x=float(linear), angular_z=float(angular))
