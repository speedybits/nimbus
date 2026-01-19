"""
ROS2 message type definitions for direct XRCE-DDS communication.

These classes mirror the ROS2 message types but can be serialized/deserialized
directly from CDR bytes, enabling communication without ROS2 installed.

Supported messages:
- sensor_msgs/LaserScan (360 LIDAR ranges)
- nav_msgs/Odometry (pose + velocity)
- geometry_msgs/Twist (velocity command)
"""

from dataclasses import dataclass, field
from typing import List, Optional
from .cdr import CDRReader, CDRWriter, parse_encapsulation, add_encapsulation
from .logger import xrce_log


# =============================================================================
# Primitive types
# =============================================================================

@dataclass
class Time:
    """builtin_interfaces/Time"""
    sec: int = 0
    nanosec: int = 0

    @classmethod
    def from_cdr(cls, reader: CDRReader) -> "Time":
        return cls(
            sec=reader.read_int32(),
            nanosec=reader.read_uint32()
        )

    def to_cdr(self, writer: CDRWriter) -> None:
        writer.write_int32(self.sec)
        writer.write_uint32(self.nanosec)


@dataclass
class Header:
    """std_msgs/Header"""
    stamp: Time = field(default_factory=Time)
    frame_id: str = ""

    @classmethod
    def from_cdr(cls, reader: CDRReader) -> "Header":
        return cls(
            stamp=Time.from_cdr(reader),
            frame_id=reader.read_string()
        )

    def to_cdr(self, writer: CDRWriter) -> None:
        self.stamp.to_cdr(writer)
        writer.write_string(self.frame_id)


@dataclass
class Vector3:
    """geometry_msgs/Vector3"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @classmethod
    def from_cdr(cls, reader: CDRReader) -> "Vector3":
        return cls(
            x=reader.read_float64(),
            y=reader.read_float64(),
            z=reader.read_float64()
        )

    def to_cdr(self, writer: CDRWriter) -> None:
        writer.write_float64(self.x)
        writer.write_float64(self.y)
        writer.write_float64(self.z)


@dataclass
class Point:
    """geometry_msgs/Point"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @classmethod
    def from_cdr(cls, reader: CDRReader) -> "Point":
        return cls(
            x=reader.read_float64(),
            y=reader.read_float64(),
            z=reader.read_float64()
        )

    def to_cdr(self, writer: CDRWriter) -> None:
        writer.write_float64(self.x)
        writer.write_float64(self.y)
        writer.write_float64(self.z)


@dataclass
class Quaternion:
    """geometry_msgs/Quaternion"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

    @classmethod
    def from_cdr(cls, reader: CDRReader) -> "Quaternion":
        return cls(
            x=reader.read_float64(),
            y=reader.read_float64(),
            z=reader.read_float64(),
            w=reader.read_float64()
        )

    def to_cdr(self, writer: CDRWriter) -> None:
        writer.write_float64(self.x)
        writer.write_float64(self.y)
        writer.write_float64(self.z)
        writer.write_float64(self.w)


@dataclass
class Pose:
    """geometry_msgs/Pose"""
    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)

    @classmethod
    def from_cdr(cls, reader: CDRReader) -> "Pose":
        return cls(
            position=Point.from_cdr(reader),
            orientation=Quaternion.from_cdr(reader)
        )

    def to_cdr(self, writer: CDRWriter) -> None:
        self.position.to_cdr(writer)
        self.orientation.to_cdr(writer)


@dataclass
class PoseWithCovariance:
    """geometry_msgs/PoseWithCovariance"""
    pose: Pose = field(default_factory=Pose)
    covariance: List[float] = field(default_factory=lambda: [0.0] * 36)

    @classmethod
    def from_cdr(cls, reader: CDRReader) -> "PoseWithCovariance":
        pose = Pose.from_cdr(reader)
        # Covariance is a fixed-size array of 36 float64s
        covariance = [reader.read_float64() for _ in range(36)]
        return cls(pose=pose, covariance=covariance)

    def to_cdr(self, writer: CDRWriter) -> None:
        self.pose.to_cdr(writer)
        for val in self.covariance:
            writer.write_float64(val)


# =============================================================================
# Twist message (geometry_msgs/Twist)
# =============================================================================

@dataclass
class Twist:
    """
    geometry_msgs/Twist - Velocity command.

    Used for publishing /cmd_vel commands.
    """
    linear: Vector3 = field(default_factory=Vector3)
    angular: Vector3 = field(default_factory=Vector3)

    @classmethod
    def from_cdr(cls, reader: CDRReader) -> "Twist":
        return cls(
            linear=Vector3.from_cdr(reader),
            angular=Vector3.from_cdr(reader)
        )

    def to_cdr(self, writer: CDRWriter) -> None:
        self.linear.to_cdr(writer)
        self.angular.to_cdr(writer)

    def serialize(self) -> bytes:
        """Serialize to CDR bytes with encapsulation."""
        writer = CDRWriter()
        self.to_cdr(writer)
        return add_encapsulation(writer.get_data())

    @classmethod
    def deserialize(cls, data: bytes) -> "Twist":
        """Deserialize from CDR bytes with encapsulation."""
        _, payload = parse_encapsulation(data)
        reader = CDRReader(payload)
        return cls.from_cdr(reader)

    @classmethod
    def create(cls, linear_x: float = 0.0, angular_z: float = 0.0) -> "Twist":
        """Create a Twist message with typical 2D motion values."""
        return cls(
            linear=Vector3(x=linear_x, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=angular_z)
        )


@dataclass
class TwistWithCovariance:
    """geometry_msgs/TwistWithCovariance"""
    twist: Twist = field(default_factory=Twist)
    covariance: List[float] = field(default_factory=lambda: [0.0] * 36)

    @classmethod
    def from_cdr(cls, reader: CDRReader) -> "TwistWithCovariance":
        twist = Twist.from_cdr(reader)
        covariance = [reader.read_float64() for _ in range(36)]
        return cls(twist=twist, covariance=covariance)

    def to_cdr(self, writer: CDRWriter) -> None:
        self.twist.to_cdr(writer)
        for val in self.covariance:
            writer.write_float64(val)


# =============================================================================
# LaserScan message (sensor_msgs/LaserScan)
# =============================================================================

@dataclass
class LaserScan:
    """
    sensor_msgs/LaserScan - LIDAR scan data.

    The Yahboom robot sends 360 range measurements.
    """
    header: Header = field(default_factory=Header)
    angle_min: float = 0.0       # Start angle (radians)
    angle_max: float = 0.0       # End angle (radians)
    angle_increment: float = 0.0  # Angular step (radians)
    time_increment: float = 0.0   # Time between measurements
    scan_time: float = 0.0        # Time for full scan
    range_min: float = 0.0        # Minimum valid range
    range_max: float = 0.0        # Maximum valid range
    ranges: List[float] = field(default_factory=list)      # Range data
    intensities: List[float] = field(default_factory=list)  # Intensity data

    @classmethod
    def from_cdr(cls, reader: CDRReader) -> "LaserScan":
        """
        Deserialize from CDR - ESP32 format.

        The ESP32 Micro-ROS sends a simplified format:
        - 4 bytes (sequence/padding)
        - frame_id string (no timestamp!)
        - angle_min, angle_max, etc.
        - ranges array
        - intensities array
        """
        # Skip 4 bytes at start (ESP32 format quirk)
        reader.read_uint32()

        # Read frame_id directly (no Header.stamp in ESP32 format)
        frame_id = reader.read_string()

        angle_min = reader.read_float32()
        angle_max = reader.read_float32()
        angle_increment = reader.read_float32()
        time_increment = reader.read_float32()
        scan_time = reader.read_float32()
        range_min = reader.read_float32()
        range_max = reader.read_float32()
        ranges = reader.read_float32_array()
        xrce_log(f"LaserScan: {len(ranges)} ranges, frame_id={frame_id}")
        intensities = reader.read_float32_array()
        return cls(
            header=Header(frame_id=frame_id),
            angle_min=angle_min,
            angle_max=angle_max,
            angle_increment=angle_increment,
            time_increment=time_increment,
            scan_time=scan_time,
            range_min=range_min,
            range_max=range_max,
            ranges=ranges,
            intensities=intensities
        )

    def to_cdr(self, writer: CDRWriter) -> None:
        self.header.to_cdr(writer)
        writer.write_float32(self.angle_min)
        writer.write_float32(self.angle_max)
        writer.write_float32(self.angle_increment)
        writer.write_float32(self.time_increment)
        writer.write_float32(self.scan_time)
        writer.write_float32(self.range_min)
        writer.write_float32(self.range_max)
        writer.write_float32_array(self.ranges)
        writer.write_float32_array(self.intensities)

    def serialize(self) -> bytes:
        """Serialize to CDR bytes with encapsulation."""
        writer = CDRWriter()
        self.to_cdr(writer)
        return add_encapsulation(writer.get_data())

    @classmethod
    def deserialize(cls, data: bytes) -> "LaserScan":
        """Deserialize from CDR bytes with encapsulation."""
        _, payload = parse_encapsulation(data)
        reader = CDRReader(payload)
        return cls.from_cdr(reader)


# =============================================================================
# Odometry message (nav_msgs/Odometry)
# =============================================================================

@dataclass
class Odometry:
    """
    nav_msgs/Odometry - Robot pose and velocity.

    The Yahboom robot publishes this on /odom_raw.
    """
    header: Header = field(default_factory=Header)
    child_frame_id: str = ""
    pose: PoseWithCovariance = field(default_factory=PoseWithCovariance)
    twist: TwistWithCovariance = field(default_factory=TwistWithCovariance)

    @classmethod
    def from_cdr(cls, reader: CDRReader) -> "Odometry":
        """
        Deserialize from CDR - ESP32 format.

        The ESP32 Micro-ROS sends a simplified format:
        - 4 bytes (sequence/padding)
        - frame_id string (no timestamp!)
        - child_frame_id string
        - PoseWithCovariance
        - TwistWithCovariance
        """
        # Skip 4 bytes at start (ESP32 format quirk)
        reader.read_uint32()

        # Read frame_id directly (no Header.stamp in ESP32 format)
        frame_id = reader.read_string()

        # Standard fields
        child_frame_id = reader.read_string()
        pose = PoseWithCovariance.from_cdr(reader)
        twist = TwistWithCovariance.from_cdr(reader)

        return cls(
            header=Header(frame_id=frame_id),
            child_frame_id=child_frame_id,
            pose=pose,
            twist=twist
        )

    def to_cdr(self, writer: CDRWriter) -> None:
        self.header.to_cdr(writer)
        writer.write_string(self.child_frame_id)
        self.pose.to_cdr(writer)
        self.twist.to_cdr(writer)

    def serialize(self) -> bytes:
        """Serialize to CDR bytes with encapsulation."""
        writer = CDRWriter()
        self.to_cdr(writer)
        return add_encapsulation(writer.get_data())

    @classmethod
    def deserialize(cls, data: bytes) -> "Odometry":
        """Deserialize from CDR bytes with encapsulation."""
        _, payload = parse_encapsulation(data)
        reader = CDRReader(payload)
        return cls.from_cdr(reader)


# =============================================================================
# Message registry
# =============================================================================

# Map topic names to message types
MESSAGE_TYPES = {
    "/scan": LaserScan,
    "/odom_raw": Odometry,
    "/cmd_vel": Twist,
    "rt/scan": LaserScan,
    "rt/odom_raw": Odometry,
    "rt/cmd_vel": Twist,
}


def get_message_type(topic: str):
    """Get the message type for a topic name."""
    return MESSAGE_TYPES.get(topic)


def deserialize_message(topic: str, data: bytes):
    """Deserialize a message based on topic name."""
    msg_type = get_message_type(topic)
    if msg_type is None:
        raise ValueError(f"Unknown topic: {topic}")
    return msg_type.deserialize(data)
