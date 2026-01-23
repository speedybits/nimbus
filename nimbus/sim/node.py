"""
Simulator node for Nimbus.

Provides the same interface as XRCENode but generates synthetic
sensor data from a simulated 2D world.
"""

from typing import Any, Callable, Optional
import threading
import time
import math

from nimbus.core.node import MockNimbusNode, TopicBuffer, MockPublisher
from nimbus.core.xrce.messages import (
    LaserScan, Odometry, Twist,
    Header, Time, Point, Quaternion, Pose, Vector3,
    PoseWithCovariance, TwistWithCovariance
)
from .world import SimulatedWorld


class SimulatorNode:
    """
    Simulator node that generates sensor data from a virtual world.

    Provides the same interface as XRCENode:
    - subscribe(topic, msg_type, buffer_size) -> TopicBuffer
    - publisher(topic, msg_type) -> Publisher
    - start() / shutdown()

    Internally runs a physics loop that:
    1. Reads velocity commands from /cmd_vel
    2. Updates robot pose in the simulated world
    3. Ray-casts to generate synthetic LIDAR
    4. Injects LaserScan and Odometry messages into buffers
    """

    def __init__(
        self,
        world: SimulatedWorld,
        tick_rate: float = 10.0,  # Hz
        name: str = "nimbus_sim",
    ):
        """
        Initialize the simulator node.

        Args:
            world: SimulatedWorld instance
            tick_rate: Physics update rate in Hz
            name: Node name for logging
        """
        self._world = world
        self._tick_rate = tick_rate
        self._name = name

        # Internal mock node for buffer management
        self._mock = MockNimbusNode(name)

        # Current velocity command
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._cmd_lock = threading.Lock()

        # Simulation thread
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Start the simulation loop."""
        if self._running:
            return

        self._mock.start()
        self._running = True

        # Start physics thread
        self._thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        """Stop the simulation loop."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._mock.shutdown()

    def subscribe(
        self,
        topic: str,
        msg_type: type,
        buffer_size: int = 1,
        callback: Optional[Callable[[Any], None]] = None
    ) -> TopicBuffer:
        """Subscribe to a topic."""
        return self._mock.subscribe(topic, msg_type, buffer_size, callback)

    def publisher(self, topic: str, msg_type: type) -> "SimPublisher":
        """Get a publisher for a topic."""
        # For /cmd_vel, we intercept the messages
        if topic == "/cmd_vel":
            return SimPublisher(topic, self._on_cmd_vel)
        return self._mock.publisher(topic, msg_type)

    def get_buffer(self, topic: str) -> Optional[TopicBuffer]:
        """Get the buffer for a topic."""
        return self._mock.get_buffer(topic)

    def inject_message(self, topic: str, msg: Any) -> None:
        """Inject a message into a topic buffer."""
        self._mock.inject_message(topic, msg)

    @property
    def is_running(self) -> bool:
        """Check if running."""
        return self._running

    @property
    def world(self) -> SimulatedWorld:
        """Get the simulated world."""
        return self._world

    def _on_cmd_vel(self, msg: Any) -> None:
        """Handle velocity command."""
        with self._cmd_lock:
            if hasattr(msg, 'linear') and hasattr(msg, 'angular'):
                self._cmd_linear = float(msg.linear.x)
                self._cmd_angular = float(msg.angular.z)

    def _simulation_loop(self) -> None:
        """Main simulation loop running at tick_rate Hz."""
        period = 1.0 / self._tick_rate
        last_time = time.time()

        while self._running:
            loop_start = time.time()
            dt = loop_start - last_time
            last_time = loop_start

            # 1. Apply velocity command
            with self._cmd_lock:
                linear = self._cmd_linear
                angular = self._cmd_angular

            self._world.apply_velocity(linear, angular, dt)

            # 2. Generate sensor data
            self._generate_lidar()
            self._generate_odometry(linear, angular)

            # 3. Maintain loop rate
            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _generate_lidar(self) -> None:
        """Generate synthetic LIDAR data from ray-casting."""
        # Ray-cast from robot position
        ranges = self._world.raycast(num_rays=360, max_range=3.5)

        # Create LaserScan message
        scan = LaserScan(
            header=Header(frame_id="laser"),
            angle_min=0.0,
            angle_max=2 * math.pi,
            angle_increment=2 * math.pi / 360,
            time_increment=0.0,
            scan_time=0.1,
            range_min=0.05,
            range_max=3.5,
            ranges=ranges,
            intensities=[],
        )

        # Inject into buffer
        self._mock.inject_message("/scan", scan)

    def _generate_odometry(self, linear: float, angular: float) -> None:
        """Generate odometry data from robot pose."""
        x, y, theta = self._world.get_robot_pose()

        # Convert theta to quaternion
        quat = self._euler_to_quaternion(0, 0, theta)

        # Create Odometry message
        odom = Odometry(
            header=Header(frame_id="odom"),
            child_frame_id="base_link",
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(x=x, y=y, z=0.0),
                    orientation=quat,
                ),
                covariance=[0.0] * 36,
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(x=linear, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=angular),
                ),
                covariance=[0.0] * 36,
            ),
        )

        # Inject into buffer
        self._mock.inject_message("/odom_raw", odom)

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        """Convert Euler angles to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return Quaternion(
            w=cr * cp * cy + sr * sp * sy,
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
        )


class SimPublisher:
    """
    Publisher that intercepts messages for simulation.

    Used for /cmd_vel to capture velocity commands.
    """

    def __init__(self, topic: str, callback: Callable[[Any], None]):
        self.topic = topic
        self._callback = callback
        self.published_messages: list[Any] = []

    def publish(self, msg: Any) -> None:
        """Publish a message (calls the callback)."""
        self._callback(msg)
        self.published_messages.append(msg)

    @property
    def last_message(self) -> Optional[Any]:
        """Get the last published message."""
        return self.published_messages[-1] if self.published_messages else None
