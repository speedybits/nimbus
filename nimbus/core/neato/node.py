"""
Neato node for Nimbus.

Provides the same interface as XRCENode and SimulatorNode but
reads sensors and sends motor commands over serial to a Neato vacuum.

Follows the SimulatorNode pattern exactly:
- Wraps MockNimbusNode for buffer/publisher infrastructure
- Intercepts /cmd_vel via NeatoPublisher
- Background daemon threads for scan, odometry, and velocity
- Injects LaserScan and Odometry messages into topic buffers
"""

from typing import Any, Callable, Optional
import logging
import threading
import time
import math

logger = logging.getLogger(__name__)

from nimbus.core.node import MockNimbusNode, TopicBuffer, MockPublisher
from nimbus.core.xrce.messages import (
    LaserScan, Odometry, Twist,
    Header, Point, Quaternion, Pose, Vector3,
    PoseWithCovariance, TwistWithCovariance,
)
from .transport import NeatoTransport, SerialDisconnectedError, NeatoBumperData
from .kinematics import velocity_to_wheel_speeds, wheel_speeds_to_motor_command, update_odometry


class NeatoNode:
    """
    Neato serial node — same interface as XRCENode / SimulatorNode.

    Three background threads handle all serial I/O:
    1. _scan_loop (~5Hz)   — GetLDSScan → LaserScan messages
    2. _odom_loop (~10Hz)  — GetMotors → encoder deltas → Odometry messages
    3. _velocity_loop (~10Hz) — read cmd_vel → SetMotor commands

    All serial access is guarded by _serial_lock.
    """

    def __init__(
        self,
        transport: Optional[NeatoTransport] = None,
        port: str = "/dev/ttyACM0",
        scan_rate_hz: float = 5.0,
        odom_rate_hz: float = 10.0,
        cmd_rate_hz: float = 10.0,
        sensor_rate_hz: float = 2.0,
        name: str = "nimbus_neato",
    ):
        self._transport = transport or NeatoTransport(port)
        self._scan_rate = scan_rate_hz
        self._odom_rate = odom_rate_hz
        self._cmd_rate = cmd_rate_hz
        self._sensor_rate = sensor_rate_hz
        self._name = name

        # Internal mock node for buffer management
        self._mock = MockNimbusNode(name)

        # Velocity command state
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._cmd_lock = threading.Lock()

        # Odometry state
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0
        self._odom_lock = threading.Lock()
        self._last_left_mm = None  # Previous encoder reading (None until first read)
        self._last_right_mm = None

        # Serial access lock (one port, three threads)
        self._serial_lock = threading.Lock()

        # Latest bumper/battery state (read by runner via properties)
        self._bumper_data = None
        self._battery_data = None
        self._bumper_lock = threading.Lock()

        # Serial health tracking
        self._serial_healthy = True
        self._consecutive_errors = 0
        self._max_consecutive_errors = 3

        # Thread control
        self._running = False
        self._scan_thread: Optional[threading.Thread] = None
        self._odom_thread: Optional[threading.Thread] = None
        self._velocity_thread: Optional[threading.Thread] = None
        self._sensor_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Connect to Neato and start background threads."""
        if self._running:
            return

        with self._serial_lock:
            self._transport.connect()

        self._mock.start()
        self._running = True

        self._scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._odom_thread = threading.Thread(target=self._odom_loop, daemon=True)
        self._velocity_thread = threading.Thread(target=self._velocity_loop, daemon=True)
        self._sensor_thread = threading.Thread(target=self._sensor_loop, daemon=True)

        self._scan_thread.start()
        self._odom_thread.start()
        self._velocity_thread.start()
        self._sensor_thread.start()

    def shutdown(self) -> None:
        """Stop motors, join threads, disconnect."""
        self._running = False

        # Wait for threads to finish
        for thread in (self._scan_thread, self._odom_thread, self._velocity_thread, self._sensor_thread):
            if thread:
                thread.join(timeout=2.0)

        self._scan_thread = None
        self._odom_thread = None
        self._velocity_thread = None
        self._sensor_thread = None

        # Clean shutdown: stop motors and disconnect
        with self._serial_lock:
            self._transport.disconnect()

        self._mock.shutdown()

    def subscribe(
        self,
        topic: str,
        msg_type: type,
        buffer_size: int = 1,
        callback: Optional[Callable[[Any], None]] = None,
    ) -> TopicBuffer:
        """Subscribe to a topic."""
        return self._mock.subscribe(topic, msg_type, buffer_size, callback)

    def publisher(self, topic: str, msg_type: type) -> "NeatoPublisher":
        """Get a publisher for a topic."""
        if topic == "/cmd_vel":
            return NeatoPublisher(topic, self._on_cmd_vel)
        return self._mock.publisher(topic, msg_type)

    def get_buffer(self, topic: str) -> Optional[TopicBuffer]:
        """Get the buffer for a topic."""
        return self._mock.get_buffer(topic)

    def inject_message(self, topic: str, msg: Any) -> None:
        """Inject a message into a topic buffer."""
        self._mock.inject_message(topic, msg)

    @property
    def is_running(self) -> bool:
        return self._running

    # --- Command handling ---

    def _on_cmd_vel(self, msg: Any) -> None:
        """Handle velocity command from behavior system."""
        with self._cmd_lock:
            if hasattr(msg, "linear") and hasattr(msg, "angular"):
                self._cmd_linear = float(msg.linear.x)
                self._cmd_angular = float(msg.angular.z)

    # --- Serial health ---

    def _record_success(self) -> None:
        """Reset error counter on successful serial operation."""
        self._consecutive_errors = 0
        if not self._serial_healthy:
            logger.info("Serial connection restored")
            self._serial_healthy = True

    def _record_error(self, exc: Exception, context: str) -> None:
        """Track serial errors; mark unhealthy after repeated failures."""
        self._consecutive_errors += 1
        if isinstance(exc, SerialDisconnectedError):
            logger.warning("Serial disconnected during %s: %s", context, exc)
            self._serial_healthy = False
        else:
            logger.warning("Serial error in %s (%d/%d): %s",
                           context, self._consecutive_errors,
                           self._max_consecutive_errors, exc)
            if self._consecutive_errors >= self._max_consecutive_errors:
                logger.error("Serial marked unhealthy after %d consecutive errors",
                             self._consecutive_errors)
                self._serial_healthy = False

    def _reconnect(self) -> bool:
        """Attempt to reconnect the serial transport. Returns True on success."""
        try:
            logger.info("Attempting serial reconnect...")
            with self._serial_lock:
                self._transport.disconnect()
                self._transport.connect()
            logger.info("Serial reconnect successful")
            self._serial_healthy = True
            self._consecutive_errors = 0
            return True
        except Exception as e:
            logger.warning("Serial reconnect failed: %s", e)
            return False

    # --- Background threads ---

    def _scan_loop(self) -> None:
        """Read LIDAR at scan_rate_hz and inject LaserScan messages."""
        period = 1.0 / self._scan_rate

        while self._running:
            loop_start = time.time()

            if not self._serial_healthy:
                time.sleep(period)
                continue

            try:
                with self._serial_lock:
                    scan_data = self._transport.get_lds_scan()

                self._record_success()

                # Convert to LaserScan message
                # Neato returns distances in mm; convert to meters
                # Error code != 0 means invalid reading -> inf
                ranges = []
                for i in range(360):
                    if i < len(scan_data.distances_mm):
                        if scan_data.error_codes[i] != 0 or scan_data.distances_mm[i] == 0:
                            ranges.append(float("inf"))
                        else:
                            ranges.append(scan_data.distances_mm[i] / 1000.0)
                    else:
                        ranges.append(float("inf"))

                scan = LaserScan(
                    header=Header(frame_id="laser"),
                    angle_min=0.0,
                    angle_max=2 * math.pi,
                    angle_increment=2 * math.pi / 360,
                    time_increment=0.0,
                    scan_time=1.0 / self._scan_rate,
                    range_min=0.02,
                    range_max=5.0,
                    ranges=ranges,
                    intensities=[float(i) for i in scan_data.intensities] if scan_data.intensities else [],
                )

                self._mock.inject_message("/scan", scan)

            except Exception as e:
                self._record_error(e, "scan_loop")

            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _odom_loop(self) -> None:
        """Read wheel encoders at odom_rate_hz and inject Odometry messages."""
        period = 1.0 / self._odom_rate

        while self._running:
            loop_start = time.time()

            if not self._serial_healthy:
                time.sleep(period)
                continue

            try:
                with self._serial_lock:
                    motor_data = self._transport.get_motors()

                left_mm = motor_data.left_wheel_position_mm
                right_mm = motor_data.right_wheel_position_mm

                # Compute deltas (skip first reading to establish baseline)
                if self._last_left_mm is not None:
                    delta_left_m = (left_mm - self._last_left_mm) / 1000.0
                    delta_right_m = (right_mm - self._last_right_mm) / 1000.0

                    with self._odom_lock:
                        self._odom_x, self._odom_y, self._odom_theta = update_odometry(
                            self._odom_x, self._odom_y, self._odom_theta,
                            delta_left_m, delta_right_m,
                        )
                        x, y, theta = self._odom_x, self._odom_y, self._odom_theta

                    # Read current velocity for twist
                    with self._cmd_lock:
                        linear = self._cmd_linear
                        angular = self._cmd_angular

                    quat = self._euler_to_quaternion(0, 0, theta)

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

                    self._mock.inject_message("/odom_raw", odom)

                self._last_left_mm = left_mm
                self._last_right_mm = right_mm

                self._record_success()

            except Exception as e:
                self._record_error(e, "odom_loop")

            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _velocity_loop(self) -> None:
        """Send motor commands at cmd_rate_hz."""
        period = 1.0 / self._cmd_rate
        cmd_duration = period  # Each command covers one period

        while self._running:
            loop_start = time.time()

            if not self._serial_healthy:
                time.sleep(period)
                continue

            try:
                with self._cmd_lock:
                    linear = self._cmd_linear
                    angular = self._cmd_angular

                if linear == 0.0 and angular == 0.0:
                    with self._serial_lock:
                        self._transport.stop_motors()
                else:
                    left_mps, right_mps = velocity_to_wheel_speeds(linear, angular)
                    left_mm, right_mm, speed_mms = wheel_speeds_to_motor_command(
                        left_mps, right_mps, cmd_duration,
                    )
                    with self._serial_lock:
                        self._transport.set_motor(left_mm, right_mm, speed_mms)

                self._record_success()

            except Exception as e:
                self._record_error(e, "velocity_loop")

            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _sensor_loop(self) -> None:
        """Read bumpers at sensor_rate_hz, battery every ~5 seconds.

        Also owns reconnection: when serial is unhealthy, attempts
        reconnect with exponential backoff (0.5s → 10s cap).
        """
        period = 1.0 / self._sensor_rate
        battery_interval = 5.0  # seconds between battery reads
        last_battery_time = 0.0
        reconnect_delay = 0.5   # exponential backoff start

        while self._running:
            loop_start = time.time()

            # Reconnection logic — this thread owns it
            if not self._serial_healthy:
                logger.info("Serial unhealthy, waiting %.1fs before reconnect...", reconnect_delay)
                time.sleep(reconnect_delay)
                if self._reconnect():
                    reconnect_delay = 0.5
                else:
                    reconnect_delay = min(reconnect_delay * 2, 10.0)
                continue

            try:
                with self._serial_lock:
                    bumper_data = self._transport.get_digital_sensors()
                    analog_data = self._transport.get_analog_sensors()

                self._record_success()

                # Cliff detection triggers bumper emergency
                if analog_data.cliff_detected:
                    logger.warning("Cliff detected! L=%dmm R=%dmm",
                                   analog_data.left_drop_mm, analog_data.right_drop_mm)
                    bumper_data = NeatoBumperData(
                        left_side_bumper=bumper_data.left_side_bumper,
                        left_front_bumper=bumper_data.left_front_bumper or (analog_data.left_drop_mm > analog_data.CLIFF_THRESHOLD_MM),
                        right_side_bumper=bumper_data.right_side_bumper,
                        right_front_bumper=bumper_data.right_front_bumper or (analog_data.right_drop_mm > analog_data.CLIFF_THRESHOLD_MM),
                        left_wheel_drop=bumper_data.left_wheel_drop,
                        right_wheel_drop=bumper_data.right_wheel_drop,
                    )

                with self._bumper_lock:
                    self._bumper_data = bumper_data

                self._mock.inject_message("/bumpers", bumper_data)

                # Battery on wall-clock interval
                if time.time() - last_battery_time >= battery_interval:
                    with self._serial_lock:
                        battery_data = self._transport.get_charger()

                    with self._bumper_lock:
                        self._battery_data = battery_data

                    self._mock.inject_message("/battery", battery_data)
                    last_battery_time = time.time()

            except Exception as e:
                self._record_error(e, "sensor_loop")

            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    @property
    def bumper_data(self):
        """Latest bumper reading (thread-safe)."""
        with self._bumper_lock:
            return self._bumper_data

    @property
    def battery_data(self):
        """Latest battery reading (thread-safe)."""
        with self._bumper_lock:
            return self._battery_data

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


class NeatoPublisher:
    """
    Publisher that intercepts messages for Neato motor control.

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
