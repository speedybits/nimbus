"""
Neato node for Nimbus.

Provides the same interface as XRCENode and SimulatorNode but
reads sensors and sends motor commands over serial to a Neato vacuum.

Uses a single serial loop to avoid contention — one thread owns the
serial port and round-robins through operations each tick:
  1. Motor command (every tick — smooth motion)
  2. Odometry read (every tick — accurate tracking)
  3. LIDAR scan (every N ticks — ~2-3 Hz effective)
  4. Bumpers/battery (every M ticks — ~1 Hz)
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
from .transport import NeatoTransport, SerialDisconnectedError, NeatoBumperData, NeatoBatteryData
from .kinematics import velocity_to_wheel_speeds, wheel_speeds_to_motor_command, update_odometry


class NeatoNode:
    """
    Neato serial node — same interface as XRCENode / SimulatorNode.

    A single background thread owns all serial I/O, eliminating lock
    contention. Each loop iteration (~10 Hz target):
      - Always: send motor command + read odometry
      - Every 3rd tick: read LIDAR scan
      - Every 10th tick: read bumpers/sensors
      - Every 50th tick: read battery
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
        self._name = name

        # Internal mock node for buffer management
        self._mock = MockNimbusNode(name)

        # Velocity command state (written by behavior thread, read by serial loop)
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._cmd_lock = threading.Lock()

        # Odometry state
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0
        self._odom_lock = threading.Lock()
        self._last_left_mm = None
        self._last_right_mm = None

        # Latest bumper/battery/cliff state
        self._bumper_data = None
        self._battery_data = None
        self._analog_data = None
        self._bumper_lock = threading.Lock()

        # Serial health tracking
        self._serial_healthy = True
        self._consecutive_errors = 0
        self._max_consecutive_errors = 6

        # Motor state tracking (avoid re-sending stop every tick)
        self._motors_stopped = True

        # Thread control
        self._running = False
        self._loop_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Connect to Neato and start the serial loop."""
        if self._running:
            return

        self._transport.connect()
        self._mock.start()
        self._running = True

        self._loop_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._loop_thread.start()

    def shutdown(self) -> None:
        """Stop motors, join thread, disconnect."""
        self._running = False

        if self._loop_thread:
            self._loop_thread.join(timeout=3.0)
        self._loop_thread = None

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
            self._transport.disconnect()
            self._transport.connect()
            logger.info("Serial reconnect successful")
            self._serial_healthy = True
            self._consecutive_errors = 0
            return True
        except Exception as e:
            logger.warning("Serial reconnect failed: %s", e)
            return False

    # --- Single serial loop ---

    def _serial_loop(self) -> None:
        """Single thread that owns all serial I/O.

        Runs at ~10 Hz base rate. Each tick:
          - Always: motor command + odometry (fast, essential for smooth motion)
          - Every 3rd tick: LIDAR scan (~3 Hz)
          - Every 10th tick: bumpers/cliff (~1 Hz)
          - Every 50th tick: battery (~0.2 Hz)
        """
        tick = 0
        base_period = 0.1  # 10 Hz
        reconnect_delay = 0.5
        cmd_duration = 1.0  # Motor command covers 1s — always in-flight for smooth motion

        while self._running:
            loop_start = time.time()

            # Reconnection when unhealthy
            if not self._serial_healthy:
                logger.info("Serial unhealthy, waiting %.1fs before reconnect...",
                            reconnect_delay)
                time.sleep(reconnect_delay)
                if self._reconnect():
                    reconnect_delay = 0.5
                    tick = 0
                else:
                    reconnect_delay = min(reconnect_delay * 2, 10.0)
                continue

            try:
                # Priority: scan and motor alternate for safety + smooth motion.
                # Each serial command takes ~150-200ms, so actual loop rate
                # is ~3-5 Hz depending on which commands run each tick.
                #
                # Tick pattern (repeats every 4):
                #   0: motor + scan    (safety-critical pair)
                #   1: motor + odom
                #   2: motor + scan
                #   3: motor + odom (+ sensors every 4th cycle)

                self._do_motor_command(cmd_duration)

                if tick % 2 == 0:
                    self._do_scan()
                else:
                    self._do_odometry()

                # Bumpers/cliff every 8th tick
                if tick % 8 == 3:
                    self._do_sensors()

                # Battery every 8th tick (offset from bumpers, isolated from errors)
                if tick % 8 == 7:
                    self._do_battery()

                self._record_success()

            except Exception as e:
                self._record_error(e, "serial_loop")

            tick += 1
            elapsed = time.time() - loop_start
            sleep_time = base_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _do_motor_command(self, cmd_duration: float) -> None:
        """Send current velocity as a motor command."""
        with self._cmd_lock:
            linear = self._cmd_linear
            angular = self._cmd_angular

        if linear == 0.0 and angular == 0.0:
            if not self._motors_stopped:
                self._transport.stop_motors()
                self._motors_stopped = True
        else:
            left_mps, right_mps = velocity_to_wheel_speeds(linear, angular)
            left_mm, right_mm, speed_mms = wheel_speeds_to_motor_command(
                left_mps, right_mps, cmd_duration,
            )
            self._transport.set_motor(left_mm, right_mm, speed_mms)
            self._motors_stopped = False

    def _do_odometry(self) -> None:
        """Read encoders and update odometry."""
        motor_data = self._transport.get_motors()

        left_mm = motor_data.left_wheel_position_mm
        right_mm = motor_data.right_wheel_position_mm

        if self._last_left_mm is not None:
            delta_left_m = (left_mm - self._last_left_mm) / 1000.0
            delta_right_m = (right_mm - self._last_right_mm) / 1000.0

            with self._odom_lock:
                self._odom_x, self._odom_y, self._odom_theta = update_odometry(
                    self._odom_x, self._odom_y, self._odom_theta,
                    delta_left_m, delta_right_m,
                )
                x, y, theta = self._odom_x, self._odom_y, self._odom_theta

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

    def _do_scan(self) -> None:
        """Read LIDAR and inject LaserScan message."""
        scan_data = self._transport.get_lds_scan()

        # Index by explicit angle from hardware.
        # Neato convention: angle 0 = forward, increases CCW.
        ranges = [float("inf")] * 360
        intensities = [0.0] * 360
        for i in range(len(scan_data.angles)):
            angle_idx = scan_data.angles[i]
            if 0 <= angle_idx < 360:
                if scan_data.error_codes[i] != 0 or scan_data.distances_mm[i] == 0:
                    ranges[angle_idx] = float("inf")
                else:
                    ranges[angle_idx] = scan_data.distances_mm[i] / 1000.0
                if i < len(scan_data.intensities):
                    intensities[angle_idx] = float(scan_data.intensities[i])

        scan = LaserScan(
            header=Header(frame_id="laser"),
            angle_min=0.0,
            angle_max=2 * math.pi,
            angle_increment=2 * math.pi / 360,
            time_increment=0.0,
            scan_time=0.33,
            range_min=0.02,
            range_max=5.0,
            ranges=ranges,
            intensities=intensities,
        )

        self._mock.inject_message("/scan", scan)

    def _do_sensors(self) -> None:
        """Read bumpers and cliff sensors."""
        bumper_data = self._transport.get_digital_sensors()
        analog_data = self._transport.get_analog_sensors()

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
            self._analog_data = analog_data

        self._mock.inject_message("/bumpers", bumper_data)
        self._mock.inject_message("/cliff", analog_data)

    def _do_battery(self) -> None:
        """Read battery status."""
        try:
            battery_data = self._transport.get_charger()
        except Exception:
            return

        with self._bumper_lock:
            self._battery_data = battery_data

        self._mock.inject_message("/battery", battery_data)

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

    @property
    def analog_data(self):
        """Latest cliff/analog sensor reading (thread-safe)."""
        with self._bumper_lock:
            return self._analog_data

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
