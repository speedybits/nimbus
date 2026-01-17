"""
Lightweight ROS2 topic wrapper for Nimbus.

This module provides a thin abstraction over rclpy that:
- Handles ROS2 initialization and shutdown
- Provides simple topic subscription with buffering
- Manages publishers
- Runs the ROS2 spinner in a background thread

No ROS2 complexity leaks outside this module.
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Generic, Optional, TypeVar
from collections import deque
import threading
import time

# Type variable for generic message types
T = TypeVar('T')


@dataclass
class TopicBuffer(Generic[T]):
    """
    Thread-safe buffer for ROS2 topic messages.

    Stores the most recent N messages from a topic.
    """

    capacity: int = 1
    _buffer: deque = field(default_factory=deque)
    _lock: threading.Lock = field(default_factory=threading.Lock)
    _last_update: float = field(default=0.0)

    def push(self, msg: T) -> None:
        """Add a message to the buffer."""
        with self._lock:
            if len(self._buffer) >= self.capacity:
                self._buffer.popleft()
            self._buffer.append(msg)
            self._last_update = time.time()

    def latest(self) -> Optional[T]:
        """Get the most recent message, or None if empty."""
        with self._lock:
            return self._buffer[-1] if self._buffer else None

    def all(self) -> list[T]:
        """Get all buffered messages (oldest first)."""
        with self._lock:
            return list(self._buffer)

    def clear(self) -> None:
        """Clear all buffered messages."""
        with self._lock:
            self._buffer.clear()

    @property
    def age(self) -> float:
        """Seconds since last message (infinity if no messages)."""
        with self._lock:
            if self._last_update == 0.0:
                return float('inf')
            return time.time() - self._last_update

    @property
    def is_stale(self) -> bool:
        """Check if data is stale (> 1 second old)."""
        return self.age > 1.0


class NimbusNode:
    """
    Lightweight ROS2 node wrapper.

    Provides simple topic subscription and publishing without
    exposing ROS2 internals to the rest of the codebase.

    Usage:
        node = NimbusNode("nimbus")
        node.start()

        scan_buffer = node.subscribe("/scan", LaserScan)
        cmd_pub = node.publisher("/cmd_vel", Twist)

        # Later...
        latest_scan = scan_buffer.latest()
        cmd_pub.publish(twist_msg)

        node.shutdown()
    """

    def __init__(self, name: str = "nimbus"):
        self._name = name
        self._node: Any = None
        self._executor: Any = None
        self._spin_thread: Optional[threading.Thread] = None
        self._buffers: dict[str, TopicBuffer] = {}
        self._publishers: dict[str, Any] = {}
        self._running = False
        self._rclpy_initialized = False

    def start(self) -> None:
        """
        Initialize ROS2 and start the background spinner.

        Call this before subscribing or publishing.
        """
        try:
            import rclpy
            from rclpy.node import Node as ROS2Node
            from rclpy.executors import SingleThreadedExecutor
        except ImportError as e:
            raise RuntimeError(
                "ROS2 (rclpy) not available. "
                "Ensure ROS2 Humble is sourced: source /opt/ros/humble/setup.bash"
            ) from e

        if not rclpy.ok():
            rclpy.init()
            self._rclpy_initialized = True

        self._node = ROS2Node(self._name)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._running = True
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self) -> None:
        """Background spinner thread."""
        import rclpy
        while self._running and rclpy.ok():
            self._executor.spin_once(timeout_sec=0.01)

    def subscribe(
        self,
        topic: str,
        msg_type: type,
        buffer_size: int = 1,
        callback: Optional[Callable[[Any], None]] = None
    ) -> TopicBuffer:
        """
        Subscribe to a topic.

        Args:
            topic: Topic name (e.g., "/scan")
            msg_type: ROS2 message type (e.g., LaserScan)
            buffer_size: Number of messages to buffer
            callback: Optional callback for each message

        Returns:
            TopicBuffer for reading messages
        """
        if self._node is None:
            raise RuntimeError("Node not started. Call start() first.")

        buffer = TopicBuffer(capacity=buffer_size)
        self._buffers[topic] = buffer

        def _callback(msg):
            buffer.push(msg)
            if callback:
                try:
                    callback(msg)
                except Exception:
                    pass  # Don't crash ROS spinner on callback errors

        self._node.create_subscription(msg_type, topic, _callback, 10)
        return buffer

    def publisher(self, topic: str, msg_type: type) -> Any:
        """
        Get or create a publisher for a topic.

        Args:
            topic: Topic name (e.g., "/cmd_vel")
            msg_type: ROS2 message type (e.g., Twist)

        Returns:
            ROS2 publisher
        """
        if self._node is None:
            raise RuntimeError("Node not started. Call start() first.")

        if topic not in self._publishers:
            self._publishers[topic] = self._node.create_publisher(msg_type, topic, 10)
        return self._publishers[topic]

    def get_buffer(self, topic: str) -> Optional[TopicBuffer]:
        """Get the buffer for a subscribed topic."""
        return self._buffers.get(topic)

    def shutdown(self) -> None:
        """
        Gracefully shutdown ROS2.

        Stops the spinner and cleans up resources.
        """
        self._running = False

        if self._spin_thread:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None

        if self._node:
            self._node.destroy_node()
            self._node = None

        if self._rclpy_initialized:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()

    @property
    def is_running(self) -> bool:
        """Check if the node is running."""
        return self._running and self._node is not None


class MockNimbusNode:
    """
    Mock node for testing without ROS2.

    Provides the same interface as NimbusNode but doesn't
    require ROS2 to be running.
    """

    def __init__(self, name: str = "nimbus_mock"):
        self._name = name
        self._buffers: dict[str, TopicBuffer] = {}
        self._publishers: dict[str, MockPublisher] = {}
        self._running = False

    def start(self) -> None:
        """Start the mock node."""
        self._running = True

    def subscribe(
        self,
        topic: str,
        msg_type: type,
        buffer_size: int = 1,
        callback: Optional[Callable[[Any], None]] = None
    ) -> TopicBuffer:
        """Subscribe to a topic (returns empty buffer)."""
        buffer = TopicBuffer(capacity=buffer_size)
        self._buffers[topic] = buffer
        return buffer

    def publisher(self, topic: str, msg_type: type) -> "MockPublisher":
        """Get a mock publisher."""
        if topic not in self._publishers:
            self._publishers[topic] = MockPublisher(topic)
        return self._publishers[topic]

    def inject_message(self, topic: str, msg: Any) -> None:
        """Inject a message into a topic buffer (for testing)."""
        if topic in self._buffers:
            self._buffers[topic].push(msg)

    def get_buffer(self, topic: str) -> Optional[TopicBuffer]:
        """Get the buffer for a topic."""
        return self._buffers.get(topic)

    def shutdown(self) -> None:
        """Shutdown the mock node."""
        self._running = False

    @property
    def is_running(self) -> bool:
        """Check if running."""
        return self._running


class MockPublisher:
    """Mock publisher for testing."""

    def __init__(self, topic: str):
        self.topic = topic
        self.published_messages: list[Any] = []

    def publish(self, msg: Any) -> None:
        """Record a published message."""
        self.published_messages.append(msg)

    def clear(self) -> None:
        """Clear published messages."""
        self.published_messages.clear()

    @property
    def last_message(self) -> Optional[Any]:
        """Get the last published message."""
        return self.published_messages[-1] if self.published_messages else None


class DirectNode:
    """
    Direct XRCE-DDS node for ROS2-free communication with ESP32.

    Provides the same interface as NimbusNode but communicates directly
    with the Yahboom ESP32's Micro-ROS firmware without requiring ROS2
    or a Micro-ROS agent.

    Usage:
        node = DirectNode(esp32_ip="192.168.1.100")
        node.start()

        scan_buffer = node.subscribe("/scan", LaserScan)
        cmd_pub = node.publisher("/cmd_vel", Twist)

        # Later...
        latest_scan = scan_buffer.latest()
        cmd_pub.publish(twist_msg)

        node.shutdown()
    """

    def __init__(
        self,
        name: str = "nimbus_direct",
        transport: str = "udp",
        esp32_ip: str = "",
        port: int = 8090,
        device: str = "/dev/ttyACM0",
        baudrate: int = 115200,
    ):
        """
        Initialize the DirectNode.

        Args:
            name: Node name (for logging)
            transport: "udp" for WiFi or "serial" for USB
            esp32_ip: ESP32 IP address (for UDP mode)
            port: UDP port (default 8090)
            device: Serial device (for serial mode)
            baudrate: Serial baud rate
        """
        self._name = name
        self._transport_type = transport
        self._esp32_ip = esp32_ip
        self._port = port
        self._device = device
        self._baudrate = baudrate

        self._client = None
        self._buffers: dict[str, TopicBuffer] = {}
        self._publishers: dict[str, "DirectPublisher"] = {}
        self._running = False

    def start(self) -> None:
        """
        Connect to the ESP32 and start receiving data.
        """
        from nimbus.core.direct import XRCEClient, UDPTransport, SerialTransport
        from nimbus.core.direct.transport import UDPConfig, SerialConfig

        if self._running:
            return

        # Create transport
        if self._transport_type == "udp":
            config = UDPConfig(
                local_port=self._port,
                remote_ip=self._esp32_ip,
                remote_port=self._port
            )
            transport = UDPTransport(config)
        else:
            config = SerialConfig(
                device=self._device,
                baudrate=self._baudrate
            )
            transport = SerialTransport(config)

        # Create and start client
        self._client = XRCEClient(transport)
        if not self._client.start(timeout=10.0):
            raise RuntimeError(
                f"Failed to connect to ESP32 via {self._transport_type}. "
                f"Check that the robot is powered on and {'connected to WiFi' if self._transport_type == 'udp' else 'connected via USB'}."
            )

        self._running = True

    def subscribe(
        self,
        topic: str,
        msg_type: type,
        buffer_size: int = 1,
        callback: Optional[Callable[[Any], None]] = None
    ) -> TopicBuffer:
        """
        Subscribe to a topic.

        Args:
            topic: Topic name (e.g., "/scan")
            msg_type: Message type (ignored for direct mode, auto-detected)
            buffer_size: Number of messages to buffer
            callback: Optional callback for each message

        Returns:
            TopicBuffer for reading messages
        """
        if self._client is None:
            raise RuntimeError("Node not started. Call start() first.")

        buffer = TopicBuffer(capacity=buffer_size)
        self._buffers[topic] = buffer

        def _buffer_callback(msg):
            buffer.push(msg)
            if callback:
                try:
                    callback(msg)
                except Exception:
                    pass

        self._client.subscribe(topic, _buffer_callback)
        return buffer

    def publisher(self, topic: str, msg_type: type) -> "DirectPublisher":
        """
        Get or create a publisher for a topic.

        Args:
            topic: Topic name (e.g., "/cmd_vel")
            msg_type: Message type (ignored for direct mode)

        Returns:
            DirectPublisher for publishing messages
        """
        if self._client is None:
            raise RuntimeError("Node not started. Call start() first.")

        if topic not in self._publishers:
            self._publishers[topic] = DirectPublisher(self._client, topic)
        return self._publishers[topic]

    def get_buffer(self, topic: str) -> Optional[TopicBuffer]:
        """Get the buffer for a subscribed topic."""
        return self._buffers.get(topic)

    def shutdown(self) -> None:
        """
        Disconnect and cleanup resources.
        """
        self._running = False

        if self._client:
            self._client.stop()
            self._client = None

    @property
    def is_running(self) -> bool:
        """Check if the node is running."""
        return self._running and self._client is not None and self._client.is_connected


class DirectPublisher:
    """
    Publisher for DirectNode that wraps XRCE client publishing.

    Provides the same interface as ROS2 publishers.
    """

    def __init__(self, client, topic: str):
        self._client = client
        self._topic = topic
        self.published_messages: list[Any] = []

    def publish(self, msg: Any) -> None:
        """
        Publish a message.

        Args:
            msg: Message to publish (can be direct message or ROS2 Twist)
        """
        from nimbus.core.direct.messages import Twist

        # Convert to direct message format if needed
        if hasattr(msg, 'linear') and hasattr(msg, 'angular'):
            # Looks like a Twist message (ROS2 or direct)
            if not isinstance(msg, Twist):
                # Convert ROS2 Twist to direct Twist
                direct_msg = Twist.create(
                    linear_x=float(msg.linear.x),
                    angular_z=float(msg.angular.z)
                )
                data = direct_msg.serialize()
            else:
                data = msg.serialize()
        elif hasattr(msg, 'serialize'):
            data = msg.serialize()
        else:
            raise ValueError(f"Cannot serialize message type: {type(msg)}")

        self._client.publish(self._topic, data)
        self.published_messages.append(msg)

    @property
    def topic(self) -> str:
        """Get the topic name."""
        return self._topic

    @property
    def last_message(self) -> Optional[Any]:
        """Get the last published message."""
        return self.published_messages[-1] if self.published_messages else None
