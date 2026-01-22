"""
XRCE-DDS node abstraction for Nimbus.

This module provides topic subscription and publishing via:
- XRCENode: Pure Python XRCE-DDS agent for ESP32 communication
- MockNimbusNode: Mock node for testing without hardware

No ROS2 dependencies - communicates directly with ESP32 via XRCE-DDS.
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


class MockNimbusNode:
    """
    Mock node for testing without hardware.

    Provides the same interface as XRCENode but doesn't
    require the ESP32 to be connected.
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


class XRCENode:
    """
    XRCE-DDS node for ROS2-free communication with ESP32.

    Uses a pure Python XRCE-DDS agent to communicate directly with the
    Yahboom ESP32's Micro-ROS firmware without requiring ROS2 or Docker.

    Usage:
        node = XRCENode(port=8888)
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
        name: str = "nimbus_xrce",
        port: int = 8888,
    ):
        """
        Initialize the XRCENode.

        Args:
            name: Node name (for logging)
            port: UDP port to listen on (ESP32 connects to this)
        """
        self._name = name
        self._port = port

        self._agent = None
        self._buffers: dict[str, TopicBuffer] = {}
        self._publishers: dict[str, "XRCEPublisher"] = {}
        self._running = False

    def start(self) -> None:
        """
        Start the XRCE agent and wait for ESP32 to connect.
        """
        from nimbus.core.xrce import XRCEAgent

        if self._running:
            return

        # Create and start agent
        self._agent = XRCEAgent(bind_port=self._port)
        if not self._agent.start():
            raise RuntimeError(
                f"Failed to start XRCE agent on port {self._port}. "
                "Check that the port is not already in use."
            )

        self._running = True

        # Wait for ESP32 to connect
        if not self._agent.wait_for_connection(timeout=15.0):
            print(f"Waiting for ESP32 connection on port {self._port}...")
            # Keep running - ESP32 may connect later

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
            msg_type: Message type (ignored for XRCE mode, auto-detected)
            buffer_size: Number of messages to buffer
            callback: Optional callback for each message

        Returns:
            TopicBuffer for reading messages
        """
        if self._agent is None:
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

        self._agent.subscribe(topic, _buffer_callback)
        return buffer

    def publisher(self, topic: str, msg_type: type) -> "XRCEPublisher":
        """
        Get or create a publisher for a topic.

        Args:
            topic: Topic name (e.g., "/cmd_vel")
            msg_type: Message type (ignored for XRCE mode)

        Returns:
            XRCEPublisher for publishing messages
        """
        if self._agent is None:
            raise RuntimeError("Node not started. Call start() first.")

        if topic not in self._publishers:
            self._publishers[topic] = XRCEPublisher(self._agent, topic)
        return self._publishers[topic]

    def get_buffer(self, topic: str) -> Optional[TopicBuffer]:
        """Get the buffer for a subscribed topic."""
        return self._buffers.get(topic)

    def shutdown(self) -> None:
        """
        Stop the agent and cleanup resources.
        """
        self._running = False

        if self._agent:
            self._agent.stop()
            self._agent = None

    @property
    def is_running(self) -> bool:
        """Check if the node is running."""
        return self._running and self._agent is not None and self._agent.is_connected


class XRCEPublisher:
    """
    Publisher for XRCENode that wraps XRCE agent publishing.

    Provides the same interface as ROS2 publishers.
    """

    def __init__(self, agent, topic: str):
        self._agent = agent
        self._topic = topic
        self.published_messages: list[Any] = []

    def publish(self, msg: Any) -> None:
        """
        Publish a message.

        Args:
            msg: XRCE Twist message to publish
        """
        from nimbus.core.xrce.messages import Twist

        # Serialize the message
        if hasattr(msg, 'linear') and hasattr(msg, 'angular'):
            if not isinstance(msg, Twist):
                # Convert to XRCE Twist format
                xrce_msg = Twist.create(
                    linear_x=float(msg.linear.x),
                    angular_z=float(msg.angular.z)
                )
                data = xrce_msg.serialize()
            else:
                data = msg.serialize()
        elif hasattr(msg, 'serialize'):
            data = msg.serialize()
        else:
            raise ValueError(f"Cannot serialize message type: {type(msg)}")

        self._agent.publish(self._topic, data)
        self.published_messages.append(msg)

    @property
    def topic(self) -> str:
        """Get the topic name."""
        return self._topic

    @property
    def last_message(self) -> Optional[Any]:
        """Get the last published message."""
        return self.published_messages[-1] if self.published_messages else None
