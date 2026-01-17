"""
High-level XRCE-DDS client for direct ESP32 communication.

This module provides the XRCEClient class, which manages the entire
lifecycle of communicating with the ESP32's Micro-ROS firmware:
- Session establishment
- Entity creation (participants, topics, readers, writers)
- Data reception and publishing
- Background receive thread

Usage:
    client = XRCEClient(UDPTransport(UDPConfig(local_port=8090)))
    client.start()

    client.subscribe("/scan", on_scan_callback)
    client.subscribe("/odom_raw", on_odom_callback)

    client.publish("/cmd_vel", twist_msg.serialize())

    client.stop()
"""

import time
import threading
from typing import Optional, Callable, Dict, Any, List
from dataclasses import dataclass, field
from queue import Queue, Empty

from .transport import Transport, UDPTransport, SerialTransport, UDPConfig, SerialConfig
from .session import (
    SessionState, MessageBuilder, SubmessageId, StatusValue, ObjectKind,
    parse_message, extract_data_from_submessage, extract_status_from_submessage,
    get_xrce_topic_name, ParsedSubmessage
)
from .messages import (
    LaserScan, Odometry, Twist, MESSAGE_TYPES, deserialize_message
)


@dataclass
class SubscriptionInfo:
    """Information about a topic subscription."""
    topic: str
    callback: Optional[Callable[[Any], None]]
    msg_type: type
    last_data: Optional[Any] = None
    last_update: float = 0.0
    message_count: int = 0


@dataclass
class PublisherInfo:
    """Information about a topic publisher."""
    topic: str
    msg_type: type
    message_count: int = 0


class XRCEClient:
    """
    High-level XRCE-DDS client.

    Manages the complete lifecycle of XRCE communication:
    1. Transport connection
    2. Session creation
    3. Entity setup (participant, pub/sub, readers/writers)
    4. Background data reception
    5. Message publishing

    Thread-safe: Can be used from multiple threads.
    """

    # Timeouts and intervals
    CONNECT_TIMEOUT = 5.0       # Seconds to wait for connection
    RECEIVE_TIMEOUT = 0.1       # Receive poll interval
    STATUS_TIMEOUT = 2.0        # Seconds to wait for status response
    POLL_INTERVAL = 0.01        # Background thread poll interval

    def __init__(self, transport: Transport):
        """
        Initialize the XRCE client.

        Args:
            transport: Transport instance (UDP or Serial)
        """
        self._transport = transport
        self._state = SessionState()
        self._builder = MessageBuilder(self._state)

        # Subscriptions and publishers
        self._subscriptions: Dict[str, SubscriptionInfo] = {}
        self._publishers: Dict[str, PublisherInfo] = {}

        # Threading
        self._running = False
        self._recv_thread: Optional[threading.Thread] = None
        self._lock = threading.RLock()

        # Status tracking
        self._pending_status: Dict[int, Optional[StatusValue]] = {}
        self._status_events: Dict[int, threading.Event] = {}

    def start(self, timeout: float = CONNECT_TIMEOUT) -> bool:
        """
        Start the client: connect transport and establish session.

        Args:
            timeout: Maximum time to wait for connection

        Returns:
            True if successfully connected
        """
        with self._lock:
            if self._running:
                return True

            # Open transport
            if not self._transport.open():
                return False

            # Start background receive thread
            self._running = True
            self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self._recv_thread.start()

            # Establish session
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self._establish_session():
                    return True
                time.sleep(0.5)

            # Timeout - cleanup
            self._running = False
            self._recv_thread.join(timeout=1.0)
            self._transport.close()
            return False

    def stop(self) -> None:
        """Stop the client and cleanup resources."""
        with self._lock:
            self._running = False

            if self._recv_thread:
                self._recv_thread.join(timeout=1.0)
                self._recv_thread = None

            self._transport.close()
            self._state = SessionState()
            self._subscriptions.clear()
            self._publishers.clear()

    def _establish_session(self) -> bool:
        """
        Establish XRCE session with the agent.

        Creates:
        - Client session
        - DomainParticipant
        - Publisher (for cmd_vel)
        - Subscriber (for scan, odom)
        """
        # Send CREATE_CLIENT
        msg = self._builder.create_client()
        self._transport.send(msg)

        # Wait for response (agent may not respond, just proceed)
        time.sleep(0.2)

        # Create participant
        msg = self._builder.create_participant()
        if not self._send_and_wait_status(msg):
            # May already exist from previous session
            pass

        # Create publisher and subscriber
        msg = self._builder.create_publisher()
        self._send_and_wait_status(msg, timeout=0.5)

        msg = self._builder.create_subscriber()
        self._send_and_wait_status(msg, timeout=0.5)

        self._state.is_connected = True
        return True

    def _send_and_wait_status(self, msg: bytes, timeout: float = STATUS_TIMEOUT) -> bool:
        """Send a message and wait for status response."""
        seq = self._state.sequence_num - 1  # Get the sequence we just used
        event = threading.Event()

        with self._lock:
            self._status_events[seq] = event
            self._pending_status[seq] = None

        self._transport.send(msg)

        # Wait for status
        if event.wait(timeout=timeout):
            with self._lock:
                status = self._pending_status.pop(seq, None)
                self._status_events.pop(seq, None)
                return status == StatusValue.OK or status == StatusValue.OK_MATCHED
        else:
            with self._lock:
                self._pending_status.pop(seq, None)
                self._status_events.pop(seq, None)
            return False

    def subscribe(
        self,
        topic: str,
        callback: Optional[Callable[[Any], None]] = None
    ) -> bool:
        """
        Subscribe to a topic.

        Args:
            topic: Topic name (e.g., "/scan", "/odom_raw")
            callback: Optional callback for each message

        Returns:
            True if subscription created successfully
        """
        with self._lock:
            if topic in self._subscriptions:
                # Update callback
                self._subscriptions[topic].callback = callback
                return True

            # Get message type
            msg_type = MESSAGE_TYPES.get(topic) or MESSAGE_TYPES.get(get_xrce_topic_name(topic))
            if not msg_type:
                print(f"Unknown topic type: {topic}")
                return False

            # Create topic if not exists
            if topic not in self._state.topics:
                msg = self._builder.create_topic(topic)
                self._send_and_wait_status(msg, timeout=0.5)

            # Create datareader
            msg = self._builder.create_datareader(topic)
            self._send_and_wait_status(msg, timeout=0.5)

            # Store subscription info
            self._subscriptions[topic] = SubscriptionInfo(
                topic=topic,
                callback=callback,
                msg_type=msg_type
            )

            return True

    def publish(self, topic: str, data: bytes) -> bool:
        """
        Publish data to a topic.

        Args:
            topic: Topic name (e.g., "/cmd_vel")
            data: Serialized message data (CDR with encapsulation)

        Returns:
            True if published successfully
        """
        with self._lock:
            # Setup publisher if first time
            if topic not in self._publishers:
                msg_type = MESSAGE_TYPES.get(topic) or MESSAGE_TYPES.get(get_xrce_topic_name(topic))
                if not msg_type:
                    print(f"Unknown topic type: {topic}")
                    return False

                # Create topic if not exists
                if topic not in self._state.topics:
                    msg = self._builder.create_topic(topic)
                    self._send_and_wait_status(msg, timeout=0.5)

                # Create datawriter
                msg = self._builder.create_datawriter(topic)
                self._send_and_wait_status(msg, timeout=0.5)

                self._publishers[topic] = PublisherInfo(
                    topic=topic,
                    msg_type=msg_type
                )

            # Send data
            msg = self._builder.write_data(topic, data)
            success = self._transport.send(msg)

            if success:
                self._publishers[topic].message_count += 1

            return success

    def get_latest(self, topic: str) -> Optional[Any]:
        """Get the latest received message for a topic."""
        with self._lock:
            sub = self._subscriptions.get(topic)
            if sub:
                return sub.last_data
            return None

    def get_data_age(self, topic: str) -> float:
        """Get the age of the latest data for a topic (seconds)."""
        with self._lock:
            sub = self._subscriptions.get(topic)
            if sub and sub.last_update > 0:
                return time.time() - sub.last_update
            return float('inf')

    def request_data(self, topic: str) -> bool:
        """
        Request data from a datareader.

        For topics without continuous streams, this requests a sample.
        """
        with self._lock:
            if topic not in self._state.datareaders:
                return False

            msg = self._builder.read_data(topic)
            return self._transport.send(msg)

    def _receive_loop(self) -> None:
        """Background thread: receive and process messages."""
        while self._running:
            try:
                data = self._transport.receive(timeout=self.RECEIVE_TIMEOUT)
                if data:
                    self._process_message(data)
            except Exception as e:
                if self._running:
                    print(f"Receive error: {e}")

            time.sleep(self.POLL_INTERVAL)

    def _process_message(self, data: bytes) -> None:
        """Process a received XRCE message."""
        parsed = parse_message(data)
        if not parsed:
            return

        for submsg in parsed.submessages:
            if submsg.submessage_id == SubmessageId.STATUS:
                self._handle_status(submsg)
            elif submsg.submessage_id == SubmessageId.DATA:
                self._handle_data(submsg)

    def _handle_status(self, submsg: ParsedSubmessage) -> None:
        """Handle a STATUS submessage."""
        result = extract_status_from_submessage(submsg)
        if not result:
            return

        obj_id, request_id, status = result

        with self._lock:
            # Check if anyone is waiting for this status
            if request_id in self._status_events:
                self._pending_status[request_id] = status
                self._status_events[request_id].set()

    def _handle_data(self, submsg: ParsedSubmessage) -> None:
        """Handle a DATA submessage."""
        cdr_data = extract_data_from_submessage(submsg)
        if not cdr_data:
            return

        # Find which subscription this data belongs to
        # The object_id in the submessage tells us the datareader
        if len(submsg.payload) < 2:
            return

        from .session import ObjectId
        obj_id = ObjectId.unpack(submsg.payload)

        # Find the topic for this datareader
        topic = None
        with self._lock:
            for t, info in self._state.datareaders.items():
                if info.object_id.id == obj_id.id:
                    topic = t
                    break

        if not topic:
            return

        # Get subscription and deserialize
        with self._lock:
            sub = self._subscriptions.get(topic)
            if not sub:
                return

            try:
                # The cdr_data should have encapsulation header
                msg = sub.msg_type.deserialize(cdr_data)
                sub.last_data = msg
                sub.last_update = time.time()
                sub.message_count += 1

                # Call callback outside lock
                callback = sub.callback
            except Exception as e:
                print(f"Deserialize error for {topic}: {e}")
                return

        if callback:
            try:
                callback(msg)
            except Exception as e:
                print(f"Callback error for {topic}: {e}")

    @property
    def is_connected(self) -> bool:
        """Check if connected to the agent."""
        return self._running and self._state.is_connected

    @property
    def subscriptions(self) -> List[str]:
        """List of subscribed topics."""
        with self._lock:
            return list(self._subscriptions.keys())

    @property
    def publishers(self) -> List[str]:
        """List of topics with publishers."""
        with self._lock:
            return list(self._publishers.keys())

    def get_stats(self) -> Dict[str, Any]:
        """Get client statistics."""
        with self._lock:
            stats = {
                "connected": self.is_connected,
                "subscriptions": {},
                "publishers": {},
            }

            for topic, sub in self._subscriptions.items():
                stats["subscriptions"][topic] = {
                    "message_count": sub.message_count,
                    "last_update": sub.last_update,
                    "data_age": time.time() - sub.last_update if sub.last_update > 0 else None
                }

            for topic, pub in self._publishers.items():
                stats["publishers"][topic] = {
                    "message_count": pub.message_count
                }

            return stats


# =============================================================================
# Convenience factory functions
# =============================================================================

def create_udp_client(
    local_port: int = 8090,
    remote_ip: str = "",
    remote_port: int = 8090
) -> XRCEClient:
    """
    Create an XRCE client using UDP transport.

    Args:
        local_port: Local port to bind (default 8090)
        remote_ip: ESP32 IP address (optional, learned from packets)
        remote_port: ESP32 port (default 8090)

    Returns:
        XRCEClient instance
    """
    config = UDPConfig(
        local_port=local_port,
        remote_ip=remote_ip,
        remote_port=remote_port
    )
    transport = UDPTransport(config)
    return XRCEClient(transport)


def create_serial_client(
    device: str = "/dev/ttyACM0",
    baudrate: int = 115200
) -> XRCEClient:
    """
    Create an XRCE client using serial transport.

    Args:
        device: Serial device path
        baudrate: Baud rate (default 115200)

    Returns:
        XRCEClient instance
    """
    config = SerialConfig(
        device=device,
        baudrate=baudrate
    )
    transport = SerialTransport(config)
    return XRCEClient(transport)
