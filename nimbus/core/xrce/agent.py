"""
Pure Python XRCE-DDS Agent for ESP32 communication.

This module implements an XRCE-DDS agent that replaces the Docker-based
Micro-ROS agent. It communicates directly with the ESP32's Micro-ROS
client firmware over UDP.

Protocol Flow:
    ESP32 (Client)                    Python Agent
         │                                 │
         │──── CREATE_CLIENT ─────────────►│  Session establishment
         │◄─── STATUS_AGENT ──────────────│
         │                                 │
         │──── TIMESTAMP ─────────────────►│  Time synchronization
         │◄─── TIMESTAMP_REPLY ───────────│
         │                                 │
         │──── CREATE (Participant) ──────►│  Entity creation
         │◄─── STATUS OK ─────────────────│
         │──── CREATE (Topic) ────────────►│
         │◄─── STATUS OK ─────────────────│
         │──── CREATE (DataWriter) ───────►│  /scan, /odom_raw
         │◄─── STATUS OK ─────────────────│
         │──── CREATE (DataReader) ───────►│  /cmd_vel
         │◄─── STATUS OK ─────────────────│
         │                                 │
         │──── WRITE_DATA (/scan) ────────►│  Sensor data flow
         │──── WRITE_DATA (/odom_raw) ────►│
         │                                 │
         │◄─── DATA (/cmd_vel) ───────────│  Command flow
         │                                 │
         │◄───► HEARTBEAT / ACKNACK ──────│  Reliability

Usage:
    agent = XRCEAgent(bind_port=8888)
    agent.start()

    agent.subscribe("/scan", on_scan_callback)
    agent.subscribe("/odom_raw", on_odom_callback)

    agent.publish("/cmd_vel", twist_msg.serialize())

    agent.stop()
"""

import struct
import time
import threading
from dataclasses import dataclass, field
from typing import Optional, Callable, Dict, Any, List

from .protocol import (
    SubmessageId, ObjectKind, StreamId,
    MessageHeader, SubmessageHeader, ObjectId,
    parse_message, ParsedSubmessage,
    build_status_agent, build_timestamp_reply, build_status_ok, build_acknack, build_data,
    build_heartbeat_probe,
)
from .session import SessionManager
from .entities import EntityManager, normalize_topic_name
from .transport import UDPTransport, UDPConfig
from .messages import LaserScan, Odometry, Twist, MESSAGE_TYPES
from .logger import xrce_log, LogLevel
from .assertions import assert_comm


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


class XRCEAgent:
    """
    Pure Python XRCE-DDS agent for ESP32 communication.

    Acts as an XRCE agent that the ESP32's Micro-ROS client connects to.
    Handles the full protocol lifecycle:
    - Session establishment (CREATE_CLIENT, TIMESTAMP)
    - Entity creation (CREATE for participants, topics, datawriters, datareaders)
    - Data flow (WRITE_DATA from ESP32, DATA to ESP32)
    - Reliability (HEARTBEAT, ACKNACK)

    Thread-safe: Can be used from multiple threads.
    """

    # Timeouts and intervals
    RECEIVE_TIMEOUT = 0.1       # Receive poll interval
    POLL_INTERVAL = 0.001       # Background thread poll interval

    # Silence detection (graduated response)
    SILENCE_WARNING_TIMEOUT = 2.0    # Seconds before warning
    SILENCE_STALE_TIMEOUT = 3.0      # Seconds before "stale" state
    SILENCE_DISCONNECT_TIMEOUT = 5.0 # Seconds before disconnect

    # Active keepalive probing
    PROBE_INTERVAL = 2.0        # Send probe every 2s when idle
    MAX_PROBE_FAILURES = 3      # Disconnect after 3 failed probes

    def __init__(self, bind_port: int = 8888):
        """
        Initialize the XRCE agent.

        Args:
            bind_port: UDP port to bind to (ESP32 sends to this port)
        """
        self._config = UDPConfig(bind_port=bind_port)
        self._transport = UDPTransport(self._config)
        self._session = SessionManager()
        self._entities = EntityManager()

        # Subscriptions (topic -> info)
        self._subscriptions: Dict[str, SubscriptionInfo] = {}
        self._publishers: Dict[str, PublisherInfo] = {}

        # Threading
        self._running = False
        self._recv_thread: Optional[threading.Thread] = None
        self._lock = threading.RLock()

        # Client address (learned from first packet)
        self._client_addr: Optional[tuple] = None

        # Fragment reassembly buffer
        self._fragment_buffer: bytes = b''
        self._fragment_count: int = 0
        self._fragment_start_time: float = 0.0

        # Heartbeat monitoring
        self._last_message_time: float = 0.0

        # Active keepalive probing state
        self._last_probe_time: float = 0.0
        self._probe_failures: int = 0

        # Duplicate CREATE_CLIENT detection
        self._last_create_client_time: float = 0.0
        self._last_create_client_key: Optional[bytes] = None

    def start(self) -> bool:
        """
        Start the agent: bind port and begin listening.

        Returns:
            True if started successfully
        """
        with self._lock:
            if self._running:
                return True

            if not self._transport.open():
                return False

            self._running = True
            self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self._recv_thread.start()
            return True

    def stop(self) -> None:
        """Stop the agent and cleanup resources."""
        with self._lock:
            self._running = False

            if self._recv_thread:
                self._recv_thread.join(timeout=1.0)
                self._recv_thread = None

            self._transport.close()
            self._session.reset()
            self._entities.clear()
            self._subscriptions.clear()
            self._publishers.clear()
            self._client_addr = None

    def subscribe(
        self,
        topic: str,
        callback: Optional[Callable[[Any], None]] = None
    ) -> bool:
        """
        Subscribe to a topic.

        When the ESP32 publishes to this topic (via WRITE_DATA), the callback
        will be invoked with the deserialized message.

        Args:
            topic: Topic name (e.g., "/scan", "/odom_raw")
            callback: Optional callback for each message

        Returns:
            True if subscription registered successfully
        """
        normalized = normalize_topic_name(topic)

        with self._lock:
            if normalized in self._subscriptions:
                # Update callback
                self._subscriptions[normalized].callback = callback
                return True

            # Get message type
            msg_type = MESSAGE_TYPES.get(topic) or MESSAGE_TYPES.get(normalized)
            if not msg_type:
                xrce_log(f"[yellow]Unknown topic type: {topic}[/yellow]", level=LogLevel.NORMAL)
                return False

            self._subscriptions[normalized] = SubscriptionInfo(
                topic=normalized,
                callback=callback,
                msg_type=msg_type
            )
            return True

    def publish(self, topic: str, data: bytes) -> bool:
        """
        Publish data to a topic.

        Sends DATA submessage to the ESP32's datareader for this topic.

        Args:
            topic: Topic name (e.g., "/cmd_vel")
            data: Serialized message data (CDR with encapsulation)

        Returns:
            True if published successfully
        """
        normalized = normalize_topic_name(topic)

        with self._lock:
            if not self._client_addr:
                assert_comm(
                    "PUB1", False,
                    f"Cannot publish {normalized}: ESP32 not connected",
                    "yellow", 5.0
                )
                return False

            # Setup publisher tracking
            if normalized not in self._publishers:
                msg_type = MESSAGE_TYPES.get(topic) or MESSAGE_TYPES.get(normalized)
                if not msg_type:
                    xrce_log(f"[yellow]Unknown topic type: {topic}[/yellow]", level=LogLevel.NORMAL)
                    return False
                self._publishers[normalized] = PublisherInfo(
                    topic=normalized,
                    msg_type=msg_type
                )

            # Find ESP32's datareader for this topic
            dr_id = self._entities.get_datareader_for_topic(normalized)
            if dr_id is None:
                # ESP32 hasn't created a datareader for this topic yet
                subscribed = self._entities.get_subscribed_topics()
                assert_comm(
                    "PUB2", False,
                    f"ESP32 not subscribed to {normalized}",
                    "red", 5.0,
                    xrce_detail=f"ESP32 subscribed topics: {subscribed}"
                )
                return False

            # Build and send DATA message
            seq = self._session.next_sequence()
            msg = build_data(dr_id, data, seq)
            success = self._transport.send(msg, self._client_addr)

            if success:
                self._publishers[normalized].message_count += 1
                xrce_log(f"[green]Published to {normalized}: {len(data)} bytes[/green]", level=LogLevel.DEBUG)
            else:
                assert_comm(
                    "PUB3", False,
                    f"Send failed for {normalized}",
                    "red", 5.0
                )

            return success

    def get_latest(self, topic: str) -> Optional[Any]:
        """Get the latest received message for a topic."""
        normalized = normalize_topic_name(topic)
        with self._lock:
            sub = self._subscriptions.get(normalized)
            if sub:
                return sub.last_data
            return None

    def get_data_age(self, topic: str) -> float:
        """Get the age of the latest data for a topic (seconds)."""
        normalized = normalize_topic_name(topic)
        with self._lock:
            sub = self._subscriptions.get(normalized)
            if sub and sub.last_update > 0:
                return time.time() - sub.last_update
            return float('inf')

    def wait_for_connection(self, timeout: float = 10.0) -> bool:
        """
        Wait for ESP32 to connect.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if connected, False if timeout
        """
        start = time.time()
        while time.time() - start < timeout:
            if self.is_connected:
                return True
            time.sleep(0.1)
        # Fire assertion on timeout
        assert_comm(
            "S1", False,
            "ESP32 not responding. Please power cycle the robot.",
            "bold red", 30.0,
            level=LogLevel.CRITICAL
        )
        return False

    @property
    def is_connected(self) -> bool:
        """Check if ESP32 is connected."""
        return self._running and self._session.is_connected

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
        """Get agent statistics."""
        with self._lock:
            stats = {
                "connected": self.is_connected,
                "client_address": self._client_addr,
                "subscriptions": {},
                "publishers": {},
                "esp32_topics": {
                    "publishes": list(self._entities.get_published_topics()),
                    "subscribes": list(self._entities.get_subscribed_topics()),
                }
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

    # =========================================================================
    # Internal: Receive loop
    # =========================================================================

    def _receive_loop(self) -> None:
        """Background thread: receive and process messages."""
        while self._running:
            try:
                result = self._transport.receive(timeout=self.RECEIVE_TIMEOUT)
                if result:
                    data, addr = result
                    self._last_message_time = time.time()
                    self._probe_failures = 0  # Reset on successful receive
                    xrce_log(f"Received {len(data)} bytes from {addr}", level=LogLevel.DEBUG)
                    self._client_addr = addr
                    self._process_message(data)
                elif self._session.is_connected and self._last_message_time > 0:
                    # Graduated silence detection
                    now = time.time()
                    silence = now - self._last_message_time

                    if silence > self.SILENCE_DISCONNECT_TIMEOUT:
                        assert_comm(
                            "S3", False,
                            "Lost connection to ESP32",
                            "bold red", 10.0,
                            level=LogLevel.CRITICAL
                        )
                    elif silence > self.SILENCE_STALE_TIMEOUT:
                        assert_comm(
                            "S4", False,
                            f"ESP32 silent for {silence:.1f}s",
                            "yellow", 2.0,
                            level=LogLevel.NORMAL
                        )
                    elif silence > self.SILENCE_WARNING_TIMEOUT:
                        assert_comm(
                            "S4", False,
                            f"ESP32 silent for {silence:.1f}s",
                            "yellow", 2.0,
                            level=LogLevel.NORMAL
                        )

                    # Active keepalive probing when connected but idle
                    if self._client_addr and now - self._last_probe_time > self.PROBE_INTERVAL:
                        self._send_keepalive_probe()
                        self._last_probe_time = now

            except Exception as e:
                if self._running:
                    assert_comm("T2", False, f"Receive loop error: {e}", "yellow", 5.0, level=LogLevel.DEBUG)

            time.sleep(self.POLL_INTERVAL)

    def _process_message(self, data: bytes) -> None:
        """Process a received XRCE message."""
        if len(data) < 4:
            assert_comm("P1", False, f"Malformed packet: {len(data)} bytes", "yellow", 5.0, level=LogLevel.DEBUG)
            return

        parsed = parse_message(data)
        if not parsed:
            return

        stream_id = parsed.header.stream_id
        seq_num = parsed.header.sequence_nr

        for submsg in parsed.submessages:
            try:
                self._handle_submessage(submsg, seq_num)
            except Exception as e:
                assert_comm(
                    "P3", False,
                    f"Error handling {submsg.submessage_id.name}: {e}",
                    "yellow", 5.0,
                    level=LogLevel.DEBUG
                )

        # Send ACKNACK for reliable stream messages (stream >= 0x80)
        if stream_id >= 0x80:
            self._send_acknack(seq_num)

    def _handle_submessage(self, submsg: ParsedSubmessage, seq_num: int) -> None:
        """Route submessage to appropriate handler."""
        submsg_id = submsg.submessage_id

        if submsg_id == SubmessageId.CREATE_CLIENT:
            self._handle_create_client(submsg)

        elif submsg_id == SubmessageId.TIMESTAMP:
            self._handle_timestamp(submsg)

        elif submsg_id == SubmessageId.CREATE:
            # Skip duplicate/retransmitted CREATE messages
            if not self._session.is_duplicate(submsg_id, seq_num):
                self._handle_create(submsg)

        elif submsg_id == SubmessageId.WRITE_DATA:
            self._handle_write_data(submsg)

        elif submsg_id == SubmessageId.HEARTBEAT:
            self._handle_heartbeat(submsg)

        elif submsg_id == SubmessageId.READ_DATA:
            # Skip duplicate/retransmitted READ_DATA messages
            if not self._session.is_duplicate(submsg_id, seq_num):
                self._handle_read_data(submsg)

        elif submsg_id == SubmessageId.FRAGMENT:
            self._handle_fragment(submsg)

        # ACKNACK, etc. are acknowledged but not specifically handled

    # =========================================================================
    # Internal: Message handlers
    # =========================================================================

    def _handle_create_client(self, submsg: ParsedSubmessage) -> None:
        """
        Handle CREATE_CLIENT from ESP32.

        Responds with STATUS_AGENT to acknowledge the client.
        Handles reconnection by clearing stale state.
        Ignores duplicate CREATE_CLIENT packets (ESP32 retransmits).
        """
        if len(submsg.payload) < 4:
            return

        client_key = submsg.payload[:4]
        now = time.time()

        # Detect duplicate CREATE_CLIENT (ESP32 retransmits within ~100ms)
        if (self._last_create_client_key == client_key and
            now - self._last_create_client_time < 0.5):
            # Duplicate - just resend STATUS_AGENT, don't reset state
            xrce_log("[dim]Duplicate CREATE_CLIENT, resending STATUS_AGENT[/dim]", level=LogLevel.DEBUG)
            msg = build_status_agent(client_key)
            self._transport.send(msg, self._client_addr, max_time=0.5)
            return

        self._last_create_client_time = now
        self._last_create_client_key = client_key

        was_connected = self._session.is_connected
        old_key = self._session.client_key

        # Handle reconnection - clear stale state only for genuine reconnects
        if was_connected:
            if old_key != client_key:
                assert_comm("S2", True, "ESP32 reconnected (new session)", "green", 5.0, level=LogLevel.NORMAL)
            else:
                assert_comm("S6", True, "ESP32 reconnected (same session)", "cyan", 5.0, level=LogLevel.NORMAL)
            self._soft_reset()

        self._session.handle_create_client(client_key)

        # Reset probe timer to prevent probing during handshake
        self._last_probe_time = time.time()

        # Send STATUS_AGENT response with retry
        msg = build_status_agent(client_key)
        success = self._transport.send(msg, self._client_addr, max_time=0.5)
        xrce_log(f"[green]STATUS_AGENT sent (success={success})[/green]", level=LogLevel.DEBUG)

    def _handle_timestamp(self, submsg: ParsedSubmessage) -> None:
        """
        Handle TIMESTAMP from ESP32.

        Responds with TIMESTAMP_REPLY for time synchronization.
        """
        if len(submsg.payload) < 8:
            return

        transmit_ts = struct.unpack('<Q', submsg.payload[:8])[0]
        now_ns = int(time.time() * 1_000_000_000)

        msg = build_timestamp_reply(transmit_ts, now_ns)
        success = self._transport.send(msg, self._client_addr)
        xrce_log(f"[green]TIMESTAMP_REPLY sent (success={success})[/green]", level=LogLevel.DEBUG)

    def _handle_create(self, submsg: ParsedSubmessage) -> None:
        """
        Handle CREATE from ESP32.

        Tracks the created entity and responds with STATUS OK.
        """
        if len(submsg.payload) < 5:
            return

        # Parse object_id and request_id
        raw_obj_id = struct.unpack('<H', submsg.payload[0:2])[0]
        request_id = struct.unpack('<H', submsg.payload[2:4])[0]

        # Object kind is at byte 4
        obj_kind_value = submsg.payload[4]
        try:
            obj_kind = ObjectKind(obj_kind_value)
        except ValueError:
            obj_kind = ObjectKind.INVALID

        # Track the entity
        self._entities.handle_create(raw_obj_id, obj_kind, submsg.payload)

        # Log entity creation
        xrce_log(f"[cyan]CREATE {obj_kind.name} id={raw_obj_id}[/cyan]", level=LogLevel.DEBUG)
        if obj_kind == ObjectKind.DATAREADER:
            topics = self._entities.get_subscribed_topics()
            xrce_log(f"[cyan]  ESP32 now subscribes to: {topics}[/cyan]", level=LogLevel.NORMAL)

        # Send STATUS OK
        seq = self._session.next_sequence()
        msg = build_status_ok(raw_obj_id, request_id, seq)
        success = self._transport.send(msg, self._client_addr)
        xrce_log(f"[cyan]  -> STATUS OK sent (obj={raw_obj_id}, req={request_id}, seq={seq}, success={success})[/cyan]", level=LogLevel.DEBUG)
        xrce_log(f"[dim]     hex: {msg.hex()}[/dim]", level=LogLevel.DEBUG)

    def _handle_write_data(self, submsg: ParsedSubmessage) -> None:
        """
        Handle WRITE_DATA from ESP32.

        Deserializes the message and delivers to subscription callback.

        Note: The ESP32's WRITE_DATA object_ids don't match CREATE object_ids,
        so we detect message type from payload content (frame_id strings).

        WRITE_DATA payload structure:
        - Bytes 0-1: object_id
        - Bytes 2-3: request_id
        - Bytes 4-7: format flags (e.g., 0x0f 0x00 0x00 0x00)
        - Bytes 8+: CDR data (NO encapsulation header)
        """
        if len(submsg.payload) < 12:
            return

        # Extract CDR data (skip object_id + request_id + format_flags = 8 bytes)
        # Note: CDR data has NO encapsulation header, starts directly with message fields
        cdr_data = submsg.payload[8:]

        # Detect topic from payload content
        topic = self._detect_topic_from_payload(submsg.payload)
        if not topic:
            assert_comm(
                "D1", False,
                f"Unknown message type ({len(submsg.payload)} bytes)",
                "yellow", 5.0,
                level=LogLevel.NORMAL
            )
            return
        xrce_log(f"Detected topic: [cyan]{topic}[/cyan]", level=LogLevel.NORMAL)

        # Get subscription
        with self._lock:
            sub = self._subscriptions.get(topic)
            if not sub:
                return

            try:
                # Deserialize directly from CDR (no encapsulation header)
                from .cdr import CDRReader
                reader = CDRReader(cdr_data)
                msg = sub.msg_type.from_cdr(reader)
                sub.last_data = msg
                sub.last_update = time.time()
                sub.message_count += 1
                xrce_log(f"[green]{topic}[/green]: msg #{sub.message_count}, ranges={len(msg.ranges) if hasattr(msg, 'ranges') else 'N/A'}", level=LogLevel.DEBUG)
                callback = sub.callback
            except Exception as e:
                assert_comm(
                    "D2", False,
                    f"Failed to parse {topic}: {e}",
                    "red", 5.0,
                    xrce_detail=f"CDR data: {len(cdr_data)} bytes",
                    level=LogLevel.NORMAL
                )
                return

        # Call callback outside lock
        if callback:
            try:
                callback(msg)
            except Exception:
                pass

    def _detect_topic_from_payload(self, payload: bytes) -> Optional[str]:
        """
        Detect topic type from WRITE_DATA payload content.

        The ESP32's WRITE_DATA uses different object_ids than CREATE,
        so we detect message type by looking for frame_id strings in the payload.

        Args:
            payload: Full WRITE_DATA payload

        Returns:
            Normalized topic name ("/scan", "/odom_raw", etc.) or None
        """
        try:
            # Convert to string for pattern matching
            data = payload.decode('latin-1')

            # Detect by frame_id patterns in CDR data
            if 'odom' in data and 'imu' not in data:
                return '/odom_raw'
            elif 'laser' in data or 'scan' in data:
                return '/scan'
            elif 'imu' in data:
                return '/imu'
            elif 'battery' in data or 'batt' in data:
                return '/battery'

            # Fallback: detect by payload size
            # LaserScan is typically much larger (700+ bytes) due to ranges array
            # Odometry/IMU are smaller (300-400 bytes)
            payload_len = len(payload)
            if payload_len > 600:
                # Large payload - likely LaserScan
                return '/scan'

        except Exception:
            pass

        return None

    def _handle_heartbeat(self, submsg: ParsedSubmessage) -> None:
        """
        Handle HEARTBEAT from ESP32.

        Responds with ACKNACK to acknowledge received messages.
        """
        if len(submsg.payload) < 4:
            return

        # HEARTBEAT contains first_seq and last_seq
        last_seq = struct.unpack('<H', submsg.payload[2:4])[0]
        self._send_acknack(last_seq)

    def _handle_read_data(self, submsg: ParsedSubmessage) -> None:
        """
        Handle READ_DATA from ESP32.

        The ESP32 requests data; we respond with STATUS OK.
        In continuous publishing mode, we don't need to do anything special.
        """
        if len(submsg.payload) < 4:
            return

        raw_obj_id = struct.unpack('<H', submsg.payload[0:2])[0]
        request_id = struct.unpack('<H', submsg.payload[2:4])[0]

        # Just send STATUS OK
        seq = self._session.next_sequence()
        msg = build_status_ok(raw_obj_id, request_id, seq)
        self._transport.send(msg, self._client_addr)

    def _handle_fragment(self, submsg: ParsedSubmessage) -> None:
        """
        Handle FRAGMENT submessage for large messages (e.g., LaserScan).

        Fragments are reassembled and then processed as WRITE_DATA.
        Detection of fragment boundaries:
        - First fragment starts with 0x07 (WRITE_DATA indicator)
        - Continuation fragments start with other bytes
        """
        payload = submsg.payload
        if len(payload) < 8:
            return

        # Check for fragment timeout (stale buffer)
        if self._fragment_buffer and self._fragment_start_time > 0:
            if time.time() - self._fragment_start_time > 3.0:  # Extended for WiFi jitter
                assert_comm(
                    "F1", False,
                    "Fragment timeout: discarding incomplete data",
                    "yellow", 5.0,
                    xrce_detail=f"Buffer: {len(self._fragment_buffer)} bytes, {self._fragment_count} fragments",
                    level=LogLevel.DEBUG
                )
                self._fragment_buffer = b''
                self._fragment_count = 0
                self._fragment_start_time = 0.0

        # Check for fragment buffer overflow
        if len(self._fragment_buffer) > 65536:
            assert_comm("F2", False, "Fragment overflow: discarding", "red", 5.0, level=LogLevel.DEBUG)
            self._fragment_buffer = b''
            self._fragment_count = 0
            self._fragment_start_time = 0.0

        # Check if this is a first fragment (starts with WRITE_DATA indicator 0x07)
        is_first = payload[0] == 0x07

        if is_first:
            # If we have a previous buffer, try to process it
            if self._fragment_buffer and self._fragment_count >= 2:
                self._process_reassembled_fragment()

            # Start new buffer - skip the fragment header (first 12 bytes contain metadata)
            # The actual CDR data starts after: submsg_id(1) + flags(1) + stream(2) + seq(2) + more(6)
            self._fragment_buffer = payload[12:]  # Skip fragment header
            self._fragment_count = 1
            self._fragment_start_time = time.time()
        else:
            # Continuation fragment - append entire payload
            self._fragment_buffer += payload
            self._fragment_count += 1

            # Check if we have enough data for a LaserScan (typically ~2900 bytes)
            if len(self._fragment_buffer) > 2800 and self._fragment_count >= 3:
                self._process_reassembled_fragment()

    def _process_reassembled_fragment(self) -> None:
        """Process a reassembled fragment buffer as LaserScan."""
        if not self._fragment_buffer:
            return

        # The reassembled data should be LaserScan CDR
        # Create a fake WRITE_DATA-like structure for processing
        # We'll directly try to deserialize as LaserScan
        cdr_data = self._fragment_buffer
        self._fragment_buffer = b''
        self._fragment_count = 0
        self._fragment_start_time = 0.0

        # Check if we have a /scan subscription
        with self._lock:
            sub = self._subscriptions.get('/scan')
            if not sub:
                return

            try:
                from .cdr import CDRReader
                reader = CDRReader(cdr_data)
                msg = sub.msg_type.from_cdr(reader)
                sub.last_data = msg
                sub.last_update = time.time()
                sub.message_count += 1
                callback = sub.callback
            except Exception:
                # Deserialization failed - might need format adjustment
                return

        # Call callback outside lock
        if callback:
            try:
                callback(msg)
            except Exception:
                pass

    def _send_acknack(self, seq_num: int) -> None:
        """Send ACKNACK to acknowledge received messages."""
        msg = build_acknack(seq_num + 1)
        success = self._transport.send(msg, self._client_addr)
        xrce_log(f"[dim]ACKNACK sent (next_seq={seq_num + 1}, success={success})[/dim]", level=LogLevel.DEBUG)

    def _send_keepalive_probe(self) -> None:
        """Send HEARTBEAT probe to verify ESP32 connectivity."""
        if not self._client_addr:
            return
        seq = self._session.next_sequence()
        msg = build_heartbeat_probe(seq)
        success = self._transport.send(msg, self._client_addr)
        xrce_log(f"[yellow]KEEPALIVE probe sent (seq={seq}, success={success})[/yellow]", level=LogLevel.DEBUG)
        if not success:
            self._probe_failures += 1
            if self._probe_failures >= self.MAX_PROBE_FAILURES:
                assert_comm(
                    "S5", False,
                    "ESP32 not reachable (probe failed)",
                    "red", 5.0,
                    level=LogLevel.NORMAL
                )

    def _soft_reset(self) -> None:
        """Reset state without stopping agent - for reconnection."""
        self._entities.clear()
        self._fragment_buffer = b''
        self._fragment_count = 0
        self._fragment_start_time = 0.0
        self._probe_failures = 0
        self._last_create_client_time = 0.0
        self._last_create_client_key = None
        self._session.reset()


# =============================================================================
# Convenience factory
# =============================================================================

def create_agent(bind_port: int = 8888) -> XRCEAgent:
    """
    Create an XRCE agent.

    Args:
        bind_port: UDP port to bind to (default 8888)

    Returns:
        XRCEAgent instance
    """
    return XRCEAgent(bind_port=bind_port)
