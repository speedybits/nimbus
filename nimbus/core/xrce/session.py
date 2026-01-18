"""
XRCE-DDS Session and stream management.

This module manages the session lifecycle and stream state for
communication with ESP32 Micro-ROS clients.

Session Flow:
    1. ESP32 sends CREATE_CLIENT -> Agent responds with STATUS_AGENT
    2. ESP32 sends TIMESTAMP -> Agent responds with TIMESTAMP_REPLY
    3. ESP32 sends CREATE messages for entities -> Agent responds with STATUS OK
    4. ESP32 starts sending WRITE_DATA with sensor data
    5. Agent can send DATA messages for commands like /cmd_vel
"""

import time
import threading
from dataclasses import dataclass, field
from typing import Dict, Optional, Callable, Any


@dataclass
class StreamState:
    """
    State for a single XRCE stream.

    Tracks sequence numbers and pending messages for reliable delivery.
    """
    stream_id: int
    next_seq_out: int = 0  # Next sequence number to send
    last_seq_in: int = 0   # Last sequence number received

    def next_sequence(self) -> int:
        """Get and increment outbound sequence number."""
        seq = self.next_seq_out
        self.next_seq_out = (self.next_seq_out + 1) & 0xFFFF
        return seq


@dataclass
class XRCESession:
    """
    XRCE session state.

    Tracks the session lifecycle, sequence numbers, and connection state.
    """
    session_id: int = 0x81  # Session with key
    client_key: Optional[bytes] = None  # ESP32's client key (4 bytes)
    is_connected: bool = False
    last_activity: float = field(default_factory=time.time)

    # Stream management
    _streams: Dict[int, StreamState] = field(default_factory=dict)
    _lock: threading.Lock = field(default_factory=threading.Lock)

    # Track seen requests to handle retransmissions
    _seen_requests: set = field(default_factory=set)

    def __post_init__(self):
        # Initialize built-in streams
        self._streams[0x00] = StreamState(0x00)  # NONE
        self._streams[0x01] = StreamState(0x01)  # Best-effort
        self._streams[0x80] = StreamState(0x80)  # Reliable

    def get_stream(self, stream_id: int) -> StreamState:
        """Get or create a stream state."""
        with self._lock:
            if stream_id not in self._streams:
                self._streams[stream_id] = StreamState(stream_id)
            return self._streams[stream_id]

    def next_sequence(self, stream_id: int = 0x80) -> int:
        """Get next sequence number for a stream."""
        return self.get_stream(stream_id).next_sequence()

    def record_received(self, stream_id: int, seq_num: int) -> None:
        """Record a received sequence number."""
        with self._lock:
            stream = self.get_stream(stream_id)
            stream.last_seq_in = seq_num

    def is_duplicate_request(self, submsg_id: int, seq_num: int) -> bool:
        """
        Check if this is a duplicate/retransmitted request.

        The ESP32 may retransmit CREATE messages if it doesn't receive
        STATUS responses quickly enough. We track seen requests to avoid
        re-processing them.
        """
        key = (submsg_id, seq_num)
        with self._lock:
            if key in self._seen_requests:
                return True
            self._seen_requests.add(key)
            return False

    def mark_activity(self) -> None:
        """Update last activity timestamp."""
        self.last_activity = time.time()

    def mark_connected(self, client_key: bytes) -> None:
        """Mark session as connected with client key."""
        self.client_key = client_key
        self.is_connected = True
        self.mark_activity()

    def reset(self) -> None:
        """Reset session state."""
        with self._lock:
            self.client_key = None
            self.is_connected = False
            self._seen_requests.clear()
            for stream in self._streams.values():
                stream.next_seq_out = 0
                stream.last_seq_in = 0


class SessionManager:
    """
    Manages XRCE sessions for the agent.

    Currently supports a single session (the ESP32), but the architecture
    allows for multiple clients in the future.
    """

    def __init__(self):
        self._session = XRCESession()
        self._lock = threading.Lock()

    @property
    def session(self) -> XRCESession:
        """Get the current session."""
        return self._session

    @property
    def is_connected(self) -> bool:
        """Check if a client is connected."""
        return self._session.is_connected

    def handle_create_client(self, client_key: bytes) -> None:
        """
        Handle CREATE_CLIENT from ESP32.

        Marks the session as connected and stores the client key.
        """
        with self._lock:
            self._session.mark_connected(client_key)

    def next_sequence(self, stream_id: int = 0x80) -> int:
        """Get next sequence number for responses."""
        return self._session.next_sequence(stream_id)

    def is_duplicate(self, submsg_id: int, seq_num: int) -> bool:
        """Check if this is a duplicate request."""
        return self._session.is_duplicate_request(submsg_id, seq_num)

    def reset(self) -> None:
        """Reset all session state."""
        with self._lock:
            self._session.reset()
