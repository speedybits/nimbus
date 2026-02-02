"""
Transport layer for XRCE-DDS communication.

Provides UDP transport for WiFi communication with the ESP32's
Micro-ROS firmware. The agent binds to a port and listens for
incoming XRCE-DDS messages from the ESP32.
"""

import socket
import select
import threading
from dataclasses import dataclass
from typing import Optional, Tuple

from .assertions import assert_comm
from .logger import LogLevel


@dataclass
class UDPConfig:
    """UDP transport configuration."""
    bind_ip: str = "0.0.0.0"     # Address to bind to
    bind_port: int = 8888        # Port to listen on (ESP32 sends to this)
    recv_timeout: float = 0.1   # Receive poll timeout


class UDPTransport:
    """
    UDP transport for WiFi communication with ESP32.

    The ESP32 Micro-ROS firmware sends UDP packets to a configured
    agent IP/port. This transport binds to that port and handles
    bidirectional communication with the ESP32.

    As an agent, we:
    - Bind to a port and wait for incoming packets
    - Learn the ESP32's address from the first incoming packet
    - Send responses back to the learned address

    Usage:
        transport = UDPTransport(UDPConfig(bind_port=8888))
        if transport.open():
            data, addr = transport.receive()
            transport.send(response, addr)
        transport.close()
    """

    def __init__(self, config: Optional[UDPConfig] = None):
        self.config = config or UDPConfig()
        self._socket: Optional[socket.socket] = None
        self._client_addr: Optional[Tuple[str, int]] = None
        self._lock = threading.Lock()

    def open(self) -> bool:
        """
        Bind UDP socket to listen for ESP32 connections.

        Returns:
            True if socket bound successfully
        """
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._socket.bind((self.config.bind_ip, self.config.bind_port))
            self._socket.setblocking(False)
            return True
        except Exception as e:
            assert_comm("T1", False, f"Cannot bind UDP port {self.config.bind_port}: {e}", "bold red", 60.0, level=LogLevel.CRITICAL)
            return False

    def close(self) -> None:
        """Close UDP socket."""
        with self._lock:
            if self._socket:
                try:
                    self._socket.close()
                except Exception:
                    pass
                self._socket = None
                self._client_addr = None

    def receive(self, timeout: Optional[float] = None) -> Optional[Tuple[bytes, Tuple[str, int]]]:
        """
        Receive data from ESP32.

        Returns:
            Tuple of (data, (ip, port)) if data received, None otherwise
        """
        timeout = timeout if timeout is not None else self.config.recv_timeout

        with self._lock:
            if not self._socket:
                return None

            try:
                readable, _, _ = select.select([self._socket], [], [], timeout)
                if not readable:
                    return None

                data, addr = self._socket.recvfrom(65535)

                # Remember client address for sending
                self._client_addr = addr

                return data, addr

            except BlockingIOError:
                return None
            except Exception as e:
                assert_comm("T2", False, f"UDP receive error: {e}", "yellow", 5.0, level=LogLevel.DEBUG)
                return None

    def send(self, data: bytes, addr: Optional[Tuple[str, int]] = None,
             retries: int = 0, max_time: float = 1.0,
             base_delay: float = 0.01, max_delay: float = 0.2) -> bool:
        """
        Send data to ESP32 with exponential backoff retry.

        Retries continuously with exponential backoff until max_time is reached.
        Releases lock between retries to avoid blocking receive thread.

        Args:
            data: Bytes to send
            addr: Optional target address; uses learned client address if not provided
            retries: Deprecated, ignored (kept for compatibility)
            max_time: Maximum total time to spend retrying (default 1.0s)
            base_delay: Initial delay between retries (default 10ms)
            max_delay: Maximum delay between retries (default 200ms)

        Returns:
            True if send succeeded
        """
        import time as _time

        # Get socket and target under lock, then release for send attempts
        with self._lock:
            if not self._socket:
                return False
            sock = self._socket
            target = addr or self._client_addr
            if not target:
                return False

        # Retry loop without holding lock (allows receive to proceed)
        start_time = _time.time()
        attempt = 0
        delay = base_delay

        while True:
            try:
                sock.sendto(data, target)
                return True
            except Exception as e:
                attempt += 1
                elapsed = _time.time() - start_time

                if elapsed >= max_time:
                    assert_comm(
                        "T3", False,
                        f"Send failed after {attempt} attempts ({elapsed:.2f}s): {e}",
                        "red", 5.0,
                        level=LogLevel.DEBUG
                    )
                    return False

                # Exponential backoff (no lock held during sleep)
                _time.sleep(delay)
                delay = min(delay * 2, max_delay)

    @property
    def is_open(self) -> bool:
        """Check if socket is open."""
        return self._socket is not None

    @property
    def client_address(self) -> Optional[Tuple[str, int]]:
        """Get the learned ESP32 address."""
        return self._client_addr

    @property
    def local_address(self) -> Optional[Tuple[str, int]]:
        """Get the local bound address."""
        if self._socket:
            try:
                return self._socket.getsockname()
            except Exception:
                pass
        return None
