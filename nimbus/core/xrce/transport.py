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
            print(f"UDP transport bind failed: {e}")
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
                print(f"UDP receive failed: {e}")
                return None

    def send(self, data: bytes, addr: Optional[Tuple[str, int]] = None) -> bool:
        """
        Send data to ESP32.

        Args:
            data: Bytes to send
            addr: Optional target address; uses learned client address if not provided

        Returns:
            True if send succeeded
        """
        with self._lock:
            if not self._socket:
                return False

            # Use provided address or learned client address
            target = addr or self._client_addr
            if not target:
                return False

            try:
                self._socket.sendto(data, target)
                return True
            except Exception as e:
                print(f"UDP send failed: {e}")
                return False

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
