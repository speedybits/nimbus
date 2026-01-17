"""
Transport layer for XRCE-DDS communication.

Provides UDP (WiFi) and Serial (USB) transports for communicating
with the Yahboom ESP32's Micro-ROS firmware.
"""

import socket
import select
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Callable
from queue import Queue, Empty


@dataclass
class TransportConfig:
    """Base transport configuration."""
    recv_timeout: float = 0.1  # Seconds to wait for data


@dataclass
class UDPConfig(TransportConfig):
    """UDP transport configuration."""
    local_ip: str = "0.0.0.0"    # Bind address
    local_port: int = 8090       # Local port to bind
    remote_ip: str = ""          # ESP32 IP (learned from incoming packets)
    remote_port: int = 8090      # ESP32 port


@dataclass
class SerialConfig(TransportConfig):
    """Serial transport configuration."""
    device: str = "/dev/ttyACM0"
    baudrate: int = 115200
    timeout: float = 0.1


class Transport(ABC):
    """
    Abstract base class for XRCE-DDS transports.

    A transport handles the low-level sending and receiving of
    XRCE-DDS packets over a physical medium (UDP or Serial).
    """

    @abstractmethod
    def open(self) -> bool:
        """Open the transport. Returns True on success."""
        pass

    @abstractmethod
    def close(self) -> None:
        """Close the transport."""
        pass

    @abstractmethod
    def send(self, data: bytes) -> bool:
        """Send data. Returns True on success."""
        pass

    @abstractmethod
    def receive(self, timeout: Optional[float] = None) -> Optional[bytes]:
        """Receive data. Returns None if no data available."""
        pass

    @property
    @abstractmethod
    def is_open(self) -> bool:
        """Check if transport is open."""
        pass


class UDPTransport(Transport):
    """
    UDP transport for WiFi communication with the ESP32.

    The ESP32 Micro-ROS firmware sends UDP packets to a configured
    agent IP/port. This transport binds to that port and communicates
    bidirectionally with the ESP32.

    Usage:
        transport = UDPTransport(UDPConfig(local_port=8090))
        transport.open()
        transport.send(data)
        response = transport.receive()
        transport.close()
    """

    def __init__(self, config: Optional[UDPConfig] = None):
        self.config = config or UDPConfig()
        self._socket: Optional[socket.socket] = None
        self._remote_addr: Optional[tuple] = None
        self._lock = threading.Lock()

    def open(self) -> bool:
        """Bind UDP socket to local port."""
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._socket.bind((self.config.local_ip, self.config.local_port))
            self._socket.setblocking(False)

            # If remote address is pre-configured, use it
            if self.config.remote_ip:
                self._remote_addr = (self.config.remote_ip, self.config.remote_port)

            return True
        except Exception as e:
            print(f"UDP transport open failed: {e}")
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

    def send(self, data: bytes) -> bool:
        """
        Send data to the ESP32.

        The remote address is learned from incoming packets if not
        pre-configured.
        """
        with self._lock:
            if not self._socket:
                return False

            if not self._remote_addr:
                # Can't send until we know where to send
                return False

            try:
                self._socket.sendto(data, self._remote_addr)
                return True
            except Exception as e:
                print(f"UDP send failed: {e}")
                return False

    def receive(self, timeout: Optional[float] = None) -> Optional[bytes]:
        """
        Receive data from the ESP32.

        Also learns the remote address from incoming packets.
        """
        timeout = timeout if timeout is not None else self.config.recv_timeout

        with self._lock:
            if not self._socket:
                return None

            try:
                # Use select for timeout
                readable, _, _ = select.select([self._socket], [], [], timeout)
                if not readable:
                    return None

                data, addr = self._socket.recvfrom(65535)

                # Learn remote address from first incoming packet
                if self._remote_addr is None:
                    self._remote_addr = addr

                return data
            except BlockingIOError:
                return None
            except Exception as e:
                print(f"UDP receive failed: {e}")
                return None

    @property
    def is_open(self) -> bool:
        """Check if socket is open."""
        return self._socket is not None

    @property
    def remote_address(self) -> Optional[tuple]:
        """Get the learned remote address (IP, port)."""
        return self._remote_addr


class SerialTransport(Transport):
    """
    Serial transport for USB communication with the ESP32.

    Uses the Micro-ROS framing protocol for serial communication.
    Each frame has the format:
    - Start flag (0x7E)
    - Length (2 bytes, little-endian)
    - Payload
    - CRC16 (2 bytes)

    Usage:
        transport = SerialTransport(SerialConfig(device="/dev/ttyACM0"))
        transport.open()
        transport.send(data)
        response = transport.receive()
        transport.close()
    """

    # Serial framing constants
    FRAME_START = 0x7E
    FRAME_ESCAPE = 0x7D
    FRAME_XOR = 0x20

    def __init__(self, config: Optional[SerialConfig] = None):
        self.config = config or SerialConfig()
        self._serial = None
        self._lock = threading.Lock()
        self._recv_buffer = bytearray()

    def open(self) -> bool:
        """Open serial port."""
        try:
            import serial
            self._serial = serial.Serial(
                port=self.config.device,
                baudrate=self.config.baudrate,
                timeout=self.config.timeout
            )
            return True
        except ImportError:
            print("pyserial not installed. Install with: pip install pyserial")
            return False
        except Exception as e:
            print(f"Serial transport open failed: {e}")
            return False

    def close(self) -> None:
        """Close serial port."""
        with self._lock:
            if self._serial:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None

    def send(self, data: bytes) -> bool:
        """
        Send data with serial framing.

        Frames the data with start flag, length, and CRC.
        """
        with self._lock:
            if not self._serial:
                return False

            try:
                frame = self._frame_data(data)
                self._serial.write(frame)
                return True
            except Exception as e:
                print(f"Serial send failed: {e}")
                return False

    def receive(self, timeout: Optional[float] = None) -> Optional[bytes]:
        """
        Receive and unframe data from serial.
        """
        timeout = timeout if timeout is not None else self.config.recv_timeout

        with self._lock:
            if not self._serial:
                return None

            try:
                # Read available data
                if self._serial.in_waiting > 0:
                    data = self._serial.read(self._serial.in_waiting)
                    self._recv_buffer.extend(data)

                # Try to extract a frame
                return self._extract_frame()

            except Exception as e:
                print(f"Serial receive failed: {e}")
                return None

    def _frame_data(self, data: bytes) -> bytes:
        """Add serial framing to data."""
        # Calculate CRC
        crc = self._crc16(data)

        # Build frame
        frame = bytearray()
        frame.append(self.FRAME_START)

        # Add length (little-endian)
        length = len(data)
        frame.append(length & 0xFF)
        frame.append((length >> 8) & 0xFF)

        # Add escaped payload
        for byte in data:
            if byte in (self.FRAME_START, self.FRAME_ESCAPE):
                frame.append(self.FRAME_ESCAPE)
                frame.append(byte ^ self.FRAME_XOR)
            else:
                frame.append(byte)

        # Add CRC (little-endian)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)

        return bytes(frame)

    def _extract_frame(self) -> Optional[bytes]:
        """Extract a complete frame from the receive buffer."""
        # Find start flag
        try:
            start_idx = self._recv_buffer.index(self.FRAME_START)
        except ValueError:
            self._recv_buffer.clear()
            return None

        # Remove bytes before start
        if start_idx > 0:
            del self._recv_buffer[:start_idx]

        # Need at least start + length (3 bytes)
        if len(self._recv_buffer) < 3:
            return None

        # Get length
        length = self._recv_buffer[1] | (self._recv_buffer[2] << 8)

        # Estimate minimum frame size (start + length + payload + crc)
        min_frame_size = 3 + length + 2

        if len(self._recv_buffer) < min_frame_size:
            return None

        # Extract and unescape payload
        payload = bytearray()
        idx = 3
        while len(payload) < length and idx < len(self._recv_buffer) - 2:
            byte = self._recv_buffer[idx]
            if byte == self.FRAME_ESCAPE:
                idx += 1
                if idx < len(self._recv_buffer):
                    payload.append(self._recv_buffer[idx] ^ self.FRAME_XOR)
            else:
                payload.append(byte)
            idx += 1

        if len(payload) < length:
            return None

        # Extract CRC
        crc_received = self._recv_buffer[idx] | (self._recv_buffer[idx + 1] << 8)
        crc_calculated = self._crc16(payload)

        # Remove frame from buffer
        del self._recv_buffer[:idx + 2]

        if crc_received != crc_calculated:
            return None  # CRC mismatch

        return bytes(payload)

    def _crc16(self, data: bytes) -> int:
        """Calculate CRC-16 (CCITT)."""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    @property
    def is_open(self) -> bool:
        """Check if serial port is open."""
        return self._serial is not None and self._serial.is_open


class BufferedTransport:
    """
    Wrapper that adds a receive buffer with background thread.

    Useful for continuous reception without blocking the main thread.
    """

    def __init__(self, transport: Transport, buffer_size: int = 100):
        self._transport = transport
        self._buffer: Queue = Queue(maxsize=buffer_size)
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> bool:
        """Start the background receive thread."""
        if not self._transport.is_open:
            if not self._transport.open():
                return False

        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        """Stop the background thread and close transport."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._transport.close()

    def _receive_loop(self) -> None:
        """Background receive loop."""
        while self._running:
            data = self._transport.receive(timeout=0.1)
            if data:
                try:
                    self._buffer.put_nowait(data)
                except Exception:
                    pass  # Buffer full, drop oldest

    def send(self, data: bytes) -> bool:
        """Send data through the transport."""
        return self._transport.send(data)

    def receive(self, timeout: float = 0.0) -> Optional[bytes]:
        """Get data from the buffer."""
        try:
            return self._buffer.get(timeout=timeout if timeout > 0 else None)
        except Empty:
            return None

    @property
    def is_open(self) -> bool:
        """Check if running."""
        return self._running and self._transport.is_open
