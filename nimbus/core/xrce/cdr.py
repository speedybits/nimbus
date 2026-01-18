"""
CDR (Common Data Representation) serialization for XRCE-DDS.

CDR is the wire format used by DDS/RTPS for serializing messages.
This module implements the subset needed for ROS2 message types:
LaserScan, Odometry, and Twist.

Key CDR rules:
- Little-endian encoding (for x86/ARM)
- Alignment: primitives align to their size
- Strings: 4-byte length + chars + null terminator
- Arrays: 4-byte count + elements
"""

import struct
from typing import List, Tuple


class CDRReader:
    """
    Deserialize CDR-encoded data.

    Usage:
        reader = CDRReader(data)
        value = reader.read_float32()
        array = reader.read_float32_array()
    """

    def __init__(self, data: bytes):
        self._data = data
        self._pos = 0

    def _align(self, size: int) -> None:
        """Align position to the given boundary."""
        remainder = self._pos % size
        if remainder != 0:
            self._pos += size - remainder

    def read_uint8(self) -> int:
        """Read unsigned 8-bit integer."""
        value = self._data[self._pos]
        self._pos += 1
        return value

    def read_int8(self) -> int:
        """Read signed 8-bit integer."""
        value = struct.unpack_from('<b', self._data, self._pos)[0]
        self._pos += 1
        return value

    def read_uint16(self) -> int:
        """Read unsigned 16-bit integer."""
        self._align(2)
        value = struct.unpack_from('<H', self._data, self._pos)[0]
        self._pos += 2
        return value

    def read_int16(self) -> int:
        """Read signed 16-bit integer."""
        self._align(2)
        value = struct.unpack_from('<h', self._data, self._pos)[0]
        self._pos += 2
        return value

    def read_uint32(self) -> int:
        """Read unsigned 32-bit integer."""
        self._align(4)
        value = struct.unpack_from('<I', self._data, self._pos)[0]
        self._pos += 4
        return value

    def read_int32(self) -> int:
        """Read signed 32-bit integer."""
        self._align(4)
        value = struct.unpack_from('<i', self._data, self._pos)[0]
        self._pos += 4
        return value

    def read_uint64(self) -> int:
        """Read unsigned 64-bit integer."""
        self._align(8)
        value = struct.unpack_from('<Q', self._data, self._pos)[0]
        self._pos += 8
        return value

    def read_int64(self) -> int:
        """Read signed 64-bit integer."""
        self._align(8)
        value = struct.unpack_from('<q', self._data, self._pos)[0]
        self._pos += 8
        return value

    def read_float32(self) -> float:
        """Read 32-bit float."""
        self._align(4)
        value = struct.unpack_from('<f', self._data, self._pos)[0]
        self._pos += 4
        return value

    def read_float64(self) -> float:
        """Read 64-bit float (double)."""
        self._align(8)
        value = struct.unpack_from('<d', self._data, self._pos)[0]
        self._pos += 8
        return value

    def read_string(self) -> str:
        """Read a CDR string (length-prefixed, null-terminated)."""
        length = self.read_uint32()  # Includes null terminator
        if length == 0:
            return ""
        # Read string bytes (excluding null terminator)
        data = self._data[self._pos:self._pos + length - 1]
        self._pos += length
        return data.decode('utf-8')

    def read_float32_array(self) -> List[float]:
        """Read a sequence of 32-bit floats."""
        count = self.read_uint32()
        self._align(4)
        values = []
        for _ in range(count):
            values.append(struct.unpack_from('<f', self._data, self._pos)[0])
            self._pos += 4
        return values

    def read_float64_array(self) -> List[float]:
        """Read a sequence of 64-bit floats."""
        count = self.read_uint32()
        self._align(8)
        values = []
        for _ in range(count):
            values.append(struct.unpack_from('<d', self._data, self._pos)[0])
            self._pos += 8
        return values

    def read_bytes(self, count: int) -> bytes:
        """Read raw bytes without alignment."""
        data = self._data[self._pos:self._pos + count]
        self._pos += count
        return data

    def skip(self, count: int) -> None:
        """Skip bytes."""
        self._pos += count

    def skip_string(self) -> None:
        """Skip a CDR string."""
        length = self.read_uint32()
        self._pos += length

    @property
    def position(self) -> int:
        """Current read position."""
        return self._pos

    @property
    def remaining(self) -> int:
        """Bytes remaining."""
        return len(self._data) - self._pos


class CDRWriter:
    """
    Serialize data to CDR format.

    Usage:
        writer = CDRWriter()
        writer.write_float32(1.5)
        data = writer.get_data()
    """

    def __init__(self, initial_capacity: int = 256):
        self._buffer = bytearray(initial_capacity)
        self._pos = 0

    def _ensure_capacity(self, needed: int) -> None:
        """Ensure buffer has enough space."""
        while self._pos + needed > len(self._buffer):
            self._buffer.extend(bytearray(len(self._buffer)))

    def _align(self, size: int) -> None:
        """Align position to the given boundary, padding with zeros."""
        remainder = self._pos % size
        if remainder != 0:
            padding = size - remainder
            self._ensure_capacity(padding)
            for _ in range(padding):
                self._buffer[self._pos] = 0
                self._pos += 1

    def write_uint8(self, value: int) -> None:
        """Write unsigned 8-bit integer."""
        self._ensure_capacity(1)
        self._buffer[self._pos] = value & 0xFF
        self._pos += 1

    def write_int8(self, value: int) -> None:
        """Write signed 8-bit integer."""
        self._ensure_capacity(1)
        struct.pack_into('<b', self._buffer, self._pos, value)
        self._pos += 1

    def write_uint16(self, value: int) -> None:
        """Write unsigned 16-bit integer."""
        self._align(2)
        self._ensure_capacity(2)
        struct.pack_into('<H', self._buffer, self._pos, value)
        self._pos += 2

    def write_int16(self, value: int) -> None:
        """Write signed 16-bit integer."""
        self._align(2)
        self._ensure_capacity(2)
        struct.pack_into('<h', self._buffer, self._pos, value)
        self._pos += 2

    def write_uint32(self, value: int) -> None:
        """Write unsigned 32-bit integer."""
        self._align(4)
        self._ensure_capacity(4)
        struct.pack_into('<I', self._buffer, self._pos, value)
        self._pos += 4

    def write_int32(self, value: int) -> None:
        """Write signed 32-bit integer."""
        self._align(4)
        self._ensure_capacity(4)
        struct.pack_into('<i', self._buffer, self._pos, value)
        self._pos += 4

    def write_uint64(self, value: int) -> None:
        """Write unsigned 64-bit integer."""
        self._align(8)
        self._ensure_capacity(8)
        struct.pack_into('<Q', self._buffer, self._pos, value)
        self._pos += 8

    def write_int64(self, value: int) -> None:
        """Write signed 64-bit integer."""
        self._align(8)
        self._ensure_capacity(8)
        struct.pack_into('<q', self._buffer, self._pos, value)
        self._pos += 8

    def write_float32(self, value: float) -> None:
        """Write 32-bit float."""
        self._align(4)
        self._ensure_capacity(4)
        struct.pack_into('<f', self._buffer, self._pos, value)
        self._pos += 4

    def write_float64(self, value: float) -> None:
        """Write 64-bit float (double)."""
        self._align(8)
        self._ensure_capacity(8)
        struct.pack_into('<d', self._buffer, self._pos, value)
        self._pos += 8

    def write_string(self, value: str) -> None:
        """Write a CDR string (length-prefixed, null-terminated)."""
        encoded = value.encode('utf-8')
        length = len(encoded) + 1  # Include null terminator
        self.write_uint32(length)
        self._ensure_capacity(length)
        for i, b in enumerate(encoded):
            self._buffer[self._pos + i] = b
        self._buffer[self._pos + len(encoded)] = 0  # Null terminator
        self._pos += length

    def write_float32_array(self, values: List[float]) -> None:
        """Write a sequence of 32-bit floats."""
        self.write_uint32(len(values))
        self._align(4)
        self._ensure_capacity(len(values) * 4)
        for value in values:
            struct.pack_into('<f', self._buffer, self._pos, value)
            self._pos += 4

    def write_float64_array(self, values: List[float]) -> None:
        """Write a sequence of 64-bit floats."""
        self.write_uint32(len(values))
        self._align(8)
        self._ensure_capacity(len(values) * 8)
        for value in values:
            struct.pack_into('<d', self._buffer, self._pos, value)
            self._pos += 8

    def write_bytes(self, data: bytes) -> None:
        """Write raw bytes without alignment."""
        self._ensure_capacity(len(data))
        for i, b in enumerate(data):
            self._buffer[self._pos + i] = b
        self._pos += len(data)

    def get_data(self) -> bytes:
        """Get the serialized data."""
        return bytes(self._buffer[:self._pos])

    @property
    def size(self) -> int:
        """Current data size."""
        return self._pos


def parse_encapsulation(data: bytes) -> Tuple[bool, bytes]:
    """
    Parse CDR encapsulation header (4 bytes).

    Returns:
        Tuple of (is_little_endian, payload)
    """
    if len(data) < 4:
        raise ValueError("Data too short for encapsulation header")

    # First byte: 0x00 (CDR representation identifier)
    # Second byte: 0x01 = little endian, 0x00 = big endian
    # Third/fourth bytes: options (usually 0x00, 0x00)
    is_little_endian = data[1] == 0x01
    return is_little_endian, data[4:]


def add_encapsulation(data: bytes, little_endian: bool = True) -> bytes:
    """
    Add CDR encapsulation header to payload.

    Args:
        data: Payload bytes
        little_endian: Use little-endian encoding (default True)

    Returns:
        Encapsulated data with 4-byte header
    """
    header = bytes([
        0x00,                          # CDR identifier
        0x01 if little_endian else 0x00,  # Endianness
        0x00, 0x00                     # Options
    ])
    return header + data
