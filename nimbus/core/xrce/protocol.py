"""
XRCE-DDS Protocol structures and constants.

This module defines the wire format structures for the DDS-XRCE protocol,
based on the OMG DDS-XRCE specification and eProsima Micro-XRCE-DDS-Agent.

Message Structure:
    ┌─────────────────────────────────────────┐
    │ Message Header (4 bytes)                │
    │ ├─ session_id:  uint8  (1 byte)         │
    │ ├─ stream_id:   uint8  (1 byte)         │
    │ └─ sequence_nr: uint16 (2 bytes)        │
    ├─────────────────────────────────────────┤
    │ Submessage Header (4 bytes)             │
    │ ├─ submessage_id: uint8  (1 byte)       │
    │ ├─ flags:         uint8  (1 byte)       │
    │ └─ length:        uint16 (2 bytes)      │
    ├─────────────────────────────────────────┤
    │ Submessage Payload (variable)           │
    └─────────────────────────────────────────┘
"""

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, List

from .logger import xrce_log


# =============================================================================
# Protocol Constants (from eProsima Micro-XRCE-DDS source)
# =============================================================================

class SubmessageId(IntEnum):
    """XRCE submessage identifiers."""
    CREATE_CLIENT = 0x00
    CREATE = 0x01
    GET_INFO = 0x02
    DELETE = 0x03
    STATUS_AGENT = 0x04
    STATUS = 0x05
    INFO = 0x06
    WRITE_DATA = 0x07
    READ_DATA = 0x08
    DATA = 0x09
    ACKNACK = 0x0A
    HEARTBEAT = 0x0B
    RESET = 0x0C
    FRAGMENT = 0x0D
    TIMESTAMP = 0x0E
    TIMESTAMP_REPLY = 0x0F


class ObjectKind(IntEnum):
    """XRCE object types."""
    INVALID = 0x00
    PARTICIPANT = 0x01
    TOPIC = 0x02
    PUBLISHER = 0x03
    SUBSCRIBER = 0x04
    DATAWRITER = 0x05
    DATAREADER = 0x06
    REQUESTER = 0x07
    REPLIER = 0x08


class StatusValue(IntEnum):
    """XRCE status values."""
    OK = 0x00
    OK_MATCHED = 0x01
    ERR_DDS_ERROR = 0x80
    ERR_MISMATCH = 0x81
    ERR_ALREADY_EXISTS = 0x82
    ERR_DENIED = 0x83
    ERR_UNKNOWN_REFERENCE = 0x84
    ERR_INVALID_DATA = 0x85
    ERR_INCOMPATIBLE = 0x86
    ERR_RESOURCES = 0x87


class StreamId(IntEnum):
    """XRCE stream identifiers."""
    NONE = 0x00
    BUILTIN_BEST_EFFORT = 0x01
    BUILTIN_RELIABLE = 0x80


# Session and submessage flags
FLAG_LITTLE_ENDIAN = 0x01
FLAG_REUSE = 0x02
FLAG_REPLACE = 0x04

# Data format flags (in submessage flags)
FORMAT_DATA = 0x00
FORMAT_SAMPLE = 0x02
FORMAT_DATA_SEQ = 0x08
FORMAT_SAMPLE_SEQ = 0x0A
FORMAT_PACKED_SAMPLES = 0x0E


# =============================================================================
# Wire Format Structures
# =============================================================================

@dataclass
class MessageHeader:
    """
    XRCE message header (4 bytes).

    Format:
    - session_id: 1 byte (session identifier, 0x81 = session with key)
    - stream_id: 1 byte (stream identifier)
    - sequence_nr: 2 bytes (sequence number, little-endian)
    """
    session_id: int = 0x81
    stream_id: int = StreamId.BUILTIN_BEST_EFFORT
    sequence_nr: int = 0

    def pack(self) -> bytes:
        """Serialize header to bytes."""
        return struct.pack('<BBH',
            self.session_id,
            self.stream_id,
            self.sequence_nr
        )

    @classmethod
    def unpack(cls, data: bytes, offset: int = 0) -> "MessageHeader":
        """Deserialize header from bytes."""
        session_id, stream_id, seq = struct.unpack_from('<BBH', data, offset)
        return cls(session_id, stream_id, seq)

    @classmethod
    def size(cls) -> int:
        """Header size in bytes."""
        return 4


@dataclass
class SubmessageHeader:
    """
    XRCE submessage header (4 bytes).

    Format:
    - submessage_id: 1 byte
    - flags: 1 byte (bit 0 = little endian)
    - length: 2 bytes (payload length, little-endian)
    """
    submessage_id: int
    flags: int = FLAG_LITTLE_ENDIAN
    length: int = 0

    def pack(self) -> bytes:
        """Serialize submessage header."""
        return struct.pack('<BBH',
            self.submessage_id,
            self.flags,
            self.length
        )

    @classmethod
    def unpack(cls, data: bytes, offset: int = 0) -> "SubmessageHeader":
        """Deserialize submessage header."""
        sub_id, flags, length = struct.unpack_from('<BBH', data, offset)
        return cls(sub_id, flags, length)

    @classmethod
    def size(cls) -> int:
        """Header size in bytes."""
        return 4


@dataclass
class ObjectId:
    """
    XRCE object identifier (2 bytes).

    The ESP32 uses a simple 2-byte little-endian ID without embedded kind.
    We track kind separately for our own bookkeeping.
    """
    raw_id: int  # The actual 2-byte ID value
    kind: ObjectKind = ObjectKind.INVALID  # Our tracked kind

    def pack(self) -> bytes:
        """Serialize object ID as 2-byte little-endian."""
        return struct.pack('<H', self.raw_id)

    @classmethod
    def unpack(cls, data: bytes, offset: int = 0) -> "ObjectId":
        """Deserialize object ID from 2-byte little-endian."""
        raw_id = struct.unpack_from('<H', data, offset)[0]
        return cls(raw_id=raw_id)

    @classmethod
    def create(cls, raw_id: int, kind: ObjectKind = ObjectKind.INVALID) -> "ObjectId":
        """Create an object ID with the given ID and optional kind."""
        return cls(raw_id=raw_id, kind=kind)


# =============================================================================
# Parsed Message Structures
# =============================================================================

@dataclass
class ParsedSubmessage:
    """A parsed XRCE submessage."""
    submessage_id: int
    flags: int
    payload: bytes


@dataclass
class ParsedMessage:
    """A parsed XRCE message with header and submessages."""
    header: MessageHeader
    submessages: List[ParsedSubmessage]


def parse_message(data: bytes) -> Optional[ParsedMessage]:
    """
    Parse an XRCE message from raw bytes.

    Args:
        data: Raw message bytes

    Returns:
        ParsedMessage if successful, None if parsing fails
    """
    if len(data) < MessageHeader.size():
        return None

    header = MessageHeader.unpack(data)
    submessages = []
    offset = MessageHeader.size()

    while offset + SubmessageHeader.size() <= len(data):
        submsg_header = SubmessageHeader.unpack(data, offset)
        offset += SubmessageHeader.size()

        payload_end = offset + submsg_header.length
        if payload_end > len(data):
            xrce_log(f"[red]Parse error: need {submsg_header.length} bytes, have {len(data) - offset}[/red]")
            break

        payload = data[offset:payload_end]
        offset = payload_end

        submessages.append(ParsedSubmessage(
            submessage_id=submsg_header.submessage_id,
            flags=submsg_header.flags,
            payload=payload
        ))

    return ParsedMessage(header, submessages)


# =============================================================================
# Message Builders
# =============================================================================

def build_status_agent(client_key: bytes) -> bytes:
    """
    Build STATUS_AGENT response to acknowledge CREATE_CLIENT.

    This tells the ESP32 that an XRCE agent is present and ready.

    Args:
        client_key: The client key from CREATE_CLIENT (4 bytes)

    Returns:
        Complete XRCE message bytes
    """
    # STATUS_AGENT payload format (11 bytes) - matches Docker Micro-ROS agent:
    # - object_id (2 bytes): reference to client object (0x0000)
    # - xrce_cookie (4 bytes): "XRCE"
    # - xrce_version (2 bytes): 1.0 (little-endian)
    # - xrce_vendor_id (2 bytes): eProsima (0x010F, little-endian)
    # - status (1 byte): OK
    payload = bytes([0x00, 0x00])        # object_id reference
    payload += b'XRCE'                    # XRCE cookie
    payload += bytes([0x01, 0x00])        # version 1.0
    payload += bytes([0x01, 0x0F])        # vendor ID (eProsima)
    payload += bytes([StatusValue.OK])    # status OK

    submsg = SubmessageHeader(SubmessageId.STATUS_AGENT, FLAG_LITTLE_ENDIAN, len(payload))

    # Use session_id=0x81, stream=NONE, seq=0 (matching Docker agent)
    header = MessageHeader(
        session_id=0x81,
        stream_id=StreamId.NONE,
        sequence_nr=0
    )
    return header.pack() + submsg.pack() + payload


def build_timestamp_reply(transmit_ts: int, receive_ts: int) -> bytes:
    """
    Build TIMESTAMP_REPLY response.

    Args:
        transmit_ts: Original transmit timestamp from client (nanoseconds)
        receive_ts: Our receive/transmit time (nanoseconds)

    Returns:
        Complete XRCE message bytes
    """
    # TIMESTAMP_REPLY payload format (24 bytes):
    # - transmit_timestamp (8 bytes): original timestamp from client
    # - receive_timestamp (8 bytes): when we received it
    # - originate_timestamp (8 bytes): when we're sending reply
    payload = struct.pack('<Q', transmit_ts)  # transmit (from client)
    payload += struct.pack('<Q', receive_ts)   # receive
    payload += struct.pack('<Q', receive_ts)   # originate (same as receive)

    submsg = SubmessageHeader(SubmessageId.TIMESTAMP_REPLY, FLAG_LITTLE_ENDIAN, len(payload))

    header = MessageHeader(
        session_id=0x81,
        stream_id=StreamId.NONE,
        sequence_nr=0
    )
    return header.pack() + submsg.pack() + payload


def build_status_ok(object_id: int, request_id: int, seq_num: int) -> bytes:
    """
    Build STATUS OK response for CREATE request.

    Args:
        object_id: The object ID from the CREATE request
        request_id: The request ID from the CREATE request
        seq_num: Sequence number for reliable stream

    Returns:
        Complete XRCE message bytes
    """
    # STATUS payload format (6 bytes) - matches Docker Micro-ROS agent:
    # - object_id (2 bytes)
    # - request_id (2 bytes)
    # - status (2 bytes): 0x0000 = OK
    payload = struct.pack('<H', object_id)
    payload += struct.pack('<H', request_id)
    payload += struct.pack('<H', 0x0000)  # status OK (2 bytes)

    submsg = SubmessageHeader(SubmessageId.STATUS, FLAG_LITTLE_ENDIAN, len(payload))

    # Use stream_id=0x80 (reliable stream) for STATUS responses
    header = MessageHeader(
        session_id=0x81,
        stream_id=0x80,  # Reliable stream
        sequence_nr=seq_num
    )
    return header.pack() + submsg.pack() + payload


def build_acknack(next_seq: int) -> bytes:
    """
    Build ACKNACK response to acknowledge received messages.

    Args:
        next_seq: Next expected sequence number

    Returns:
        Complete XRCE message bytes
    """
    # ACKNACK payload format (5 bytes):
    # - seq_num (2 bytes): next expected sequence number
    # - bitmap (2 bytes): NACK bitmap (0x0000 = all received)
    # - stream_id (1 byte): stream being acknowledged (0x80 = reliable)
    payload = struct.pack('<H', next_seq)
    payload += struct.pack('<H', 0x0000)  # empty NACK bitmap
    payload += bytes([0x80])  # reliable stream

    submsg = SubmessageHeader(SubmessageId.ACKNACK, FLAG_LITTLE_ENDIAN, len(payload))

    header = MessageHeader(
        session_id=0x81,
        stream_id=StreamId.NONE,
        sequence_nr=0
    )
    return header.pack() + submsg.pack() + payload


def build_data(object_id: int, cdr_data: bytes, seq_num: int, request_id: int = 0) -> bytes:
    """
    Build DATA submessage to send data to a datareader.

    Args:
        object_id: The datareader's object ID
        cdr_data: CDR-serialized message data (with encapsulation header)
        seq_num: Sequence number for reliable stream
        request_id: Request ID from READ_DATA (0 for unsolicited)

    Returns:
        Complete XRCE message bytes

    DATA submessage format (mirrors WRITE_DATA from ESP32):
        - object_id (2 bytes): datareader object ID
        - request_id (2 bytes): from READ_DATA or 0
        - format_flags (4 bytes): 0x0f 0x00 0x00 0x00
        - CDR data WITHOUT encapsulation header
    """
    # Strip CDR encapsulation if present
    if len(cdr_data) >= 4 and cdr_data[:2] == b'\x00\x01':
        raw_cdr = cdr_data[4:]
    else:
        raw_cdr = cdr_data

    # DATA payload matching WRITE_DATA format
    payload = struct.pack('<H', object_id)      # object_id (2 bytes)
    payload += struct.pack('<H', request_id)    # request_id (2 bytes)
    payload += bytes([0x0f, 0x00, 0x00, 0x00])  # format_flags (4 bytes)
    payload += raw_cdr                          # CDR data without encapsulation

    submsg = SubmessageHeader(
        SubmessageId.DATA,
        FLAG_LITTLE_ENDIAN | FORMAT_DATA,
        len(payload)
    )

    header = MessageHeader(
        session_id=0x81,
        stream_id=StreamId.BUILTIN_RELIABLE,
        sequence_nr=seq_num
    )
    return header.pack() + submsg.pack() + payload


def build_heartbeat_probe(seq_num: int) -> bytes:
    """
    Build HEARTBEAT message for keepalive probing.

    Sends a minimal HEARTBEAT to verify bidirectional connectivity
    with the ESP32. The ESP32 should respond, resetting the silence timer.

    Args:
        seq_num: Sequence number for the message

    Returns:
        Complete XRCE message bytes
    """
    # HEARTBEAT payload format (4 bytes):
    # - first_seq (2 bytes): first unacknowledged sequence
    # - last_seq (2 bytes): last sent sequence
    payload = struct.pack('<HH', 0, seq_num)

    submsg = SubmessageHeader(
        SubmessageId.HEARTBEAT,
        FLAG_LITTLE_ENDIAN,
        len(payload)
    )

    header = MessageHeader(
        session_id=0x81,
        stream_id=StreamId.NONE,
        sequence_nr=seq_num
    )
    return header.pack() + submsg.pack() + payload
