"""
XRCE-DDS session and entity management.

This module implements the DDS-XRCE protocol structures for
managing sessions and entities with the ESP32 Micro-ROS agent.

XRCE-DDS Protocol Overview:
- Session: A communication context with the agent
- Participant: A DDS DomainParticipant
- Publisher/Subscriber: Containers for DataWriters/DataReaders
- DataWriter: Writes data to a topic
- DataReader: Reads data from a topic
"""

import struct
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Callable, Any
from enum import IntEnum
import threading


# =============================================================================
# XRCE Protocol Constants
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
    USER_RELIABLE_START = 0x80


# Session flags
FLAG_LITTLE_ENDIAN = 0x01
FLAG_REUSE = 0x02
FLAG_REPLACE = 0x04

# Data format flags
FORMAT_DATA = 0x00
FORMAT_SAMPLE = 0x02
FORMAT_DATA_SEQ = 0x08
FORMAT_SAMPLE_SEQ = 0x0A
FORMAT_PACKED_SAMPLES = 0x0E


# =============================================================================
# XRCE Message Structures
# =============================================================================

@dataclass
class XRCEHeader:
    """
    XRCE message header (4 bytes).

    Format:
    - session_id: 1 byte (session identifier)
    - stream_id: 1 byte (stream identifier)
    - sequence_num: 2 bytes (sequence number, little-endian)
    """
    session_id: int = 0x81  # Session with key
    stream_id: int = StreamId.BUILTIN_BEST_EFFORT
    sequence_num: int = 0

    def pack(self) -> bytes:
        """Serialize header to bytes."""
        return struct.pack('<BBH',
            self.session_id,
            self.stream_id,
            self.sequence_num
        )

    @classmethod
    def unpack(cls, data: bytes) -> "XRCEHeader":
        """Deserialize header from bytes."""
        session_id, stream_id, seq = struct.unpack_from('<BBH', data)
        return cls(session_id, stream_id, seq)


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


@dataclass
class ObjectId:
    """
    XRCE object identifier (2 bytes).

    Format: [kind << 4 | (id >> 8), id & 0xFF]
    """
    kind: ObjectKind
    id: int

    def pack(self) -> bytes:
        """Serialize object ID."""
        first = (self.kind << 4) | ((self.id >> 8) & 0x0F)
        second = self.id & 0xFF
        return bytes([first, second])

    @classmethod
    def unpack(cls, data: bytes, offset: int = 0) -> "ObjectId":
        """Deserialize object ID."""
        first = data[offset]
        second = data[offset + 1]
        kind = ObjectKind(first >> 4)
        obj_id = ((first & 0x0F) << 8) | second
        return cls(kind, obj_id)

    @classmethod
    def create(cls, kind: ObjectKind, index: int) -> "ObjectId":
        """Create an object ID with the given kind and index."""
        return cls(kind, index)


# =============================================================================
# Entity Representations (for CREATE submessages)
# =============================================================================

def pack_string(s: str) -> bytes:
    """Pack a string in XRCE format (length + data)."""
    encoded = s.encode('utf-8')
    return struct.pack('<I', len(encoded) + 1) + encoded + b'\x00'


def create_participant_xml(domain_id: int = 0) -> str:
    """Create XML representation for a DomainParticipant."""
    return f'<dds><participant><rtps><name>nimbus_direct</name></rtps></participant></dds>'


def create_topic_xml(name: str, type_name: str) -> str:
    """Create XML representation for a Topic."""
    return f'<dds><topic><name>{name}</name><dataType>{type_name}</dataType></topic></dds>'


def create_datawriter_xml(topic_name: str) -> str:
    """Create XML representation for a DataWriter."""
    return f'<dds><data_writer><topic><kind>NO_KEY</kind><name>{topic_name}</name></topic></data_writer></dds>'


def create_datareader_xml(topic_name: str) -> str:
    """Create XML representation for a DataReader."""
    return f'<dds><data_reader><topic><kind>NO_KEY</kind><name>{topic_name}</name></topic></data_reader></dds>'


def create_publisher_xml() -> str:
    """Create XML representation for a Publisher."""
    return '<dds><publisher><partition><name>rt</name></partition></publisher></dds>'


def create_subscriber_xml() -> str:
    """Create XML representation for a Subscriber."""
    return '<dds><subscriber><partition><name>rt</name></partition></subscriber></dds>'


# =============================================================================
# Message Types for Topics
# =============================================================================

TOPIC_TYPES = {
    "/scan": "sensor_msgs::msg::dds_::LaserScan_",
    "/odom_raw": "nav_msgs::msg::dds_::Odometry_",
    "/cmd_vel": "geometry_msgs::msg::dds_::Twist_",
    "rt/scan": "sensor_msgs::msg::dds_::LaserScan_",
    "rt/odom_raw": "nav_msgs::msg::dds_::Odometry_",
    "rt/cmd_vel": "geometry_msgs::msg::dds_::Twist_",
}


def get_xrce_topic_name(ros_topic: str) -> str:
    """Convert ROS2 topic name to XRCE topic name."""
    # XRCE uses rt/ prefix for ROS2 topics
    if ros_topic.startswith('/'):
        return 'rt' + ros_topic
    return ros_topic


def get_topic_type(topic: str) -> str:
    """Get the DDS type name for a topic."""
    return TOPIC_TYPES.get(topic, TOPIC_TYPES.get(get_xrce_topic_name(topic), ""))


# =============================================================================
# Session State
# =============================================================================

@dataclass
class EntityInfo:
    """Information about a created XRCE entity."""
    object_id: ObjectId
    name: str
    created: bool = False


@dataclass
class SessionState:
    """
    Tracks the state of an XRCE session.
    """
    session_id: int = 0x81
    client_key: bytes = field(default_factory=lambda: bytes([0xAA, 0xBB, 0xCC, 0xDD]))
    sequence_num: int = 0
    is_connected: bool = False

    # Entity tracking
    participant: Optional[EntityInfo] = None
    publisher: Optional[EntityInfo] = None
    subscriber: Optional[EntityInfo] = None
    topics: Dict[str, EntityInfo] = field(default_factory=dict)
    datawriters: Dict[str, EntityInfo] = field(default_factory=dict)
    datareaders: Dict[str, EntityInfo] = field(default_factory=dict)

    # Object ID counter
    _next_id: int = 1

    def next_object_id(self, kind: ObjectKind) -> ObjectId:
        """Generate the next object ID."""
        obj_id = ObjectId.create(kind, self._next_id)
        self._next_id += 1
        return obj_id

    def next_sequence(self) -> int:
        """Get and increment sequence number."""
        seq = self.sequence_num
        self.sequence_num = (self.sequence_num + 1) & 0xFFFF
        return seq


# =============================================================================
# Message Builders
# =============================================================================

class MessageBuilder:
    """
    Builds XRCE protocol messages.
    """

    def __init__(self, state: SessionState):
        self.state = state

    def _build_message(self, submessages: List[bytes], stream_id: int = StreamId.BUILTIN_BEST_EFFORT) -> bytes:
        """Build a complete XRCE message with header and submessages."""
        header = XRCEHeader(
            session_id=self.state.session_id,
            stream_id=stream_id,
            sequence_num=self.state.next_sequence()
        )
        return header.pack() + b''.join(submessages)

    def create_client(self) -> bytes:
        """Build CREATE_CLIENT message."""
        # Client key (4 bytes)
        payload = self.state.client_key

        # Session properties
        # - mtu: 2 bytes (max transmission unit)
        payload += struct.pack('<H', 512)

        submsg_header = SubmessageHeader(
            submessage_id=SubmessageId.CREATE_CLIENT,
            flags=FLAG_LITTLE_ENDIAN | FLAG_REUSE,
            length=len(payload)
        )

        return self._build_message([submsg_header.pack() + payload])

    def create_participant(self, domain_id: int = 0) -> bytes:
        """Build CREATE message for participant."""
        obj_id = self.state.next_object_id(ObjectKind.PARTICIPANT)
        self.state.participant = EntityInfo(obj_id, "participant")

        xml = create_participant_xml(domain_id)
        xml_bytes = pack_string(xml)

        # Object representation: object_id (2) + object_representation
        payload = obj_id.pack()
        payload += struct.pack('<H', 0x0103)  # BY_REFERENCE + XML representation
        payload += xml_bytes

        submsg_header = SubmessageHeader(
            submessage_id=SubmessageId.CREATE,
            flags=FLAG_LITTLE_ENDIAN,
            length=len(payload)
        )

        return self._build_message([submsg_header.pack() + payload])

    def create_topic(self, topic_name: str) -> bytes:
        """Build CREATE message for topic."""
        if not self.state.participant:
            raise RuntimeError("Participant must be created first")

        obj_id = self.state.next_object_id(ObjectKind.TOPIC)
        self.state.topics[topic_name] = EntityInfo(obj_id, topic_name)

        type_name = get_topic_type(topic_name)
        xrce_name = get_xrce_topic_name(topic_name)
        xml = create_topic_xml(xrce_name, type_name)
        xml_bytes = pack_string(xml)

        # Payload: object_id + participant_id + representation
        payload = obj_id.pack()
        payload += self.state.participant.object_id.pack()
        payload += struct.pack('<H', 0x0103)
        payload += xml_bytes

        submsg_header = SubmessageHeader(
            submessage_id=SubmessageId.CREATE,
            flags=FLAG_LITTLE_ENDIAN,
            length=len(payload)
        )

        return self._build_message([submsg_header.pack() + payload])

    def create_publisher(self) -> bytes:
        """Build CREATE message for publisher."""
        if not self.state.participant:
            raise RuntimeError("Participant must be created first")

        obj_id = self.state.next_object_id(ObjectKind.PUBLISHER)
        self.state.publisher = EntityInfo(obj_id, "publisher")

        xml = create_publisher_xml()
        xml_bytes = pack_string(xml)

        payload = obj_id.pack()
        payload += self.state.participant.object_id.pack()
        payload += struct.pack('<H', 0x0103)
        payload += xml_bytes

        submsg_header = SubmessageHeader(
            submessage_id=SubmessageId.CREATE,
            flags=FLAG_LITTLE_ENDIAN,
            length=len(payload)
        )

        return self._build_message([submsg_header.pack() + payload])

    def create_subscriber(self) -> bytes:
        """Build CREATE message for subscriber."""
        if not self.state.participant:
            raise RuntimeError("Participant must be created first")

        obj_id = self.state.next_object_id(ObjectKind.SUBSCRIBER)
        self.state.subscriber = EntityInfo(obj_id, "subscriber")

        xml = create_subscriber_xml()
        xml_bytes = pack_string(xml)

        payload = obj_id.pack()
        payload += self.state.participant.object_id.pack()
        payload += struct.pack('<H', 0x0103)
        payload += xml_bytes

        submsg_header = SubmessageHeader(
            submessage_id=SubmessageId.CREATE,
            flags=FLAG_LITTLE_ENDIAN,
            length=len(payload)
        )

        return self._build_message([submsg_header.pack() + payload])

    def create_datawriter(self, topic_name: str) -> bytes:
        """Build CREATE message for datawriter."""
        if not self.state.publisher:
            raise RuntimeError("Publisher must be created first")
        if topic_name not in self.state.topics:
            raise RuntimeError(f"Topic {topic_name} must be created first")

        obj_id = self.state.next_object_id(ObjectKind.DATAWRITER)
        self.state.datawriters[topic_name] = EntityInfo(obj_id, topic_name)

        xrce_name = get_xrce_topic_name(topic_name)
        xml = create_datawriter_xml(xrce_name)
        xml_bytes = pack_string(xml)

        payload = obj_id.pack()
        payload += self.state.publisher.object_id.pack()
        payload += struct.pack('<H', 0x0103)
        payload += xml_bytes

        submsg_header = SubmessageHeader(
            submessage_id=SubmessageId.CREATE,
            flags=FLAG_LITTLE_ENDIAN,
            length=len(payload)
        )

        return self._build_message([submsg_header.pack() + payload])

    def create_datareader(self, topic_name: str) -> bytes:
        """Build CREATE message for datareader."""
        if not self.state.subscriber:
            raise RuntimeError("Subscriber must be created first")
        if topic_name not in self.state.topics:
            raise RuntimeError(f"Topic {topic_name} must be created first")

        obj_id = self.state.next_object_id(ObjectKind.DATAREADER)
        self.state.datareaders[topic_name] = EntityInfo(obj_id, topic_name)

        xrce_name = get_xrce_topic_name(topic_name)
        xml = create_datareader_xml(xrce_name)
        xml_bytes = pack_string(xml)

        payload = obj_id.pack()
        payload += self.state.subscriber.object_id.pack()
        payload += struct.pack('<H', 0x0103)
        payload += xml_bytes

        submsg_header = SubmessageHeader(
            submessage_id=SubmessageId.CREATE,
            flags=FLAG_LITTLE_ENDIAN,
            length=len(payload)
        )

        return self._build_message([submsg_header.pack() + payload])

    def read_data(self, topic_name: str, max_samples: int = 1) -> bytes:
        """Build READ_DATA message to request data from a datareader."""
        if topic_name not in self.state.datareaders:
            raise RuntimeError(f"DataReader for {topic_name} must be created first")

        reader = self.state.datareaders[topic_name]

        # Read specification
        # - data_format: 1 byte
        # - max_samples: 2 bytes
        # - max_elapsed_time: 4 bytes
        # - max_rate: 4 bytes
        payload = reader.object_id.pack()
        payload += struct.pack('<BHI', FORMAT_DATA, max_samples, 0)  # Unlimited time

        submsg_header = SubmessageHeader(
            submessage_id=SubmessageId.READ_DATA,
            flags=FLAG_LITTLE_ENDIAN,
            length=len(payload)
        )

        return self._build_message([submsg_header.pack() + payload])

    def write_data(self, topic_name: str, data: bytes) -> bytes:
        """Build WRITE_DATA message to publish data."""
        if topic_name not in self.state.datawriters:
            raise RuntimeError(f"DataWriter for {topic_name} must be created first")

        writer = self.state.datawriters[topic_name]

        # Payload: object_id + data
        payload = writer.object_id.pack()
        payload += data

        submsg_header = SubmessageHeader(
            submessage_id=SubmessageId.WRITE_DATA,
            flags=FLAG_LITTLE_ENDIAN | FORMAT_DATA,
            length=len(payload)
        )

        return self._build_message([submsg_header.pack() + payload], StreamId.BUILTIN_RELIABLE)

    def delete_entity(self, obj_id: ObjectId) -> bytes:
        """Build DELETE message for an entity."""
        payload = obj_id.pack()

        submsg_header = SubmessageHeader(
            submessage_id=SubmessageId.DELETE,
            flags=FLAG_LITTLE_ENDIAN,
            length=len(payload)
        )

        return self._build_message([submsg_header.pack() + payload])


# =============================================================================
# Message Parser
# =============================================================================

@dataclass
class ParsedSubmessage:
    """A parsed XRCE submessage."""
    submessage_id: SubmessageId
    flags: int
    payload: bytes


@dataclass
class ParsedMessage:
    """A parsed XRCE message."""
    header: XRCEHeader
    submessages: List[ParsedSubmessage]


def parse_message(data: bytes) -> Optional[ParsedMessage]:
    """Parse an XRCE message from raw bytes."""
    if len(data) < 4:
        return None

    header = XRCEHeader.unpack(data)
    submessages = []
    offset = 4

    while offset + 4 <= len(data):
        submsg_header = SubmessageHeader.unpack(data, offset)
        offset += 4

        payload_end = offset + submsg_header.length
        if payload_end > len(data):
            break

        payload = data[offset:payload_end]
        offset = payload_end

        try:
            submsg_id = SubmessageId(submsg_header.submessage_id)
        except ValueError:
            submsg_id = submsg_header.submessage_id

        submessages.append(ParsedSubmessage(
            submessage_id=submsg_id,
            flags=submsg_header.flags,
            payload=payload
        ))

    return ParsedMessage(header, submessages)


def extract_data_from_submessage(submsg: ParsedSubmessage) -> Optional[bytes]:
    """Extract CDR data from a DATA submessage."""
    if submsg.submessage_id != SubmessageId.DATA:
        return None

    # DATA payload: object_id (2 bytes) + data
    if len(submsg.payload) <= 2:
        return None

    return submsg.payload[2:]


def extract_status_from_submessage(submsg: ParsedSubmessage) -> Optional[tuple]:
    """Extract status information from a STATUS submessage."""
    if submsg.submessage_id != SubmessageId.STATUS:
        return None

    if len(submsg.payload) < 4:
        return None

    # STATUS payload: object_id (2) + request_id (2) + status (1)
    obj_id = ObjectId.unpack(submsg.payload)
    request_id = struct.unpack_from('<H', submsg.payload, 2)[0]
    status = submsg.payload[4] if len(submsg.payload) > 4 else 0

    return (obj_id, request_id, StatusValue(status) if status in StatusValue._value2member_map_ else status)
