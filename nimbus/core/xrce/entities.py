"""
XRCE Entity tracking for ESP32 communication.

This module tracks the entities (participants, topics, datawriters, datareaders)
created by the ESP32 client. The agent needs to maintain this mapping because
WRITE_DATA and DATA messages use object IDs, and we need to know which topic
each ID corresponds to.

Entity Creation Flow:
    1. ESP32 sends CREATE for PARTICIPANT
    2. ESP32 sends CREATE for TOPIC(s) - contains topic name in XML
    3. ESP32 sends CREATE for PUBLISHER/SUBSCRIBER
    4. ESP32 sends CREATE for DATAWRITER/DATAREADER - references topic
    5. Agent tracks object_id -> topic_name mapping
"""

import re
import struct
import threading
from dataclasses import dataclass, field
from typing import Dict, Optional, Set

from .protocol import ObjectKind


# =============================================================================
# Topic Type Mappings
# =============================================================================

# Map topic names to DDS type names
TOPIC_TYPES = {
    "/scan": "sensor_msgs::msg::dds_::LaserScan_",
    "/odom_raw": "nav_msgs::msg::dds_::Odometry_",
    "/cmd_vel": "geometry_msgs::msg::dds_::Twist_",
    "rt/scan": "sensor_msgs::msg::dds_::LaserScan_",
    "rt/odom_raw": "nav_msgs::msg::dds_::Odometry_",
    "rt/cmd_vel": "geometry_msgs::msg::dds_::Twist_",
}


def normalize_topic_name(topic: str) -> str:
    """
    Normalize topic name to ROS2 format.

    XRCE uses 'rt/scan' while ROS2 uses '/scan'.

    Args:
        topic: Topic name (e.g., 'rt/scan' or '/scan')

    Returns:
        Normalized topic name (e.g., '/scan')
    """
    if topic.startswith('rt/'):
        return '/' + topic[3:]
    return topic


def to_xrce_topic_name(topic: str) -> str:
    """
    Convert ROS2 topic name to XRCE format.

    Args:
        topic: ROS2 topic name (e.g., '/scan')

    Returns:
        XRCE topic name (e.g., 'rt/scan')
    """
    if topic.startswith('/'):
        return 'rt' + topic
    return topic


def get_topic_type(topic: str) -> Optional[str]:
    """Get the DDS type name for a topic."""
    return TOPIC_TYPES.get(topic) or TOPIC_TYPES.get(normalize_topic_name(topic))


# =============================================================================
# Entity Structures
# =============================================================================

@dataclass
class Entity:
    """Base entity information."""
    object_id: int
    kind: ObjectKind
    name: str = ""


@dataclass
class TopicEntity(Entity):
    """Topic entity with type information."""
    type_name: str = ""


@dataclass
class DataWriterEntity(Entity):
    """DataWriter entity linked to a topic."""
    topic_name: str = ""


@dataclass
class DataReaderEntity(Entity):
    """DataReader entity linked to a topic."""
    topic_name: str = ""


# =============================================================================
# Entity Manager
# =============================================================================

class EntityManager:
    """
    Tracks XRCE entities created by the ESP32.

    When the ESP32 creates entities (participant, topics, datawriters, datareaders),
    we parse the CREATE messages to extract the object IDs and topic names.
    This mapping is essential for routing WRITE_DATA messages to the correct
    topic handlers.

    Entity creation order from ESP32:
        TOPIC -> PUBLISHER -> DATAWRITER (repeats for each topic)
        TOPIC -> SUBSCRIBER -> DATAREADER (for subscribed topics)

    The DATAWRITER/DATAREADER is implicitly associated with the most recently
    created TOPIC in the sequence.
    """

    def __init__(self):
        self._lock = threading.Lock()

        # ESP32 entity tracking
        # object_id -> entity
        self._participants: Dict[int, Entity] = {}
        self._topics: Dict[int, TopicEntity] = {}
        self._publishers: Dict[int, Entity] = {}
        self._subscribers: Dict[int, Entity] = {}
        self._datawriters: Dict[int, DataWriterEntity] = {}
        self._datareaders: Dict[int, DataReaderEntity] = {}

        # Reverse mappings for quick lookup
        # topic_name -> datawriter object_id
        self._topic_to_datawriter: Dict[str, int] = {}
        # topic_name -> datareader object_id
        self._topic_to_datareader: Dict[str, int] = {}

        # Track last created topic for implicit association
        self._last_topic_name: Optional[str] = None
        self._last_topic_id: Optional[int] = None

    def handle_create(self, object_id: int, kind: ObjectKind, payload: bytes) -> None:
        """
        Handle a CREATE submessage from ESP32.

        Parses the payload to extract entity information and stores it.

        Args:
            object_id: The object ID from the CREATE message
            kind: The object kind (PARTICIPANT, TOPIC, DATAWRITER, etc.)
            payload: The full CREATE submessage payload
        """
        from .logger import xrce_log, LogLevel

        # Also extract kind from object_id bytes (XRCE encodes kind in low nibble of byte 1)
        kind_from_wire = None
        if len(payload) >= 2:
            kind_from_wire_val = payload[1] & 0x0F
            try:
                kind_from_wire = ObjectKind(kind_from_wire_val)
            except ValueError:
                pass

        xrce_log(
            f"[bold magenta]  EntityManager: kind_from_byte4={kind.name}({kind.value}) "
            f"kind_from_objid={kind_from_wire.name if kind_from_wire else 'None'}"
            f"({kind_from_wire_val if len(payload) >= 2 else '?'}) "
            f"obj_id={object_id} payload_hex={payload[:8].hex()}[/bold magenta]",
            level=LogLevel.NORMAL
        )

        # If the kind from byte 4 doesn't match the kind from the object_id,
        # prefer the object_id-derived kind (XRCE standard encoding)
        if kind_from_wire and kind_from_wire != kind:
            xrce_log(
                f"[bold yellow]  KIND MISMATCH: byte4={kind.name} objid={kind_from_wire.name} — using objid[/bold yellow]",
                level=LogLevel.NORMAL
            )
            kind = kind_from_wire

        with self._lock:
            if kind == ObjectKind.PARTICIPANT:
                self._participants[object_id] = Entity(object_id, kind, "participant")

            elif kind == ObjectKind.TOPIC:
                topic_name, type_name = self._extract_topic_info(payload)
                if topic_name:
                    self._topics[object_id] = TopicEntity(
                        object_id, kind, topic_name, type_name or ""
                    )
                    # Track for implicit association with subsequent DATAWRITER/DATAREADER
                    self._last_topic_name = topic_name
                    self._last_topic_id = object_id
                else:
                    self._topics[object_id] = TopicEntity(
                        object_id, kind, f"topic_{object_id}", type_name or ""
                    )

            elif kind == ObjectKind.PUBLISHER:
                self._publishers[object_id] = Entity(object_id, kind, "publisher")

            elif kind == ObjectKind.SUBSCRIBER:
                self._subscribers[object_id] = Entity(object_id, kind, "subscriber")

            elif kind == ObjectKind.DATAWRITER:
                # Try to extract topic name from payload first
                topic_name = self._extract_topic_name(payload)

                # Fallback: use the last created topic (implicit association)
                if not topic_name and self._last_topic_name:
                    topic_name = self._last_topic_name
                    xrce_log(f"[cyan]  DATAWRITER using last_topic: {topic_name}[/cyan]", level=LogLevel.NORMAL)

                if topic_name:
                    self._datawriters[object_id] = DataWriterEntity(
                        object_id, kind, f"datawriter_{object_id}", topic_name
                    )
                    self._topic_to_datawriter[topic_name] = object_id
                    # Also map normalized name
                    normalized = normalize_topic_name(topic_name)
                    if normalized != topic_name:
                        self._topic_to_datawriter[normalized] = object_id
                    xrce_log(f"[bold green]  DATAWRITER {object_id} -> {topic_name}[/bold green]", level=LogLevel.NORMAL)
                else:
                    xrce_log(f"[yellow]  DATAWRITER {object_id}: no topic association![/yellow]", level=LogLevel.NORMAL)
                    xrce_log(f"[dim]  DW payload: {payload.hex()}[/dim]", level=LogLevel.NORMAL)

            elif kind == ObjectKind.DATAREADER:
                # Try to extract topic name from payload first
                topic_name = self._extract_topic_name(payload)

                # Fallback: use the last created topic (implicit association)
                if not topic_name and self._last_topic_name:
                    topic_name = self._last_topic_name
                    xrce_log(f"[cyan]  DATAREADER using last_topic: {topic_name}[/cyan]", level=LogLevel.NORMAL)

                if topic_name:
                    self._datareaders[object_id] = DataReaderEntity(
                        object_id, kind, f"datareader_{object_id}", topic_name
                    )
                    self._topic_to_datareader[topic_name] = object_id
                    # Also map normalized name
                    normalized = normalize_topic_name(topic_name)
                    if normalized != topic_name:
                        self._topic_to_datareader[normalized] = object_id
                    xrce_log(f"[bold green]  DATAREADER {object_id} -> {topic_name}[/bold green]", level=LogLevel.NORMAL)
                else:
                    xrce_log(f"[yellow]  DATAREADER {object_id}: no topic association![/yellow]", level=LogLevel.NORMAL)
                    xrce_log(f"[dim]  DR payload: {payload.hex()}[/dim]", level=LogLevel.NORMAL)

    def get_datawriter_topic(self, object_id: int) -> Optional[str]:
        """
        Get the topic name for a datawriter object ID.

        This is called when we receive WRITE_DATA from ESP32 to determine
        which topic the data belongs to.

        Args:
            object_id: The datawriter's object ID

        Returns:
            Topic name if found, None otherwise
        """
        with self._lock:
            dw = self._datawriters.get(object_id)
            if dw:
                return dw.topic_name
            return None

    def get_datareader_topic(self, object_id: int) -> Optional[str]:
        """
        Get the topic name for a datareader object ID.

        Args:
            object_id: The datareader's object ID

        Returns:
            Topic name if found, None otherwise
        """
        with self._lock:
            dr = self._datareaders.get(object_id)
            if dr:
                return dr.topic_name
            return None

    def get_datareader_for_topic(self, topic: str) -> Optional[int]:
        """
        Get the datareader object ID for a topic.

        This is called when we want to send DATA to ESP32's datareader
        (e.g., for /cmd_vel commands).

        Args:
            topic: Topic name

        Returns:
            Datareader object ID if found, None otherwise
        """
        with self._lock:
            # Try exact match first
            obj_id = self._topic_to_datareader.get(topic)
            if obj_id is not None:
                return obj_id
            # Try XRCE format
            xrce_name = to_xrce_topic_name(topic)
            return self._topic_to_datareader.get(xrce_name)

    def get_datawriter_for_topic(self, topic: str) -> Optional[int]:
        """
        Get the datawriter object ID for a topic.

        Args:
            topic: Topic name

        Returns:
            Datawriter object ID if found, None otherwise
        """
        with self._lock:
            # Try exact match first
            obj_id = self._topic_to_datawriter.get(topic)
            if obj_id is not None:
                return obj_id
            # Try XRCE format
            xrce_name = to_xrce_topic_name(topic)
            return self._topic_to_datawriter.get(xrce_name)

    def get_published_topics(self) -> Set[str]:
        """
        Get all topics that ESP32 publishes to (has datawriters for).

        These are topics where ESP32 sends data (like /scan, /odom_raw).
        """
        with self._lock:
            topics = set()
            for dw in self._datawriters.values():
                topics.add(normalize_topic_name(dw.topic_name))
            return topics

    def get_subscribed_topics(self) -> Set[str]:
        """
        Get all topics that ESP32 subscribes to (has datareaders for).

        These are topics where ESP32 receives data (like /cmd_vel).
        """
        with self._lock:
            topics = set()
            for dr in self._datareaders.values():
                topics.add(normalize_topic_name(dr.topic_name))
            return topics

    def get_diagnostic_info(self) -> dict:
        """Get full diagnostic info about tracked entities."""
        with self._lock:
            return {
                "participants": {str(k): v.name for k, v in self._participants.items()},
                "topics": {str(k): {"name": v.name, "type": v.type_name} for k, v in self._topics.items()},
                "publishers": {str(k): v.name for k, v in self._publishers.items()},
                "subscribers": {str(k): v.name for k, v in self._subscribers.items()},
                "datawriters": {str(k): {"name": v.name, "topic": v.topic_name} for k, v in self._datawriters.items()},
                "datareaders": {str(k): {"name": v.name, "topic": v.topic_name} for k, v in self._datareaders.items()},
                "topic_to_datawriter": dict(self._topic_to_datawriter),
                "topic_to_datareader": dict(self._topic_to_datareader),
                "last_topic_name": self._last_topic_name,
                "last_topic_id": self._last_topic_id,
            }

    def clear(self) -> None:
        """Clear all tracked entities."""
        with self._lock:
            self._participants.clear()
            self._topics.clear()
            self._publishers.clear()
            self._subscribers.clear()
            self._datawriters.clear()
            self._datareaders.clear()
            self._topic_to_datawriter.clear()
            self._topic_to_datareader.clear()
            self._last_topic_name = None
            self._last_topic_id = None

    def _extract_topic_info(self, payload: bytes) -> tuple:
        """
        Extract topic name and type from CREATE TOPIC payload.

        Uses multiple extraction strategies in order of reliability:
        1. Pattern matching (most reliable for known topics)
        2. Binary format parsing
        3. XML format parsing

        Returns:
            Tuple of (topic_name, type_name)
        """
        from .logger import xrce_log, LogLevel

        # Log payload hex for diagnostics
        xrce_log(f"[dim]  TOPIC payload ({len(payload)} bytes): {payload[:40].hex()}...[/dim]", level=LogLevel.NORMAL)

        # Strategy 1: Pattern search (most reliable — finds known topic names anywhere in bytes)
        topic_name = self._find_topic_pattern(payload)
        if topic_name:
            xrce_log(f"[green]  Topic name from pattern: {topic_name}[/green]", level=LogLevel.NORMAL)

        # Strategy 2: Binary format extraction (fragile — offset assumptions may not match firmware)
        type_name = None
        if not topic_name:
            topic_name = self._extract_binary_string(payload, string_index=0)
            if topic_name:
                xrce_log(f"[cyan]  Topic name from binary: {topic_name}[/cyan]", level=LogLevel.NORMAL)
        type_name = self._extract_binary_string(payload, string_index=1)

        # Strategy 3: XML format
        if not topic_name:
            topic_name = self._extract_xml_element(payload, 'name')
            if topic_name:
                xrce_log(f"[cyan]  Topic name from XML: {topic_name}[/cyan]", level=LogLevel.NORMAL)
        if not type_name:
            type_name = self._extract_xml_element(payload, 'dataType')

        if not topic_name:
            xrce_log(f"[yellow]  Could not extract topic name from payload[/yellow]", level=LogLevel.NORMAL)

        return topic_name, type_name

    def _extract_topic_name(self, payload: bytes) -> Optional[str]:
        """
        Extract topic name from CREATE DATAWRITER/DATAREADER payload.

        Uses pattern matching first (most reliable), then binary/XML fallbacks.

        Args:
            payload: The CREATE submessage payload

        Returns:
            Topic name if found, None otherwise
        """
        from .logger import xrce_log, LogLevel

        # Strategy 1: Pattern search (most reliable)
        topic_name = self._find_topic_pattern(payload)
        if topic_name:
            xrce_log(f"[green]  DW/DR topic from pattern: {topic_name}[/green]", level=LogLevel.NORMAL)
            return topic_name

        # Strategy 2: Binary format
        topic_name = self._extract_binary_string(payload, string_index=0)
        if topic_name:
            xrce_log(f"[cyan]  DW/DR topic from binary: {topic_name}[/cyan]", level=LogLevel.NORMAL)
            return topic_name

        # Strategy 3: XML format
        topic_name = self._extract_xml_element(payload, 'name')
        if topic_name:
            xrce_log(f"[cyan]  DW/DR topic from XML: {topic_name}[/cyan]", level=LogLevel.NORMAL)
            return topic_name

        return None

    def _extract_binary_string(self, payload: bytes, string_index: int = 0) -> Optional[str]:
        """
        Extract a length-prefixed string from binary payload.

        Binary format after header (obj_id, req_id, kind):
            - format (2 bytes)
            - padding (1 byte)
            - total_length (4 bytes)
            - string1_length (4 bytes)
            - string1 (null-terminated)
            - string2_length (4 bytes)
            - string2 (null-terminated)
            - ...

        Args:
            payload: The CREATE submessage payload
            string_index: Which string to extract (0 = first, 1 = second, etc.)

        Returns:
            The extracted string, or None if not found
        """
        if len(payload) < 16:
            return None

        try:
            # Skip header: obj_id (2) + req_id (2) + kind (1) + format (2) + padding (1) + total_len (4)
            offset = 12

            # Navigate to the requested string
            for i in range(string_index + 1):
                if offset + 4 > len(payload):
                    return None

                str_len = struct.unpack_from('<I', payload, offset)[0]
                offset += 4

                if str_len == 0 or str_len > 256:  # Sanity check
                    return None

                if i == string_index:
                    # This is the string we want
                    if offset + str_len > len(payload):
                        return None
                    # Extract string (str_len includes null terminator)
                    string_bytes = payload[offset:offset + str_len - 1]
                    return string_bytes.decode('utf-8', errors='ignore')
                else:
                    # Skip this string
                    offset += str_len
                    # Align to 4 bytes if needed
                    if str_len % 4 != 0:
                        offset += 4 - (str_len % 4)

        except Exception:
            pass

        return None

    def _find_topic_pattern(self, payload: bytes) -> Optional[str]:
        """
        Search for known topic name patterns in binary payload.

        This is a fallback when structured parsing fails.

        Args:
            payload: The payload bytes

        Returns:
            Topic name if found, None otherwise
        """
        try:
            data = payload.decode('latin-1')
        except Exception:
            return None

        # Known topic patterns (XRCE format with rt/ prefix)
        patterns = [
            'rt/scan', 'rt/odom_raw', 'rt/cmd_vel',
            '/scan', '/odom_raw', '/cmd_vel',
        ]

        for pattern in patterns:
            idx = data.find(pattern)
            if idx >= 0:
                # Extract the full topic name (until null or non-printable)
                end = idx
                while end < len(data) and data[end] >= ' ' and data[end] <= '~':
                    end += 1
                topic = data[idx:end]
                if topic:
                    return topic

        return None

    def _extract_xml_element(self, data, element: str) -> Optional[str]:
        """
        Extract an XML element value using regex.

        Args:
            data: XML string or bytes
            element: Element name to find

        Returns:
            Element content if found, None otherwise
        """
        if isinstance(data, bytes):
            try:
                data = data.decode('utf-8', errors='ignore')
            except Exception:
                return None

        match = re.search(rf'<{element}>([^<]+)</{element}>', data)
        if match:
            return match.group(1)
        return None
