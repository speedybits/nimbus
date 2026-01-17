"""
Direct ESP32 communication module for Nimbus.

This module implements a minimal XRCE-DDS client that communicates
directly with the Yahboom ESP32's Micro-ROS firmware, eliminating
the need for a ROS2 installation or Docker-based Micro-ROS agent.

Architecture:
    ESP32 (Micro-ROS) <--XRCE-DDS--> DirectNode (Python) <--> Nimbus

Components:
    - cdr: CDR (Common Data Representation) serialization
    - messages: ROS2 message type definitions (LaserScan, Odometry, Twist)
    - transport: UDP and Serial transport layers
    - session: XRCE session and entity management
    - client: High-level XRCE client API
"""

from .client import XRCEClient
from .transport import UDPTransport, SerialTransport
from .messages import LaserScan, Odometry, Twist

__all__ = [
    "XRCEClient",
    "UDPTransport",
    "SerialTransport",
    "LaserScan",
    "Odometry",
    "Twist",
]
