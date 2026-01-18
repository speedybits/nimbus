"""
Pure Python XRCE-DDS Agent for Nimbus.

This module implements an XRCE-DDS agent that replaces the Docker-based
Micro-ROS agent. It communicates directly with the Yahboom ESP32's
Micro-ROS firmware without requiring ROS2 or Docker.

Architecture:
    ESP32 (Micro-ROS Client) <--XRCE-DDS/UDP--> XRCEAgent (Python)

Components:
    - agent: Main XRCEAgent class
    - protocol: XRCE-DDS wire format structures
    - session: Session and stream management
    - entities: Entity tracking (participants, topics, datawriters, datareaders)
    - transport: UDP transport layer
    - cdr: CDR (Common Data Representation) serialization
    - messages: ROS2 message type definitions

Usage:
    from nimbus.core.xrce import XRCEAgent, LaserScan, Odometry, Twist

    agent = XRCEAgent(bind_port=8888)
    agent.start()

    # Subscribe to ESP32's sensor data
    agent.subscribe("/scan", lambda msg: print(f"Got {len(msg.ranges)} ranges"))
    agent.subscribe("/odom_raw", lambda msg: print(f"Pose: {msg.pose}"))

    # Wait for ESP32 to connect
    if agent.wait_for_connection(timeout=10.0):
        # Publish velocity commands
        twist = Twist.create(linear_x=0.1, angular_z=0.0)
        agent.publish("/cmd_vel", twist.serialize())

    agent.stop()
"""

from .agent import XRCEAgent, create_agent
from .transport import UDPTransport, UDPConfig
from .messages import LaserScan, Odometry, Twist

__all__ = [
    "XRCEAgent",
    "create_agent",
    "UDPTransport",
    "UDPConfig",
    "LaserScan",
    "Odometry",
    "Twist",
]
