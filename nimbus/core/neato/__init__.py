"""
Neato serial transport for Nimbus.

Talks directly to a Neato vacuum's MCU over pyserial,
using the vacuum's built-in LIDAR, motors, and sensors.
"""

from .node import NeatoNode
from .transport import (
    NeatoTransport, NeatoBumperData, NeatoBatteryData,
    NeatoAnalogData, SerialDisconnectedError,
)
from .sim_transport import SimNeatoTransport

__all__ = [
    "NeatoNode", "NeatoTransport", "SimNeatoTransport",
    "NeatoBumperData", "NeatoBatteryData", "NeatoAnalogData",
    "SerialDisconnectedError",
]
