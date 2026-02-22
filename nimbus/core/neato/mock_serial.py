"""
Mock Neato transport for testing.

Subclasses NeatoTransport and overrides hardware methods with
synthetic data. Motor commands accumulate encoder positions so
odometry integration tests work correctly.
"""

import math
from .transport import (
    NeatoTransport,
    NeatoScanData,
    NeatoMotorData,
    NeatoBumperData,
    NeatoBatteryData,
)


class MockNeatoTransport(NeatoTransport):
    """
    Mock transport that generates synthetic data instead of serial I/O.

    Encoder positions accumulate from set_motor() calls, so odometry
    integration produces realistic results.
    """

    def __init__(self):
        super().__init__(port="MOCK", baudrate=115200)
        self._connected = False

        # Encoder state (accumulates from set_motor calls)
        self._left_position_mm = 0
        self._right_position_mm = 0

        # Configurable wall distance for scan simulation
        self._wall_distance_mm = 2000  # 2 meters default
        self._scan_noise = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        """Simulate connection (no serial port needed)."""
        self._connected = True

    def disconnect(self) -> None:
        """Simulate disconnection."""
        self._connected = False

    def get_lds_scan(self) -> NeatoScanData:
        """Return a synthetic 360-degree scan at configured wall distance."""
        scan = NeatoScanData()
        for angle in range(360):
            scan.angles.append(angle)
            scan.distances_mm.append(self._wall_distance_mm)
            scan.intensities.append(50)
            scan.error_codes.append(0)
        return scan

    def set_motor(self, left_mm: int, right_mm: int, speed_mms: int) -> None:
        """Accumulate encoder positions from motor commands."""
        self._left_position_mm += left_mm
        self._right_position_mm += right_mm

    def stop_motors(self) -> None:
        """Stop (no-op, just don't accumulate)."""
        pass

    def get_motors(self) -> NeatoMotorData:
        """Return accumulated encoder positions."""
        return NeatoMotorData(
            left_wheel_position_mm=self._left_position_mm,
            right_wheel_position_mm=self._right_position_mm,
            left_wheel_speed_mms=0,
            right_wheel_speed_mms=0,
        )

    def get_digital_sensors(self) -> NeatoBumperData:
        """Return all-clear bumper state."""
        return NeatoBumperData()

    def get_charger(self) -> NeatoBatteryData:
        """Return 100% battery."""
        return NeatoBatteryData(battery_percent=100, charging=False, voltage_mv=14400)

    # --- Test helpers ---

    def set_wall_distance(self, distance_mm: int) -> None:
        """Set the simulated wall distance for scan data."""
        self._wall_distance_mm = distance_mm

    def reset_encoders(self) -> None:
        """Reset encoder positions to zero."""
        self._left_position_mm = 0
        self._right_position_mm = 0
