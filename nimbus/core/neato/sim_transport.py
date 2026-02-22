"""
Simulated Neato transport backed by SimulatedWorld.

Provides the same NeatoTransport interface but uses SimulatedWorld for
physics and ray-casting instead of a real serial port. Motor commands
drive SimulatedWorld kinematics; scans come from ray-casting; encoder
positions are derived from the world's robot pose.

This lets NeatoNode's 3-thread architecture run unchanged in simulation,
exercising the full Neato code path: encoder-delta odometry, scan
conversion, and velocity-to-wheel-speed kinematics.
"""

import math
import threading
from typing import Optional

from .transport import (
    NeatoTransport,
    NeatoScanData,
    NeatoMotorData,
    NeatoBumperData,
    NeatoBatteryData,
)
from .kinematics import WHEEL_BASE_M


class SimNeatoTransport(NeatoTransport):
    """
    NeatoTransport backed by SimulatedWorld physics.

    Motor commands are converted from Neato mm-based SetMotor format
    into differential-drive velocity and applied to the world. Encoder
    positions track cumulative wheel travel so NeatoNode's encoder-delta
    odometry works unchanged.

    Thread safety: SimulatedWorld pose access is guarded by _world_lock.
    NeatoNode already serialises all transport calls through its own
    _serial_lock, but we add our own lock for world state that may be
    read externally (e.g. by tests).
    """

    def __init__(self, world, num_rays: int = 360, max_range: float = 3.5):
        super().__init__(port="SIM_NEATO", baudrate=0)
        self._world = world
        self._num_rays = num_rays
        self._max_range = max_range
        self._connected = False

        # Cumulative encoder positions (mm), matching real Neato behaviour
        self._left_encoder_mm = 0
        self._right_encoder_mm = 0

        # Lock for world pose mutations
        self._world_lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def world(self):
        """Access the underlying SimulatedWorld (for tests/inspection)."""
        return self._world

    def connect(self) -> None:
        """No-op — simulation is always ready."""
        self._connected = True

    def disconnect(self) -> None:
        """No-op — nothing to tear down."""
        self._connected = False

    def get_lds_scan(self) -> NeatoScanData:
        """
        Ray-cast from SimulatedWorld and return as Neato scan data.

        Ranges are converted from meters to mm. Angles 0-359 map to
        the same convention as real Neato LDS output.
        """
        with self._world_lock:
            ranges_m = self._world.raycast(
                num_rays=self._num_rays, max_range=self._max_range,
            )

        scan = NeatoScanData()
        for angle, dist_m in enumerate(ranges_m):
            scan.angles.append(angle)
            scan.distances_mm.append(int(round(dist_m * 1000.0)))
            scan.intensities.append(50)
            scan.error_codes.append(0)
        return scan

    def set_motor(self, left_mm: int, right_mm: int, speed_mms: int) -> None:
        """
        Apply a Neato SetMotor command to the simulated world.

        Converts wheel distances (mm) into a velocity applied over
        an implied duration, then steps the world physics.
        """
        if speed_mms == 0:
            return

        # Accumulate encoder positions (matches real Neato behaviour)
        self._left_encoder_mm += left_mm
        self._right_encoder_mm += right_mm

        # Derive duration from distance/speed: t = max(|d|) / speed
        max_dist = max(abs(left_mm), abs(right_mm))
        if max_dist == 0:
            return
        dt = max_dist / speed_mms  # seconds

        # Convert wheel distances to wheel velocities (m/s)
        left_mps = (left_mm / 1000.0) / dt
        right_mps = (right_mm / 1000.0) / dt

        # Differential drive → unicycle
        linear = (left_mps + right_mps) / 2.0
        angular = (right_mps - left_mps) / WHEEL_BASE_M

        with self._world_lock:
            self._world.apply_velocity(linear, angular, dt)

    def stop_motors(self) -> None:
        """No-op — simulation has no inertia."""
        pass

    def get_motors(self) -> NeatoMotorData:
        """Return cumulative encoder positions."""
        return NeatoMotorData(
            left_wheel_position_mm=self._left_encoder_mm,
            right_wheel_position_mm=self._right_encoder_mm,
            left_wheel_speed_mms=0,
            right_wheel_speed_mms=0,
        )

    def get_digital_sensors(self) -> NeatoBumperData:
        """All-clear — no bumpers in simulation."""
        return NeatoBumperData()

    def get_charger(self) -> NeatoBatteryData:
        """Infinite battery in simulation."""
        return NeatoBatteryData(battery_percent=100, charging=False, voltage_mv=14400)
