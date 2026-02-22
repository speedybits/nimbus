"""
Neato serial transport protocol.

Implements the Neato XV-series serial protocol for direct MCU communication.
All commands use TestMode. The serial port runs at 115200 baud.

Protocol basics:
- Send ASCII command + newline
- Read response lines until ctrl-z (0x1A) or timeout
- Most responses are key,value CSV pairs

``import serial`` is deferred to connect() so nimbus loads without pyserial.
"""

from dataclasses import dataclass, field
from typing import Optional
import time


@dataclass
class NeatoScanData:
    """360-degree LIDAR scan from GetLDSScan."""
    angles: list[int] = field(default_factory=list)           # 0-359
    distances_mm: list[int] = field(default_factory=list)     # mm, 0 = error
    intensities: list[int] = field(default_factory=list)
    error_codes: list[int] = field(default_factory=list)      # 0 = valid


@dataclass
class NeatoMotorData:
    """Wheel encoder data from GetMotors."""
    left_wheel_position_mm: int = 0
    right_wheel_position_mm: int = 0
    left_wheel_speed_mms: int = 0
    right_wheel_speed_mms: int = 0


@dataclass
class NeatoBumperData:
    """Digital sensor (bumper) state from GetDigitalSensors."""
    left_bumper: bool = False
    right_bumper: bool = False
    left_wheel_drop: bool = False
    right_wheel_drop: bool = False
    wall_sensor: bool = False


@dataclass
class NeatoBatteryData:
    """Battery state from GetCharger."""
    battery_percent: int = 0
    charging: bool = False
    voltage_mv: int = 0


class NeatoTransport:
    """
    Low-level serial protocol for Neato XV-series vacuum.

    NOT thread-safe — caller must hold a lock for all serial access.

    Usage:
        transport = NeatoTransport("/dev/ttyACM0")
        transport.connect()
        scan = transport.get_lds_scan()
        transport.set_motor(100, 100, 200)
        transport.disconnect()
    """

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200):
        self._port = port
        self._baudrate = baudrate
        self._serial = None
        self._timeout = 2.0  # seconds per command

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self) -> None:
        """Open serial port, enable TestMode and LDS rotation."""
        import serial
        self._serial = serial.Serial(
            self._port,
            self._baudrate,
            timeout=self._timeout,
        )
        # Flush any stale data
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        time.sleep(0.1)

        # Enter test mode and start LIDAR
        self._send_command("TestMode On")
        self._send_command("SetLDSRotation On")

    def disconnect(self) -> None:
        """Stop motors, LDS, exit TestMode, close port."""
        if not self.is_connected:
            return
        try:
            self.stop_motors()
            self._send_command("SetLDSRotation Off")
            self._send_command("TestMode Off")
        except Exception:
            pass
        finally:
            if self._serial:
                self._serial.close()
                self._serial = None

    def get_lds_scan(self) -> NeatoScanData:
        """
        Get a 360-degree LIDAR scan.

        Sends GetLDSScan and parses the CSV response.
        Each line: AngleInDegrees,DistInMM,Intensity,ErrorCodeHEX
        """
        lines = self._send_command("GetLDSScan")
        scan = NeatoScanData()

        for line in lines:
            parts = line.split(",")
            if len(parts) != 4:
                continue
            try:
                angle = int(parts[0].strip())
                dist = int(parts[1].strip())
                intensity = int(parts[2].strip())
                error = int(parts[3].strip())
                scan.angles.append(angle)
                scan.distances_mm.append(dist)
                scan.intensities.append(intensity)
                scan.error_codes.append(error)
            except ValueError:
                continue

        return scan

    def set_motor(self, left_mm: int, right_mm: int, speed_mms: int) -> None:
        """
        Send SetMotor command with clamped values.

        Args:
            left_mm: Left wheel distance in mm (negative = backward)
            right_mm: Right wheel distance in mm (negative = backward)
            speed_mms: Speed in mm/s (always positive, clamped to 300)
        """
        from .kinematics import MAX_SPEED_MMS
        speed_mms = min(abs(speed_mms), MAX_SPEED_MMS)
        self._send_command(
            f"SetMotor LWheelDist {left_mm} RWheelDist {right_mm} Speed {speed_mms}"
        )

    def stop_motors(self) -> None:
        """Stop both wheels."""
        self.set_motor(0, 0, 0)

    def get_motors(self) -> NeatoMotorData:
        """Get wheel encoder positions and speeds."""
        lines = self._send_command("GetMotors")
        data = self._parse_key_value(lines)
        return NeatoMotorData(
            left_wheel_position_mm=int(data.get("LeftWheel_PositionInMM", "0")),
            right_wheel_position_mm=int(data.get("RightWheel_PositionInMM", "0")),
            left_wheel_speed_mms=int(data.get("LeftWheel_SpeedInMMS", "0")),
            right_wheel_speed_mms=int(data.get("RightWheel_SpeedInMMS", "0")),
        )

    def get_digital_sensors(self) -> NeatoBumperData:
        """Get bumper and wheel drop sensor states."""
        lines = self._send_command("GetDigitalSensors")
        data = self._parse_key_value(lines)
        return NeatoBumperData(
            left_bumper=data.get("SNSR_LEFT_BUMPER", "0") != "0",
            right_bumper=data.get("SNSR_RIGHT_BUMPER", "0") != "0",
            left_wheel_drop=data.get("SNSR_LEFT_WHEEL_EXTENDED", "0") != "0",
            right_wheel_drop=data.get("SNSR_RIGHT_WHEEL_EXTENDED", "0") != "0",
            wall_sensor=data.get("SNSR_WALL_DETECT", "0") != "0",
        )

    def get_charger(self) -> NeatoBatteryData:
        """Get battery/charger state."""
        lines = self._send_command("GetCharger")
        data = self._parse_key_value(lines)
        return NeatoBatteryData(
            battery_percent=int(data.get("FuelPercent", "0")),
            charging=data.get("ChargingActive", "0") != "0",
            voltage_mv=int(data.get("VBattV", "0")),
        )

    def _send_command(self, cmd: str) -> list[str]:
        """
        Send a command and read the response.

        Writes cmd + newline, reads lines until ctrl-z (0x1A) or timeout.

        Returns:
            List of response lines (stripped, non-empty)
        """
        if not self.is_connected:
            raise RuntimeError("Not connected to Neato")

        self._serial.write(f"{cmd}\n".encode("ascii"))
        self._serial.flush()

        lines = []
        deadline = time.time() + self._timeout

        while time.time() < deadline:
            raw = self._serial.readline()
            if not raw:
                break
            line = raw.decode("ascii", errors="replace").strip()
            if "\x1a" in line:
                # ctrl-z marks end of response
                clean = line.replace("\x1a", "").strip()
                if clean:
                    lines.append(clean)
                break
            if line:
                lines.append(line)

        return lines

    @staticmethod
    def _parse_key_value(lines: list[str]) -> dict[str, str]:
        """
        Parse Neato key,value response lines.

        Format: "Parameter,Value" — first line is often a header.
        """
        result = {}
        for line in lines:
            parts = line.split(",", 1)
            if len(parts) == 2:
                key = parts[0].strip()
                value = parts[1].strip()
                # Skip header lines
                if key.lower() not in ("parameter", "header"):
                    result[key] = value
        return result
