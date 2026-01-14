"""
ESP32 robot configuration via serial protocol.

Sends WiFi credentials and agent settings to Yahboom ESP32 firmware.
The robot must be connected via USB for configuration, then operates
wirelessly after reboot.

Based on Yahboom's MicroROS robot protocol.
"""

import struct
import time
from dataclasses import dataclass
from typing import Optional

import serial


# Protocol command addresses
_COMMANDS = {
    "WIFI_SSID": 0x01,
    "WIFI_PASSWD": 0x02,
    "AGENT_IP": 0x03,
    "AGENT_PORT": 0x04,
    "CAR_TYPE": 0x05,
    "DOMAIN_ID": 0x06,
    "SERIAL_BAUDRATE": 0x07,
    "SERVO_OFFSET": 0x08,
    "MOTOR_PID": 0x09,
    "IMU_YAW_PID": 0x0A,
    "ROS_NAMESPACE": 0x0B,
    "REBOOT_DEVICE": 0x20,
    "RESET_CONFIG": 0x21,
    "REQUEST_DATA": 0x50,
    "FIRMWARE_VERSION": 0x51,
}

# Transport/car type constants
TRANSPORT_WIFI_UDP = 0
TRANSPORT_SERIAL = 1


@dataclass
class WiFiCredentials:
    """WiFi network credentials."""

    ssid: str
    password: str


@dataclass
class UDPAgentConfig:
    """UDP agent connection settings."""

    ip: str        # e.g., "192.168.1.100"
    port: int = 8090


@dataclass
class RobotInfo:
    """Robot configuration read from ESP32."""

    firmware_version: Optional[str] = None
    wifi_ssid: Optional[str] = None
    agent_ip: Optional[str] = None
    agent_port: Optional[int] = None
    car_type: Optional[str] = None
    ros_domain_id: Optional[int] = None
    ros_baudrate: Optional[int] = None


class RobotConfigurator:
    """
    Configure Yahboom ESP32 robot via serial.

    Sends WiFi credentials and Micro-ROS agent settings
    to the ESP32 firmware. Robot must be connected via USB.

    Example:
        configurator = RobotConfigurator("/dev/ttyUSB0")
        configurator.set_wifi(WiFiCredentials("MyNetwork", "password123"))
        configurator.set_udp_agent(UDPAgentConfig("192.168.1.100", 8090))
        configurator.set_transport(TRANSPORT_WIFI_UDP)
        configurator.reboot()
    """

    # Protocol constants
    _HEAD = 0xFF
    _DEVICE_ID = 0xF8
    _RETURN_ID = 0xF7
    _READ_CMD = 0x50
    _RX_BUF_SIZE = 40

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.05):
        """
        Initialize connection to robot.

        Args:
            port: Serial port path (e.g., "/dev/ttyUSB0")
            baudrate: Serial baud rate (default: 115200)
            timeout: Read timeout in seconds
        """
        self._serial = serial.Serial(port, baudrate, timeout=timeout)
        self._send_delay = 0.01
        self._read_delay = 0.01

    def close(self) -> None:
        """Close serial connection."""
        if self._serial and self._serial.is_open:
            self._serial.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # --- Configuration Methods ---

    def set_wifi(self, credentials: WiFiCredentials) -> None:
        """
        Configure WiFi SSID and password.

        Changes take effect after reboot.
        """
        ssid_bytes = credentials.ssid.encode("utf-8")
        self._send_string(_COMMANDS["WIFI_SSID"], ssid_bytes)

        passwd_bytes = credentials.password.encode("utf-8")
        self._send_string(_COMMANDS["WIFI_PASSWD"], passwd_bytes)

    def set_udp_agent(self, config: UDPAgentConfig) -> None:
        """
        Configure UDP agent IP address and port.

        Changes take effect after reboot.
        """
        # Parse IP into 4 octets
        ip_parts = [int(x) & 0xFF for x in config.ip.split(".")]
        if len(ip_parts) != 4:
            raise ValueError(f"Invalid IP address: {config.ip}")

        self._send_bytes(_COMMANDS["AGENT_IP"], bytes(ip_parts))

        # Port as little-endian 16-bit
        port_bytes = struct.pack("<H", config.port)
        self._send_bytes(_COMMANDS["AGENT_PORT"], port_bytes)

    def set_transport(self, mode: int) -> None:
        """
        Set transport mode.

        Args:
            mode: TRANSPORT_WIFI_UDP (0) or TRANSPORT_SERIAL (1)
        """
        self._send_bytes(_COMMANDS["CAR_TYPE"], bytes([mode & 0xFF, 0]))

    def set_ros_domain_id(self, domain_id: int) -> None:
        """
        Set ROS2 domain ID (0-100).

        Changes take effect after reboot.
        """
        domain_id = max(0, min(100, domain_id))
        domain_bytes = struct.pack("<h", domain_id)
        self._send_bytes(_COMMANDS["DOMAIN_ID"], domain_bytes)

    def set_ros_baudrate(self, baudrate: int) -> None:
        """
        Set ROS serial communication baud rate.

        Changes take effect after reboot.
        """
        baud_bytes = struct.pack("<i", baudrate)
        self._send_bytes(_COMMANDS["SERIAL_BAUDRATE"], baud_bytes)

    def set_ros_namespace(self, namespace: str) -> None:
        """
        Set ROS namespace.

        Changes take effect after reboot.
        """
        if namespace:
            ns_bytes = namespace.encode("utf-8")
        else:
            ns_bytes = bytes([0])
        self._send_string(_COMMANDS["ROS_NAMESPACE"], ns_bytes)

    def reboot(self) -> None:
        """
        Reboot the ESP32 to apply configuration changes.

        Uses DTR/RTS signals to trigger hardware reset.
        """
        self._serial.setDTR(False)
        self._serial.setRTS(True)
        time.sleep(0.1)
        self._serial.setDTR(True)
        self._serial.setRTS(True)
        time.sleep(2)

    def reset_to_factory(self) -> None:
        """
        Reset all configuration to factory defaults.

        Requires reboot to take effect.
        """
        self._send_bytes(_COMMANDS["RESET_CONFIG"], bytes([0x5F, 0x5F]))

    # --- Read Methods ---

    def read_config(self) -> RobotInfo:
        """
        Read current configuration from ESP32.

        Returns:
            RobotInfo with current settings
        """
        info = RobotInfo()

        info.firmware_version = self._read_string(_COMMANDS["FIRMWARE_VERSION"])
        info.wifi_ssid = self._read_string(_COMMANDS["WIFI_SSID"])
        info.agent_ip = self._read_ip(_COMMANDS["AGENT_IP"])
        info.agent_port = self._read_int16(_COMMANDS["AGENT_PORT"])
        info.car_type = self._read_car_type(_COMMANDS["CAR_TYPE"])
        info.ros_domain_id = self._read_int16(_COMMANDS["DOMAIN_ID"])
        info.ros_baudrate = self._read_int32(_COMMANDS["SERIAL_BAUDRATE"])

        return info

    def read_firmware_version(self) -> Optional[str]:
        """Read firmware version string."""
        return self._read_version(_COMMANDS["FIRMWARE_VERSION"])

    # --- Internal Protocol Methods ---

    def _send_bytes(self, command: int, data: bytes) -> None:
        """Send a command with byte data."""
        length = len(data) + 5
        checksum = (self._HEAD + self._DEVICE_ID + length + command + sum(data)) % 256

        packet = bytes([self._HEAD, self._DEVICE_ID, length, command]) + data + bytes([checksum])
        self._serial.write(packet)

        if self._send_delay > 0:
            time.sleep(self._send_delay)

    def _send_string(self, command: int, data: bytes) -> None:
        """Send a command with string data."""
        self._send_bytes(command, data)

    def _request(self, address: int) -> Optional[bytes]:
        """Request data from a register address."""
        # Build request packet
        length = 7
        checksum = (self._HEAD + self._DEVICE_ID + length + self._READ_CMD + address) % 256
        packet = bytes([self._HEAD, self._DEVICE_ID, length, self._READ_CMD, address, 0, checksum])

        # Clear buffers
        self._serial.flushInput()
        self._serial.flushOutput()

        # Send request
        self._serial.write(packet)
        time.sleep(self._read_delay)

        # Parse response
        return self._read_response()

    def _read_response(self) -> Optional[bytes]:
        """Read and parse response packet."""
        data = self._serial.read_all()
        if not data:
            return None

        # Parse packet: HEAD, RETURN_ID, LEN, ADDR, DATA..., CHECKSUM
        state = 0
        rx_len = 0
        rx_data = bytearray(self._RX_BUF_SIZE)
        rx_count = 0

        for byte in data:
            if state == 0:  # Looking for HEAD
                if byte == self._HEAD:
                    state = 1
            elif state == 1:  # Looking for RETURN_ID
                if byte == self._RETURN_ID:
                    state = 2
                else:
                    state = 0
            elif state == 2:  # Length byte
                rx_len = byte
                state = 3
            elif state == 3:  # Address byte
                state = 4
                rx_count = 0
            elif state == 4:  # Data bytes
                if rx_count < rx_len - 5 and rx_count < self._RX_BUF_SIZE:
                    rx_data[rx_count] = byte
                    rx_count += 1
                if rx_count >= rx_len - 5:
                    state = 5
            elif state == 5:  # Checksum
                # Verify checksum
                expected = (self._HEAD + self._RETURN_ID + rx_len + sum(rx_data[:rx_count])) % 256
                if byte == expected:
                    return bytes(rx_data[:rx_count])
                return None

        return None

    def _read_string(self, address: int) -> Optional[str]:
        """Read a string value from address."""
        data = self._request(address)
        if data:
            try:
                return data.rstrip(b"\x00").decode("utf-8")
            except UnicodeDecodeError:
                return None
        return None

    def _read_version(self, address: int) -> Optional[str]:
        """Read firmware version (3 bytes: major.minor.patch)."""
        data = self._request(address)
        if data and len(data) >= 3:
            return f"{data[0]}.{data[1]}.{data[2]}"
        return None

    def _read_ip(self, address: int) -> Optional[str]:
        """Read IP address (4 bytes)."""
        data = self._request(address)
        if data and len(data) >= 4:
            return f"{data[0]}.{data[1]}.{data[2]}.{data[3]}"
        return None

    def _read_int16(self, address: int) -> Optional[int]:
        """Read 16-bit little-endian integer."""
        data = self._request(address)
        if data and len(data) >= 2:
            return struct.unpack("<h", data[:2])[0]
        return None

    def _read_int32(self, address: int) -> Optional[int]:
        """Read 32-bit little-endian integer."""
        data = self._request(address)
        if data and len(data) >= 4:
            return struct.unpack("<i", data[:4])[0]
        return None

    def _read_car_type(self, address: int) -> Optional[str]:
        """Read car/transport type."""
        value = self._read_int16(address)
        if value == TRANSPORT_WIFI_UDP:
            return "wifi"
        elif value == TRANSPORT_SERIAL:
            return "serial"
        return "unknown"
