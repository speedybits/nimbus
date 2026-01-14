"""
Network utilities for WiFi support.

Provides IP detection, hostname resolution, and serial port discovery
for configuring and communicating with Yahboom robots over WiFi.
"""

import glob
import os
import socket
import subprocess
from typing import List, Optional


def get_local_ip() -> str:
    """
    Get the best local IP address for robot communication.

    Prioritizes physical network interfaces (wlan, eth) over
    virtual ones (docker, veth). Returns the IP that should be
    used as the Micro-ROS agent address.

    Returns:
        Local IP address as string, or "127.0.0.1" if detection fails
    """
    # Try using ip command to get interfaces
    try:
        result = subprocess.run(
            ["ip", "-4", "addr", "show"],
            capture_output=True,
            text=True,
            timeout=5
        )

        if result.returncode == 0:
            interfaces = _parse_ip_output(result.stdout)

            # Priority order: prefer physical interfaces
            priority_patterns = ["wlo", "wlan", "eth", "enp", "ens"]

            for pattern in priority_patterns:
                for iface, ip in interfaces.items():
                    if iface.startswith(pattern) and not ip.startswith("127."):
                        return ip

            # Fallback: any non-localhost, non-virtual IP
            for iface, ip in interfaces.items():
                if _is_physical_interface(iface) and not ip.startswith("127."):
                    return ip

    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass

    # Last resort: socket method
    return _get_ip_via_socket()


def _parse_ip_output(output: str) -> dict:
    """Parse output from 'ip -4 addr show' command."""
    interfaces = {}
    current_iface = None

    for line in output.split("\n"):
        # Parse interface name (e.g., "2: wlo1: <BROADCAST...")
        if ": " in line and not line.startswith(" "):
            parts = line.split(": ")
            if len(parts) >= 2:
                current_iface = parts[1].split("@")[0]
        # Parse IP address (e.g., "    inet 192.168.1.100/24...")
        elif "inet " in line and current_iface:
            ip = line.strip().split()[1].split("/")[0]
            interfaces[current_iface] = ip

    return interfaces


def _is_physical_interface(iface: str) -> bool:
    """Check if interface name looks like a physical (non-virtual) interface."""
    virtual_prefixes = ["docker", "br-", "veth", "virbr", "lo"]
    return not any(iface.startswith(prefix) for prefix in virtual_prefixes)


def _get_ip_via_socket() -> str:
    """Get IP address by connecting to external address."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def resolve_hostname(hostname: str) -> str:
    """
    Resolve hostname to IP address.

    Supports both regular hostnames and mDNS (.local) addresses.
    If the hostname is already an IP address, returns it unchanged.

    Args:
        hostname: Hostname to resolve (e.g., "myhost.local" or "192.168.1.100")

    Returns:
        Resolved IP address

    Raises:
        ValueError: If hostname cannot be resolved
    """
    # Check if already an IP address
    try:
        socket.inet_aton(hostname)
        return hostname
    except socket.error:
        pass

    # Handle local machine hostname specially
    local_hostname = socket.gethostname()
    local_variants = [local_hostname, f"{local_hostname}.local", "localhost.local"]

    if hostname in local_variants:
        return get_local_ip()

    # Standard hostname resolution
    try:
        ip = socket.gethostbyname(hostname)

        # Check for virtual interface IPs that might be wrong
        if hostname.endswith(".local") and _looks_like_virtual_ip(ip):
            # This might be our local machine with wrong interface
            if hostname == f"{local_hostname}.local":
                return get_local_ip()

        return ip

    except socket.gaierror as e:
        hint = ""
        if hostname.endswith(".local"):
            hint = (
                "\nFor mDNS (.local) resolution:\n"
                "1. Install avahi: sudo apt install avahi-daemon\n"
                "2. Ensure target device advertises via mDNS\n"
                "3. Both devices must be on same network"
            )
        raise ValueError(f"Cannot resolve hostname '{hostname}': {e}{hint}")


def _looks_like_virtual_ip(ip: str) -> bool:
    """Check if IP looks like it's from a virtual interface."""
    virtual_prefixes = ["172.17.", "172.18.", "172.16.", "10.0."]
    return any(ip.startswith(prefix) for prefix in virtual_prefixes)


def find_serial_ports() -> List[str]:
    """
    Auto-detect available serial ports.

    Scans for common USB serial device patterns across
    Linux, macOS, and WSL environments.

    Returns:
        Sorted list of available serial port paths
    """
    ports = []

    # Common USB serial device patterns
    patterns = [
        "/dev/ttyUSB*",   # Linux USB-Serial adapters
        "/dev/ttyACM*",   # Linux USB CDC-ACM (Arduino, ESP32)
        "/dev/cu.usb*",   # macOS USB
        "/dev/cu.wch*",   # macOS CH340/CH341 chips
        "/dev/tty.usb*",  # macOS alternative
    ]

    for pattern in patterns:
        ports.extend(glob.glob(pattern))

    # WSL fallback ports
    wsl_ports = ["/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3"]
    ports.extend(p for p in wsl_ports if os.path.exists(p))

    return sorted(set(ports))


def is_port_open(host: str, port: int, timeout: float = 2.0) -> bool:
    """
    Check if a UDP port appears to be listening.

    Note: UDP is connectionless, so this only verifies
    the host is reachable, not that the specific service is running.

    Args:
        host: IP address or hostname
        port: UDP port number
        timeout: Connection timeout in seconds

    Returns:
        True if host is reachable
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(timeout)
        sock.connect((host, port))
        sock.close()
        return True
    except Exception:
        return False
