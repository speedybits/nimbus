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


def get_local_subnet() -> Optional[tuple]:
    """
    Get the local subnet information for the primary interface.

    Returns:
        Tuple of (network_address, prefix_length, gateway, local_ip) or None if detection fails.
        Example: ("192.168.1.0", 24, "192.168.1.1", "192.168.1.100")
    """
    try:
        # Get route to internet to find primary interface
        result = subprocess.run(
            ["ip", "route", "get", "8.8.8.8"],
            capture_output=True,
            text=True,
            timeout=5
        )

        if result.returncode != 0:
            return None

        # Parse output: "8.8.8.8 via 192.168.1.1 dev wlo1 src 192.168.1.100 uid 1000"
        output = result.stdout.strip()
        gateway = None
        local_ip = None
        interface = None

        parts = output.split()
        for i, part in enumerate(parts):
            if part == "via" and i + 1 < len(parts):
                gateway = parts[i + 1]
            elif part == "src" and i + 1 < len(parts):
                local_ip = parts[i + 1]
            elif part == "dev" and i + 1 < len(parts):
                interface = parts[i + 1]

        if not local_ip or not interface:
            return None

        # Get subnet mask from interface
        result = subprocess.run(
            ["ip", "-4", "addr", "show", interface],
            capture_output=True,
            text=True,
            timeout=5
        )

        if result.returncode != 0:
            return None

        # Parse: "inet 192.168.1.100/24 brd 192.168.1.255 scope global ..."
        for line in result.stdout.split("\n"):
            if "inet " in line:
                parts = line.strip().split()
                if len(parts) >= 2:
                    ip_cidr = parts[1]  # e.g., "192.168.1.100/24"
                    if "/" in ip_cidr:
                        ip, prefix = ip_cidr.split("/")
                        prefix_len = int(prefix)

                        # Calculate network address
                        ip_parts = [int(x) for x in ip.split(".")]
                        mask = (0xFFFFFFFF << (32 - prefix_len)) & 0xFFFFFFFF
                        network_int = (
                            (ip_parts[0] << 24) + (ip_parts[1] << 16) +
                            (ip_parts[2] << 8) + ip_parts[3]
                        ) & mask
                        network = ".".join([
                            str((network_int >> 24) & 0xFF),
                            str((network_int >> 16) & 0xFF),
                            str((network_int >> 8) & 0xFF),
                            str(network_int & 0xFF)
                        ])

                        return (network, prefix_len, gateway, local_ip)

        return None

    except (subprocess.TimeoutExpired, FileNotFoundError, ValueError):
        return None


def _ping_host(ip: str, timeout: float) -> Optional[dict]:
    """
    Ping a single host and return result if reachable.

    Args:
        ip: IP address to ping
        timeout: Timeout in seconds

    Returns:
        Dict with ip and latency_ms if reachable, None otherwise
    """
    try:
        # Use ping with count=1 and timeout
        result = subprocess.run(
            ["ping", "-c", "1", "-W", str(int(timeout)), ip],
            capture_output=True,
            text=True,
            timeout=timeout + 1
        )

        if result.returncode == 0:
            # Parse latency from output: "time=14.5 ms"
            import re
            match = re.search(r"time[=<](\d+\.?\d*)\s*ms", result.stdout)
            latency = float(match.group(1)) if match else 0.0
            return {"ip": ip, "latency_ms": latency}

    except (subprocess.TimeoutExpired, Exception):
        pass

    return None


def listen_for_robot(port: int = 8090, timeout: float = 5.0) -> Optional[dict]:
    """
    Listen for incoming XRCE-DDS traffic to identify the robot.

    The ESP32 Micro-ROS client sends UDP packets to the agent. By listening
    on the agent port, we can detect which IP the robot traffic comes from.

    Args:
        port: UDP port to listen on (default 8090, the Micro-ROS agent port)
        timeout: Maximum time to wait for traffic

    Returns:
        Dict with robot info if found, None otherwise.
        Example: {"ip": "172.20.10.9", "port": 8090, "packets": 5}
    """
    import socket
    import select

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", port))
        sock.setblocking(False)

        start = time.time()
        robot_ip = None
        packet_count = 0

        while time.time() - start < timeout:
            readable, _, _ = select.select([sock], [], [], 0.5)
            if readable:
                try:
                    data, addr = sock.recvfrom(65535)
                    if len(data) > 4:  # Minimum XRCE header size
                        robot_ip = addr[0]
                        packet_count += 1
                        # Got traffic - wait a bit more to confirm it's sustained
                        if packet_count >= 3:
                            break
                except Exception:
                    pass

        sock.close()

        if robot_ip:
            return {
                "ip": robot_ip,
                "port": port,
                "packets": packet_count,
                "verified": True
            }
        return None

    except OSError as e:
        # Port may be in use
        if "Address already in use" in str(e) or e.errno == 98:
            return {"error": "port_in_use", "message": f"Port {port} is already in use (agent running?)"}
        return None
    except Exception:
        return None


def scan_for_esp32(timeout: float = 3.0, max_workers: int = 50) -> List[dict]:
    """
    Scan local network for ESP32 robots.

    Performs a ping sweep of the local subnet, excluding the gateway
    and local machine. Returns hosts that respond to ping, sorted by latency.

    Args:
        timeout: Ping timeout per host in seconds
        max_workers: Maximum concurrent ping operations

    Returns:
        List of dicts: [{"ip": "172.20.10.9", "latency_ms": 15.2}, ...]
        Sorted by latency (fastest first).
    """
    from concurrent.futures import ThreadPoolExecutor, as_completed

    subnet_info = get_local_subnet()
    if not subnet_info:
        return []

    network, prefix_len, gateway, local_ip = subnet_info

    # Generate IP addresses to scan (skip network address, broadcast, gateway, and self)
    network_parts = [int(x) for x in network.split(".")]
    network_int = (
        (network_parts[0] << 24) + (network_parts[1] << 16) +
        (network_parts[2] << 8) + network_parts[3]
    )

    # Calculate number of hosts in subnet
    num_hosts = (1 << (32 - prefix_len)) - 2  # Exclude network and broadcast
    if num_hosts > 254:
        num_hosts = 254  # Limit to /24 equivalent

    # Skip addresses
    skip_ips = {local_ip}
    if gateway:
        skip_ips.add(gateway)

    # Generate candidate IPs
    candidates = []
    for i in range(1, num_hosts + 1):
        ip_int = network_int + i
        ip = ".".join([
            str((ip_int >> 24) & 0xFF),
            str((ip_int >> 16) & 0xFF),
            str((ip_int >> 8) & 0xFF),
            str(ip_int & 0xFF)
        ])
        if ip not in skip_ips:
            candidates.append(ip)

    # Ping sweep in parallel
    results = []
    with ThreadPoolExecutor(max_workers=min(max_workers, len(candidates))) as executor:
        futures = {executor.submit(_ping_host, ip, timeout): ip for ip in candidates}

        for future in as_completed(futures):
            result = future.result()
            if result:
                results.append(result)

    # Sort by latency
    results.sort(key=lambda x: x["latency_ms"])
    return results
