"""
Micro-ROS agent management for Nimbus.

Handles starting/stopping the Docker-based Micro-ROS agent
that bridges ESP32 to ROS2. Supports both serial (USB) and
WiFi (UDP) transport modes.
"""

from typing import Optional
import subprocess
import time

from .config import AgentConfig


class MicroROSAgent:
    """
    Manage the Micro-ROS agent Docker container.

    The agent bridges the ESP32 microcontroller to ROS2,
    enabling topic communication with the robot hardware.

    Supports two transport modes:
    - serial: USB connection via /dev/ttyACM0 or similar
    - wifi: UDP connection over WiFi network
    """

    def __init__(self, config: Optional[AgentConfig] = None):
        self.config = config or AgentConfig()
        self._container_id: Optional[str] = None

    def start(self, transport: Optional[str] = None) -> bool:
        """
        Start the Micro-ROS agent container.

        Args:
            transport: Override config transport ("serial" or "wifi").
                      Uses config.transport if not specified.

        Returns:
            True if started successfully
        """
        # Check if already running
        if self.is_running():
            return True

        # Stop any existing container with same name
        self._stop_existing()

        # Determine transport mode
        mode = transport or self.config.transport

        if mode == "wifi":
            return self._start_wifi()
        else:
            return self._start_serial()

    def _start_serial(self) -> bool:
        """Start agent in serial mode."""
        try:
            cmd = [
                "docker", "run",
                "-d",  # Detached
                "--rm",  # Remove on stop
                "--name", self.config.container_name,
                "--device", self.config.device,
                "--net=host",  # Use host network for ROS2
                self.config.docker_image,
                "serial",
                "--dev", self.config.device,
                "-b", str(self.config.baudrate),
            ]

            return self._run_docker(cmd)

        except Exception as e:
            print(f"Error starting serial agent: {e}")
            return False

    def _start_wifi(self) -> bool:
        """Start agent in WiFi/UDP mode."""
        try:
            # Determine agent IP (auto-detect if not configured)
            agent_ip = self.config.agent_ip
            if not agent_ip:
                from .network import get_local_ip
                agent_ip = get_local_ip()

            cmd = [
                "docker", "run",
                "-d",  # Detached
                "--rm",  # Remove on stop
                "--name", self.config.container_name,
                "-v", "/dev:/dev",
                "-v", "/dev/shm:/dev/shm",
                "--privileged",
                "--net=host",  # Use host network for ROS2
                self.config.docker_image,
                "udp4",
                "--port", str(self.config.agent_port),
            ]

            return self._run_docker(cmd)

        except Exception as e:
            print(f"Error starting WiFi agent: {e}")
            return False

    def _run_docker(self, cmd: list) -> bool:
        """Execute docker command and handle result."""
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                self._container_id = result.stdout.strip()
                # Wait for agent to initialize
                time.sleep(2)
                return True
            else:
                print(f"Failed to start agent: {result.stderr}")
                return False

        except subprocess.TimeoutExpired:
            print("Timeout starting Micro-ROS agent")
            return False
        except FileNotFoundError:
            print("Docker not found. Please install Docker.")
            return False

    def stop(self) -> None:
        """Stop the Micro-ROS agent container."""
        try:
            subprocess.run(
                ["docker", "stop", self.config.container_name],
                capture_output=True,
                timeout=10
            )
        except Exception:
            pass  # Ignore errors on stop

        self._container_id = None

    def _stop_existing(self) -> None:
        """Stop any existing container with our name."""
        try:
            subprocess.run(
                ["docker", "stop", self.config.container_name],
                capture_output=True,
                timeout=10
            )
        except Exception:
            pass

    def is_running(self) -> bool:
        """Check if the agent container is running."""
        try:
            result = subprocess.run(
                ["docker", "ps", "-q", "-f", f"name={self.config.container_name}"],
                capture_output=True,
                text=True,
                timeout=5
            )
            return bool(result.stdout.strip())
        except Exception:
            return False

    def status(self) -> dict:
        """Get agent status including transport mode."""
        running = self.is_running()

        container_id = None
        if running:
            try:
                result = subprocess.run(
                    ["docker", "ps", "-q", "-f", f"name={self.config.container_name}"],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                container_id = result.stdout.strip()
            except Exception:
                pass

        status_dict = {
            "running": running,
            "transport": self.config.transport,
            "container_name": self.config.container_name,
            "container_id": container_id,
        }

        # Add transport-specific info
        if self.config.transport == "wifi":
            agent_ip = self.config.agent_ip
            if not agent_ip:
                from .network import get_local_ip
                agent_ip = get_local_ip()
            status_dict["agent_ip"] = agent_ip
            status_dict["agent_port"] = self.config.agent_port
        else:
            status_dict["device"] = self.config.device
            status_dict["baudrate"] = self.config.baudrate

        return status_dict

    def logs(self, lines: int = 50) -> str:
        """Get recent container logs."""
        try:
            result = subprocess.run(
                ["docker", "logs", "--tail", str(lines), self.config.container_name],
                capture_output=True,
                text=True,
                timeout=10
            )
            return result.stdout + result.stderr
        except Exception as e:
            return f"Error getting logs: {e}"
