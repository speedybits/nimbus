"""
Micro-ROS agent management for Nimbus.

Handles starting/stopping the Docker-based Micro-ROS agent
that bridges ESP32 to ROS2.
"""

from dataclasses import dataclass
from typing import Optional
import subprocess
import time


@dataclass
class AgentConfig:
    """Micro-ROS agent configuration."""
    docker_image: str = "microros/micro-ros-agent:humble"
    container_name: str = "nimbus_microros_agent"
    device: str = "/dev/ttyACM0"
    baudrate: int = 115200


class MicroROSAgent:
    """
    Manage the Micro-ROS agent Docker container.

    The agent bridges the ESP32 microcontroller to ROS2,
    enabling topic communication with the robot hardware.
    """

    def __init__(self, config: Optional[AgentConfig] = None):
        self.config = config or AgentConfig()
        self._container_id: Optional[str] = None

    def start(self) -> bool:
        """
        Start the Micro-ROS agent container.

        Returns:
            True if started successfully
        """
        # Check if already running
        if self.is_running():
            return True

        # Stop any existing container with same name
        self._stop_existing()

        # Start new container
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
        except Exception as e:
            print(f"Error starting agent: {e}")
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
        """Get agent status."""
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

        return {
            "running": running,
            "container_name": self.config.container_name,
            "container_id": container_id,
            "device": self.config.device,
            "baudrate": self.config.baudrate,
        }

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
