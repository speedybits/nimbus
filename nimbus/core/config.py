"""
Configuration system for Nimbus.

Supports:
- YAML config files
- Environment variable overrides (NIMBUS_*)
- Sensible defaults

Priority (highest to lowest):
1. Environment variables
2. User config (~/.nimbus/config.yaml)
3. Project config (./nimbus.yaml)
4. Default values
"""

from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional
import os
import yaml


@dataclass
class SensorConfig:
    """Sensor configuration."""

    lidar_topic: str = "/scan"
    odom_topic: str = "/odom_raw"
    imu_topic: str = "/imu"
    safety_radius: float = 0.30     # meters (robot radius + margin)
    max_range: float = 3.5          # meters (ignore beyond this)
    min_range: float = 0.05         # meters (ignore closer than this)


@dataclass
class NavigationConfig:
    """Navigation algorithm configuration."""

    max_linear_speed: float = 0.30   # m/s
    max_angular_speed: float = 1.0   # rad/s

    # VFH parameters
    vfh_sectors: int = 72            # 5-degree sectors
    vfh_threshold_high: float = 0.7  # blocked if > this
    vfh_threshold_low: float = 0.3   # clear if < this
    vfh_wide_valley_min: int = 8     # minimum sectors for "wide"

    # Safety thresholds
    emergency_distance: float = 0.15  # full stop (meters)
    caution_distance: float = 0.40    # slow down (meters)
    emergency_timeout: float = 5.0    # seconds before recovery


@dataclass
class APIConfig:
    """External API configuration."""

    rest_enabled: bool = True
    rest_host: str = "0.0.0.0"
    rest_port: int = 8080
    websocket_enabled: bool = True
    websocket_rate_hz: float = 10.0  # telemetry update rate


@dataclass
class AgentConfig:
    """Micro-ROS agent configuration."""

    auto_start: bool = True
    docker_image: str = "microros/micro-ros-agent:humble"
    device: str = "/dev/ttyACM0"
    baudrate: int = 115200
    container_name: str = "nimbus_microros_agent"


@dataclass
class NimbusConfig:
    """
    Top-level configuration container.

    Usage:
        config = NimbusConfig.load()
        print(config.navigation.max_linear_speed)
    """

    sensors: SensorConfig = field(default_factory=SensorConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    api: APIConfig = field(default_factory=APIConfig)
    agent: AgentConfig = field(default_factory=AgentConfig)

    @classmethod
    def load(cls, path: Optional[Path] = None) -> "NimbusConfig":
        """
        Load configuration from files and environment.

        Order of precedence:
        1. Environment variables (NIMBUS_*)
        2. Provided path (if any)
        3. User config (~/.nimbus/config.yaml)
        4. Local config (./nimbus.yaml)
        5. Defaults
        """
        config = cls()

        # Load from files (lowest to highest priority)
        config_paths = [
            Path("./nimbus.yaml"),
            Path.home() / ".nimbus" / "config.yaml",
        ]
        if path:
            config_paths.append(path)

        for config_path in config_paths:
            if config_path and config_path.exists():
                config = cls._merge_from_file(config, config_path)

        # Apply environment overrides (highest priority)
        config = cls._apply_env_overrides(config)

        return config

    @classmethod
    def _merge_from_file(cls, config: "NimbusConfig", path: Path) -> "NimbusConfig":
        """Merge configuration from a YAML file."""
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        except Exception:
            return config

        # Merge each section
        sections = [
            ("sensors", config.sensors),
            ("navigation", config.navigation),
            ("api", config.api),
            ("agent", config.agent),
        ]

        for section_name, section_obj in sections:
            if section_name in data:
                for key, value in data[section_name].items():
                    if hasattr(section_obj, key):
                        setattr(section_obj, key, value)

        return config

    @classmethod
    def _apply_env_overrides(cls, config: "NimbusConfig") -> "NimbusConfig":
        """Apply NIMBUS_* environment variable overrides."""

        # Mapping: ENV_VAR -> (section, key, converter)
        mappings = {
            # Sensor settings
            "NIMBUS_LIDAR_TOPIC": ("sensors", "lidar_topic", str),
            "NIMBUS_ODOM_TOPIC": ("sensors", "odom_topic", str),
            "NIMBUS_SAFETY_RADIUS": ("sensors", "safety_radius", float),

            # Navigation settings
            "NIMBUS_MAX_SPEED": ("navigation", "max_linear_speed", float),
            "NIMBUS_MAX_ANGULAR": ("navigation", "max_angular_speed", float),
            "NIMBUS_EMERGENCY_DIST": ("navigation", "emergency_distance", float),
            "NIMBUS_CAUTION_DIST": ("navigation", "caution_distance", float),

            # API settings
            "NIMBUS_API_ENABLED": ("api", "rest_enabled", lambda x: x.lower() == "true"),
            "NIMBUS_API_HOST": ("api", "rest_host", str),
            "NIMBUS_API_PORT": ("api", "rest_port", int),

            # Agent settings
            "NIMBUS_AGENT_AUTO_START": ("agent", "auto_start", lambda x: x.lower() == "true"),
            "NIMBUS_AGENT_DEVICE": ("agent", "device", str),
            "NIMBUS_AGENT_BAUDRATE": ("agent", "baudrate", int),
        }

        for env_var, (section, key, converter) in mappings.items():
            value = os.environ.get(env_var)
            if value is not None:
                section_obj = getattr(config, section)
                try:
                    setattr(section_obj, key, converter(value))
                except (ValueError, TypeError):
                    pass  # Invalid env var value, skip

        return config

    def to_yaml(self) -> str:
        """Export configuration as YAML string."""
        return yaml.dump(asdict(self), default_flow_style=False)

    def save(self, path: Path) -> None:
        """Save configuration to a YAML file."""
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as f:
            f.write(self.to_yaml())
