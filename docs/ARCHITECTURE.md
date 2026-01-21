# Nimbus Architecture Guide

This document provides a technical deep-dive into Nimbus's architecture, design decisions, and implementation details.

## Design Philosophy

Nimbus follows these core principles:

1. **Minimal** — No feature without justification
2. **Testable** — Every component mockable
3. **Observable** — Rich telemetry everywhere
4. **Safe** — Hardware safety layer always active
5. **Extensible** — Plugin system for growth
6. **Elegant** — Code that reads like poetry

## System Overview

```
┌──────────────────────────────────────────────────────────────────┐
│                         User Interface                            │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐   │
│  │  CLI (Typer)│  │  REST API   │  │  WebSocket Telemetry    │   │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
                              │
┌──────────────────────────────────────────────────────────────────┐
│                       NimbusRunner                                │
│  Main control loop orchestrating all components at 10Hz           │
└──────────────────────────────────────────────────────────────────┘
         │                    │                    │
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│ BehaviorManager │  │ SafetyController│  │ VelocitySmoother│
│                 │  │                 │  │                 │
│ - idle          │  │ - Emergency     │  │ - Acceleration  │
│ - wander        │  │ - Caution       │  │   limits        │
│ - goto          │  │ - Normal        │  │ - Jerk control  │
│ - patrol        │  │                 │  │                 │
└─────────────────┘  └─────────────────┘  └─────────────────┘
         │                    │                    │
┌──────────────────────────────────────────────────────────────────┐
│                       RobotContext                                │
│  Thread-safe central state: pose, velocity, sensors, target       │
└──────────────────────────────────────────────────────────────────┘
         │                    │
┌─────────────────┐  ┌─────────────────┐
│ LidarProcessor  │  │OdometryProcessor│
│                 │  │                 │
│ - Polar histo.  │  │ - Pose2D        │
│ - Find closest  │  │ - Velocity      │
│ - Sector ranges │  │ - Quaternion    │
└─────────────────┘  └─────────────────┘
         │                    │
┌──────────────────────────────────────────────────────────────────┐
│                     NimbusNode / XRCENode                         │
│  Unified interface for ROS2 or pure Python XRCE-DDS               │
└──────────────────────────────────────────────────────────────────┘
         │                    │                    │
    ┌────┴────┐          ┌────┴────┐          ┌────┴────┐
    │  /scan  │          │/odom_raw│          │/cmd_vel │
    └─────────┘          └─────────┘          └─────────┘
         │                    │                    │
         ├────────────────────┼────────────────────┤
         │                    │                    │
    ┌────┴────────────────────┴────────────────────┴────┐
    │                Connection Layer                     │
    │  ┌─────────────────┐    ┌─────────────────────┐   │
    │  │   ROS2 Mode     │    │     XRCE Mode       │   │
    │  │  (NimbusNode)   │    │    (XRCENode)       │   │
    │  │                 │    │                     │   │
    │  │ Micro-ROS Agent │    │ Pure Python XRCE   │   │
    │  │   (Docker)      │    │  (no Docker/ROS2)  │   │
    │  └────────┬────────┘    └──────────┬─────────┘   │
    └───────────┼─────────────────────────┼────────────┘
                │                         │
                └───────────┬─────────────┘
                            │
                  Serial (USB) or WiFi (UDP)
                            │
┌──────────────────────────────────────────────────────────────────┐
│                    ESP32 + Physical Robot                         │
│  Yahboom G1 with Micro-ROS client firmware                        │
└──────────────────────────────────────────────────────────────────┘
```

## Connection Modes

Nimbus supports two connection modes for communicating with the ESP32:

### ROS2 Mode (Default)

The traditional stack using the Micro-ROS agent:

```
PC: Nimbus (Python) → ROS2 (rclpy) → Micro-ROS Agent (Docker) → ESP32
```

**Requirements:**
- ROS2 Humble installed and sourced
- Docker for Micro-ROS agent container
- `nimbus agent start` before running

**Use when:**
- You need ROS2 ecosystem integration (rviz2, rosbag, etc.)
- Running alongside other ROS2 nodes
- Debugging with standard ROS2 tools

### XRCE Mode

Pure Python XRCE-DDS implementation that communicates directly with the ESP32:

```
PC: Nimbus (Python) → XRCEAgent (Python) → ESP32
```

**Requirements:**
- Python 3.10+ only
- No ROS2 or Docker needed

**Use when:**
- Simplified deployment without ROS2/Docker
- Lightweight installations (Raspberry Pi, etc.)
- Faster startup and lower resource usage

**Enable with:**
```bash
nimbus run --xrce --behavior wander           # Auto-discover ESP32
nimbus run --xrce --discover --behavior wander  # Explicit discovery
```

### Architecture Clarification

The Micro-ROS **agent** runs on the **PC** (via Docker), not on the robot. The robot's ESP32 runs the Micro-ROS **client** firmware. The XRCENode implementation replaces the PC-side agent by implementing XRCE-DDS directly in Python.

```
Before: ESP32 (client) ←→ PC: Docker Agent ←→ PC: ROS2 ←→ PC: Nimbus
After:  ESP32 (client) ←→ PC: XRCEAgent (Python XRCE-DDS) ←→ PC: Nimbus
```

## Core Components

### NimbusNode (`core/node.py`)

The NimbusNode provides a thin abstraction over ROS2:

```python
class NimbusNode:
    """Lightweight ROS2 wrapper."""

    def start(self) -> None:
        """Initialize ROS2 and start background spinner."""

    def subscribe(self, topic, msg_type, buffer_size=1) -> TopicBuffer:
        """Subscribe to topic, return buffer for reading."""

    def publisher(self, topic, msg_type):
        """Get or create publisher."""

    def shutdown(self) -> None:
        """Graceful shutdown."""
```

**Key design decisions:**
- Single node, multiple subscriptions
- Background spinner thread
- TopicBuffer for thread-safe message access
- MockNimbusNode for testing without ROS2

### XRCENode (`core/node.py`)

The XRCENode provides the same interface as NimbusNode but communicates directly with the ESP32 using XRCE-DDS protocol, bypassing ROS2 and the Docker-based Micro-ROS agent entirely:

```python
class XRCENode:
    """Pure Python XRCE-DDS communication with ESP32."""

    def start(self) -> None:
        """Initialize XRCE agent and wait for ESP32 connection."""

    def subscribe(self, topic, msg_type, buffer_size=1) -> TopicBuffer:
        """Subscribe to topic via XRCE-DDS."""

    def publisher(self, topic, msg_type):
        """Get or create XRCE-DDS publisher."""

    def shutdown(self) -> None:
        """Stop the XRCE agent."""
```

**Key benefits:**
- No ROS2 installation required
- No Docker container needed
- Pure Python implementation
- Same API as NimbusNode for seamless switching
- Handles ESP32's simplified message format (no Header.stamp)
- Fragment reassembly for large messages (LaserScan)

### XRCE Module (`core/xrce/`)

The XRCE module implements the XRCE-DDS protocol in pure Python:

| File | Purpose |
|------|---------|
| `agent.py` | Main XRCEAgent class - handles ESP32 connection |
| `protocol.py` | Wire format structures (submessages, headers) |
| `session.py` | Protocol session and stream management |
| `entities.py` | Entity tracking (topics, datawriters, datareaders) |
| `transport.py` | UDP transport layer |
| `cdr.py` | CDR (Common Data Representation) serialization |
| `messages.py` | ROS2 message types (LaserScan, Odometry, Twist) |

### RobotContext (`core/state.py`)

Central thread-safe state container:

```python
@dataclass(frozen=True)
class Pose2D:
    x: float      # meters
    y: float      # meters
    theta: float  # radians

@dataclass(frozen=True)
class Velocity:
    linear: float   # m/s
    angular: float  # rad/s

@dataclass(frozen=True)
class SensorSnapshot:
    timestamp: datetime
    pose: Pose2D
    velocity: Velocity
    lidar_ranges: tuple[float, ...]
    closest_obstacle: float
    obstacle_direction: float

class RobotContext:
    """Thread-safe state container."""

    @property
    def state(self) -> RobotState
    def set_state(self, new_state: RobotState)
    def on_state_change(self, callback)

    @property
    def sensors(self) -> SensorSnapshot
    def update_sensors(self, snapshot)

    @property
    def target(self) -> Pose2D
    def set_target(self, target)
```

**State machine:**
```
         ┌─────────┐
         │  IDLE   │◄──────────────────┐
         └────┬────┘                   │
              │ goto/wander            │ goal reached
              ▼                        │
         ┌─────────┐                   │
    ┌───►│NAVIGATING│──────────────────┤
    │    └────┬────┘                   │
    │         │ obstacle               │
    │         ▼                        │
    │    ┌─────────┐                   │
    └────│AVOIDING │───────────────────┘
         └────┬────┘
              │ too close
              ▼
         ┌─────────┐
         │EMERGENCY│
         │  STOP   │
         └─────────┘
```

### Configuration (`core/config.py`)

Hierarchical configuration with environment overrides:

```python
@dataclass
class NimbusConfig:
    sensors: SensorConfig
    navigation: NavigationConfig
    api: APIConfig
    agent: AgentConfig

    @classmethod
    def load(cls, path=None) -> "NimbusConfig":
        """Load with priority: env vars > user config > defaults"""
```

**Priority order:**
1. Environment variables (`NIMBUS_*`)
2. User config (`~/.nimbus/config.yaml`)
3. Project config (`./nimbus.yaml`)
4. Default values

### Network Utilities (`core/network.py`)

Utilities for WiFi connectivity:

```python
def get_local_ip() -> str:
    """Get best local IP for robot communication."""

def resolve_hostname(hostname: str) -> str:
    """Resolve hostname to IP, supports mDNS (.local)."""

def find_serial_ports() -> List[str]:
    """Auto-detect available serial ports."""
```

### Robot Configurator (`core/robot_config.py`)

Serial protocol for configuring Yahboom ESP32 robots:

```python
class RobotConfigurator:
    """Configure ESP32 via serial for WiFi operation."""

    def set_wifi(credentials: WiFiCredentials) -> None
    def set_udp_agent(config: UDPAgentConfig) -> None
    def set_transport(mode: int) -> None
    def reboot() -> None
    def read_config() -> RobotInfo
```

Used by `nimbus wifi setup` to send WiFi credentials and agent
settings to the robot before wireless operation.

## Navigation System

### LIDAR Processing (`sensors/lidar.py`)

Converts 360 LIDAR points to polar histogram:

```python
class LidarProcessor:
    def process(self, ranges: np.ndarray) -> np.ndarray:
        """
        Convert 360 ranges to 72-sector histogram.

        Each sector: 5 degrees
        Values: 0.0 (clear) to 1.0 (blocked)
        """

    def find_closest(self, ranges) -> tuple[float, float]:
        """Return (distance, angle) of nearest obstacle."""

    def get_sector_ranges(self, ranges) -> dict:
        """Get min range in named sectors (front, left, etc.)"""
```

### Vector Field Histogram (`navigation/vfh.py`)

VFH algorithm for obstacle avoidance:

```
Input: 360 LIDAR points + goal direction
Output: Steering angle + blocked flag

Algorithm:
1. Build polar histogram (72 sectors, 5° each)
2. Smooth histogram to reduce noise
3. Apply thresholds → binary blocked/clear map
4. Find "valleys" (contiguous clear sectors)
5. Score valleys by width + proximity to goal
6. Return steering toward best valley
```

**Why VFH over alternatives:**

| Algorithm | Pros | Cons |
|-----------|------|------|
| Potential Fields | Simple | Local minima, oscillation |
| Bug Algorithms | Complete | Jerky motion |
| Costmaps (Nav2) | Powerful | Heavy, complex |
| **VFH** | **Smooth, efficient** | **Needs tuning** |

### Safety Controller (`navigation/safety.py`)

**Cannot be bypassed.** Filters all velocity commands:

```python
class SafetyController:
    def limit_velocity(self, linear, angular, closest_distance):
        """
        Apply safety limits.

        EMERGENCY (< 15cm): linear = 0, allow rotation
        CAUTION (15-40cm): linear scaled by distance
        NORMAL (> 40cm): full speed
        """

    def force_stop(self) -> tuple[float, float]:
        """Return (0, 0) - immediate stop."""
```

**Safety zones:**
```
         Robot
           │
    ┌──────┴──────┐
    │  EMERGENCY  │  0 - 15cm   → Full stop
    │    ZONE     │
    ├─────────────┤
    │   CAUTION   │  15 - 40cm  → Speed reduced
    │    ZONE     │
    ├─────────────┤
    │   NORMAL    │  > 40cm     → Full speed
    │    ZONE     │
    └─────────────┘
```

## Behavior System

### Behavior Interface (`behaviors/base.py`)

```python
class Behavior(ABC):
    name: str
    description: str
    priority: int

    @abstractmethod
    def compute(self, context: RobotContext) -> Optional[Velocity]:
        """Return velocity or None to delegate."""

    def activate(self) -> None
    def deactivate(self) -> None
    def reset(self) -> None
```

### Built-in Behaviors

| Behavior | Priority | Description |
|----------|----------|-------------|
| `idle` | 0 | Return `Velocity(0, 0)` |
| `simple_wander` | 5 | Reactive 3-zone avoidance |
| `wander` | 10 | VFH-based exploration |
| `patrol` | 15 | Cycle through waypoints |
| `goto` | 20 | Navigate to coordinates |

### Behavior Manager

```python
class BehaviorManager:
    def register(self, behavior: Behavior)
    def activate(self, name: str) -> bool
    def compute(self, context: RobotContext) -> Velocity
```

## Control Loop

The main loop runs at 10Hz in `NimbusRunner`:

```python
def _control_step(self):
    # 1. Read sensors
    self._update_sensors()

    # 2. Get velocity from behavior
    velocity = self._behavior_manager.compute(self._context)

    # 3. Apply safety filtering (CANNOT SKIP)
    safe_linear, safe_angular = self._safety.limit_velocity(
        velocity.linear, velocity.angular, closest
    )

    # 4. Smooth velocity
    smooth_linear, smooth_angular = self._smoother.smooth(
        safe_linear, safe_angular
    )

    # 5. Send to motors
    self._send_velocity(smooth_linear, smooth_angular)
```

## API Layer

### REST API (`api/server.py`)

FastAPI-based REST endpoints:

```python
@app.get("/api/status")
async def get_status() -> StatusResponse

@app.post("/api/navigate")
async def navigate(request: NavigateRequest)

@app.post("/api/stop")
async def emergency_stop()

@app.post("/api/behavior/{name}")
async def set_behavior(name: str)
```

### WebSocket (`api/websocket.py`)

Real-time telemetry streaming:

```python
# Client connects to /ws/telemetry
# Receives JSON at 10Hz:
{
    "type": "telemetry",
    "timestamp": "2024-01-15T10:30:00",
    "state": "NAVIGATING",
    "pose": {"x": 1.5, "y": 2.0, "theta": 0.5},
    "velocity": {"linear": 0.2, "angular": 0.1},
    "closest_obstacle": 1.2
}
```

## Thread Model

```
┌─────────────────────────────────────────────────────────┐
│                     Main Thread                          │
│  - NimbusRunner.run() control loop                      │
│  - Behavior computation                                  │
│  - Sensor processing                                     │
└─────────────────────────────────────────────────────────┘
                         │
        ┌────────────────┼────────────────┐
        ▼                ▼                ▼
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│ ROS2 Spinner │  │  API Server  │  │  WebSocket   │
│   (daemon)   │  │   (daemon)   │  │  Broadcast   │
└──────────────┘  └──────────────┘  └──────────────┘
```

**Thread safety:**
- `RobotContext` uses `threading.RLock` for all state access
- `TopicBuffer` uses `threading.Lock` for message access
- All data classes are immutable (`frozen=True`)

## Testing Strategy

### Unit Tests
- Test each component in isolation
- Mock ROS2 with `MockNimbusNode`
- Mock sensor data with `MockLaserScan`, `MockOdometry`

### Integration Tests
- Test component interactions
- Use mock node with injected messages

### Regression Tests
- Record navigation scenarios as JSON
- Replay and verify behavior matches expected

```python
# Example test
def test_wall_ahead_steers_around(vfh_navigator, lidar_processor):
    scan = MockLaserScan.wall_ahead(distance=0.3)
    histogram = lidar_processor.process(np.array(scan.ranges))
    steering, blocked = vfh_navigator.compute_steering(histogram, 0.0)

    assert not blocked
    assert abs(steering) > 0.1  # Should turn
```

## Extension Points

### Custom Behaviors

```python
from nimbus.behaviors.base import Behavior

class MyBehavior(Behavior):
    name = "my_behavior"
    description = "Does something cool"
    priority = 25

    def compute(self, context):
        # Your logic here
        return Velocity(0.1, 0.0)

# Register
runner._behavior_manager.register(MyBehavior())
```

### Custom Sensors

Extend `NimbusNode` subscriptions:

```python
# In runner or custom code
imu_buffer = node.subscribe("/imu", Imu, buffer_size=10)
```

## Performance Considerations

- **Control loop**: 10Hz (100ms period)
- **LIDAR processing**: O(360) per frame
- **VFH computation**: O(72 sectors)
- **API latency**: < 10ms typical
- **Memory**: ~50MB typical

## Security

- No authentication on local API (trusted network)
- Safety controller cannot be disabled via API
- Emergency stop always available
- No remote code execution
