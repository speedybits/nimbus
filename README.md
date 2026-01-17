# Nimbus

> *"Simplicity is the ultimate sophistication."* — Leonardo da Vinci

**Nimbus** is a lightweight robot control platform that replaces heavy ROS2 navigation stacks with an elegant, minimal Python implementation. It provides reactive obstacle avoidance, a beautiful CLI, and clean APIs for external integration.

## Features

- **Lightweight** — No Nav2, no AMCL, no complex costmaps. Just pure Python.
- **Safe** — Hardware safety layer that cannot be bypassed
- **Beautiful CLI** — Rich terminal interface with live dashboard
- **API-First** — REST + WebSocket for Home Assistant, custom dashboards, AI integration
- **Test-Driven** — Comprehensive test suite runs without ROS2
- **Extensible** — Plugin system for custom behaviors

## Quick Start

```bash
# Clone and install
cd /home/mike/projects/nimbus
pip install -e ".[dev]"

# Run tests (no ROS2 required!)
nimbus test

# Start with mock mode (no hardware)
nimbus run --mock --behavior wander
```

### Connecting to the Robot

**Option 1: Direct Mode (Recommended)** — No ROS2 or Docker required!

```bash
# WiFi: Direct connection to ESP32
nimbus run --direct --esp32-ip 192.168.1.100 --behavior wander

# USB: Direct serial connection
nimbus run --direct --behavior wander
```

**Option 2: ROS2 Mode** — Traditional ROS2 stack with Micro-ROS agent

```bash
# Start the Micro-ROS agent first
nimbus connect

# Then run Nimbus
nimbus run --behavior wander
```

> **Note**: If you've already run `nimbus connect`, use ROS2 mode (without `--direct`).
> Direct mode and the Micro-ROS agent both use UDP port 8090 and cannot run simultaneously.

## Requirements

**Minimum (Direct Mode):**
- Python 3.10+

**Full (ROS2 Mode):**
- Python 3.10+
- ROS2 Humble
- Docker (for Micro-ROS agent)

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        Nimbus CLI                           │
│   nimbus run | status | test | calibrate | goto | stop      │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                      External APIs                          │
│              REST (FastAPI)  │  WebSocket (real-time)       │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                    Behavior Layer                           │
│         idle │ wander │ goto │ patrol │ explore │ pet       │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                   Navigation Layer                          │
│      VFH Algorithm │ Safety Controller │ Velocity Smoother  │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                    Sensor Layer                             │
│         LIDAR Processor │ Odometry │ IMU (optional)         │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                    Core Layer                               │
│           NimbusNode │ DirectNode │ State Machine           │
└─────────────────────────────────────────────────────────────┘
                 │                        │
    ┌────────────┴────────────┐           │
    │  ROS2 Mode (--no-direct)│           │  Direct Mode (--direct)
    │                         │           │
┌───────────────────┐         │    ┌──────────────────────┐
│ Micro-ROS Agent   │         │    │ XRCE-DDS Client      │
│ (Docker)          │         │    │ (Pure Python)        │
│  /scan /odom_raw  │         │    │  No ROS2 required    │
└───────────────────┘         │    └──────────────────────┘
         │                    │              │
         └────────────────────┼──────────────┘
                              │
                    ┌─────────────────┐
                    │   ESP32 + Motors │
                    │   (Micro-ROS)   │
                    └─────────────────┘
```

## CLI Commands

| Command | Description |
|---------|-------------|
| `nimbus run` | Start robot controller with live dashboard |
| `nimbus run --direct` | Start in direct mode (no ROS2 required) |
| `nimbus run --direct --esp32-ip IP` | Direct mode over WiFi |
| `nimbus connect` | Connect robot via ROS2/Micro-ROS agent |
| `nimbus status` | Show current robot state |
| `nimbus stop` | Emergency stop |
| `nimbus goto X Y` | Navigate to coordinates |
| `nimbus behaviors` | List available behaviors |
| `nimbus behavior NAME` | Set active behavior |
| `nimbus explore` | Start AI-driven exploration |
| `nimbus agent start\|stop\|status` | Manage Micro-ROS agent |
| `nimbus wifi setup` | Configure robot WiFi |
| `nimbus test` | Run test suite |
| `nimbus version` | Show version |

## API Endpoints

### REST (port 8080)

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/status` | Current robot state |
| GET | `/api/sensors` | Raw sensor readings |
| GET | `/api/behaviors` | List behaviors |
| GET | `/api/health` | Health check |
| POST | `/api/navigate` | Send navigation goal |
| POST | `/api/stop` | Emergency stop |
| POST | `/api/behavior/{name}` | Set behavior |

### WebSocket

| Endpoint | Description |
|----------|-------------|
| `/ws/telemetry` | Real-time sensor data (10Hz) |
| `/ws/lidar` | LIDAR scans for visualization |
| `/ws/events` | State changes and alerts |

## Connection Modes

Nimbus supports two ways to communicate with the ESP32 robot:

| Mode | Command | Requirements | Use Case |
|------|---------|--------------|----------|
| **Direct** | `nimbus run --direct` | Python only | Simple setup, no Docker/ROS2 |
| **ROS2** | `nimbus run` | ROS2 + Docker | Integration with ROS2 ecosystem |

### Direct Mode

Direct mode implements the XRCE-DDS protocol in pure Python, communicating directly with the ESP32's Micro-ROS firmware. No ROS2 installation or Docker required.

```bash
# WiFi (recommended for untethered operation)
nimbus run --direct --esp32-ip 192.168.1.100 --behavior wander

# Serial/USB (for development)
nimbus run --direct --behavior wander
```

### ROS2 Mode

Traditional mode using the Micro-ROS Docker agent. Use this if you need ROS2 integration or have already run `nimbus connect`.

```bash
# Start agent and wait for robot
nimbus connect

# Run with ROS2 (in another terminal)
nimbus run --behavior wander
```

> **Important**: Direct mode and ROS2 mode cannot run simultaneously — they both use UDP port 8090. If the Micro-ROS agent is running, use ROS2 mode.

## Configuration

Nimbus uses YAML configuration with environment variable overrides:

```yaml
# ~/.nimbus/config.yaml or ./nimbus.yaml
sensors:
  lidar_topic: "/scan"
  odom_topic: "/odom_raw"
  safety_radius: 0.30

navigation:
  max_linear_speed: 0.30
  max_angular_speed: 1.0
  emergency_distance: 0.15
  caution_distance: 0.40

api:
  rest_port: 8080
  websocket_enabled: true

# Direct mode settings
direct:
  enabled: false          # Set true to use direct mode by default
  esp32_ip: ""            # ESP32 IP for WiFi (empty = serial mode)
  transport: "serial"     # "serial" or "udp"
```

Environment variables (override config):
```bash
export NIMBUS_MAX_SPEED=0.25
export NIMBUS_API_PORT=9000
export NIMBUS_SAFETY_RADIUS=0.35

# Direct mode
export NIMBUS_DIRECT_MODE=true
export NIMBUS_DIRECT_ESP32_IP=192.168.1.100
```

## Behaviors

| Behavior | Description |
|----------|-------------|
| `idle` | Stay stationary, await commands |
| `wander` | Random exploration with VFH obstacle avoidance |
| `goto` | Navigate to specific coordinates |
| `patrol` | Cycle through waypoints |

## Safety System

Nimbus includes a hardware safety layer that **cannot be bypassed**:

| Level | Distance | Action |
|-------|----------|--------|
| EMERGENCY | < 15cm | Full stop, no forward motion |
| CAUTION | 15-40cm | Speed proportionally reduced |
| NORMAL | > 40cm | Full speed allowed |

## Documentation

- [Architecture Guide](docs/ARCHITECTURE.md) — Technical deep-dive
- [API Reference](docs/API.md) — REST & WebSocket documentation
- [CLI Reference](docs/CLI.md) — Command-line usage
- [Development Guide](docs/DEVELOPMENT.md) — Contributing, testing, extending

## Project Structure

```
nimbus/
├── nimbus/
│   ├── core/           # ROS2 wrapper, state machine, config
│   ├── sensors/        # LIDAR processing, odometry
│   ├── navigation/     # VFH algorithm, safety controller
│   ├── behaviors/      # idle, wander, goto, patrol
│   ├── api/            # FastAPI REST, WebSocket
│   ├── cli/            # Typer CLI, Rich dashboard
│   └── tests/          # pytest test suite
├── docs/               # Documentation
├── pyproject.toml      # Package configuration
└── requirements.txt    # Dependencies
```

## License

MIT License

## Acknowledgments

- Vector Field Histogram (VFH) algorithm by Borenstein & Koren
- Built with [Typer](https://typer.tiangolo.com/), [Rich](https://rich.readthedocs.io/), [FastAPI](https://fastapi.tiangolo.com/)
