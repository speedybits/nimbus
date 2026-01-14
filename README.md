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

# With real robot (requires ROS2 + Micro-ROS agent)
nimbus run --behavior wander
```

## Requirements

- Python 3.10+
- ROS2 Humble (for real robot operation)
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
│         idle │ wander │ goto │ patrol │ follow_wall         │
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
│      NimbusNode (ROS2 wrapper) │ State Machine │ Config     │
└─────────────────────────────────────────────────────────────┘
                              │
                    ┌─────────────────┐
                    │  Micro-ROS Agent │
                    │   /scan /odom_raw /cmd_vel              │
                    └─────────────────┘
                              │
                    ┌─────────────────┐
                    │   ESP32 + Motors │
                    └─────────────────┘
```

## CLI Commands

| Command | Description |
|---------|-------------|
| `nimbus run` | Start robot controller with live dashboard |
| `nimbus status` | Show current robot state |
| `nimbus stop` | Emergency stop |
| `nimbus goto X Y` | Navigate to coordinates |
| `nimbus behaviors` | List available behaviors |
| `nimbus behavior NAME` | Set active behavior |
| `nimbus agent start\|stop\|status` | Manage Micro-ROS agent |
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
```

Environment variables (override config):
```bash
export NIMBUS_MAX_SPEED=0.25
export NIMBUS_API_PORT=9000
export NIMBUS_SAFETY_RADIUS=0.35
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
