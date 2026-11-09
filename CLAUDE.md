# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Nimbus is a lightweight robot control platform that replaces ROS2 navigation stacks with pure Python. It controls robots via XRCE-DDS (ESP32) or serial (Neato vacuum), providing reactive obstacle avoidance, a CLI dashboard, and REST/WebSocket APIs.

**Platform**: Jetson Nano (aarch64, Ubuntu 18.04-based L4T), Python 3.10.14
**Venv**: `~/projects/nimbus-env/bin/activate`
**Jetson hostname**: `mike-jetson.local` (mDNS)

## Common Commands

```bash
# Activate environment
source ~/projects/nimbus-env/bin/activate

# Install in dev mode
pip install -e ".[dev]"

# Run tests
nimbus test                                    # all tests
pytest nimbus/tests/unit/ -v                   # unit tests only
pytest nimbus/tests/unit/test_safety.py -v     # single file
pytest nimbus/tests/unit/test_safety.py::TestSafetyController::test_emergency_stop -v  # single test
nimbus test --regression                       # regression tests

# Run nimbus
nimbus run --mock --behavior wander            # mock mode (no hardware)
nimbus run --transport neato                   # Neato serial mode
nimbus run --behavior wander                   # real ESP32

# Linting/formatting
ruff check nimbus/                             # lint (configured in pyproject.toml)
```

## Architecture

The main control loop runs at 10Hz in `NimbusRunner` (`core/runner.py`):
1. Read sensors (LIDAR + odometry)
2. Active behavior computes velocity
3. Safety controller filters velocity (cannot be bypassed)
4. Velocity smoother applies acceleration limits
5. Send to motors via XRCE-DDS or serial

**Key layers** (bottom-up):
- **core/xrce/**: Pure Python XRCE-DDS protocol implementation (agent, session, transport, CDR serialization, message types)
- **core/node.py**: `XRCENode` and `NimbusNode` — same interface, different transports. `MockNimbusNode` for testing
- **core/state.py**: `RobotContext` — thread-safe central state with immutable frozen dataclasses (`Pose2D`, `Velocity`, `SensorSnapshot`). State machine: IDLE -> NAVIGATING -> AVOIDING -> EMERGENCY_STOP
- **sensors/**: `LidarProcessor` (360 points -> 72-sector histogram), `OdometryProcessor`, `ObstacleMap` (sparse grid)
- **navigation/vfh.py**: Vector Field Histogram obstacle avoidance
- **navigation/safety.py**: `SafetyController` — EMERGENCY (<15cm: stop), CAUTION (15-40cm: scale speed), NORMAL (>40cm: full)
- **behaviors/base.py**: `Behavior` ABC with `compute(context) -> Optional[Velocity]`. Priority-based; return None to delegate
- **api/server.py**: FastAPI on port 8080. REST + WebSocket (`/ws/telemetry` at 10Hz)
- **cli/app.py**: Typer CLI entry point (`nimbus` command)

**Thread model**: Main thread runs control loop; daemon threads for API server, WebSocket broadcast, and ROS2/XRCE spinner. `RobotContext` uses `RLock`, `TopicBuffer` uses `Lock`, all data classes are `frozen=True`.

## XRCE-DDS Topics

- `/scan` — LaserScan from LIDAR (subscribed)
- `/odom_raw` — Raw odometry from ESP32 (subscribed)
- `/cmd_vel` — Velocity commands to motors (published)

LIDAR angle convention: index 0 = backward (180deg), 90 = right, 180 = forward, 270 = left.

## API

**Port 8080** (not 8000).

Core: `GET /api/status`, `GET /api/sensors`, `POST /api/navigate`, `POST /api/stop`, `WS /ws/telemetry`

Claude control (blocking, precise movement):
- `POST /api/claude/move?distance=0.5&speed=0.15&timeout=30`
- `POST /api/claude/turn?degrees=90&speed=0.5&timeout=30`
- `GET /api/claude/scan` — processed surroundings with compass bearings
- `POST /api/claude/goto?x=2.0&y=0.5&timeout=60` — VFH nav (blocking)
- `GET /api/claude/map` — ASCII obstacle map

Motor test: `POST /api/behavior/motor_test` then `POST /api/motor_test/velocity?linear=0.1&angular=0`

## Testing

Tests use pytest with fixtures from `nimbus/tests/conftest.py`. Key fixtures: `lidar_processor`, `vfh_navigator`, `safety_controller`, `robot_context`, `mock_scan_empty`, `mock_scan_wall_ahead`, `mock_scan_corridor`. Use `MockLaserScan.wall_ahead(distance=0.5)` etc. for custom scans.

Config: `pyproject.toml` sets `asyncio_mode = "auto"` and `testpaths = ["nimbus/tests"]`.

## Deployment (Neato/Jetson)

```bash
ssh mike@mike-jetson.local
source ~/projects/nimbus-env/bin/activate && nimbus run --transport neato
```

Serial port: `/dev/ttyACM0`. Deploy code changes:
```bash
git push && ssh mike@mike-jetson.local "cd ~/projects/nimbus && git pull && source ~/projects/nimbus-env/bin/activate && pip install -e ."
```

## Configuration

YAML config at `~/.nimbus/config.yaml` or `./nimbus.yaml`. Env var overrides with `NIMBUS_` prefix (e.g., `NIMBUS_MAX_SPEED`, `NIMBUS_API_PORT`, `NIMBUS_AGENT_PORT`). Priority: env vars > user config > project config > defaults.

## Critical Rules

- **NEVER launch `nimbus run` if an instance is already running.** Check with `ps aux | grep nimbus | grep -v grep`. Only one instance can run (UDP port conflict).
- **When ESP32 won't connect to XRCE agent**: immediately ask user to reboot/power cycle the robot. Don't retry silently.
- **WiFi is always correctly configured.** Never suggest re-running WiFi setup.
- **ESP32 firmware subscribes to `/cmd_vel`.** If agent says "not subscribed", the bug is in Nimbus XRCE code, not firmware.
- **Never bypass the safety controller.** Never block the main control loop.
- To check if robot is physically moving, monitor `~/.nimbus/claude_state.json` for changing x/y/theta values.
- ESP32/robot hardware docs are at `/home/mike/projects/b4m_yahboom/doc_txt`.
