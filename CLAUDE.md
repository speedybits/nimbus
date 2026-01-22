# CLAUDE.md - Nimbus Project Instructions

## Project Overview

Nimbus is a lightweight robot control platform that replaces heavy ROS2 navigation stacks with an elegant, minimal Python implementation.

## Architecture

```
nimbus/
├── core/       # XRCE agent, state machine, config
├── sensors/    # LIDAR processing, odometry
├── navigation/ # VFH algorithm, safety controller
├── behaviors/  # idle, wander, goto, patrol
├── api/        # FastAPI REST, WebSocket
├── cli/        # Typer CLI, Rich dashboard
├── tests/      # pytest test suite
└── plugins/    # Extensibility system
```

## Key Commands

```bash
# Run with default behavior
nimbus run

# Run with specific behavior
nimbus run --behavior wander

# Check status
nimbus status

# Run tests
nimbus test
pytest nimbus/tests/

# Emergency stop
nimbus stop
```

## Design Principles

1. **Minimal**: No feature without justification
2. **Testable**: Every component mockable
3. **Observable**: Rich telemetry everywhere
4. **Safe**: Hardware safety layer always active
5. **Extensible**: Plugin system for growth
6. **Elegant**: Code that reads like poetry

## XRCE-DDS Topics

- `/scan` - LaserScan from LIDAR (subscribed)
- `/odom_raw` - Raw odometry from ESP32 (subscribed)
- `/cmd_vel` - Velocity commands to motors (published)

## API Endpoints

**API runs on port 8080** (not 8000).

Core endpoints:
- `GET /api/status` - Robot state
- `GET /api/sensors` - Sensor readings
- `POST /api/navigate` - Send goal
- `POST /api/stop` - Emergency stop
- `WS /ws/telemetry` - Real-time data stream

Motor control endpoints:
- `POST /api/motor_test/velocity?linear=0.1&angular=0` - Set velocity (requires motor_test mode)
- `GET /api/motor_test/velocity` - Get current velocity setting
- `POST /api/behavior/motor_test` - Switch to motor_test mode

## Motor Control

Direct motor control via CLI (requires nimbus running):

```bash
# Quick commands
nimbus motor forward --speed 0.1    # Move forward
nimbus motor backward --speed 0.1   # Move backward
nimbus motor left --speed 0.5       # Turn left
nimbus motor right --speed 0.5      # Turn right
nimbus motor stop                   # Stop motors

# Custom velocity
nimbus motor velocity -l 0.1 -a 0.2  # linear + angular
```

Via API:
```bash
curl -X POST "http://localhost:8080/api/behavior/motor_test"
curl -X POST "http://localhost:8080/api/motor_test/velocity?linear=0.1&angular=0"
```

## Testing

```bash
# All tests
nimbus test

# Unit tests only
pytest nimbus/tests/unit/

# Regression tests
nimbus test --regression

# With coverage
pytest --cov=nimbus nimbus/tests/
```

## Robot Documentation

The ESP32/robot hardware documentation is located at `/home/mike/projects/b4m_yahboom/doc_txt`

## Robot Connection

**IMPORTANT:** When the ESP32 is not connecting to the XRCE agent, ALWAYS ask the user to reboot/power cycle the robot. Do not wait or retry silently - immediately ask for a reboot.

**NOTE:** The WiFi is always correctly configured. NEVER suggest re-running WiFi setup - it is not needed.

**NOTE:** The ESP32 firmware is correct and subscribes to `/cmd_vel`. If the agent reports "ESP32 not subscribed to /cmd_vel", the bug is in the Nimbus XRCE agent code, not the firmware.

**TIP:** To check if the robot is physically moving, monitor the pose data in `~/.nimbus/claude_state.json` - if x, y, theta values are changing, the robot is moving.

## Never Do

- Never store secrets in files - use environment variables
- Never commit runtime data files
- Never bypass the safety controller
- Never block the main control loop
- **NEVER launch `nimbus run` if an instance is already running.** Always check first with `ps aux | grep nimbus | grep -v grep`. Only ONE nimbus instance can run at a time (they compete for the UDP port). If the user already has nimbus running, debug their existing session instead of starting a new one.
