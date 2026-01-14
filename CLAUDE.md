# CLAUDE.md - Nimbus Project Instructions

## Project Overview

Nimbus is a lightweight robot control platform that replaces heavy ROS2 navigation stacks with an elegant, minimal Python implementation.

## Architecture

```
nimbus/
├── core/       # ROS2 wrapper, state machine, config
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

## ROS2 Topics Used

- `/scan` - LaserScan from LIDAR (subscribed)
- `/odom_raw` - Raw odometry from ESP32 (subscribed)
- `/cmd_vel` - Velocity commands to motors (published)

## API Endpoints

- `GET /api/status` - Robot state
- `GET /api/sensors` - Sensor readings
- `POST /api/navigate` - Send goal
- `POST /api/stop` - Emergency stop
- `WS /ws/telemetry` - Real-time data stream

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

## Never Do

- Never store secrets in files - use environment variables
- Never commit runtime data files
- Never bypass the safety controller
- Never block the main control loop
