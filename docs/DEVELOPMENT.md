# Nimbus Development Guide

This guide covers setting up a development environment, running tests, creating custom behaviors, and contributing to Nimbus.

## Development Setup

### Prerequisites

- Python 3.10+
- ROS2 Humble (optional, for real robot testing)
- Docker (for Micro-ROS agent)
- Git

### Clone and Install

```bash
# Clone the repository
git clone https://github.com/yourorg/nimbus.git
cd nimbus

# Create virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate

# Install in development mode with dev dependencies
pip install -e ".[dev]"

# Verify installation
nimbus version
nimbus test
```

### Development Dependencies

The `[dev]` extra installs:
- `pytest` - Test framework
- `pytest-asyncio` - Async test support
- `pytest-cov` - Coverage reporting
- `black` - Code formatting
- `isort` - Import sorting
- `mypy` - Type checking
- `flake8` - Linting

---

## Project Structure

```
nimbus/
├── nimbus/
│   ├── __init__.py           # Package version
│   ├── __main__.py           # python -m nimbus entry
│   ├── core/                 # Core infrastructure
│   │   ├── node.py           # ROS2 wrapper
│   │   ├── state.py          # State machine, data classes
│   │   ├── config.py         # Configuration system
│   │   ├── runner.py         # Main control loop
│   │   └── agent.py          # Micro-ROS agent management
│   ├── sensors/              # Sensor processing
│   │   ├── lidar.py          # LIDAR to histogram
│   │   └── odometry.py       # Pose extraction
│   ├── navigation/           # Navigation algorithms
│   │   ├── vfh.py            # Vector Field Histogram
│   │   └── safety.py         # Safety controller
│   ├── behaviors/            # Robot behaviors
│   │   ├── base.py           # Behavior interface
│   │   ├── idle.py           # Stay still
│   │   ├── wander.py         # Random exploration
│   │   └── goto.py           # Navigate to point
│   ├── api/                  # External APIs
│   │   ├── server.py         # FastAPI REST
│   │   ├── websocket.py      # WebSocket handlers
│   │   └── schemas.py        # Pydantic models
│   ├── cli/                  # Command line interface
│   │   ├── app.py            # Typer commands
│   │   └── dashboard.py      # Rich live display
│   ├── tests/                # Test suite
│   │   ├── conftest.py       # Fixtures
│   │   ├── unit/             # Unit tests
│   │   ├── integration/      # Integration tests
│   │   └── regression/       # Scenario tests
│   └── data/                 # Default configs
│       └── config.yaml
├── docs/                     # Documentation
├── pyproject.toml            # Package config
└── requirements.txt          # Dependencies
```

---

## Running Tests

### All Tests

```bash
# Run all tests
nimbus test

# With verbose output
nimbus test -v

# With coverage report
nimbus test --cov
```

### Specific Tests

```bash
# Run by pattern
nimbus test test_safety
nimbus test test_vfh

# Run specific file
pytest nimbus/tests/unit/test_state.py -v

# Run specific test
pytest nimbus/tests/unit/test_safety.py::TestSafetyController::test_emergency_stop -v
```

### Test Categories

```bash
# Unit tests only
pytest nimbus/tests/unit/ -v

# Integration tests only
pytest nimbus/tests/integration/ -v

# Regression tests only
nimbus test --regression
```

### Writing Tests

Tests use pytest with fixtures from `conftest.py`:

```python
# nimbus/tests/unit/test_my_feature.py

import pytest
from nimbus.my_module import MyClass


class TestMyFeature:
    """Test suite for MyFeature."""

    def test_basic_functionality(self):
        """Test basic operation."""
        obj = MyClass()
        result = obj.do_something()
        assert result == expected

    def test_with_mock_lidar(self, lidar_processor, mock_scan_empty):
        """Test using LIDAR fixtures."""
        histogram = lidar_processor.process(mock_scan_empty.ranges)
        assert len(histogram) == 72

    def test_with_context(self, robot_context):
        """Test using robot context fixture."""
        robot_context.set_state(RobotState.NAVIGATING)
        assert robot_context.state == RobotState.NAVIGATING
```

### Available Fixtures

| Fixture | Description |
|---------|-------------|
| `lidar_processor` | Configured LidarProcessor |
| `vfh_navigator` | Configured VFHNavigator |
| `safety_controller` | SafetyController instance |
| `robot_context` | Fresh RobotContext |
| `mock_scan_empty` | LaserScan with no obstacles |
| `mock_scan_wall_ahead` | LaserScan with wall in front |
| `mock_scan_corridor` | LaserScan simulating corridor |

### Creating Mock Scans

```python
from nimbus.tests.conftest import MockLaserScan

# Wall directly ahead at 0.5m
scan = MockLaserScan.wall_ahead(distance=0.5)

# Obstacle at specific angle
scan = MockLaserScan.obstacle_at(angle_deg=45, distance=0.3)

# Surrounded by obstacles
scan = MockLaserScan.surrounded(distance=0.2)

# Clear scan (no obstacles)
scan = MockLaserScan.empty()
```

---

## Creating Custom Behaviors

### Behavior Interface

All behaviors inherit from `Behavior`:

```python
from nimbus.behaviors.base import Behavior
from nimbus.core.state import RobotContext, Velocity
from typing import Optional


class MyBehavior(Behavior):
    """Custom behavior description."""

    name = "my_behavior"
    description = "Does something interesting"
    priority = 15  # Higher = takes precedence

    def __init__(self, my_param: float = 1.0):
        super().__init__()
        self.my_param = my_param

    def compute(self, context: RobotContext) -> Optional[Velocity]:
        """
        Compute velocity command.

        Args:
            context: Current robot state and sensors

        Returns:
            Velocity command, or None to delegate to lower priority behavior
        """
        sensors = context.sensors

        # Access sensor data
        pose = sensors.pose
        closest = sensors.closest_obstacle

        # Your navigation logic here
        linear = 0.2 if closest > 0.5 else 0.0
        angular = 0.1

        return Velocity(linear=linear, angular=angular)

    def activate(self) -> None:
        """Called when behavior becomes active."""
        print(f"{self.name} activated")

    def deactivate(self) -> None:
        """Called when behavior is deactivated."""
        print(f"{self.name} deactivated")

    def reset(self) -> None:
        """Reset behavior state."""
        pass
```

### Registering Behaviors

```python
from nimbus.core.runner import NimbusRunner
from my_behaviors import MyBehavior

# In your code
runner = NimbusRunner(config)
runner._behavior_manager.register(MyBehavior(my_param=2.0))

# Now available via CLI and API
# nimbus behavior my_behavior
# POST /api/behavior/my_behavior
```

### Behavior Best Practices

1. **Keep compute() fast** - Called at 10Hz, should complete in <10ms
2. **Use context.sensors** - Don't access ROS2 directly
3. **Return None to delegate** - Let lower priority behaviors handle edge cases
4. **Implement reset()** - Clear any internal state when behavior restarts
5. **Thread safety** - Context is thread-safe, but your behavior state isn't

---

## Adding New Sensors

### Subscribing to Topics

```python
from nimbus.core.node import NimbusNode

# In your code
node = NimbusNode()
node.start()

# Subscribe to new topic
from sensor_msgs.msg import Imu
imu_buffer = node.subscribe("/imu", Imu, buffer_size=10)

# Read latest message
imu_msg = imu_buffer.get()
if imu_msg:
    # Process IMU data
    angular_velocity = imu_msg.angular_velocity
```

### Creating a Sensor Processor

```python
# nimbus/sensors/imu.py

from dataclasses import dataclass
from typing import Optional
import numpy as np


@dataclass
class ImuConfig:
    """IMU processor configuration."""
    topic: str = "/imu"
    buffer_size: int = 10


class ImuProcessor:
    """Process IMU sensor data."""

    def __init__(self, config: Optional[ImuConfig] = None):
        self.config = config or ImuConfig()

    def process(self, msg) -> dict:
        """
        Extract useful data from IMU message.

        Returns:
            Dict with angular velocity, linear acceleration
        """
        return {
            "angular_velocity": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z,
            },
            "linear_acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z,
            }
        }
```

---

## API Development

### Adding REST Endpoints

```python
# In nimbus/api/server.py

from fastapi import APIRouter

router = APIRouter()

@router.get("/api/my_endpoint")
async def my_endpoint():
    """My custom endpoint."""
    return {"status": "ok", "data": "value"}

@router.post("/api/my_action")
async def my_action(request: MyRequest):
    """Perform custom action."""
    # Your logic here
    return {"status": "success"}
```

### Adding WebSocket Streams

```python
# In nimbus/api/websocket.py

async def my_stream(websocket: WebSocket, context: RobotContext):
    """Stream custom data."""
    await websocket.accept()
    try:
        while True:
            data = {
                "type": "my_data",
                "value": compute_something(context),
            }
            await websocket.send_json(data)
            await asyncio.sleep(0.1)  # 10Hz
    except WebSocketDisconnect:
        pass
```

### API Schemas

```python
# In nimbus/api/schemas.py

from pydantic import BaseModel


class MyRequest(BaseModel):
    """Request model for my endpoint."""
    param1: str
    param2: float = 1.0


class MyResponse(BaseModel):
    """Response model."""
    status: str
    result: dict
```

---

## Code Style

### Formatting

```bash
# Format with black
black nimbus/

# Sort imports
isort nimbus/

# Both
black nimbus/ && isort nimbus/
```

### Type Checking

```bash
# Run mypy
mypy nimbus/
```

### Linting

```bash
# Run flake8
flake8 nimbus/
```

### Pre-commit Checks

```bash
# Run all checks before committing
black nimbus/ && isort nimbus/ && flake8 nimbus/ && mypy nimbus/ && nimbus test
```

---

## Debugging

### Enable Debug Logging

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

### Mock Mode

Test without ROS2 or hardware:

```bash
nimbus run --mock --behavior wander
```

### API Debugging

```bash
# Test endpoints with curl
curl http://localhost:8080/api/status | jq

# Watch WebSocket stream
websocat ws://localhost:8080/ws/telemetry
```

### Dashboard Without Robot

```bash
# Run with mock node
nimbus run --mock --dashboard --behavior wander
```

---

## Architecture Decisions

### Why VFH over Nav2?

| Feature | Nav2 | Nimbus VFH |
|---------|------|------------|
| Setup complexity | High | Minimal |
| Dependencies | Many | Few |
| Configuration | Complex | Simple |
| SLAM required | Usually | No |
| Resource usage | Heavy | Light |
| Debugging | Hard | Easy |

### Why Thread-Safe Immutable State?

- **Predictable**: No race conditions
- **Debuggable**: State can be logged at any point
- **Testable**: Easy to create known states
- **Safe**: Can't accidentally modify shared state

### Why Behaviors over Finite State Machines?

- **Composable**: Behaviors can delegate to others
- **Extensible**: Add new behaviors without modifying existing code
- **Priority-based**: Higher priority behaviors naturally take over
- **Testable**: Each behavior tests in isolation

---

## Contributing

### Pull Request Process

1. Fork the repository
2. Create feature branch: `git checkout -b feature/my-feature`
3. Make changes with tests
4. Run all checks: `black . && isort . && flake8 . && nimbus test`
5. Commit with descriptive message
6. Push and create PR

### Commit Messages

```
feat: Add new patrol behavior with configurable waypoints

- Implement PatrolBehavior class
- Add waypoint configuration to config.yaml
- Add unit tests for patrol logic
- Update CLI to show patrol in behaviors list
```

### Code Review Checklist

- [ ] Tests pass
- [ ] New code has tests
- [ ] Documentation updated
- [ ] Code formatted with black
- [ ] Types checked with mypy
- [ ] No new linting errors

---

## Troubleshooting

### Tests Fail with Import Error

```bash
# Reinstall in dev mode
pip install -e ".[dev]"
```

### ROS2 Not Found

```bash
# Source ROS2 first
source /opt/ros/humble/setup.bash
nimbus run
```

### Mock Node Not Working

```bash
# Ensure not sourcing ROS2 for pure mock mode
deactivate 2>/dev/null
source venv/bin/activate
nimbus run --mock
```

### API Not Responding

```bash
# Check if port is in use
lsof -i :8080

# Use different port
NIMBUS_API_PORT=9000 nimbus run
```

---

## Resources

- [Architecture Guide](ARCHITECTURE.md) - Technical deep-dive
- [API Reference](API.md) - REST & WebSocket documentation
- [CLI Reference](CLI.md) - Command-line usage
- [VFH Paper](https://ieeexplore.ieee.org/document/88137) - Original algorithm
- [FastAPI Docs](https://fastapi.tiangolo.com/) - API framework
- [Typer Docs](https://typer.tiangolo.com/) - CLI framework
- [Rich Docs](https://rich.readthedocs.io/) - Terminal formatting
