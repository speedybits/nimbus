# Nimbus CLI Reference

Nimbus provides a beautiful command-line interface built with Typer and Rich.

## Installation

```bash
cd /home/mike/projects/nimbus
pip install -e .
```

After installation, the `nimbus` command is available globally.

## Commands Overview

```
nimbus --help
```

```
Usage: nimbus [OPTIONS] COMMAND [ARGS]...

 Nimbus: Lightweight Robot Control Platform

╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --install-completion          Install completion for the current shell.      │
│ --show-completion             Show completion for the current shell.         │
│ --help                        Show this message and exit.                    │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Commands ───────────────────────────────────────────────────────────────────╮
│ run         Start Nimbus robot controller.                                   │
│ status      Show current robot status.                                       │
│ stop        Emergency stop - immediately halt all motion.                    │
│ goto        Navigate to coordinates.                                         │
│ behaviors   List available behaviors.                                        │
│ behavior    Set active behavior.                                             │
│ test        Run Nimbus test suite.                                           │
│ agent       Manage Micro-ROS agent.                                          │
│ version     Show Nimbus version.                                             │
╰──────────────────────────────────────────────────────────────────────────────╯
```

---

## nimbus run

Start the Nimbus robot controller.

```bash
nimbus run [OPTIONS]
```

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--behavior TEXT` | `idle` | Initial behavior: `idle`, `wander`, `patrol`, `goto` |
| `--api / --no-api` | `--api` | Enable REST/WebSocket API |
| `--dashboard / --no-dashboard` | `--dashboard` | Show live terminal dashboard |
| `--config PATH` | None | Path to custom config file |
| `--mock` | False | Use mock node (no ROS2 required) |

### Examples

```bash
# Start with default settings (idle, dashboard, API)
nimbus run

# Start with wander behavior
nimbus run --behavior wander

# Start without dashboard (background mode)
nimbus run --behavior wander --no-dashboard

# Start in mock mode (testing without hardware)
nimbus run --mock --behavior wander

# Use custom config
nimbus run --config /path/to/config.yaml
```

### Live Dashboard

When running with `--dashboard` (default), you'll see a live terminal interface:

```
╭─────────────────────────── NIMBUS DASHBOARD ────────────────────────────╮
│                                                                          │
├──────────────── Sensors ─────────────────┬──────────── Status ──────────┤
│ Position X:   1.523 m                    │ State:      NAVIGATING       │
│ Position Y:   2.108 m                    │ Behavior:   wander           │
│ Heading:      45.2 deg                   │ Target:     None             │
│ Linear Vel:   0.20 m/s                   │ Safety:     OK               │
│ Angular Vel:  0.05 rad/s                 │                              │
│ Closest:      1.25 m                     │                              │
├──────────────────────────── LIDAR View ─────────────────────────────────┤
│                                                                          │
│                          .  .  .                                         │
│                    .  .        .  .                                      │
│                 .                    .                                   │
│               .          ^            .                                  │
│               .          R            .                                  │
│                 .                    .                                   │
│                    .  .        .  .                                      │
│                          .  .  .                                         │
│                                                                          │
│   Closest: 1.25m  |  Scale: 1 char = 0.15m                              │
╰──────────────────────────────────────────────────────────────────────────╯
```

Press `Ctrl+C` to stop.

---

## nimbus status

Show current robot status (requires Nimbus to be running).

```bash
nimbus status
```

### Output

```
╭──────────────────── Robot Status ─────────────────────╮
│ Property            │ Value                           │
├─────────────────────┼─────────────────────────────────┤
│ State               │ NAVIGATING                      │
│ Behavior            │ wander                          │
│ Position            │ (1.52, 2.11)                    │
│ Heading             │ 0.79 rad                        │
│ Closest Obstacle    │ 1.25 m                          │
│ Velocity            │ 0.20 m/s, 0.05 rad/s            │
╰───────────────────────────────────────────────────────╯
```

### Errors

If Nimbus is not running:
```
Error: Could not connect to Nimbus. Is it running?
```

---

## nimbus stop

Emergency stop - immediately halt all motion.

```bash
nimbus stop
```

### Output

```
EMERGENCY STOP ACTIVATED
```

This command attempts to contact the API. If unreachable:
```
Warning: Could not reach API. Robot may still be moving.
If robot is still moving, physically power it off.
```

---

## nimbus goto

Navigate to specific coordinates.

```bash
nimbus goto X Y [OPTIONS]
```

### Arguments

| Argument | Description |
|----------|-------------|
| `X` | Target X coordinate in meters |
| `Y` | Target Y coordinate in meters |

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--wait / --no-wait` | `--wait` | Wait for navigation to complete |

### Examples

```bash
# Navigate and wait for completion
nimbus goto 2.0 3.5

# Navigate without waiting
nimbus goto 2.0 3.5 --no-wait
```

### Output (with --wait)

```
Navigating to (2.0, 3.5)...
Navigation started
⠋ State: NAVIGATING
⠙ State: NAVIGATING
⠹ State: AVOIDING
⠸ State: NAVIGATING
Navigation complete
```

---

## nimbus behaviors

List available behaviors.

```bash
nimbus behaviors
```

### Output

```
╭───────────── Available Behaviors ──────────────╮
│ Name             │ Active                      │
├──────────────────┼─────────────────────────────┤
│ idle             │                             │
│ wander           │ Yes                         │
│ simple_wander    │                             │
│ goto             │                             │
│ patrol           │                             │
╰────────────────────────────────────────────────╯
```

---

## nimbus behavior

Set the active behavior.

```bash
nimbus behavior NAME
```

### Arguments

| Argument | Description |
|----------|-------------|
| `NAME` | Behavior name to activate |

### Examples

```bash
# Switch to wander mode
nimbus behavior wander

# Switch to idle (stop)
nimbus behavior idle

# Switch to patrol
nimbus behavior patrol
```

### Output

```
Behavior set to: wander
```

---

## nimbus test

Run the Nimbus test suite.

```bash
nimbus test [PATTERN] [OPTIONS]
```

### Arguments

| Argument | Description |
|----------|-------------|
| `PATTERN` | pytest-style test pattern (optional) |

### Options

| Option | Description |
|--------|-------------|
| `-v, --verbose` | Verbose output |
| `--regression` | Run only regression tests |
| `--cov` | Run with coverage report |

### Examples

```bash
# Run all tests
nimbus test

# Run with verbose output
nimbus test -v

# Run specific test
nimbus test test_vfh

# Run only regression tests
nimbus test --regression

# Run with coverage
nimbus test --cov
```

### Output

```
Running: pytest -v /home/mike/projects/nimbus/nimbus/tests

============================= test session starts ==============================
collected 34 items

test_safety.py::TestSafetyController::test_emergency_stop_when_close PASSED
test_safety.py::TestSafetyController::test_speed_reduction_in_caution PASSED
...
============================== 34 passed in 0.06s ==============================
```

---

## nimbus agent

Manage the Micro-ROS agent Docker container.

```bash
nimbus agent ACTION
```

### Actions

| Action | Description |
|--------|-------------|
| `start` | Start the Micro-ROS agent container |
| `stop` | Stop the agent container |
| `status` | Check if agent is running |

### Examples

```bash
# Start the agent
nimbus agent start
# Output: Micro-ROS agent started

# Check status
nimbus agent status
# Output: Micro-ROS agent is running
#         Container: a1b2c3d4e5f6

# Stop the agent
nimbus agent stop
# Output: Micro-ROS agent stopped
```

### Requirements

- Docker must be installed
- User must be in `docker` group or run as root
- USB device (`/dev/ttyACM0`) must be connected

---

## nimbus version

Show Nimbus version.

```bash
nimbus version
```

### Output

```
Nimbus version 0.1.0
```

---

## Shell Completion

Nimbus supports shell completion for bash, zsh, and fish.

### Install Completion

```bash
# For bash
nimbus --install-completion bash

# For zsh
nimbus --install-completion zsh

# For fish
nimbus --install-completion fish
```

After installation, restart your shell or source the completion file.

### Usage

```bash
nimbus <TAB>           # Show all commands
nimbus run --be<TAB>   # Complete to --behavior
nimbus behavior <TAB>  # Show available behaviors
```

---

## Environment Variables

These environment variables affect CLI behavior:

| Variable | Description |
|----------|-------------|
| `NIMBUS_API_PORT` | REST API port (default: 8080) |
| `NIMBUS_MAX_SPEED` | Maximum linear speed (default: 0.30) |
| `NIMBUS_SAFETY_RADIUS` | Safety radius in meters (default: 0.30) |

---

## Exit Codes

| Code | Description |
|------|-------------|
| 0 | Success |
| 1 | Error (connection failed, invalid argument, etc.) |

---

## Troubleshooting

### "Could not connect to Nimbus"

Nimbus is not running. Start it first:
```bash
nimbus run
```

### "ROS2 (rclpy) not available"

ROS2 is not sourced. Source it first:
```bash
source /opt/ros/humble/setup.bash
nimbus run
```

Or use mock mode:
```bash
nimbus run --mock
```

### "Docker not found"

Docker is required for Micro-ROS agent. Install it:
```bash
sudo apt install docker.io
sudo usermod -aG docker $USER
# Log out and back in
```

### Dashboard not displaying correctly

Try running without dashboard:
```bash
nimbus run --no-dashboard
```

Or check terminal size (minimum 80x24 recommended).
