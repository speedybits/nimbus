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
│ explore     Start AI-driven exploration with Ollama.                         │
│ test        Run Nimbus test suite.                                           │
│ agent       Manage Micro-ROS agent.                                          │
│ version     Show Nimbus version.                                             │
│ memory      Manage exploration memories                                      │
│ wifi        WiFi configuration for Yahboom robots                            │
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
| `--direct` | False | Use direct XRCE-DDS mode (no ROS2/Docker required) |
| `--esp32-ip TEXT` | None | ESP32 IP address for WiFi direct mode |

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

# Direct mode over WiFi (no ROS2/Docker needed)
nimbus run --direct --esp32-ip 192.168.1.100 --behavior wander

# Direct mode over serial
nimbus run --direct --behavior wander
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
nimbus agent ACTION [OPTIONS]
```

### Actions

| Action | Description |
|--------|-------------|
| `start` | Start the Micro-ROS agent container |
| `stop` | Stop the agent container |
| `status` | Check if agent is running |

### Options

| Option | Description |
|--------|-------------|
| `--transport TEXT` | Transport mode: `serial` or `wifi` |

### Examples

```bash
# Start with configured transport (default: serial)
nimbus agent start

# Start in WiFi/UDP mode
nimbus agent start --transport wifi
# Output: Micro-ROS agent started (wifi mode)
#         Listening on UDP port 8090
#         Agent IP: 192.168.1.100

# Start in serial mode
nimbus agent start --transport serial
# Output: Micro-ROS agent started (serial mode)
#         Device: /dev/ttyACM0

# Check status
nimbus agent status
# Output: Micro-ROS agent is running
#         Transport: wifi
#         Container: a1b2c3d4e5f6
#         Agent IP: 192.168.1.100
#         UDP Port: 8090

# Stop the agent
nimbus agent stop
# Output: Micro-ROS agent stopped
```

### Transport Modes

| Mode | Connection | Use Case |
|------|------------|----------|
| `serial` | USB cable (`/dev/ttyACM0`) | Development, debugging |
| `wifi` | UDP over WiFi (port 8090) | Untethered operation |

### Requirements

- Docker must be installed
- User must be in `docker` group or run as root
- **Serial mode**: USB device must be connected
- **WiFi mode**: Robot must be configured with `nimbus wifi setup`

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

## nimbus wifi

Configure WiFi connectivity for Yahboom robots. These commands allow you to set up the robot for wireless operation.

### nimbus wifi setup

Interactive wizard to configure WiFi on the robot via USB.

```bash
nimbus wifi setup [OPTIONS]
```

#### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--ssid, -s TEXT` | (prompt) | WiFi network name |
| `--password, -p TEXT` | (prompt) | WiFi password |
| `--port TEXT` | (auto) | Serial port for USB connection |
| `--agent-ip TEXT` | (auto) | IP address of agent host |
| `--agent-port INTEGER` | `8090` | UDP port for Micro-ROS agent |
| `--domain-id INTEGER` | `20` | ROS2 domain ID |
| `--no-reboot` | False | Don't reboot robot after configuration |

#### Examples

```bash
# Interactive setup (prompts for all values)
nimbus wifi setup

# Non-interactive setup
nimbus wifi setup --ssid MyNetwork --password secret123

# Specify agent IP manually
nimbus wifi setup --ssid MyNetwork --agent-ip 192.168.1.100

# Configure without rebooting
nimbus wifi setup --ssid MyNetwork --no-reboot
```

#### Output

```
╭─────────── Nimbus WiFi Setup Wizard ───────────╮
│     Configure robot for wireless operation      │
╰────────────────────────────────────────────────╯

Step 1: Serial Connection
Found serial ports: /dev/ttyUSB0
Using: /dev/ttyUSB0

Step 2: WiFi Credentials
WiFi network name (SSID): MyNetwork
WiFi password: ********
Network: MyNetwork

Step 3: Agent Configuration
Use detected IP address (192.168.1.100)? [Y/n]: y
Agent IP: 192.168.1.100
Agent Port: 8090
ROS Domain ID: 20

Step 4: Apply Configuration
Apply this configuration to the robot? [Y/n]: y

WiFi configuration complete!

Next steps:
  1. Disconnect the USB cable
  2. Power cycle the robot
  3. Wait 10-15 seconds for WiFi connection
  4. Run: nimbus agent start --transport wifi
```

---

### nimbus wifi status

Read current WiFi configuration from the robot (requires USB connection).

```bash
nimbus wifi status [OPTIONS]
```

#### Options

| Option | Description |
|--------|-------------|
| `--port TEXT` | Serial port (auto-detect if not specified) |

#### Output

```
╭─────────── Robot Configuration ───────────╮
│ Property         │ Value                   │
├──────────────────┼─────────────────────────┤
│ Firmware Version │ 1.2.3                   │
│ WiFi SSID        │ MyNetwork               │
│ Agent IP         │ 192.168.1.100           │
│ Agent Port       │ 8090                    │
│ Transport Mode   │ wifi                    │
│ ROS Domain ID    │ 20                      │
╰───────────────────────────────────────────╯
```

---

### nimbus wifi test

Test WiFi connectivity to the robot.

```bash
nimbus wifi test [OPTIONS]
```

#### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout, -t FLOAT` | `5.0` | Test timeout in seconds |

#### Output (Success)

```
Testing connection to 192.168.1.100:8090...
Micro-ROS agent is running
Robot topics detected!
  ✓ /scan
  ✓ /odom_raw
  ✓ /cmd_vel
```

#### Output (Agent Not Running)

```
Testing connection to 192.168.1.100:8090...
Micro-ROS agent is not running
Start it with: nimbus agent start --transport wifi
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
| `NIMBUS_AGENT_TRANSPORT` | Agent transport mode: `serial` or `wifi` (default: serial) |
| `NIMBUS_AGENT_DEVICE` | Serial device path (default: /dev/ttyACM0) |
| `NIMBUS_AGENT_IP` | Agent IP for WiFi mode (default: auto-detect) |
| `NIMBUS_AGENT_PORT` | UDP port for WiFi mode (default: 8090) |
| `NIMBUS_ROS_DOMAIN_ID` | ROS2 domain ID (default: 20) |
| `NIMBUS_DIRECT_MODE` | Enable direct mode by default (default: false) |
| `NIMBUS_DIRECT_ESP32_IP` | ESP32 IP for direct WiFi mode |

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

### Direct mode: "Connection refused" or timeout

1. Verify ESP32 is powered on and connected to WiFi
2. Check ESP32 IP address is correct:
```bash
# Ping the ESP32
ping 192.168.1.100
```
3. Ensure ESP32 firmware supports XRCE-DDS (Micro-ROS client)
4. Check firewall isn't blocking UDP port 8090

### Direct mode: "No data received"

The ESP32 may not be publishing topics yet:
```bash
# Try with verbose logging
NIMBUS_LOG_LEVEL=DEBUG nimbus run --direct --esp32-ip 192.168.1.100
```

Common causes:
- ESP32 still initializing (wait 5-10 seconds after power-on)
- Wrong transport mode on ESP32 (should be WiFi/UDP)
- Network issues between PC and ESP32

### Direct mode: Serial connection issues

```bash
# Check serial port exists
ls -la /dev/ttyACM* /dev/ttyUSB*

# Verify permissions
sudo usermod -aG dialout $USER
# Log out and back in

# Try explicit port
NIMBUS_AGENT_DEVICE=/dev/ttyUSB0 nimbus run --direct
```
