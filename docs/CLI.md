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
│ motor       Direct motor control.                                            │
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
| `--mock` | False | Use mock node (no hardware required) |
| `--discover` | False | Auto-discover ESP32 IP on network |
| `-v, --verbosity` | 1 | Log verbosity: 1=minimal, 2=normal, 3=debug |

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

# With auto-discover
nimbus run --discover --behavior wander

# Debug verbosity
nimbus run -v 3 --behavior wander
```

### Live Dashboard

When running with `--dashboard` (default), you'll see a live terminal interface with side-by-side LIDAR and Map views:

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
├────────── LIDAR View ────────┬─────────── Map View ─────────────────────┤
│        .  .  .               │  . . . . . # # . .                       │
│    .  .        .  .          │  . . ~ ~ ~ # # . .                       │
│  .                    .      │  . . ~ . . . . . .                       │
│ .          ^            .    │  . . ~ . R . . . .                       │
│ .          R            .    │  . . ~ . . . . . .                       │
│  .                    .      │  # # # # . . . . .                       │
│    .  .        .  .          │  . . . . . . . . .                       │
│        .  .  .               │                                          │
│                              │  Obs:42 Free:186                         │
│  Closest: 1.25m              │                                          │
├──────────────────────────────┴──────────────────────────────────────────┤
│                            Shortcuts                                     │
╰──────────────────────────────────────────────────────────────────────────╯
```

**Dashboard Views:**

| View | Description |
|------|-------------|
| **LIDAR View** | Real-time robot-centric view of obstacles. `R` = robot, `^` = forward direction, `.` = detected obstacle |
| **Map View** | Accumulated world map. `R` = robot, `~` = trail, `#` = obstacle, `.` = free space |

**Map View Symbols:**

| Symbol | Meaning |
|--------|---------|
| ` ` (space) | Unknown/unexplored |
| `.` (dim) | Free space (LIDAR ray passed through) |
| `#` (red) | High-confidence obstacle |
| `+` (yellow) | Lower-confidence obstacle |
| `~` (cyan) | Robot trail (recent path) |
| `R` (green) | Current robot position |

The map auto-zooms as exploration expands and stays centered on the explored area.

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

---

## nimbus stop

Emergency stop - immediately halt all motion.

```bash
nimbus stop
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

---

## nimbus behaviors

List available behaviors.

```bash
nimbus behaviors
```

---

## nimbus behavior

Set the active behavior.

```bash
nimbus behavior NAME
```

### Examples

```bash
# Switch to wander mode
nimbus behavior wander

# Switch to idle (stop)
nimbus behavior idle

# Switch to motor_test (direct control)
nimbus behavior motor_test
```

---

## nimbus motor

Direct motor control (requires Nimbus to be running).

```bash
nimbus motor ACTION [OPTIONS]
```

### Actions

| Action | Description |
|--------|-------------|
| `forward` | Move forward |
| `backward` | Move backward |
| `left` | Turn left |
| `right` | Turn right |
| `stop` | Stop motors |
| `velocity` | Custom velocity command |

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--speed, -s FLOAT` | `0.1` | Speed for preset actions |
| `--linear, -l FLOAT` | `0.0` | Linear velocity for `velocity` action |
| `--angular, -a FLOAT` | `0.0` | Angular velocity for `velocity` action |

### Examples

```bash
nimbus motor forward --speed 0.15     # Move forward
nimbus motor left --speed 0.5         # Turn left
nimbus motor stop                     # Stop motors
nimbus motor velocity -l 0.1 -a 0.2   # Custom velocity
```

---

## nimbus test

Run the Nimbus test suite.

```bash
nimbus test [PATTERN] [OPTIONS]
```

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

# Run with coverage
nimbus test --cov
```

---

## nimbus version

Show Nimbus version.

```bash
nimbus version
```

---

## nimbus wifi

Configure WiFi connectivity for Yahboom robots.

### nimbus wifi setup

Interactive 6-step wizard to configure WiFi on the robot via USB.

```bash
nimbus wifi setup [OPTIONS]
```

#### Wizard Steps

1. **Prerequisites Check** - Verifies dialout group membership for serial port access
2. **USB Connection** - Detects new USB devices when robot is connected
3. **WiFi Credentials** - Validates SSID length (max 32 chars) and password
4. **Agent Configuration** - Choose mDNS hostname, fixed IP, or custom address
5. **Configuration Summary** - Review all settings before applying
6. **Apply Configuration** - Send config to robot with retry on failure

#### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--ssid, -s TEXT` | (prompt) | WiFi network name |
| `--password, -p TEXT` | (prompt) | WiFi password |
| `--port TEXT` | (auto) | Serial port for USB connection |
| `--agent-ip TEXT` | (auto) | IP address of agent host |
| `--agent-hostname TEXT` | None | mDNS hostname (e.g., `myhost.local`) |
| `--agent-port INTEGER` | `8090` | UDP port for XRCE agent |
| `--domain-id INTEGER` | `20` | ROS2 domain ID |
| `--no-reboot` | False | Don't reboot robot after configuration |
| `--skip-dialout-check` | False | Skip dialout group verification |

#### Examples

```bash
# Interactive wizard (recommended for first-time setup)
nimbus wifi setup

# Pre-fill WiFi credentials
nimbus wifi setup --ssid MyNetwork --password mypassword

# Use mDNS hostname for agent (recommended - survives IP changes)
nimbus wifi setup --agent-hostname mycomputer.local

# Use fixed IP address
nimbus wifi setup --agent-ip 192.168.1.100

# Specify serial port directly
nimbus wifi setup --port /dev/ttyUSB0

# Non-interactive with all options
nimbus wifi setup -s MyNetwork -p mypassword --agent-ip 192.168.1.100 --skip-dialout-check
```

#### Dialout Group

Serial port access requires membership in the `dialout` group. If not a member:

```bash
sudo usermod -aG dialout $USER
# Log out and back in for changes to take effect
```

---

### nimbus wifi status

Read current WiFi configuration from the robot (requires USB connection).

```bash
nimbus wifi status [OPTIONS]
```

---

### nimbus wifi discover

Scan the local network to find ESP32 robots.

```bash
nimbus wifi discover [OPTIONS]
```

#### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout, -t FLOAT` | `3.0` | Scan timeout per host in seconds |
| `--probe, -p` | False | Listen for robot traffic on UDP port 8090 |

#### Examples

```bash
# Basic discovery
nimbus wifi discover

# Listen for robot traffic (power cycle robot while listening)
nimbus wifi discover --probe --timeout 30
```

---

## Shell Completion

Nimbus supports shell completion for bash, zsh, and fish.

```bash
# Install completion
nimbus --install-completion bash
nimbus --install-completion zsh
nimbus --install-completion fish
```

---

## Environment Variables

| Variable | Description |
|----------|-------------|
| `NIMBUS_API_PORT` | REST API port (default: 8080) |
| `NIMBUS_MAX_SPEED` | Maximum linear speed (default: 0.30) |
| `NIMBUS_SAFETY_RADIUS` | Safety radius in meters (default: 0.30) |
| `NIMBUS_AGENT_IP` | Agent IP (default: auto-detect) |
| `NIMBUS_AGENT_PORT` | UDP port for XRCE agent (default: 8090) |

---

## Troubleshooting

### "Could not connect to Nimbus"

Nimbus is not running. Start it first:
```bash
nimbus run
```

### "Connection refused" or timeout

1. Verify ESP32 is powered on and connected to WiFi
2. Check firewall isn't blocking UDP port 8090
3. Try with verbose logging:
```bash
nimbus run -v 3 --behavior wander
```

### Dashboard not displaying correctly

Try running without dashboard:
```bash
nimbus run --no-dashboard
```

Or check terminal size (minimum 80x24 recommended).
