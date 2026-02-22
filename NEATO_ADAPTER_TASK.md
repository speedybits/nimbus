# Nimbus Neato Serial Adapter Implementation

## Overview

Implement a **Neato Serial Transport** for the Nimbus robot control platform. This will allow Nimbus to run directly on a Jetson Nano (or similar SBC) mounted inside a Neato vacuum, communicating with the Neato's MCU over its debug serial port.

The goal is to bypass the existing XRCE-DDS/ESP32 transport layer and instead talk directly to the Neato hardware via pyserial.

## Architecture

```
┌─────────────────────────────────────────┐
│           Jetson Nano 2GB               │
│  ┌─────────────────────────────────┐    │
│  │         Nimbus (Python)         │    │
│  │  VFH │ Behaviors │ API │ CLI    │    │
│  └──────────────┬──────────────────┘    │
│                 │                       │
│  ┌──────────────▼──────────────────┐    │
│  │    NeatoTransport (NEW)         │    │
│  │  pyserial → /dev/ttyTHS1        │    │
│  └──────────────┬──────────────────┘    │
└─────────────────┼───────────────────────┘
                  │ UART 115200 baud (3.3V)
┌─────────────────▼───────────────────────┐
│           Neato MCU                     │
│   LIDAR │ Motors │ Sensors │ Battery   │
└─────────────────────────────────────────┘
```

## Repository Context

**Nimbus repo**: https://github.com/speedybits/nimbus

Key existing files to understand:
- `nimbus/core/` - Core abstractions, state machine, config
- `nimbus/sensors/` - LIDAR processing, odometry
- `nimbus/navigation/` - VFH algorithm, safety controller

The existing transport is XRCE-DDS based (for ESP32 + Micro-ROS). We're adding an alternative transport that talks directly to Neato hardware.

## Neato Serial Protocol

The Neato uses a simple ASCII text protocol over serial at **115200 baud**. Commands are sent as text lines, responses come back as text with key-value pairs.

### Entering Test Mode

Many commands require "Test Mode" to be enabled first:

```
TestMode On
```

### Essential Commands

#### GetLDSScan - LIDAR Data
Returns 360° scan data. Each line has: `angle,distance_mm,intensity,error_code`

```
GetLDSScan
```

Response format:
```
AngleInDegrees,DistInMM,Intensity,ErrorCodeHEX
0,622,297,0
1,0,0,8035
2,0,0,8021
3,614,460,0
...
359,0,0,8035
ROTATION_SPEED,5.12
```

- **DistInMM**: Distance in millimeters (0 = invalid reading)
- **ErrorCodeHEX**: Non-zero means invalid (e.g., 8035 = no return)
- **ROTATION_SPEED**: LDS rotation speed in Hz (should be ~5 Hz)

#### SetMotor - Wheel Control
Drive the wheels by specifying distance and speed:

```
SetMotor LWheelDist <mm> RWheelDist <mm> Speed <mm/s>
```

Parameters:
- **LWheelDist**: Left wheel distance in mm (-10000 to +10000, positive = forward)
- **RWheelDist**: Right wheel distance in mm (-10000 to +10000, positive = forward)
- **Speed**: Speed in mm/s (0-300)
- **Accel**: Optional acceleration in mm/s² (defaults to Speed)

Examples:
```
SetMotor LWheelDist 100 RWheelDist 100 Speed 100    # Drive forward 100mm at 100mm/s
SetMotor LWheelDist -100 RWheelDist 100 Speed 100   # Turn left in place
SetMotor LWheelDist 195 RWheelDist -195 Speed 100   # Turn 90° right in place
```

**Important**: The command is asynchronous - it starts the motion and returns immediately. The robot executes until the distance is reached. For continuous velocity control, you'll need to send repeated commands.

**Wheel base**: ~248mm between wheels (use for differential drive kinematics)

#### GetMotors - Odometry Data
Returns motor positions and speeds:

```
GetMotors
```

Response includes:
```
LeftWheel_PositionInMM,2219
LeftWheel_RPM,0
LeftWheel_Speed,0
RightWheel_PositionInMM,1818
RightWheel_RPM,0
RightWheel_Speed,0
```

#### SetLDSRotation - Enable/Disable LIDAR
```
SetLDSRotation On   # Start LIDAR spinning
SetLDSRotation Off  # Stop LIDAR
```

#### GetCharger - Battery Status
```
GetCharger
```

Returns:
```
FuelPercent,85
BatteryOverTemp,0
ChargingActive,0
VBattV,16.2
```

#### GetDigitalSensors - Bumpers, Cliff, etc.
```
GetDigitalSensors
```

Returns:
```
SNSR_LEFT_WHEEL_EXTENDED,0
SNSR_RIGHT_WHEEL_EXTENDED,0
LSIDEBIT,0           # Left side bumper
LFRONTBIT,0          # Left front bumper
RSIDEBIT,0           # Right side bumper
RFRONTBIT,0          # Right front bumper
```

#### GetAnalogSensors - Drop Sensors, Wall Sensor
```
GetAnalogSensors
```

Returns:
```
LeftDropInMM,150
RightDropInMM,150
WallSensorInMM,81
BatteryVoltageInmV,16200
```

### Response Parsing

Responses are newline-delimited. Most commands return key-value pairs:
```
Key,Value
Key,Value
...
```

GetLDSScan is special - it returns CSV data with angle as the implicit index.

### Error Handling

- If a command fails, you may get an error response
- GetErr can retrieve the last error
- Some commands only work in TestMode
- Invalid commands return "No recognizable parameters" or similar

## Implementation Requirements

### 1. Create `nimbus/core/neato_transport.py`

```python
"""
Neato Serial Transport for Nimbus

Provides direct communication with Neato vacuum MCU via serial port,
replacing the XRCE-DDS transport for embedded deployments.
"""

import serial
import threading
import time
from dataclasses import dataclass
from typing import Optional, Callable, List, Tuple
import math

@dataclass
class LidarScan:
    """360-degree LIDAR scan data"""
    ranges: List[float]          # Distance in meters (inf for invalid)
    intensities: List[int]       # Signal intensity
    angle_min: float = 0.0       # radians
    angle_max: float = 2 * math.pi
    angle_increment: float = math.pi / 180  # 1 degree
    timestamp: float = 0.0

@dataclass
class Odometry:
    """Robot odometry state"""
    x: float = 0.0              # meters
    y: float = 0.0              # meters
    theta: float = 0.0          # radians
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    timestamp: float = 0.0

@dataclass
class BumperState:
    """Bumper and cliff sensor state"""
    left_bumper: bool = False
    right_bumper: bool = False
    left_cliff: bool = False
    right_cliff: bool = False

class NeatoTransport:
    """
    Serial transport for Neato vacuum robots.
    
    Handles:
    - LIDAR scanning at ~5Hz
    - Differential drive motor control
    - Odometry computation from wheel encoders
    - Bumper/cliff sensor monitoring
    - Battery status
    """
    
    WHEEL_BASE_MM = 248.0  # Distance between wheels
    MAX_SPEED_MMS = 300    # Max wheel speed mm/s
    
    def __init__(
        self,
        port: str = "/dev/ttyTHS1",  # Jetson UART
        baudrate: int = 115200,
        scan_callback: Optional[Callable[[LidarScan], None]] = None,
        odom_callback: Optional[Callable[[Odometry], None]] = None,
    ):
        # Initialize serial, callbacks, state tracking
        # Start background threads for scanning and odometry
        pass
    
    def connect(self) -> bool:
        """Open serial connection and enter test mode"""
        pass
    
    def disconnect(self):
        """Clean shutdown - stop motors, disable LDS, exit test mode"""
        pass
    
    def send_velocity(self, linear_mps: float, angular_rps: float):
        """
        Send velocity command (differential drive kinematics).
        
        Args:
            linear_mps: Forward velocity in meters/second
            angular_rps: Angular velocity in radians/second
        """
        # Convert to wheel velocities using differential drive equations:
        # v_left = linear - (angular * wheel_base / 2)
        # v_right = linear + (angular * wheel_base / 2)
        pass
    
    def stop(self):
        """Emergency stop - immediately halt all motors"""
        pass
    
    def get_scan(self) -> Optional[LidarScan]:
        """Get latest LIDAR scan (non-blocking)"""
        pass
    
    def get_odometry(self) -> Odometry:
        """Get current odometry estimate"""
        pass
    
    def get_bumpers(self) -> BumperState:
        """Get current bumper/cliff state"""
        pass
    
    def get_battery_percent(self) -> int:
        """Get battery charge percentage"""
        pass
    
    # Internal methods
    def _send_command(self, cmd: str) -> List[str]:
        """Send command and read response lines"""
        pass
    
    def _parse_lds_scan(self, lines: List[str]) -> LidarScan:
        """Parse GetLDSScan response into LidarScan"""
        pass
    
    def _scan_thread(self):
        """Background thread: continuous LIDAR scanning"""
        pass
    
    def _odom_thread(self):
        """Background thread: odometry updates from wheel encoders"""
        pass
```

### 2. Differential Drive Kinematics

For converting Nimbus velocity commands to Neato wheel commands:

```python
def velocity_to_wheel_speeds(linear_mps: float, angular_rps: float, wheel_base_m: float) -> Tuple[float, float]:
    """
    Convert (linear, angular) velocity to (left, right) wheel speeds.
    
    Args:
        linear_mps: Linear velocity in m/s (positive = forward)
        angular_rps: Angular velocity in rad/s (positive = counter-clockwise)
        wheel_base_m: Distance between wheels in meters
    
    Returns:
        (left_speed_mps, right_speed_mps)
    """
    half_base = wheel_base_m / 2.0
    left = linear_mps - angular_rps * half_base
    right = linear_mps + angular_rps * half_base
    return (left, right)
```

For odometry from wheel encoders:

```python
def update_odometry(
    odom: Odometry,
    delta_left_m: float,
    delta_right_m: float,
    wheel_base_m: float,
    dt: float
) -> Odometry:
    """
    Update odometry from wheel encoder deltas.
    
    Uses midpoint integration for better accuracy during turns.
    """
    delta_dist = (delta_left_m + delta_right_m) / 2.0
    delta_theta = (delta_right_m - delta_left_m) / wheel_base_m
    
    # Midpoint heading for integration
    mid_theta = odom.theta + delta_theta / 2.0
    
    new_x = odom.x + delta_dist * math.cos(mid_theta)
    new_y = odom.y + delta_dist * math.sin(mid_theta)
    new_theta = odom.theta + delta_theta
    
    # Normalize theta to [-pi, pi]
    new_theta = math.atan2(math.sin(new_theta), math.cos(new_theta))
    
    return Odometry(
        x=new_x,
        y=new_y,
        theta=new_theta,
        linear_velocity=delta_dist / dt if dt > 0 else 0,
        angular_velocity=delta_theta / dt if dt > 0 else 0,
        timestamp=time.time()
    )
```

### 3. LIDAR Data Conversion

Convert Neato's mm distances to Nimbus's expected format:

```python
def parse_lds_scan(response_lines: List[str]) -> LidarScan:
    """
    Parse GetLDSScan response.
    
    Neato returns: angle,distance_mm,intensity,error_code
    Convert to: ranges in meters, inf for invalid readings
    """
    ranges = [float('inf')] * 360
    intensities = [0] * 360
    
    for line in response_lines:
        if line.startswith('ROTATION_SPEED'):
            continue
        if ',' not in line:
            continue
        
        parts = line.split(',')
        if len(parts) >= 4:
            try:
                angle = int(parts[0])
                dist_mm = int(parts[1])
                intensity = int(parts[2])
                error = int(parts[3])
                
                if 0 <= angle < 360:
                    if error == 0 and dist_mm > 0:
                        ranges[angle] = dist_mm / 1000.0  # mm to meters
                        intensities[angle] = intensity
            except ValueError:
                continue
    
    return LidarScan(
        ranges=ranges,
        intensities=intensities,
        timestamp=time.time()
    )
```

### 4. Velocity Command Strategy

The Neato's SetMotor command uses distance targets, not continuous velocity. For smooth motion control:

**Option A: Distance-based commands with re-issuing**
```python
def send_velocity(self, linear_mps: float, angular_rps: float):
    """Send velocity by issuing distance commands that get refreshed."""
    left_mps, right_mps = velocity_to_wheel_speeds(
        linear_mps, angular_rps, self.WHEEL_BASE_MM / 1000.0
    )
    
    # Command a short distance at the desired speed
    # Re-issue every ~100ms to maintain velocity
    COMMAND_DURATION_S = 0.2
    left_mm = int(left_mps * 1000 * COMMAND_DURATION_S)
    right_mm = int(right_mps * 1000 * COMMAND_DURATION_S)
    speed_mms = int(max(abs(left_mps), abs(right_mps)) * 1000)
    
    if speed_mms > 0:
        cmd = f"SetMotor LWheelDist {left_mm} RWheelDist {right_mm} Speed {speed_mms}"
        self._send_command(cmd)
```

**Option B: Continuous command thread**
Run a background thread that continuously re-issues the current velocity command at ~10Hz.

### 5. Integration with Nimbus

Create an adapter that makes NeatoTransport look like Nimbus's expected sensor/motor interfaces:

```python
# nimbus/core/neato_adapter.py

class NeatoAdapter:
    """
    Adapts NeatoTransport to Nimbus's sensor/motor interfaces.
    
    Replaces the XRCE agent for Neato deployments.
    """
    
    def __init__(self, port: str = "/dev/ttyTHS1"):
        self.transport = NeatoTransport(
            port=port,
            scan_callback=self._on_scan,
            odom_callback=self._on_odom,
        )
        self._latest_scan = None
        self._scan_lock = threading.Lock()
    
    def start(self):
        """Connect and start sensor streaming"""
        self.transport.connect()
    
    def stop(self):
        """Stop and disconnect"""
        self.transport.disconnect()
    
    # Nimbus sensor interface
    def get_lidar_scan(self):
        """Returns scan in Nimbus's expected format"""
        with self._scan_lock:
            return self._latest_scan
    
    def get_odometry(self):
        """Returns odom in Nimbus's expected format"""
        return self.transport.get_odometry()
    
    # Nimbus motor interface
    def send_cmd_vel(self, linear: float, angular: float):
        """Send velocity command"""
        self.transport.send_velocity(linear, angular)
    
    def emergency_stop(self):
        """Immediate stop"""
        self.transport.stop()
```

### 6. Configuration

Add Neato-specific config to `nimbus/core/config.py`:

```yaml
# config.yaml
transport:
  type: "neato"  # or "xrce" for ESP32
  neato:
    port: "/dev/ttyTHS1"
    baudrate: 115200
    wheel_base_mm: 248
    max_speed_mms: 300
    scan_rate_hz: 5
```

### 7. CLI Integration

Update `nimbus/cli/` to support selecting transport:

```bash
nimbus run --transport neato --port /dev/ttyTHS1
nimbus run --transport neato --discover  # Try common ports
```

## Testing

### Mock Mode
Create a mock Neato serial interface for testing without hardware:

```python
class MockNeatoSerial:
    """Simulates Neato serial responses for testing"""
    
    def write(self, data):
        self._pending_command = data.decode().strip()
    
    def readlines(self):
        if self._pending_command == "GetLDSScan":
            return self._generate_mock_scan()
        # etc.
```

### Hardware Testing Checklist
1. [ ] Serial connection opens successfully
2. [ ] TestMode On/Off works
3. [ ] GetLDSScan returns valid data
4. [ ] LDS rotation enables/disables
5. [ ] SetMotor moves wheels
6. [ ] Odometry accumulates correctly
7. [ ] Bumper sensors report correctly
8. [ ] Emergency stop works reliably
9. [ ] Clean disconnect (motors stop, LDS off, TestMode off)

## Reference Links

- **Neato Programmer's Manual**: https://help.neatorobotics.com/wp-content/uploads/2020/07/XV-ProgrammersManual-3_1.pdf
- **neato-driver-python** (reference implementation): https://github.com/brannonvann/neato-driver-python
- **neato-connected** (ESP32 approach): https://github.com/Philip2809/neato-connected
- **Nimbus repo**: https://github.com/speedybits/nimbus

## Hardware Notes

### Jetson Nano 2GB
- UART on `/dev/ttyTHS1` (pins 8/10 on 40-pin header)
- 3.3V logic levels (compatible with Neato)
- Needs USB WiFi dongle (no built-in WiFi)
- Power: 5V @ 2-3A via barrel jack or GPIO header

### Neato Power
- Battery: 14.4V NiMH or Li-ion (model dependent)
- Use buck converter (e.g., 5V/5A step-down) to power Jetson
- Can tap 16V rail on Neato motherboard

### Serial Connection
- Neato debug port: 3.3V UART, 115200 baud
- Connect: Jetson TX → Neato RX, Jetson RX → Neato TX, GND ↔ GND
- No level shifter needed (both 3.3V)

## Success Criteria

1. `nimbus run --transport neato` starts successfully
2. LIDAR data feeds into VFH algorithm
3. Wander behavior navigates without collisions
4. Odometry tracks position reasonably
5. Bumper hits trigger safety stops
6. Battery level displays in CLI dashboard
7. Clean shutdown with `nimbus stop` or Ctrl+C
