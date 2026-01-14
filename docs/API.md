# Nimbus API Reference

Nimbus provides REST and WebSocket APIs for external control and monitoring.

## Base URL

```
http://localhost:8080
```

The port can be configured via `NIMBUS_API_PORT` environment variable or `api.rest_port` in config.

## REST API

### Health Check

Check if Nimbus is running and healthy.

```http
GET /api/health
```

**Response:**
```json
{
  "status": "ok",
  "running": true,
  "version": "1.0.0"
}
```

---

### Get Status

Get current robot state, position, and sensor data.

```http
GET /api/status
```

**Response:**
```json
{
  "state": "NAVIGATING",
  "pose": {
    "x": 1.5,
    "y": 2.0,
    "theta": 0.785
  },
  "velocity": {
    "linear": 0.2,
    "angular": 0.1
  },
  "closest_obstacle": 1.25,
  "current_behavior": "wander",
  "target": {
    "x": 3.0,
    "y": 4.0,
    "theta": 0.0
  }
}
```

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `state` | string | One of: `IDLE`, `NAVIGATING`, `AVOIDING`, `EMERGENCY_STOP`, `CALIBRATING` |
| `pose.x` | float | X position in meters |
| `pose.y` | float | Y position in meters |
| `pose.theta` | float | Heading in radians (0 = +X axis) |
| `velocity.linear` | float | Forward velocity in m/s |
| `velocity.angular` | float | Rotational velocity in rad/s |
| `closest_obstacle` | float | Distance to nearest obstacle in meters |
| `current_behavior` | string | Active behavior name |
| `target` | object | Navigation target (null if none) |

---

### Get Sensors

Get detailed sensor readings.

```http
GET /api/sensors
```

**Response:**
```json
{
  "timestamp": "2024-01-15T10:30:00.123456",
  "pose": {
    "x": 1.5,
    "y": 2.0,
    "theta": 0.785
  },
  "velocity": {
    "linear": 0.2,
    "angular": 0.1
  },
  "closest_obstacle": 1.25,
  "obstacle_direction": 0.52,
  "lidar_histogram": [0.0, 0.1, 0.3, ...]
}
```

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `timestamp` | string | ISO 8601 timestamp |
| `obstacle_direction` | float | Angle to nearest obstacle in radians |
| `lidar_histogram` | array | 72-element polar histogram (0-1 values) |

---

### Navigate to Position

Send the robot to a specific location.

```http
POST /api/navigate
Content-Type: application/json

{
  "x": 3.0,
  "y": 4.0,
  "theta": 1.57
}
```

**Request Body:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `x` | float | Yes | Target X coordinate in meters |
| `y` | float | Yes | Target Y coordinate in meters |
| `theta` | float | No | Target orientation in radians |

**Response:**
```json
{
  "status": "navigating",
  "target": {
    "x": 3.0,
    "y": 4.0,
    "theta": 1.57
  }
}
```

**Errors:**

| Status | Description |
|--------|-------------|
| 400 | Failed to start navigation |
| 503 | Nimbus not initialized |

---

### Emergency Stop

Immediately halt all robot motion.

```http
POST /api/stop
```

**Response:**
```json
{
  "status": "stopped"
}
```

This endpoint always succeeds, even if Nimbus is not fully initialized.

---

### List Behaviors

Get available behaviors and current active behavior.

```http
GET /api/behaviors
```

**Response:**
```json
{
  "behaviors": ["idle", "wander", "simple_wander", "goto", "patrol"],
  "current": "wander"
}
```

---

### Set Behavior

Activate a specific behavior.

```http
POST /api/behavior/{name}
```

**Path Parameters:**

| Parameter | Description |
|-----------|-------------|
| `name` | Behavior name (e.g., `wander`, `idle`, `goto`) |

**Response:**
```json
{
  "status": "ok",
  "behavior": "wander"
}
```

**Errors:**

| Status | Description |
|--------|-------------|
| 404 | Unknown behavior |
| 400 | Failed to activate behavior |
| 503 | Nimbus not initialized |

---

### Get Safety Status

Get current safety controller state.

```http
GET /api/safety
```

**Response:**
```json
{
  "level": "NORMAL",
  "is_emergency": false,
  "can_move_forward": true,
  "emergency_duration": 0.0,
  "can_recover": true,
  "consecutive_emergencies": 0
}
```

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `level` | string | `NORMAL`, `CAUTION`, or `EMERGENCY` |
| `is_emergency` | boolean | True if in emergency stop |
| `can_move_forward` | boolean | True if forward motion allowed |
| `emergency_duration` | float | Seconds in emergency state |
| `can_recover` | boolean | True if recovery timeout passed |
| `consecutive_emergencies` | int | Count of back-to-back emergencies |

---

## WebSocket API

### Telemetry Stream

Real-time sensor data at 10Hz.

```
WebSocket /ws/telemetry
```

**Connect:**
```javascript
const ws = new WebSocket('ws://localhost:8080/ws/telemetry');

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log(data);
};
```

**Message Format:**
```json
{
  "type": "telemetry",
  "timestamp": "2024-01-15T10:30:00.123456",
  "state": "NAVIGATING",
  "pose": {
    "x": 1.5,
    "y": 2.0,
    "theta": 0.785
  },
  "velocity": {
    "linear": 0.2,
    "angular": 0.1
  },
  "closest_obstacle": 1.25
}
```

---

### LIDAR Stream

Full LIDAR data for visualization.

```
WebSocket /ws/lidar
```

**Message Format:**
```json
{
  "type": "lidar",
  "timestamp": "2024-01-15T10:30:00.123456",
  "ranges": [1.2, 1.3, 1.5, ...],
  "closest": 0.85,
  "closest_angle": 0.35
}
```

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `ranges` | array | Downsampled LIDAR ranges (72 points) |
| `closest` | float | Nearest obstacle distance |
| `closest_angle` | float | Angle to nearest obstacle |

---

### Event Stream

State changes, alerts, and navigation events.

```
WebSocket /ws/events
```

**Message Format:**
```json
{
  "type": "event",
  "event_type": "state_change",
  "old_state": "IDLE",
  "new_state": "NAVIGATING"
}
```

**Event Types:**

| Event | Description |
|-------|-------------|
| `state_change` | Robot state transition |
| `goal_reached` | Navigation completed |
| `emergency` | Emergency stop triggered |
| `behavior_change` | Active behavior changed |

---

## Error Responses

All error responses follow this format:

```json
{
  "detail": "Error message here"
}
```

**Common Status Codes:**

| Code | Description |
|------|-------------|
| 200 | Success |
| 400 | Bad request (invalid parameters) |
| 404 | Not found (unknown behavior, etc.) |
| 503 | Service unavailable (Nimbus not ready) |

---

## Examples

### Python (requests)

```python
import requests

# Get status
r = requests.get('http://localhost:8080/api/status')
print(r.json())

# Navigate
r = requests.post('http://localhost:8080/api/navigate', json={
    'x': 2.0,
    'y': 3.0
})
print(r.json())

# Emergency stop
requests.post('http://localhost:8080/api/stop')
```

### Python (websockets)

```python
import asyncio
import websockets
import json

async def monitor():
    uri = "ws://localhost:8080/ws/telemetry"
    async with websockets.connect(uri) as ws:
        while True:
            data = json.loads(await ws.recv())
            print(f"Position: ({data['pose']['x']:.2f}, {data['pose']['y']:.2f})")

asyncio.run(monitor())
```

### curl

```bash
# Get status
curl http://localhost:8080/api/status

# Navigate
curl -X POST http://localhost:8080/api/navigate \
  -H "Content-Type: application/json" \
  -d '{"x": 2.0, "y": 3.0}'

# Set behavior
curl -X POST http://localhost:8080/api/behavior/wander

# Emergency stop
curl -X POST http://localhost:8080/api/stop
```

### JavaScript (fetch)

```javascript
// Get status
const status = await fetch('http://localhost:8080/api/status')
  .then(r => r.json());

// Navigate
await fetch('http://localhost:8080/api/navigate', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ x: 2.0, y: 3.0 })
});

// WebSocket telemetry
const ws = new WebSocket('ws://localhost:8080/ws/telemetry');
ws.onmessage = (e) => console.log(JSON.parse(e.data));
```

---

## Home Assistant Integration

Nimbus can integrate with Home Assistant using REST sensors and commands.

### REST Sensor (configuration.yaml)

```yaml
sensor:
  - platform: rest
    name: Nimbus State
    resource: http://localhost:8080/api/status
    value_template: "{{ value_json.state }}"
    scan_interval: 5

  - platform: rest
    name: Nimbus Obstacle Distance
    resource: http://localhost:8080/api/status
    value_template: "{{ value_json.closest_obstacle | round(2) }}"
    unit_of_measurement: "m"
    scan_interval: 5
```

### RESTful Command

```yaml
rest_command:
  nimbus_goto:
    url: http://localhost:8080/api/navigate
    method: POST
    content_type: application/json
    payload: '{"x": {{ x }}, "y": {{ y }}}'

  nimbus_stop:
    url: http://localhost:8080/api/stop
    method: POST

  nimbus_wander:
    url: http://localhost:8080/api/behavior/wander
    method: POST
```

### Automation Example

```yaml
automation:
  - alias: "Robot patrol at night"
    trigger:
      - platform: time
        at: "22:00:00"
    action:
      - service: rest_command.nimbus_wander
```
