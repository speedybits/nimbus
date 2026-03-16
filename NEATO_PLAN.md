# Neato Serial Transport: Review & Fix Plan

## Context

**Repo:** `/home/mike/projects/nimbus` on branch `neato`

The Neato serial transport in `nimbus/core/neato/` is functionally complete for LIDAR scanning and basic motor control, but has several bugs and missing integrations that block hardware testing items 5-9 (motors, odometry, bumpers, e-stop, clean disconnect). This plan addresses critical bugs, missing safety integrations, and observability gaps.

## Issues Found (Prioritized)

### Critical Bugs
1. **Hex error code parsing** — `transport.py:130` uses `int()` instead of `int(x, 16)` for ErrorCodeHEX column. Hex codes with a-f digits silently drop scan lines.
2. **Wrong bumper key names** — `transport.py:174-179` looks for `SNSR_LEFT_BUMPER`/`SNSR_RIGHT_BUMPER` but Neato XV returns `LSIDEBIT`/`LFRONTBIT`/`RSIDEBIT`/`RFRONTBIT`. Bumper reads always return False on real hardware.

### Missing Safety Integrations
3. **Bumper data never polled** — `NeatoNode` has no loop calling `get_digital_sensors()`. Physical bumper presses do nothing.
4. **Safety controller ignores bumpers** — `SafetyController.limit_velocity()` only uses LIDAR distance, no bumper input.

### Missing Observability
5. **Silent serial failures** — All 3 background threads use bare `except Exception: pass`. No logging, no disconnection detection, no reconnection.
6. **Battery not displayed** — `get_charger()` exists but is never called; dashboard doesn't show battery status.

### Missing Features
7. **No GetAnalogSensors** — Cliff/drop detection (`LeftDropInMM`, `RightDropInMM`) not implemented.
8. **NeatoBumperData too simplistic** — Only 2 zones instead of 4 (LSIDE, LFRONT, RSIDE, RFRONT).

---

## Implementation Plan

### Phase 1: Critical Bug Fixes

#### 1A. Fix hex error code parsing
**File:** `nimbus/core/neato/transport.py` line 130
- Change `int(parts[3].strip())` → `int(parts[3].strip(), 16)`

#### 1B. Expand NeatoBumperData to 4 zones
**File:** `nimbus/core/neato/transport.py` lines 39-45
- Replace `left_bumper`/`right_bumper`/`wall_sensor` with `left_side_bumper`, `left_front_bumper`, `right_side_bumper`, `right_front_bumper`
- Add convenience properties: `left_bumper`, `right_bumper`, `any_bumper`, `any_front_bumper`
- Keep `left_wheel_drop`/`right_wheel_drop`

#### 1C. Fix bumper key names in `get_digital_sensors()`
**File:** `nimbus/core/neato/transport.py` lines 170-180
- Map `LSIDEBIT` → `left_side_bumper`, `LFRONTBIT` → `left_front_bumper`, etc.

#### 1D. Propagate to MockNeatoTransport
**File:** `nimbus/core/neato/mock_serial.py`
- Update `get_digital_sensors()` for new dataclass
- Add `trigger_bumper(zone)` test helper

#### 1E. Propagate to SimNeatoTransport
**File:** `nimbus/core/neato/sim_transport.py`
- Update `get_digital_sensors()` for new dataclass

#### 1F. Fix tests
**File:** `nimbus/tests/unit/test_neato_transport.py`
- Update `TestBumperParsing` to use real Neato XV key names
- Add hex error code test cases to `TestLDSScanParsing`

### Phase 2: Bumper & Battery Sensor Loop

#### 2A. Add 4th background thread to NeatoNode
**File:** `nimbus/core/neato/node.py`
- Add `_sensor_loop` thread running at ~5Hz
- Each iteration: `get_digital_sensors()` → inject `NeatoBumperData` on `/bumpers` topic
- Every 25th iteration (~0.2Hz): `get_charger()` → store in `_battery_data` + inject on `/battery`
- Guard serial access with existing `_serial_lock`

#### 2B. Add bumper input to SafetyController
**File:** `nimbus/navigation/safety.py`
- Add `bumper_triggered: bool = False` parameter to `evaluate()` and `limit_velocity()`
- When `bumper_triggered=True`: return `EMERGENCY` immediately (physical contact overrides LIDAR)
- Backwards-compatible: existing callers unchanged

#### 2C. Add `bumper_triggered` to SensorSnapshot
**File:** `nimbus/core/state.py`
- Add `bumper_triggered: bool = False` field to `SensorSnapshot` (frozen dataclass, backward-compatible default)

#### 2D. Wire through runner
**File:** `nimbus/core/runner.py`
- In `start()`: subscribe to `/bumpers` topic
- In `_update_sensors()`: read bumper buffer, set `bumper_triggered` in snapshot
- In `_control_step()`: pass `bumper_triggered` to `safety.limit_velocity()`

### Phase 3: Serial Error Handling

#### 3A. Add logging to NeatoNode
**File:** `nimbus/core/neato/node.py`
- Replace all `except Exception: pass` with `logger.warning(...)`
- Add `import logging; logger = logging.getLogger(__name__)`

#### 3B. Add serial health tracking + reconnection
**File:** `nimbus/core/neato/node.py`
- Track `_consecutive_errors` counter per thread
- After 3+ consecutive errors: set `_serial_healthy = False`
- In `_sensor_loop`: attempt reconnect with exponential backoff (0.5s → 10s cap)
- Other threads skip serial calls when `_serial_healthy is False`

#### 3C. Distinguish timeout from disconnect
**File:** `nimbus/core/neato/transport.py`
- After `_send_command()` gets empty readline, check `self._serial.is_open`
- If port closed: raise `SerialDisconnectedError` (new exception class)

### Phase 4: Battery Display & Analog Sensors

#### 4A. Expose battery in runner
**File:** `nimbus/core/runner.py`
- Subscribe to `/battery` topic in `start()`
- Add `battery_status` property reading from buffer

#### 4B. Display battery in dashboard
**File:** `nimbus/cli/dashboard.py`
- Add battery row to status panel (green >50%, yellow 20-50%, red <20%)

#### 4C. Implement GetAnalogSensors
**File:** `nimbus/core/neato/transport.py`
- Add `NeatoAnalogData` dataclass (`left_drop_mm`, `right_drop_mm`, `wall_sensor_mm`)
- Add `get_analog_sensors()` method
- Add to mock and sim transports
- Integrate cliff detection into sensor loop (large drop = emergency)

---

## Verification

1. **Unit tests**: `cd /home/mike/projects/nimbus && python -m pytest nimbus/tests/unit/test_neato_transport.py nimbus/tests/unit/test_neato_node.py nimbus/tests/unit/test_neato_kinematics.py -v`
2. **All tests pass**: `python -m pytest nimbus/tests/ -v`
3. **Smoke test (mock mode)**: `python -m nimbus run --transport neato --mock` — verify startup, scan generation, odom, bumper topic, battery display, clean shutdown with Ctrl+C
4. **Manual verification**: After each phase, run tests to catch regressions before proceeding
