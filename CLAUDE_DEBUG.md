# Claude Control Debug Session

## Status: In Progress

### What's Working
- **Emergency stop fix** (committed `4fa1d18`): `/api/stop` now reliably stops the robot by switching to idle behavior and sending reliable stop commands. Verified with wander start/stop/start/stop cycle.
- **Open-loop fallback** (committed `635fdd9`): claude_control completes move/turn commands via time-based fallback when odometry is stale.
- **XRCE cmd_vel publishing**: Hardcoded fallback values (data_obj_id=6656, request_id=1536) work correctly.
- **Turns and moves**: Verified physically working with claude_control.

### Uncommitted Changes (ready to commit after testing)

#### 1. Obstacle direction check in claude_control (`claude_control.py:216-225`)
**Bug:** claude_control checked `sensors.closest_obstacle < 0.15` without considering direction. An obstacle *behind* the robot (e.g., 0.11m at 143°) would abort a forward move.
**Fix:** Now checks obstacle is within ±45° of front before aborting.

#### 2. Removed EMERGENCY_STOP early return from control loop (`runner.py`)
**Bug:** Added an early return in `_control_step()` that skipped `behavior.compute()` when state was EMERGENCY_STOP. This meant when the safety controller triggered emergency (close obstacle in front), claude_control's compute() never ran, so commands could never timeout or detect obstacles — they hung forever returning "unknown".
**Fix:** Removed the early return. The safety controller already zeros velocity via `limit_velocity()`, and `emergency_stop()` already switches to idle. The early return was redundant and harmful.

#### 3. Garbage odom detection in claude_control (`claude_control.py`)
**Bug:** Garbage odom values (e.g., `x=2.68e+154`, `y=5.248e-315` — CDR parsing artifacts) could set `_odom_ever_changed = True`, preventing the open-loop time-based fallback from activating. Commands would then hang in the odom-based completion path with nonsensical distance calculations.
**Fix:** Added `POSE_SANITY_LIMIT = 1000.0`. In `_compute_move()`, pose values beyond ±1000m are treated as garbage — distance is forced to 0.0 and `_odom_ever_changed` stays False, ensuring open-loop fallback activates. In `_compute_turn()`, theta values outside [-π, π] are ignored for rotation tracking.

### Known Issues (Not Yet Fixed)

1. **Odometry is broken**: Pose data from `/odom_raw` is garbage (denormalized floats, extreme values). The robot physically moves but odom never reflects it. Root cause likely in CDR deserialization of odom messages or the ESP32 odom publisher itself.

2. **Entity tracking misassociates topics**: The XRCE entity tracker labels the /cmd_vel datareader as "rt/imu" due to `_last_topic_name` fallback when entity creation is interleaved across ESP32 nodes. Bypassed with hardcoded fallback values.

3. **Dashboard crash**: The dashboard crashed during a long-running claude_control command that timed out. May be related to the 16s blocking API call or garbage pose values.

4. **LIDAR histogram vs actual obstacles**: The LIDAR histogram showed 1.35m clearance ahead when the user confirmed an obstacle was actually in front. Possible bin alignment issue or the obstacle was below LIDAR scan plane.

### Key Files Modified (this session)
- `nimbus/core/runner.py` — emergency_stop switches to idle, removed EMERGENCY_STOP early return
- `nimbus/behaviors/claude_control.py` — obstacle direction check, garbage odom detection

### Testing Checklist (for next session)
- [ ] Restart nimbus after fresh power cycle
- [ ] Test wander start/stop cycle (should still work)
- [ ] Test claude_control move forward with clear path
- [ ] Test claude_control move when obstacle is behind (should not abort)
- [ ] Test claude_control move when obstacle is in front (should abort with "obstacle")
- [ ] Test claude_control turn commands
- [ ] Test full exploration sequence (multiple moves and turns)
- [ ] Commit if all tests pass

### Architecture Notes

**Control loop flow** (`runner.py:_control_step`):
1. Update sensors
2. Get velocity from behavior (behavior.compute())
3. Apply safety filtering (limit_velocity — only reacts to obstacles in ±45° forward arc)
4. Smooth velocity
5. Send to motors

**claude_control completion flow** (`claude_control.py`):
1. First 5 compute cycles: check if odom values change
2. If odom unchanged (or garbage): switch to open-loop mode
3. Open-loop: complete when elapsed >= distance/speed
4. Odom-based: complete when distance_traveled >= target

**Emergency stop flow** (`runner.py:emergency_stop`):
1. Set state to EMERGENCY_STOP
2. Switch behavior to idle (sends stop, resets smoother)
3. Send 3 reliable stop commands

**XRCE publish flow** (`agent.py:publish`):
1. Try entity-tracked datareader for topic
2. Fallback for /cmd_vel: hardcoded data_obj_id=6656, request_id from pending_read_requests[6656] or 1536
3. Build DATA message and send to ESP32
