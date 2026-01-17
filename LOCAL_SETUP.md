# Local Setup Notes

## Environment Setup
- Date: 2026-01-17
- Platform: macOS Darwin 24.6.0
- Python: 3.13.7

## Installation Steps Completed

1. **Created Python virtual environment**
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

2. **Installed Nimbus package with dev dependencies**
   ```bash
   pip install -e ".[dev]"
   ```

3. **Tested installation**
   - ✅ All 71 unit tests passed
   - ✅ Mock mode runs successfully
   - ✅ API server starts on http://localhost:8080
   - ✅ Behaviors available: idle, wander, simple_wander, goto, patrol, explore, ai_explore

## Running Nimbus

### Mock Mode (no hardware required)
```bash
./venv/bin/nimbus run --mock --behavior wander
```

### Real Robot Mode (requires ROS2 + Docker)
```bash
# Start Micro-ROS agent first
./venv/bin/nimbus agent start

# Then run Nimbus
./venv/bin/nimbus run --behavior wander
```

## Yahboom Robot Setup (In Progress)

### USB Driver Installation
- Installed CH340 driver: `brew install --cask wch-ch34x-usb-serial-driver`
- Installed CP210x driver: `brew install --cask silicon-labs-vcp-driver`

### Next Steps for Robot Connection
1. Ensure robot is powered ON
2. Connect via USB with data cable (not charge-only)
3. Configure WiFi credentials using robot configurator
4. Start Micro-ROS agent via Docker
5. Connect Nimbus to robot over network

## Docker Setup
- Docker Desktop installed and running

## Known Issues
- `/api/status` and `/api/sensors` endpoints have JSON serialization issues with infinity values
- ROS2 not installed locally (use mock mode or Docker for real robot)