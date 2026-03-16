# Jetson Nano 2GB Setup Guide for Nimbus

Complete setup instructions for preparing a Jetson Nano 2GB to run Nimbus on a Neato vacuum robot.

---

## Table of Contents

1. [Hardware Requirements](#hardware-requirements)
2. [Download and Flash JetPack](#download-and-flash-jetpack)
3. [Initial Boot and Setup](#initial-boot-and-setup)
4. [WiFi Configuration](#wifi-configuration)
5. [System Configuration](#system-configuration)
6. [USB Setup for Neato](#usb-setup-for-neato)
7. [Python Environment](#python-environment)
8. [Install Nimbus](#install-nimbus)
9. [Power Setup for Neato Installation](#power-setup-for-neato-installation)
10. [Testing](#testing)
11. [Troubleshooting](#troubleshooting)

---

## Hardware Requirements

### Essential
| Item | Notes |
|------|-------|
| Jetson Nano 2GB Developer Kit | The 2GB model (not 4GB) |
| MicroSD Card | 64GB+ recommended, UHS-1 or faster |
| USB WiFi Adapter | The 2GB model has no built-in WiFi |
| 5V 3A Power Supply | Barrel jack (5.5mm OD / 2.1mm ID) recommended |
| USB Keyboard | For initial setup |
| Monitor + HDMI Cable | For initial setup |
| MicroUSB Cable | Alternative for initial setup (headless mode) |

### For Neato Integration
| Item | Notes |
|------|-------|
| Micro USB to USB-A Cable | To connect Jetson USB port to Neato's debug USB port (behind dustbin) |
| 5V 5A Buck Converter | To power Jetson from Neato's 16V rail |
| JST Connectors | Optional, for clean wiring |

### Recommended USB WiFi Adapters
These are known to work well with the Jetson Nano:
- **Edimax EW-7811Un** (RTL8188CUS) — tiny, cheap, works out of box
- **TP-Link TL-WN725N** (RTL8188EUS) — compact, reliable
- **Panda PAU06** (RT5372) — good range, dual-band not needed

Avoid adapters requiring manual driver compilation if possible.

---

## Download and Flash JetPack

### Step 1: Download the Image

Go to NVIDIA's Jetson Download Center:
```
https://developer.nvidia.com/embedded/downloads
```

Download **JetPack 4.6.x** for Jetson Nano 2GB:
- Look for: "Jetson Nano 2GB Developer Kit SD Card Image"
- File will be named something like: `jetson-nano-2gb-jp46-sd-card-image.zip`

> **Note**: JetPack 4.6.x is recommended for stability. JetPack 5.x is NOT supported on Nano.

### Step 2: Flash the SD Card

#### Option A: Using balenaEtcher (Recommended)

1. Download [balenaEtcher](https://www.balena.io/etcher/)
2. Insert your microSD card
3. Open Etcher
4. Select the downloaded `.zip` file (no need to extract)
5. Select your SD card
6. Click "Flash!"
7. Wait ~10-15 minutes for completion

#### Option B: Using dd (Linux/Mac)

```bash
# Find your SD card device
lsblk

# Unmount if mounted
sudo umount /dev/sdX*

# Extract and flash (CAREFUL: verify device name!)
unzip -p jetson-nano-2gb-jp46-sd-card-image.zip | sudo dd of=/dev/sdX bs=4M status=progress

# Sync and eject
sync
sudo eject /dev/sdX
```

---

## Initial Boot and Setup

### Step 1: First Boot

1. Insert the flashed SD card into the Jetson Nano
2. Connect keyboard, monitor, and WiFi adapter
3. Connect power (5V barrel jack recommended)
4. The green LED will light up, and boot takes ~1-2 minutes

### Step 2: Initial Configuration Wizard

The first boot launches a setup wizard:

1. **Accept License Agreement**
2. **Select Language**: English (or your preference)
3. **Select Keyboard Layout**: US (or your preference)
4. **Select Time Zone**: Your timezone
5. **Create User Account**:
   ```
   Username: nimbus (or your preference)
   Password: [choose a strong password]
   Computer Name: jetson-neato (or your preference)
   ```
6. **APP Partition Size**: Use maximum (default)
7. **Select Nvpmodel Mode**: Choose "MAXN" for maximum performance
8. Wait for setup to complete (~5-10 minutes)

### Step 3: First Login

After reboot, log in with your created credentials. You'll land on the Ubuntu 18.04 desktop.

---

## WiFi Configuration

The Jetson Nano 2GB has no built-in WiFi — you must use a USB adapter.

### Step 1: Verify Adapter Detection

```bash
# Check if adapter is recognized
lsusb

# Look for your adapter, e.g.:
# Bus 001 Device 003: ID 0bda:8179 Realtek Semiconductor Corp. RTL8188EUS
```

### Step 2: Connect via GUI

1. Click the network icon in the top-right
2. Select your WiFi network
3. Enter password
4. Wait for connection

### Step 3: Connect via Command Line (Headless)

```bash
# List available networks
nmcli device wifi list

# Connect to network
nmcli device wifi connect "YOUR_SSID" password "YOUR_PASSWORD"

# Verify connection
nmcli connection show
ip addr show wlan0
```

### Step 4: Set Static IP (Recommended for Robot)

Before setting a static IP, you must know your actual network subnet. **Do not guess — using the wrong subnet will make the Jetson unreachable over SSH.**

First, check what IP the Jetson received from DHCP and note the subnet mask:

```bash
hostname -I
ip addr show wlan0
```

Also check your laptop's IP to confirm they match:

```bash
# On your laptop (Linux/Mac)
ip addr show
```

Both should be in the same subnet (e.g. both `192.168.68.x`, or both `192.168.1.x`). Once confirmed, pick a static address in the same range that's outside your router's DHCP pool, and note the correct prefix length from `ip addr` output (commonly `/24` but may differ — e.g. `/22`).

```bash
# Replace values to match YOUR network
# Example shown for a 192.168.68.x/22 network:
nmcli connection modify "YOUR_SSID" \
  ipv4.addresses "192.168.68.200/22" \
  ipv4.gateway "192.168.68.1" \
  ipv4.dns "8.8.8.8,8.8.4.4" \
  ipv4.method "manual"

# Restart connection
nmcli connection down "YOUR_SSID"
nmcli connection up "YOUR_SSID"

# Verify
ip addr show wlan0
hostname -I
```

> **Warning**: If SSH stops working after setting a static IP, the address or prefix length is wrong. Reconnect a monitor/keyboard, run `nmcli connection modify "YOUR_SSID" ipv4.method auto` to revert to DHCP, then recheck your subnet before retrying.

### Step 5: Enable SSH

```bash
# SSH should be enabled by default, verify:
sudo systemctl status ssh

# If not running:
sudo systemctl enable ssh
sudo systemctl start ssh

# Get IP address for remote access
hostname -I
```

Now you can disconnect the monitor/keyboard and work via SSH:
```bash
ssh YOUR_USERNAME@YOUR_JETSON_IP
```

---

## System Configuration

### Step 1: Update System Packages

```bash
sudo apt update
sudo apt upgrade -y
```

> **Note**: This may take 15-30 minutes. Don't interrupt.

### Step 2: Install Essential Tools

```bash
sudo apt install -y \
  python3-pip \
  python3-dev \
  python3-venv \
  git \
  htop \
  nano \
  curl \
  wget \
  screen \
  tmux \
  i2c-tools
```

### Step 3: Configure Swap (Important!)

The 2GB model needs swap for stability:

```bash
# Check current swap
free -h

# Create 4GB swap file
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make permanent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Verify
free -h
```

### Step 4: Optimize Performance

```bash
# Set to maximum performance mode
sudo nvpmodel -m 0

# Enable all CPU cores
sudo jetson_clocks

# Make jetson_clocks persistent (optional)
sudo cp /etc/rc.local /etc/rc.local.bak 2>/dev/null || true
echo '#!/bin/bash
sleep 10
/usr/bin/jetson_clocks
exit 0' | sudo tee /etc/rc.local
sudo chmod +x /etc/rc.local
```

### Step 5: Disable GUI (Optional but Recommended)

For a headless robot, disable the desktop to save ~500MB RAM:

```bash
# Disable GUI on boot
sudo systemctl set-default multi-user.target

# To re-enable later if needed:
# sudo systemctl set-default graphical.target

# Reboot to apply
sudo reboot
```

---

## USB Setup for Neato

The Neato D4 exposes a CDC ACM serial device over its micro USB port, located behind the dustbin. On Linux this requires no driver and enumerates automatically as `/dev/ttyACM0`.

### Step 1: Connect the Cable

Remove the Neato's dustbin to expose the micro USB port on the robot's body. Connect a micro USB to USB-A cable from that port to any USB-A port on the Jetson Nano.

> **Note**: The Jetson Nano 2GB has two USB-A ports. One is typically occupied by the WiFi adapter, leaving one for the Neato. If you need additional USB devices, use a powered USB hub.

### Step 2: Set USB Serial Permissions

```bash
# Add user to dialout group for serial access
sudo usermod -aG dialout $USER

# Log out and back in (or reboot) for the group change to take effect
```

### Step 3: Verify Device Enumeration

After plugging in the Neato (with the robot powered on):

```bash
# Check kernel detected the device
dmesg | grep ttyACM

# Verify device node exists
ls -la /dev/ttyACM*

# Should show:
# crw-rw---- 1 root dialout 166, 0 ... /dev/ttyACM0

# Test with Python
python3 -c "import serial; print(serial.Serial('/dev/ttyACM0', 115200, timeout=1))"
```

If another ACM device is already present, the Neato may enumerate as `/dev/ttyACM1`. Check `dmesg` output to confirm which node is the Neato.

---

## Python Environment

JetPack 4.6 ships with Python 3.6, and the deadsnakes PPA only provides up to Python 3.8 for Ubuntu 18.04 (bionic). Nimbus requires Python >=3.10, so Python 3.10 must be built from source. This takes approximately 10–15 minutes to compile on the Jetson Nano.

### Step 1: Install Build Dependencies

```bash
sudo apt install -y build-essential libssl-dev zlib1g-dev \
  libncurses5-dev libreadline-dev libsqlite3-dev \
  libgdbm-dev libbz2-dev libexpat1-dev liblzma-dev \
  libffi-dev uuid-dev
```

### Step 2: Build Python 3.10 from Source

```bash
cd /tmp
wget https://www.python.org/ftp/python/3.10.14/Python-3.10.14.tgz
tar xf Python-3.10.14.tgz
cd Python-3.10.14

# Configure (--enable-optimizations adds ~5 min but improves runtime performance)
./configure --enable-optimizations

# Build using all 4 CPU cores
make -j4

# Install alongside system Python — does NOT overwrite python3 symlink
sudo make altinstall
```

Verify the install:

```bash
python3.10 --version
# Should print: Python 3.10.14
```

### Step 3: Create Virtual Environment

```bash
mkdir -p ~/projects
cd ~/projects

python3.10 -m venv nimbus-env
source nimbus-env/bin/activate

pip install --upgrade pip wheel setuptools
```

### Step 4: Add Activation to Bashrc (Optional)

```bash
echo 'source ~/projects/nimbus-env/bin/activate' >> ~/.bashrc
```

---

## Install Nimbus

### Step 1: Clone Repository

```bash
cd ~/projects
git clone https://github.com/speedybits/nimbus.git
cd nimbus
```

### Step 2: Install Nimbus

```bash
# Ensure venv is active
source ~/projects/nimbus-env/bin/activate

# Install in development mode
pip install -e ".[dev]"

# Verify installation
nimbus version
```

### Step 3: Configure for Neato

Create/edit the configuration file:

```bash
mkdir -p ~/.nimbus
nano ~/.nimbus/config.yaml
```

Add Neato-specific configuration:

```yaml
# Nimbus Configuration for Neato + Jetson Nano

transport:
  type: neato
  neato:
    port: /dev/ttyACM0
    baudrate: 115200
    wheel_base_mm: 248
    max_speed_mms: 300

sensors:
  safety_radius: 0.30

navigation:
  max_linear_speed: 0.25
  max_angular_speed: 1.0
  emergency_distance: 0.15
  caution_distance: 0.40

api:
  rest_port: 8080
  websocket_enabled: true
```

---

## Power Setup for Neato Installation

### Option 1: External Power (Development)

For testing before permanent installation:
- Power Jetson via 5V barrel jack from wall adapter
- Connect USB cable to Neato (Neato powered separately)

### Option 2: Neato Battery Power (Production)

For permanent installation inside the Neato:

#### Components Needed
- 5V 5A Buck Converter (e.g., Pololu D24V50F5)
- Wire, connectors, heat shrink

#### Wiring Diagram

```
Neato 16V Rail ─────┬───────────────────────┐
                    │                       │
               [Buck Converter]             │
                IN+   IN-                   │
                 │     │                    │
                 │     └──── GND ───────────┤
                 │                          │
                OUT+  OUT-                  │
                 │     │                    │
                 │     └──── GND ──────┐    │
                 │                     │    │
                5V ────────────────────┼────┼───► Jetson 5V (Pin 2 or 4)
                                       │    │
                                      GND ──┴───► Jetson GND (Pin 6)
```

#### Step-by-Step

1. **Locate Neato's 16V connector** on the motherboard
2. **Connect buck converter input** to 16V and GND
3. **Verify output is 5V** with multimeter before connecting Jetson!
4. **Connect to Jetson GPIO header**:
   - Pin 2 or 4 (5V)
   - Pin 6 (GND)
5. **Secure all connections** with heat shrink and hot glue

> **Warning**: Double-check polarity and voltage before powering the Jetson. Incorrect voltage will destroy the board.

---

## Testing

### Test 1: Serial Communication

```bash
# Activate environment
source ~/projects/nimbus-env/bin/activate

# Test serial connection
python3 << 'EOF'
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
ser.write(b'TestMode On\n')
time.sleep(0.1)
response = ser.read(1000).decode('utf-8', errors='ignore')
print("TestMode response:", response)

# Try GetVersion
ser.write(b'GetVersion\n')
time.sleep(0.5)
response = ser.read(2000).decode('utf-8', errors='ignore')
print("GetVersion response:", response[:500])

ser.close()
EOF
```

### Test 2: LIDAR Scan

```bash
python3 << 'EOF'
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
time.sleep(0.5)

# Enable test mode and LIDAR
ser.write(b'TestMode On\n')
time.sleep(0.1)
ser.read(1000)

ser.write(b'SetLDSRotation On\n')
time.sleep(2)  # Wait for LIDAR to spin up
ser.read(1000)

# Get scan
ser.write(b'GetLDSScan\n')
time.sleep(0.5)
response = ser.read(10000).decode('utf-8', errors='ignore')

# Count valid readings
lines = response.strip().split('\n')
valid = sum(1 for l in lines if ',' in l and not l.startswith('ROTATION'))
print(f"Got {len(lines)} lines, {valid} valid readings")
print("First 10 lines:")
for line in lines[:10]:
    print(f"  {line}")

# Cleanup
ser.write(b'SetLDSRotation Off\n')
time.sleep(0.1)
ser.write(b'TestMode Off\n')
ser.close()
EOF
```

### Test 3: Motor Control

```bash
python3 << 'EOF'
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
time.sleep(0.5)

# Enable test mode
ser.write(b'TestMode On\n')
time.sleep(0.1)
ser.read(1000)

# Move forward 50mm at 50mm/s
print("Moving forward 50mm...")
ser.write(b'SetMotor LWheelDist 50 RWheelDist 50 Speed 50\n')
time.sleep(2)

# Move backward 50mm
print("Moving backward 50mm...")
ser.write(b'SetMotor LWheelDist -50 RWheelDist -50 Speed 50\n')
time.sleep(2)

# Cleanup
ser.write(b'TestMode Off\n')
ser.close()
print("Done!")
EOF
```

### Test 4: Run Nimbus

```bash
# Start Nimbus with Neato transport
nimbus run --transport neato

# Or in mock mode first to verify CLI works
nimbus run --mock
```

---

## Troubleshooting

### Serial Port Access Denied

```bash
# Error: Permission denied: '/dev/ttyACM0'

# Fix: Add user to dialout group
sudo usermod -aG dialout $USER

# Must log out and back in (or reboot)
```

### No Response from Neato

1. **Check USB cable**: Try a different micro USB cable (some are charge-only with no data lines)
2. **Check device enumeration**: Run `dmesg | grep ttyACM` after plugging in — you should see a CDC ACM device appear
3. **Check baud rate**: Must be 115200
4. **Check Neato power**: Robot must be powered on
5. **Check dustbin**: The USB port is behind the dustbin — confirm it's fully removed and the cable is seated

```bash
# On laptop, verify Neato responds:
screen /dev/ttyACM0 115200
# Type: Help
# Should see command list
# Exit screen: Ctrl+A then K
```

### WiFi Adapter Not Working

```bash
# Check if detected
lsusb
dmesg | tail -20

# Check for driver issues
lsmod | grep -i rtl
lsmod | grep -i 80211

# Try different USB port
# Some adapters need powered hub
```

### Out of Memory Errors

```bash
# Check memory
free -h

# Ensure swap is enabled
swapon --show

# Kill memory hogs
htop
# Sort by memory (press M), kill unused processes

# Disable GUI if not already
sudo systemctl set-default multi-user.target
sudo reboot
```

### Jetson Won't Boot

1. Re-flash SD card
2. Try different SD card (some have compatibility issues)
3. Verify power supply is 5V 3A minimum
4. Check barrel jack connection (should feel solid)

### Python Version / Nimbus Install Errors

JetPack 4.6 ships with Python 3.6. Nimbus requires >=3.10. If you see errors like:

```
Package 'nimbus' requires a different Python: 3.6.x not in '>=3.10'
```
or:
```
Could not find a version that satisfies the requirement setuptools>=65.0
```

This means the venv was created with the wrong Python. The deadsnakes PPA only provides up to Python 3.8 for Ubuntu 18.04 — Python 3.10 must be built from source. Follow the Python Environment section above.

If you have an old venv, remove it first before rebuilding:

```bash
deactivate   # if currently inside the venv
rm -rf ~/projects/nimbus-env
python3.10 -m venv ~/projects/nimbus-env
source ~/projects/nimbus-env/bin/activate
pip install --upgrade pip wheel setuptools
```

### LIDAR Not Spinning

```bash
# Ensure test mode is on
TestMode On

# Enable rotation
SetLDSRotation On

# Wait 2-3 seconds for spin-up
# Check rotation speed in GetLDSScan response

# If still not working, LIDAR motor may be damaged
```

---

## Quick Reference

### Common Commands

```bash
# Activate environment
source ~/projects/nimbus-env/bin/activate

# Start Nimbus
nimbus run --transport neato

# SSH to Jetson
ssh YOUR_USERNAME@YOUR_JETSON_IP

# Serial terminal to Neato (for debugging)
screen /dev/ttyACM0 115200
# Exit screen: Ctrl+A, then K, then Y

# Check system resources
htop
free -h
df -h

# Check Jetson power mode
sudo nvpmodel -q

# View logs
journalctl -f
```

### Important File Locations

| Path | Description |
|------|-------------|
| `/dev/ttyACM0` | USB serial to Neato |
| `~/.nimbus/config.yaml` | Nimbus configuration |
| `~/projects/nimbus/` | Nimbus source code |
| `~/projects/nimbus-env/` | Python virtual environment |
| `/boot/extlinux/extlinux.conf` | Boot configuration |

### Network

```bash
# Show IP address
hostname -I

# Restart networking
sudo systemctl restart NetworkManager

# Scan for WiFi networks
nmcli device wifi list
```

---

## Next Steps

1. Complete Neato hardware integration (power wiring)
2. Mount Jetson inside Neato chassis
3. Implement Neato transport adapter for Nimbus
4. Test navigation behaviors
5. Set up auto-start on boot:

```bash
# Create systemd service
sudo nano /etc/systemd/system/nimbus.service
```

```ini
[Unit]
Description=Nimbus Robot Controller
After=network.target

[Service]
Type=simple
User=nimbus
WorkingDirectory=/home/nimbus/projects/nimbus
Environment="PATH=/home/nimbus/projects/nimbus-env/bin"
ExecStart=/home/nimbus/projects/nimbus-env/bin/nimbus run --transport neato
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
# Enable service
sudo systemctl enable nimbus
sudo systemctl start nimbus
sudo systemctl status nimbus
```

---

*Last updated: February 2026*
