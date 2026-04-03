#!/bin/bash
# Jetson Multi-Network Setup Script
# Run this ON the Jetson: ssh -t mike@192.168.68.200 "bash -s" < scripts/jetson_network_setup.sh
#
# Sets up:
# 1. shwashwa WiFi as priority connection
# 2. MK_iphone hotspot as fallback
# 3. Verifies Avahi/mDNS is working

set -e

echo "=== Jetson Multi-Network Setup ==="
echo ""

# 1. Set shwashwa priority
echo "[1/4] Setting shwashwa as priority WiFi (100)..."
sudo nmcli connection modify 'shwashwa' connection.autoconnect yes connection.autoconnect-priority 100
echo "  Done."

# 2. Add iPhone hotspot
echo ""
echo "[2/4] Adding MK_iphone hotspot profile..."

if nmcli connection show 'MK_iphone' &>/dev/null; then
    echo "  MK_iphone profile already exists, updating priority..."
    sudo nmcli connection modify 'MK_iphone' connection.autoconnect yes connection.autoconnect-priority 0
else
    read -sp "  Enter MK_iphone hotspot password: " HOTSPOT_PW
    echo ""
    sudo nmcli connection add type wifi ifname wlan0 con-name 'MK_iphone' ssid 'MK_iphone' \
        wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$HOTSPOT_PW" \
        connection.autoconnect yes connection.autoconnect-priority 0
fi
echo "  Done."

# 3. Verify Avahi
echo ""
echo "[3/4] Checking Avahi/mDNS..."
if systemctl is-active --quiet avahi-daemon; then
    echo "  Avahi is running. Hostname: $(hostname)"
else
    echo "  Avahi not running, starting..."
    sudo systemctl enable avahi-daemon
    sudo systemctl start avahi-daemon
    echo "  Started. Hostname: $(hostname)"
fi

# 4. Show results
echo ""
echo "[4/4] Network profiles:"
nmcli -f NAME,AUTOCONNECT,AUTOCONNECT-PRIORITY connection show
echo ""
echo "=== Setup complete ==="
echo "From your laptop, test with: ping mike-jetson.local"
echo "Then SSH with: ssh mike@mike-jetson.local"
