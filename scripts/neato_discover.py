#!/usr/bin/env python3
"""
Neato serial protocol discovery tool.

Connects to the Neato, lists all available commands (Help),
dumps full GetCharger response, and logs LIDAR intensity data
to help calibrate IR dock detection.

Usage:
    source ~/projects/nimbus-env/bin/activate
    python scripts/neato_discover.py [--port /dev/ttyACM0] [--intensity-log]
"""

import argparse
import sys
import time

sys.path.insert(0, ".")
from nimbus.core.neato.transport import NeatoTransport


def main():
    parser = argparse.ArgumentParser(description="Neato serial protocol discovery")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument(
        "--intensity-log", action="store_true",
        help="Log LIDAR intensity data for IR dock calibration"
    )
    parser.add_argument(
        "--num-scans", type=int, default=5,
        help="Number of scans to collect for intensity logging"
    )
    args = parser.parse_args()

    transport = NeatoTransport(port=args.port)

    print(f"Connecting to Neato on {args.port}...")
    transport.connect()
    print("Connected.\n")

    # 1. List all available commands
    print("=" * 60)
    print("AVAILABLE COMMANDS (Help)")
    print("=" * 60)
    try:
        lines = transport.raw_command("Help")
        for line in lines:
            print(f"  {line}")
    except Exception as e:
        print(f"  Error: {e}")

    # 2. Full GetCharger response
    print("\n" + "=" * 60)
    print("FULL GetCharger RESPONSE")
    print("=" * 60)
    try:
        lines = transport.raw_command("GetCharger")
        for line in lines:
            print(f"  {line}")
        print()
        # Also parse via normal method to show structured data
        battery = transport.get_charger()
        print(f"  Parsed: percent={battery.battery_percent}%, "
              f"charging={battery.charging}, voltage={battery.voltage_mv}mV, "
              f"ext_power={battery.ext_power_present}, "
              f"over_temp={battery.battery_over_temp}")
        print(f"  Raw fields: {battery.raw}")
    except Exception as e:
        print(f"  Error: {e}")

    # 3. Check for dock-related commands
    print("\n" + "=" * 60)
    print("PROBING DOCK-RELATED COMMANDS")
    print("=" * 60)
    probe_commands = [
        "GetVersion",
        "GetAccel",
        "GetButtons",
        "GetCalInfo",
    ]
    for cmd in probe_commands:
        print(f"\n  --- {cmd} ---")
        try:
            lines = transport.raw_command(cmd)
            for line in lines:
                print(f"    {line}")
        except Exception as e:
            print(f"    Error: {e}")

    # 4. LIDAR intensity logging for IR dock calibration
    if args.intensity_log:
        print("\n" + "=" * 60)
        print(f"LIDAR INTENSITY LOG ({args.num_scans} scans)")
        print("=" * 60)
        print("Position the robot so the charger is visible, then compare")
        print("with a scan where the charger is NOT visible.\n")

        for scan_num in range(args.num_scans):
            scan = transport.get_lds_scan()
            # Find intensity peaks
            if scan.intensities:
                valid = [
                    (angle, dist, intensity)
                    for angle, dist, intensity, err
                    in zip(scan.angles, scan.distances_mm, scan.intensities, scan.error_codes)
                    if err == 0 and dist > 0
                ]
                if valid:
                    intensities_only = [v[2] for v in valid]
                    avg_intensity = sum(intensities_only) / len(intensities_only)
                    max_intensity = max(intensities_only)
                    max_idx = intensities_only.index(max_intensity)
                    max_angle = valid[max_idx][0]
                    max_dist = valid[max_idx][1]

                    # Find all readings above 2x average
                    peaks = [
                        (a, d, i) for a, d, i in valid
                        if i > avg_intensity * 2
                    ]

                    print(f"  Scan {scan_num + 1}:")
                    print(f"    Valid readings: {len(valid)}")
                    print(f"    Avg intensity: {avg_intensity:.1f}")
                    print(f"    Max intensity: {max_intensity} at {max_angle}° ({max_dist}mm)")
                    if peaks:
                        print(f"    Peaks (>2x avg):")
                        for a, d, i in sorted(peaks, key=lambda x: -x[2])[:10]:
                            print(f"      {a:3d}°  dist={d:5d}mm  intensity={i}")
                    else:
                        print(f"    No peaks above 2x average")
                    print()

            time.sleep(0.5)

    transport.disconnect()
    print("\nDone.")


if __name__ == "__main__":
    main()
