"""
Neato differential drive kinematics.

Pure functions for converting between velocity commands, wheel speeds,
and odometry updates. Zero dependencies beyond math.

Neato XV-series specs:
- Wheel base: 248mm (center-to-center)
- Max wheel speed: 300mm/s
- SetMotor command: distance in mm + speed in mm/s
"""

import math

# Neato XV-series physical constants
WHEEL_BASE_MM = 248.0
WHEEL_BASE_M = WHEEL_BASE_MM / 1000.0
MAX_SPEED_MMS = 300


def velocity_to_wheel_speeds(linear_mps: float, angular_rps: float) -> tuple[float, float]:
    """
    Convert unicycle velocity command to differential wheel speeds.

    Args:
        linear_mps: Forward velocity in meters/second
        angular_rps: Rotational velocity in radians/second

    Returns:
        (left_mps, right_mps) wheel speeds in meters/second
    """
    half_base = WHEEL_BASE_M / 2.0
    left_mps = linear_mps - angular_rps * half_base
    right_mps = linear_mps + angular_rps * half_base
    return left_mps, right_mps


def wheel_speeds_to_motor_command(
    left_mps: float,
    right_mps: float,
    duration_s: float = 0.2,
) -> tuple[int, int, int]:
    """
    Convert wheel speeds to Neato SetMotor parameters.

    The Neato SetMotor command takes:
    - LWheelDist: distance for left wheel in mm
    - RWheelDist: distance for right wheel in mm
    - Speed: max speed in mm/s (applied to both wheels)

    We compute distance = speed * duration, then clamp speed to MAX_SPEED_MMS.

    Args:
        left_mps: Left wheel speed in m/s
        right_mps: Right wheel speed in m/s
        duration_s: Duration of the command in seconds

    Returns:
        (left_mm, right_mm, speed_mms) integers for SetMotor command
    """
    left_mms = left_mps * 1000.0
    right_mms = right_mps * 1000.0

    # Clamp to max speed, preserving ratio
    max_abs = max(abs(left_mms), abs(right_mms))
    if max_abs > MAX_SPEED_MMS:
        scale = MAX_SPEED_MMS / max_abs
        left_mms *= scale
        right_mms *= scale
        max_abs = MAX_SPEED_MMS

    left_mm = int(round(left_mms * duration_s))
    right_mm = int(round(right_mms * duration_s))
    speed_mms = int(round(max_abs))

    return left_mm, right_mm, speed_mms


def update_odometry(
    x: float,
    y: float,
    theta: float,
    delta_left_m: float,
    delta_right_m: float,
) -> tuple[float, float, float]:
    """
    Update pose using midpoint integration from wheel encoder deltas.

    Args:
        x, y: Current position in meters
        theta: Current heading in radians
        delta_left_m: Left wheel displacement in meters
        delta_right_m: Right wheel displacement in meters

    Returns:
        (x, y, theta) updated pose, theta normalized to [-pi, pi]
    """
    delta_center = (delta_left_m + delta_right_m) / 2.0
    delta_theta = (delta_right_m - delta_left_m) / WHEEL_BASE_M

    # Midpoint integration: use heading at midpoint of arc
    mid_theta = theta + delta_theta / 2.0
    x += delta_center * math.cos(mid_theta)
    y += delta_center * math.sin(mid_theta)
    theta += delta_theta

    # Normalize theta to [-pi, pi]
    theta = math.atan2(math.sin(theta), math.cos(theta))

    return x, y, theta
