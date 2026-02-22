"""Tests for Neato differential drive kinematics."""

import math
import pytest
from nimbus.core.neato.kinematics import (
    WHEEL_BASE_M,
    MAX_SPEED_MMS,
    velocity_to_wheel_speeds,
    wheel_speeds_to_motor_command,
    update_odometry,
)


class TestVelocityToWheelSpeeds:
    """Test unicycle-to-differential conversion."""

    def test_pure_forward(self):
        left, right = velocity_to_wheel_speeds(0.2, 0.0)
        assert left == pytest.approx(0.2)
        assert right == pytest.approx(0.2)

    def test_pure_rotation(self):
        left, right = velocity_to_wheel_speeds(0.0, 1.0)
        half_base = WHEEL_BASE_M / 2.0
        assert left == pytest.approx(-half_base)
        assert right == pytest.approx(half_base)

    def test_arc_turn(self):
        left, right = velocity_to_wheel_speeds(0.2, 0.5)
        assert left < right  # turning left: right wheel faster
        assert left == pytest.approx(0.2 - 0.5 * WHEEL_BASE_M / 2)
        assert right == pytest.approx(0.2 + 0.5 * WHEEL_BASE_M / 2)

    def test_zero_velocity(self):
        left, right = velocity_to_wheel_speeds(0.0, 0.0)
        assert left == 0.0
        assert right == 0.0

    def test_backward(self):
        left, right = velocity_to_wheel_speeds(-0.15, 0.0)
        assert left == pytest.approx(-0.15)
        assert right == pytest.approx(-0.15)


class TestWheelSpeedsToMotorCommand:
    """Test conversion to SetMotor parameters."""

    def test_forward_200ms(self):
        left_mm, right_mm, speed = wheel_speeds_to_motor_command(0.1, 0.1, 0.2)
        assert left_mm == 20   # 100mm/s * 0.2s = 20mm
        assert right_mm == 20
        assert speed == 100

    def test_zero_speed(self):
        left_mm, right_mm, speed = wheel_speeds_to_motor_command(0.0, 0.0, 0.2)
        assert left_mm == 0
        assert right_mm == 0
        assert speed == 0

    def test_clamping(self):
        # 0.5 m/s = 500 mm/s > MAX_SPEED_MMS (300)
        left_mm, right_mm, speed = wheel_speeds_to_motor_command(0.5, 0.5, 0.2)
        assert speed == MAX_SPEED_MMS
        assert left_mm == 60   # 300mm/s * 0.2s = 60mm
        assert right_mm == 60

    def test_clamping_preserves_ratio(self):
        # Left=400mm/s, Right=200mm/s -> scale by 300/400=0.75
        left_mm, right_mm, speed = wheel_speeds_to_motor_command(0.4, 0.2, 0.2)
        assert speed == MAX_SPEED_MMS
        # Left: 400*0.75=300mm/s * 0.2s = 60mm
        # Right: 200*0.75=150mm/s * 0.2s = 30mm
        assert left_mm == 60
        assert right_mm == 30

    def test_backward_command(self):
        left_mm, right_mm, speed = wheel_speeds_to_motor_command(-0.1, -0.1, 0.2)
        assert left_mm == -20
        assert right_mm == -20
        assert speed == 100


class TestUpdateOdometry:
    """Test midpoint integration odometry."""

    def test_straight_forward(self):
        x, y, theta = update_odometry(0.0, 0.0, 0.0, 0.1, 0.1)
        assert x == pytest.approx(0.1)
        assert y == pytest.approx(0.0, abs=1e-10)
        assert theta == pytest.approx(0.0, abs=1e-10)

    def test_pure_rotation_left(self):
        # Equal and opposite wheel deltas -> pure rotation
        delta = WHEEL_BASE_M * math.pi / 4  # quarter turn worth
        x, y, theta = update_odometry(0.0, 0.0, 0.0, -delta / 2, delta / 2)
        assert x == pytest.approx(0.0, abs=1e-10)
        assert y == pytest.approx(0.0, abs=1e-10)
        assert theta > 0  # turned left (positive angular)

    def test_theta_normalization(self):
        # Start near pi, push past it
        x, y, theta = update_odometry(0.0, 0.0, math.pi - 0.01, -0.01, 0.01)
        assert -math.pi <= theta <= math.pi

    def test_accumulation(self):
        # Drive forward in 10 small steps
        x, y, theta = 0.0, 0.0, 0.0
        for _ in range(10):
            x, y, theta = update_odometry(x, y, theta, 0.01, 0.01)
        assert x == pytest.approx(0.1, abs=1e-6)
        assert y == pytest.approx(0.0, abs=1e-6)

    def test_circle_returns_near_origin(self):
        # Drive in a circle: left wheel stationary, right wheel moves
        # This should curve to the left. After enough steps, approximate a loop.
        x, y, theta = 0.0, 0.0, 0.0
        steps = 1000
        # Full circle: total delta_theta = 2*pi
        # delta_theta per step = (right - left) / WHEEL_BASE_M
        # With left=0, need right = 2*pi*WHEEL_BASE_M / steps per step
        right_per_step = 2.0 * math.pi * WHEEL_BASE_M / steps
        for _ in range(steps):
            x, y, theta = update_odometry(x, y, theta, 0.0, right_per_step)
        # Should be approximately back near origin
        assert abs(x) < 0.05
        assert abs(y) < 0.05
        assert theta == pytest.approx(0.0, abs=0.05)
