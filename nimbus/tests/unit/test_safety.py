"""
Unit tests for safety controller.
"""

import pytest
from nimbus.navigation.safety import SafetyController, SafetyConfig, SafetyLevel


class TestSafetyController:
    """Tests for safety controller."""

    def test_emergency_stop_when_close(self, safety_controller):
        """Should trigger emergency stop when obstacle very close."""
        linear, angular = safety_controller.limit_velocity(0.3, 0.0, 0.10)

        assert linear == 0.0  # Should stop forward motion
        assert safety_controller.is_emergency

    def test_speed_reduction_in_caution_zone(self, safety_controller):
        """Should reduce speed when in caution zone."""
        linear, angular = safety_controller.limit_velocity(0.3, 0.0, 0.30)

        assert 0.0 < linear < 0.3  # Should be reduced but not stopped

    def test_full_speed_when_clear(self, safety_controller):
        """Should allow full speed when path is clear."""
        linear, angular = safety_controller.limit_velocity(0.3, 0.0, 2.0)

        assert linear == 0.3  # No reduction

    def test_respects_maximum_speeds(self, safety_controller):
        """Should clamp to maximum speeds."""
        linear, angular = safety_controller.limit_velocity(1.0, 5.0, 2.0)

        assert linear <= safety_controller.config.max_linear_speed
        assert angular <= safety_controller.config.max_angular_speed

    def test_evaluate_emergency(self, safety_controller):
        """Should evaluate emergency level correctly."""
        level = safety_controller.evaluate(0.10)
        assert level == SafetyLevel.EMERGENCY

    def test_evaluate_caution(self, safety_controller):
        """Should evaluate caution level correctly."""
        level = safety_controller.evaluate(0.30)
        assert level == SafetyLevel.CAUTION

    def test_evaluate_normal(self, safety_controller):
        """Should evaluate normal level correctly."""
        level = safety_controller.evaluate(2.0)
        assert level == SafetyLevel.NORMAL

    def test_allows_rotation_in_emergency(self, safety_controller):
        """Should allow rotation even in emergency (to find clear path)."""
        linear, angular = safety_controller.limit_velocity(0.3, 0.5, 0.10)

        assert linear == 0.0  # No forward
        # Angular should be allowed (clamped to max)

    def test_allows_reverse_in_emergency(self, safety_controller):
        """Should allow slow reverse in emergency to back away."""
        linear, angular = safety_controller.limit_velocity(-0.1, 0.0, 0.10)

        # Reverse should be allowed (may be limited)
        assert linear <= 0.0

    def test_force_stop(self, safety_controller):
        """Force stop should return zero velocities."""
        linear, angular = safety_controller.force_stop()

        assert linear == 0.0
        assert angular == 0.0

    def test_status_dict(self, safety_controller):
        """Status should return proper dict."""
        safety_controller.evaluate(2.0)
        status = safety_controller.status

        assert "level" in status
        assert "is_emergency" in status
        assert "can_move_forward" in status
