"""Tests for IR-based dock detection via LIDAR intensity."""

import pytest
from nimbus.sensors.dock_detector import DockDetector, DockDetectorConfig, DockDetection


def _make_scan(
    base_range_m: float = 2.0,
    base_intensity: float = 600.0,
    peak_angle: int | None = None,
    peak_width: int = 8,
    peak_intensity: float = 4500.0,
    peak_range_m: float | None = None,
):
    """Create synthetic LIDAR scan data with optional IR intensity peak.

    Default values calibrated from real Neato D4 BotVac data:
    - base_intensity ~600 (typical room surfaces)
    - peak_intensity ~4500 (charger IR emitter at close range)

    Args:
        base_range_m: Default range for all angles
        base_intensity: Default intensity for all angles
        peak_angle: Center angle of IR peak (None = no peak)
        peak_width: Angular width of peak in degrees
        peak_intensity: Max intensity at peak center
        peak_range_m: Range at peak angles (defaults to base_range_m)
    """
    ranges = [base_range_m] * 360
    intensities = [base_intensity] * 360

    if peak_angle is not None:
        if peak_range_m is None:
            peak_range_m = base_range_m
        half = peak_width // 2
        for offset in range(-half, half + 1):
            angle = (peak_angle + offset) % 360
            # Intensity tapers from center
            taper = 1.0 - abs(offset) / (half + 1)
            intensities[angle] = base_intensity + (peak_intensity - base_intensity) * taper
            ranges[angle] = peak_range_m

    return tuple(ranges), tuple(intensities)


class TestDockDetector:
    """Test IR dock detection from LIDAR intensity data."""

    def setup_method(self):
        self.detector = DockDetector(DockDetectorConfig(
            intensity_threshold=400.0,
            intensity_ratio=3.5,
            min_peak_width_deg=2,
            max_peak_width_deg=30,
        ))

    def test_no_peak_returns_none(self):
        """Uniform intensity should produce no detection."""
        ranges, intensities = _make_scan()
        result = self.detector.detect(ranges, intensities)
        assert result is None

    def test_detect_peak_at_zero(self):
        """IR peak directly ahead (0°)."""
        ranges, intensities = _make_scan(peak_angle=0)
        result = self.detector.detect(ranges, intensities)
        assert result is not None
        assert result.detected is True
        assert abs(result.angle_deg - 0) <= 2
        assert result.confidence > 0.0

    def test_detect_peak_at_90(self):
        """IR peak to the left (90°)."""
        ranges, intensities = _make_scan(peak_angle=90)
        result = self.detector.detect(ranges, intensities)
        assert result is not None
        assert result.detected is True
        assert abs(result.angle_deg - 90) <= 2

    def test_detect_peak_at_180(self):
        """IR peak behind (180°)."""
        ranges, intensities = _make_scan(peak_angle=180)
        result = self.detector.detect(ranges, intensities)
        assert result is not None
        assert abs(result.angle_deg - 180) <= 2

    def test_detect_peak_at_270(self):
        """IR peak to the right (270°)."""
        ranges, intensities = _make_scan(peak_angle=270)
        result = self.detector.detect(ranges, intensities)
        assert result is not None
        assert abs(result.angle_deg - 270) <= 2

    def test_wraparound_peak(self):
        """IR peak spanning 358-2° (across zero boundary)."""
        ranges, intensities = _make_scan(peak_angle=0, peak_width=6)
        result = self.detector.detect(ranges, intensities)
        assert result is not None
        assert result.detected is True
        # Should be near 0/360
        assert result.angle_deg <= 5 or result.angle_deg >= 355

    def test_distance_reported_correctly(self):
        """Detection should report the range at the peak angle."""
        ranges, intensities = _make_scan(
            peak_angle=45, peak_range_m=1.5
        )
        result = self.detector.detect(ranges, intensities)
        assert result is not None
        assert abs(result.distance_m - 1.5) < 0.1

    def test_peak_too_wide_rejected(self):
        """Very wide intensity bump (like a bright wall) should be rejected."""
        detector = DockDetector(DockDetectorConfig(max_peak_width_deg=20))
        ranges, intensities = _make_scan(
            peak_angle=90, peak_width=40
        )
        result = detector.detect(ranges, intensities)
        assert result is None

    def test_peak_too_narrow_rejected(self):
        """Single-angle spike should be rejected (noise)."""
        detector = DockDetector(DockDetectorConfig(min_peak_width_deg=3))
        ranges = [2.0] * 360
        intensities = [600.0] * 360
        intensities[100] = 5000.0  # single spike
        result = detector.detect(tuple(ranges), tuple(intensities))
        assert result is None

    def test_peak_beyond_max_range_ignored(self):
        """IR signal at very long range should be ignored."""
        ranges, intensities = _make_scan(
            peak_angle=45, peak_range_m=5.0
        )
        result = self.detector.detect(ranges, intensities)
        assert result is None

    def test_low_intensity_peak_ignored(self):
        """Peak below absolute threshold should be ignored."""
        ranges, intensities = _make_scan(
            base_intensity=80.0, peak_angle=90,
            peak_intensity=200.0,  # 2.5× ratio but below absolute threshold of 400
        )
        result = self.detector.detect(ranges, intensities)
        assert result is None

    def test_intensity_peak_field(self):
        """The detection should report the max intensity value."""
        ranges, intensities = _make_scan(peak_angle=0, peak_intensity=4500.0)
        result = self.detector.detect(ranges, intensities)
        assert result is not None
        assert result.intensity_peak >= 4000.0  # allowing for taper

    def test_empty_scan_returns_none(self):
        """All-inf ranges should return None (not enough valid readings)."""
        ranges = tuple([float('inf')] * 360)
        intensities = tuple([0.0] * 360)
        result = self.detector.detect(ranges, intensities)
        assert result is None

    def test_wrong_size_returns_none(self):
        """Non-360 arrays should return None."""
        result = self.detector.detect(tuple([1.0] * 100), tuple([50.0] * 100))
        assert result is None

    def test_higher_intensity_gives_higher_confidence(self):
        """Stronger IR signal should produce higher confidence."""
        ranges1, intensities1 = _make_scan(peak_angle=0, peak_intensity=2500.0)
        ranges2, intensities2 = _make_scan(peak_angle=0, peak_intensity=4500.0)
        result1 = self.detector.detect(ranges1, intensities1)
        result2 = self.detector.detect(ranges2, intensities2)
        assert result1 is not None and result2 is not None
        assert result2.confidence >= result1.confidence
