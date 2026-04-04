"""
IR-based dock detection using LIDAR intensity data.

The Neato charging base emits an IR signal that the LIDAR picks up
as anomalously high intensity values at specific angles. This module
detects those intensity spikes to determine the bearing and distance
to the charging dock.

Neato LIDAR convention: index 0 = forward, increases CCW.
"""

from dataclasses import dataclass
from typing import Optional
import logging
import math

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class DockDetection:
    """Result of dock detection from a single LIDAR scan."""
    detected: bool
    confidence: float       # 0.0–1.0
    angle_deg: int          # angle to dock IR source (0=front)
    distance_m: float       # LIDAR distance at peak angle
    intensity_peak: float   # peak intensity value


@dataclass(frozen=True)
class DockDetectorConfig:
    """Tunable parameters for IR dock detection.

    Calibrated from real Neato D4 BotVac data (2026-04-03):
    - Baseline intensity: ~610 (varies with surface reflectivity)
    - Charger IR peak: ~4500 (7.4× baseline)
    - Peak width: ~8° at 163mm range
    - Secondary reflections visible at ~20-30° offset from main peak
    """
    intensity_threshold: float = 1500.0  # min absolute intensity for IR signal
    intensity_ratio: float = 3.0         # peak must be N× median intensity
    min_peak_width_deg: int = 2          # minimum angular width of IR peak
    max_peak_width_deg: int = 30         # maximum (reject wide reflections)
    min_valid_readings: int = 60         # need enough data to compute median
    max_detection_range_m: float = 3.0   # ignore peaks beyond this range


class DockDetector:
    """Detect Neato charging dock from LIDAR intensity spikes.

    The charging base IR emitter creates a directional beam that the
    LIDAR sensor picks up as high intensity readings at the bearing
    of the charger.

    Usage:
        detector = DockDetector()
        result = detector.detect(ranges, intensities)
        if result and result.detected:
            print(f"Dock at {result.angle_deg}°, {result.distance_m:.2f}m")
    """

    def __init__(self, config: Optional[DockDetectorConfig] = None):
        self.config = config or DockDetectorConfig()

    def detect(
        self,
        ranges: tuple[float, ...],
        intensities: tuple[float, ...],
    ) -> Optional[DockDetection]:
        """Find dock bearing from LIDAR intensity spikes.

        Args:
            ranges: 360 range values (meters), inf = no reading
            intensities: 360 intensity values (aligned to ranges)

        Returns:
            DockDetection if a likely dock signal is found, None otherwise
        """
        if len(ranges) != 360 or len(intensities) != 360:
            return None

        # Collect valid readings (where we have both range and intensity)
        valid = []
        for i in range(360):
            if (ranges[i] != float('inf')
                    and ranges[i] > 0
                    and intensities[i] > 0):
                valid.append((i, ranges[i], intensities[i]))

        if len(valid) < self.config.min_valid_readings:
            return None

        # Compute median intensity as baseline
        valid_intensities = sorted(v[2] for v in valid)
        median_intensity = valid_intensities[len(valid_intensities) // 2]

        if median_intensity <= 0:
            return None

        # Find angles that exceed both thresholds
        threshold = max(
            self.config.intensity_threshold,
            median_intensity * self.config.intensity_ratio,
        )

        hot_angles = set()
        for angle, dist, intensity in valid:
            if (intensity >= threshold
                    and dist <= self.config.max_detection_range_m):
                hot_angles.add(angle)

        if not hot_angles:
            return None

        # Cluster adjacent hot angles into peaks (handling 360→0 wraparound)
        peaks = self._cluster_angles(hot_angles)

        if not peaks:
            return None

        # Score each peak and pick the best
        best_detection = None
        best_score = 0.0

        for peak_angles in peaks:
            width = len(peak_angles)
            if (width < self.config.min_peak_width_deg
                    or width > self.config.max_peak_width_deg):
                continue

            # Find the angle with highest intensity within the peak
            max_intensity = 0.0
            max_angle = 0
            for a in peak_angles:
                if intensities[a] > max_intensity:
                    max_intensity = intensities[a]
                    max_angle = a

            # Compute intensity-weighted center angle
            center_angle = self._weighted_center(peak_angles, intensities)

            # Distance at peak
            distance = ranges[max_angle]
            if distance == float('inf') or distance <= 0:
                # Use average distance across peak
                dists = [ranges[a] for a in peak_angles
                         if ranges[a] != float('inf') and ranges[a] > 0]
                if not dists:
                    continue
                distance = sum(dists) / len(dists)

            # Score: ratio above median, peak shape quality
            ratio = max_intensity / median_intensity if median_intensity > 0 else 0
            # Confidence: sigmoid-like mapping of ratio
            confidence = min(1.0, (ratio - self.config.intensity_ratio) /
                           (self.config.intensity_ratio * 2))
            confidence = max(0.0, confidence)

            # Prefer narrower peaks (more directional = more likely real IR)
            width_score = 1.0 - abs(width - 8) / 20.0
            width_score = max(0.0, min(1.0, width_score))

            score = confidence * 0.7 + width_score * 0.3

            if score > best_score:
                best_score = score
                best_detection = DockDetection(
                    detected=True,
                    confidence=score,
                    angle_deg=center_angle,
                    distance_m=distance,
                    intensity_peak=max_intensity,
                )

        if best_detection:
            logger.debug(
                "Dock IR detected at %d° dist=%.2fm intensity=%.0f conf=%.2f",
                best_detection.angle_deg, best_detection.distance_m,
                best_detection.intensity_peak, best_detection.confidence,
            )

        return best_detection

    def _cluster_angles(self, angles: set[int]) -> list[list[int]]:
        """Group adjacent angles into clusters, handling 360→0 wraparound."""
        if not angles:
            return []

        sorted_angles = sorted(angles)
        clusters = []
        current = [sorted_angles[0]]

        for i in range(1, len(sorted_angles)):
            if sorted_angles[i] - sorted_angles[i - 1] <= 2:
                # Allow gap of up to 2° (noisy readings)
                current.append(sorted_angles[i])
            else:
                clusters.append(current)
                current = [sorted_angles[i]]
        clusters.append(current)

        # Check if first and last cluster should merge (wraparound)
        if (len(clusters) >= 2
                and clusters[-1][-1] >= 358
                and clusters[0][0] <= 1):
            clusters[0] = clusters[-1] + clusters[0]
            clusters.pop()

        return clusters

    def _weighted_center(
        self, angles: list[int], intensities: tuple[float, ...]
    ) -> int:
        """Compute intensity-weighted center angle, handling wraparound."""
        # Use vector averaging to handle 360/0 boundary
        sin_sum = 0.0
        cos_sum = 0.0
        for a in angles:
            weight = intensities[a]
            rad = math.radians(a)
            sin_sum += weight * math.sin(rad)
            cos_sum += weight * math.cos(rad)

        center_rad = math.atan2(sin_sum, cos_sum)
        center_deg = int(round(math.degrees(center_rad))) % 360
        return center_deg
