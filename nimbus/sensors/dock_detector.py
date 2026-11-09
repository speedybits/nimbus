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
    - On dock: baseline ~610, peak ~4500 (7.4×)
    - Facing dock at 6in: baseline ~150, peak ~950 (6×)
    - Peak width: ~8° at close range
    The ratio check is the primary filter; the absolute threshold just
    rejects scans where everything is uniformly low/noisy.
    """
    intensity_threshold: float = 400.0   # min absolute intensity for IR signal
    intensity_ratio: float = 3.5         # peak must be N× median intensity
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

        # Filter peaks by width
        valid_peaks = []
        for peak_angles in peaks:
            width = len(peak_angles)
            if (width >= self.config.min_peak_width_deg
                    and width <= self.config.max_peak_width_deg):
                valid_peaks.append(peak_angles)

        if not valid_peaks:
            return None

        # Merge ALL valid peaks into a single detection.
        # The Neato charger has dual IR emitters that create two peaks
        # (e.g., at 325° and 30°). Computing the intensity-weighted center
        # across all hot angles gives the true charger bearing (~0°).
        all_hot = []
        for peak_angles in valid_peaks:
            all_hot.extend(peak_angles)

        # Intensity-weighted center across all peaks
        center_angle = self._weighted_center(all_hot, intensities)

        # Find overall max intensity and its distance
        max_intensity = 0.0
        max_angle = 0
        for a in all_hot:
            if intensities[a] > max_intensity:
                max_intensity = intensities[a]
                max_angle = a

        distance = ranges[max_angle]
        if distance == float('inf') or distance <= 0:
            dists = [ranges[a] for a in all_hot
                     if ranges[a] != float('inf') and ranges[a] > 0]
            if not dists:
                return None
            distance = sum(dists) / len(dists)

        # Score based on intensity ratio above median
        ratio = max_intensity / median_intensity if median_intensity > 0 else 0
        confidence = min(1.0, (ratio - self.config.intensity_ratio) /
                       (self.config.intensity_ratio * 2))
        confidence = max(0.0, confidence)

        total_width = len(all_hot)
        width_score = 1.0 - abs(total_width - 12) / 30.0
        width_score = max(0.0, min(1.0, width_score))

        score = confidence * 0.7 + width_score * 0.3

        detection = DockDetection(
            detected=True,
            confidence=score,
            angle_deg=center_angle,
            distance_m=distance,
            intensity_peak=max_intensity,
        )

        logger.debug(
            "Dock IR detected at %d° dist=%.2fm intensity=%.0f conf=%.2f "
            "(%d peaks, %d total hot angles)",
            center_angle, distance, max_intensity, score,
            len(valid_peaks), total_width,
        )

        return detection

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
