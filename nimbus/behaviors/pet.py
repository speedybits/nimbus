"""
Pet behavior - curious cat personality.

A behavior that expresses a cat-like personality through:
- Multiple mood states (resting, alert, curious, investigating, playful)
- Expressive movement patterns (wiggling, circling, stalking)
- Interest detection and investigation
- Natural timing with 50/50 active/resting balance
"""

from typing import Optional
from dataclasses import dataclass
from enum import Enum, auto
import random
import time
import math
import numpy as np

from nimbus.core.state import RobotContext, Velocity, RobotState, Pose2D
from nimbus.navigation.vfh import VFHNavigator, VFHConfig
from nimbus.sensors.lidar import LidarProcessor, LidarConfig
from .base import Behavior


class Mood(Enum):
    """Cat mood states."""
    RESTING = auto()
    ALERT = auto()
    CURIOUS = auto()
    INVESTIGATING = auto()
    PLAYFUL = auto()


class MovementPhase(Enum):
    """Movement sub-phases within moods."""
    MOVING = auto()
    PAUSING = auto()
    SCANNING = auto()
    TURNING = auto()


class InvestigationPhase(Enum):
    """Phases of investigating something interesting."""
    APPROACHING = auto()
    CIRCLING = auto()
    INSPECTING = auto()


@dataclass
class InterestPoint:
    """A point of interest detected by sensors."""
    angle: float          # Angle from robot (radians)
    distance: float       # Distance to point (meters)
    reason: str           # Why it's interesting
    world_x: float = 0.0  # World coordinates
    world_y: float = 0.0

    def update_world_coords(self, robot_pose: Pose2D) -> None:
        """Convert robot-relative to world coordinates."""
        theta = robot_pose.theta + self.angle
        self.world_x = robot_pose.x + self.distance * math.cos(theta)
        self.world_y = robot_pose.y + self.distance * math.sin(theta)


@dataclass
class PetConfig:
    """Configuration for cat-like behavior timing."""

    # Mood durations (seconds)
    resting_duration_min: float = 10.0
    resting_duration_max: float = 30.0
    alert_duration_min: float = 5.0
    alert_duration_max: float = 15.0
    curious_duration_min: float = 15.0
    curious_duration_max: float = 45.0
    investigating_duration_min: float = 5.0
    investigating_duration_max: float = 20.0
    playful_duration_min: float = 5.0
    playful_duration_max: float = 15.0

    # Movement speeds (m/s)
    creep_speed: float = 0.05       # Stalking/slow curious
    walk_speed: float = 0.08        # Normal curious movement
    trot_speed: float = 0.15        # Alert/moderate movement
    zoomie_speed: float = 0.30      # Playful burst

    # Turn speeds (rad/s)
    slow_scan_speed: float = 0.1    # Resting scan
    alert_scan_speed: float = 0.4   # Quick head turns
    curious_turn_speed: float = 0.3 # Direction changes
    zoomie_turn_speed: float = 0.8  # Sharp playful turns

    # Wiggle parameters
    wiggle_frequency: float = 3.0   # Hz
    wiggle_amplitude: float = 0.15  # rad/s

    # Circle parameters
    circle_radius: float = 0.5      # meters
    circle_speed: float = 0.1       # m/s linear
    circle_angular: float = 0.4     # rad/s

    # Pause timing
    curious_pause_min: float = 1.0  # seconds
    curious_pause_max: float = 3.0
    curious_move_min: float = 2.0
    curious_move_max: float = 4.0

    # Interest detection
    interest_range_threshold: float = 0.5  # meters change to trigger
    interest_max_distance: float = 1.5     # max distance to investigate
    gap_width_min: float = 0.4             # narrow gap min
    gap_width_max: float = 1.0             # narrow gap max


class PetBehavior(Behavior):
    """
    Curious cat personality behavior.

    Implements a hierarchical state machine:
    - Top level: Mood (RESTING, ALERT, CURIOUS, INVESTIGATING, PLAYFUL)
    - Bottom level: Movement patterns within each mood

    Expression through movement:
    - Speed variations (creep, walk, trot, zoomie)
    - Angular patterns (wiggle, circle, scan, sharp turns)
    - Timing patterns (pause-move-pause for curious, bursts for playful)
    """

    name = "pet"
    description = "Curious cat personality with mood states"
    priority = 12  # Between wander (10) and explore (15)

    def __init__(self, config: Optional[PetConfig] = None):
        super().__init__()
        self.config = config or PetConfig()

        # Initialize VFH for obstacle avoidance during movement
        self._lidar = LidarProcessor(LidarConfig(safety_radius=0.4))
        self._vfh = VFHNavigator(VFHConfig(threshold_high=0.5, threshold_low=0.2))

        # Mood state
        self._mood = Mood.RESTING
        self._mood_start_time = 0.0
        self._mood_duration = 0.0

        # Movement state
        self._movement_phase = MovementPhase.PAUSING
        self._phase_start_time = 0.0
        self._phase_duration = 0.0

        # Investigation state
        self._investigation_phase = InvestigationPhase.APPROACHING
        self._interest_point: Optional[InterestPoint] = None
        self._circle_direction = 1
        self._circle_progress = 0.0

        # Tracking
        self._last_ranges: Optional[np.ndarray] = None
        self._goal_direction = 0.0
        self._scan_direction = 1
        self._scan_target_angle = 0.0
        self._current_scan_angle = 0.0

        # Statistics for 50/50 balance
        self._time_active = 0.0
        self._time_resting = 0.0
        self._last_compute_time = 0.0

    def activate(self) -> None:
        """Start pet behavior in resting state."""
        super().activate()
        self._set_mood(Mood.RESTING)
        self._last_ranges = None
        self._time_active = 0.0
        self._time_resting = 0.0
        self._last_compute_time = time.time()

    def deactivate(self) -> None:
        """Stop pet behavior."""
        super().deactivate()

    def reset(self) -> None:
        """Reset to initial resting state."""
        self._set_mood(Mood.RESTING)
        self._interest_point = None
        self._last_ranges = None

    def _set_mood(self, mood: Mood) -> None:
        """Transition to a new mood."""
        self._mood = mood
        self._mood_start_time = time.time()
        self._mood_duration = self._get_mood_duration(mood)
        self._movement_phase = MovementPhase.PAUSING
        self._phase_start_time = time.time()

        # Reset mood-specific state
        if mood == Mood.ALERT:
            self._scan_direction = random.choice([-1, 1])
            self._scan_target_angle = random.uniform(math.pi / 4, math.pi / 2)
            self._current_scan_angle = 0.0
        elif mood == Mood.CURIOUS:
            self._pick_random_direction()
            self._phase_duration = self._random_duration(
                self.config.curious_pause_min, self.config.curious_pause_max
            )
        elif mood == Mood.INVESTIGATING:
            self._investigation_phase = InvestigationPhase.APPROACHING
            self._circle_progress = 0.0
            self._circle_direction = random.choice([-1, 1])
        elif mood == Mood.PLAYFUL:
            self._pick_random_direction()

    def _get_mood_duration(self, mood: Mood) -> float:
        """Get random duration for a mood."""
        cfg = self.config
        durations = {
            Mood.RESTING: (cfg.resting_duration_min, cfg.resting_duration_max),
            Mood.ALERT: (cfg.alert_duration_min, cfg.alert_duration_max),
            Mood.CURIOUS: (cfg.curious_duration_min, cfg.curious_duration_max),
            Mood.INVESTIGATING: (cfg.investigating_duration_min, cfg.investigating_duration_max),
            Mood.PLAYFUL: (cfg.playful_duration_min, cfg.playful_duration_max),
        }
        min_d, max_d = durations[mood]
        return self._random_duration(min_d, max_d)

    def _random_duration(self, min_val: float, max_val: float) -> float:
        """Generate natural-feeling random duration."""
        # Beta distribution for more natural timing (peaks in middle)
        normalized = random.betavariate(2, 2)
        return min_val + normalized * (max_val - min_val)

    def _pick_random_direction(self) -> None:
        """Pick a new random preferred direction."""
        # Bias toward forward
        self._goal_direction = random.gauss(0.0, math.pi / 4)
        self._goal_direction = max(-math.pi / 2, min(math.pi / 2, self._goal_direction))

    def compute(self, context: RobotContext) -> Optional[Velocity]:
        """Main compute loop - dispatch to current mood handler."""
        sensors = context.sensors
        if sensors is None:
            return Velocity.stop()

        # Track activity ratio
        self._update_activity_tracking()

        # Check for mood transitions
        self._check_mood_transitions(context)

        # Dispatch to mood-specific handler
        if self._mood == Mood.RESTING:
            return self._compute_resting(context)
        elif self._mood == Mood.ALERT:
            return self._compute_alert(context)
        elif self._mood == Mood.CURIOUS:
            return self._compute_curious(context)
        elif self._mood == Mood.INVESTIGATING:
            return self._compute_investigating(context)
        elif self._mood == Mood.PLAYFUL:
            return self._compute_playful(context)

        return Velocity.stop()

    def _update_activity_tracking(self) -> None:
        """Track time spent active vs resting."""
        now = time.time()
        dt = now - self._last_compute_time
        self._last_compute_time = now

        if self._mood == Mood.RESTING:
            self._time_resting += dt
        else:
            self._time_active += dt

    def _check_mood_transitions(self, context: RobotContext) -> None:
        """Check if we should transition to a new mood."""
        elapsed = time.time() - self._mood_start_time

        # Check for interest point (can trigger INVESTIGATING from ALERT or CURIOUS)
        if self._mood in (Mood.ALERT, Mood.CURIOUS):
            interest = self._detect_interesting(context)
            if interest is not None:
                self._interest_point = interest
                self._set_mood(Mood.INVESTIGATING)
                return

        # Time-based transitions
        if elapsed >= self._mood_duration:
            self._transition_from_timeout()
            return

        # Random transitions (probability per compute cycle ~10Hz)
        self._check_random_transition()

    def _transition_from_timeout(self) -> None:
        """Handle mood timeout - move to next mood."""
        if self._mood == Mood.RESTING:
            # Resting -> Alert or Curious
            self._set_mood(random.choice([Mood.ALERT, Mood.CURIOUS]))
        elif self._mood == Mood.ALERT:
            # Alert -> Curious (done scanning)
            self._set_mood(Mood.CURIOUS)
        elif self._mood == Mood.CURIOUS:
            # Curious -> Resting (tired of wandering)
            self._set_mood(Mood.RESTING)
        elif self._mood == Mood.INVESTIGATING:
            # Done investigating -> Curious or Resting
            self._interest_point = None
            self._set_mood(random.choice([Mood.CURIOUS, Mood.RESTING]))
        elif self._mood == Mood.PLAYFUL:
            # Tired from zoomies -> Resting
            self._set_mood(Mood.RESTING)

    def _check_random_transition(self) -> None:
        """Check for random mood transitions."""
        # Lower probability = less frequent random changes
        roll = random.random()

        if self._mood == Mood.RESTING:
            if roll < 0.005:  # 0.5% chance per cycle
                self._set_mood(Mood.ALERT)
        elif self._mood == Mood.CURIOUS:
            if roll < 0.002:  # Rare zoomie trigger
                self._set_mood(Mood.PLAYFUL)
        elif self._mood == Mood.ALERT:
            if roll < 0.008:
                self._set_mood(Mood.CURIOUS)

    # ==================== MOOD HANDLERS ====================

    def _compute_resting(self, context: RobotContext) -> Velocity:
        """Cat is resting - minimal movement with occasional ear-perk scan."""
        elapsed = time.time() - self._phase_start_time

        if self._movement_phase == MovementPhase.PAUSING:
            # Mostly stay still, occasionally start a scan
            if elapsed > self._phase_duration:
                if random.random() < 0.3:  # 30% chance to scan
                    self._movement_phase = MovementPhase.SCANNING
                    self._phase_start_time = time.time()
                    self._phase_duration = random.uniform(2.0, 4.0)
                    self._scan_direction = random.choice([-1, 1])
                else:
                    # Reset pause timer
                    self._phase_start_time = time.time()
                    self._phase_duration = random.uniform(3.0, 8.0)
            return Velocity.stop()

        elif self._movement_phase == MovementPhase.SCANNING:
            # Slow rotation to look around
            if elapsed > self._phase_duration:
                self._movement_phase = MovementPhase.PAUSING
                self._phase_start_time = time.time()
                self._phase_duration = random.uniform(5.0, 10.0)
                return Velocity.stop()
            return Velocity(linear=0.0, angular=self.config.slow_scan_speed * self._scan_direction)

        return Velocity.stop()

    def _compute_alert(self, context: RobotContext) -> Velocity:
        """Cat is alert - scanning, ears perked, ready to investigate."""
        elapsed = time.time() - self._phase_start_time

        if self._movement_phase == MovementPhase.PAUSING:
            # Pause between scans
            if elapsed > 1.0:  # 1 second pause
                self._movement_phase = MovementPhase.TURNING
                self._phase_start_time = time.time()
                self._scan_direction *= -1  # Alternate direction
                self._scan_target_angle = random.uniform(math.pi / 6, math.pi / 3)
                self._current_scan_angle = 0.0
            return Velocity.stop()

        elif self._movement_phase == MovementPhase.TURNING:
            # Quick turn to scan angle
            dt = time.time() - self._phase_start_time
            self._current_scan_angle = abs(self.config.alert_scan_speed * dt)

            if self._current_scan_angle >= self._scan_target_angle:
                self._movement_phase = MovementPhase.PAUSING
                self._phase_start_time = time.time()
                return Velocity.stop()

            return Velocity(linear=0.0, angular=self.config.alert_scan_speed * self._scan_direction)

        return Velocity.stop()

    def _compute_curious(self, context: RobotContext) -> Velocity:
        """Cat is curious - slow exploration with frequent pauses."""
        sensors = context.sensors
        elapsed = time.time() - self._phase_start_time

        if self._movement_phase == MovementPhase.PAUSING:
            # Pause and look around
            if elapsed > self._phase_duration:
                self._movement_phase = MovementPhase.MOVING
                self._phase_start_time = time.time()
                self._phase_duration = self._random_duration(
                    self.config.curious_move_min, self.config.curious_move_max
                )
                self._pick_random_direction()
            return Velocity.stop()

        elif self._movement_phase == MovementPhase.MOVING:
            # Move with wiggle
            if elapsed > self._phase_duration:
                self._movement_phase = MovementPhase.PAUSING
                self._phase_start_time = time.time()
                self._phase_duration = self._random_duration(
                    self.config.curious_pause_min, self.config.curious_pause_max
                )
                return Velocity.stop()

            # Process LIDAR for obstacle avoidance
            if len(sensors.lidar_ranges) < 10:
                return Velocity.stop()

            histogram = self._lidar.process(np.array(sensors.lidar_ranges))
            steering, blocked = self._vfh.compute_steering(histogram, self._goal_direction)

            if blocked:
                # Obstacle - pause and think
                self._movement_phase = MovementPhase.PAUSING
                self._phase_start_time = time.time()
                self._phase_duration = 1.5
                return Velocity.stop()

            # Slow forward with wiggle
            wiggle = math.sin(time.time() * self.config.wiggle_frequency) * self.config.wiggle_amplitude
            return Velocity(linear=self.config.walk_speed, angular=steering + wiggle)

        return Velocity.stop()

    def _compute_investigating(self, context: RobotContext) -> Velocity:
        """Cat investigating something interesting - approach and circle."""
        sensors = context.sensors

        if self._interest_point is None:
            self._set_mood(Mood.CURIOUS)
            return Velocity.stop()

        elapsed = time.time() - self._phase_start_time

        if self._investigation_phase == InvestigationPhase.APPROACHING:
            # Slow, stalking approach
            if len(sensors.lidar_ranges) < 10:
                return Velocity.stop()

            # Check if we're close enough
            target_angle = self._interest_point.angle
            target_dist = self._interest_point.distance

            if target_dist < 0.5:  # Close enough, start circling
                self._investigation_phase = InvestigationPhase.CIRCLING
                self._phase_start_time = time.time()
                return Velocity.stop()

            # Approach slowly
            histogram = self._lidar.process(np.array(sensors.lidar_ranges))
            steering, blocked = self._vfh.compute_steering(histogram, target_angle)

            if blocked:
                # Can't approach, circle instead
                self._investigation_phase = InvestigationPhase.CIRCLING
                self._phase_start_time = time.time()
                return Velocity.stop()

            return Velocity(linear=self.config.creep_speed, angular=steering * 0.8)

        elif self._investigation_phase == InvestigationPhase.CIRCLING:
            # Circle around the interesting point
            self._circle_progress += 0.05  # Increment per compute cycle

            if self._circle_progress >= 2 * math.pi:  # Full circle
                self._investigation_phase = InvestigationPhase.INSPECTING
                self._phase_start_time = time.time()
                return Velocity.stop()

            return Velocity(
                linear=self.config.circle_speed,
                angular=self.config.circle_angular * self._circle_direction
            )

        elif self._investigation_phase == InvestigationPhase.INSPECTING:
            # Stop and inspect
            if elapsed > 2.0:  # 2 seconds of inspection
                self._interest_point = None
                self._set_mood(Mood.CURIOUS)
            return Velocity.stop()

        return Velocity.stop()

    def _compute_playful(self, context: RobotContext) -> Velocity:
        """Cat having zoomies - fast, erratic movement."""
        sensors = context.sensors

        if len(sensors.lidar_ranges) < 10:
            return Velocity.stop()

        # Random direction changes
        if random.random() < 0.08:  # 8% chance per cycle
            self._pick_random_direction()

        # Process LIDAR for safety
        histogram = self._lidar.process(np.array(sensors.lidar_ranges))
        steering, blocked = self._vfh.compute_steering(histogram, self._goal_direction)

        if blocked:
            # Sharp turn away
            return Velocity(linear=0.0, angular=self.config.zoomie_turn_speed * random.choice([-1, 1]))

        # Occasional sharp turns for playfulness
        if random.random() < 0.03:
            return Velocity(
                linear=self.config.trot_speed,
                angular=random.choice([-1, 1]) * self.config.zoomie_turn_speed
            )

        # Fast forward with steering
        return Velocity(linear=self.config.zoomie_speed, angular=steering * 1.5)

    # ==================== INTEREST DETECTION ====================

    def _detect_interesting(self, context: RobotContext) -> Optional[InterestPoint]:
        """Detect things a curious cat would investigate."""
        sensors = context.sensors
        if sensors is None or len(sensors.lidar_ranges) < 10:
            return None

        ranges = np.array(sensors.lidar_ranges)

        # 1. New close object (wasn't there last cycle)
        if self._last_ranges is not None and len(self._last_ranges) == len(ranges):
            range_diff = np.abs(ranges - self._last_ranges)
            # Find significant changes that are now close
            mask = (range_diff > self.config.interest_range_threshold) & \
                   (ranges < self.config.interest_max_distance) & \
                   (ranges > 0.2)  # Not too close

            changed_indices = np.where(mask)[0]
            if len(changed_indices) > 0:
                idx = changed_indices[0]
                angle = self._idx_to_angle(idx, len(ranges))
                distance = ranges[idx]
                self._last_ranges = ranges.copy()
                return InterestPoint(angle=angle, distance=distance, reason="new_object")

        # 2. Narrow gap detection (cat curiosity about openings)
        gap = self._find_narrow_gap(ranges)
        if gap is not None:
            self._last_ranges = ranges.copy()
            return gap

        self._last_ranges = ranges.copy()
        return None

    def _find_narrow_gap(self, ranges: np.ndarray) -> Optional[InterestPoint]:
        """Find narrow gaps that might be interesting to investigate."""
        # Look for sudden depth changes indicating a gap/opening
        threshold = 0.8  # meters difference
        min_gap_width_deg = 10  # minimum gap width in degrees
        max_gap_width_deg = 30  # maximum gap width in degrees

        for i in range(len(ranges) - 1):
            # Look for a significant drop in range (opening)
            if ranges[i] > 1.0 and ranges[i] - ranges[(i + 1) % len(ranges)] > threshold:
                # Found potential gap start
                gap_start = i
                gap_end = i

                # Find gap end
                for j in range(i + 1, min(i + max_gap_width_deg + 1, len(ranges))):
                    if ranges[j] < ranges[gap_start] - threshold / 2:
                        gap_end = j
                    else:
                        break

                gap_width = gap_end - gap_start
                if min_gap_width_deg <= gap_width <= max_gap_width_deg:
                    center_idx = (gap_start + gap_end) // 2
                    angle = self._idx_to_angle(center_idx, len(ranges))
                    distance = np.min(ranges[gap_start:gap_end + 1])
                    return InterestPoint(angle=angle, distance=distance, reason="narrow_gap")

        return None

    def _idx_to_angle(self, idx: int, total: int) -> float:
        """Convert LIDAR index to angle in radians."""
        # Assuming 360 degree LIDAR, index 0 = front
        angle_deg = idx if idx <= total // 2 else idx - total
        return math.radians(angle_deg)

    # ==================== STATUS ====================

    def get_status(self) -> dict:
        """Get current pet behavior status for API."""
        total_time = self._time_active + self._time_resting
        activity_ratio = self._time_active / total_time if total_time > 0 else 0.5

        return {
            "mood": self._mood.name,
            "mood_elapsed": time.time() - self._mood_start_time,
            "mood_duration": self._mood_duration,
            "movement_phase": self._movement_phase.name,
            "activity_ratio": round(activity_ratio, 2),
            "investigating": self._interest_point is not None,
            "interest_reason": self._interest_point.reason if self._interest_point else None,
        }
