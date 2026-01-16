"""
Main control loop for Nimbus.

The NimbusRunner orchestrates:
- ROS2 topic subscriptions
- Sensor processing
- Behavior execution
- Safety filtering
- Motor command publishing
- API server (optional)
"""

from typing import Optional, Callable
from datetime import datetime
import signal
import threading
import time
import numpy as np

from .node import NimbusNode, MockNimbusNode
from .state import RobotContext, RobotState, Pose2D, Velocity, SensorSnapshot
from .config import NimbusConfig
from nimbus.sensors.lidar import LidarProcessor
from nimbus.sensors.odometry import OdometryProcessor, SensorFusion, create_twist_msg
from nimbus.navigation.safety import SafetyController, VelocitySmoother
from nimbus.behaviors.base import BehaviorManager
from nimbus.behaviors.idle import IdleBehavior
from nimbus.behaviors.wander import WanderBehavior, SimpleWanderBehavior
from nimbus.behaviors.goto import GoToBehavior, PatrolBehavior
from nimbus.behaviors.ai_explore import AIExploreBehavior
from nimbus.behaviors.explore import ExploreBehavior


class NimbusRunner:
    """
    Main Nimbus control loop.

    Coordinates all components:
    - Subscribes to ROS2 topics
    - Processes sensor data
    - Executes behaviors
    - Filters through safety controller
    - Publishes velocity commands

    Usage:
        runner = NimbusRunner()
        runner.run()  # Blocking

        # Or with callback:
        runner.run(on_update=my_callback)
    """

    def __init__(
        self,
        config: Optional[NimbusConfig] = None,
        mock: bool = False,  # Use mock node for testing
    ):
        self.config = config or NimbusConfig.load()
        self._mock = mock

        # Core components
        self._node: Optional[NimbusNode] = None
        self._context = RobotContext()
        self._behavior_manager = BehaviorManager()
        self._safety = SafetyController(self.config.navigation)
        self._smoother = VelocitySmoother()

        # Sensor processors
        self._lidar_processor = LidarProcessor()
        self._odom_processor = OdometryProcessor()
        self._fusion = SensorFusion()

        # Topic buffers (set during start)
        self._scan_buffer = None
        self._odom_buffer = None
        self._cmd_publisher = None

        # Control
        self._running = False
        self._shutdown_event = threading.Event()
        self._control_rate = 10.0  # Hz
        self._last_cmd_time = 0.0

        # Register built-in behaviors
        self._register_behaviors()

        # Signal handling for graceful shutdown
        self._setup_signal_handlers()

    def _register_behaviors(self) -> None:
        """Register built-in behaviors."""
        self._behavior_manager.register(IdleBehavior())
        self._behavior_manager.register(WanderBehavior(
            forward_speed=self.config.navigation.max_linear_speed * 0.7,
            turn_speed=self.config.navigation.max_angular_speed * 0.5,
        ))
        self._behavior_manager.register(SimpleWanderBehavior())
        self._behavior_manager.register(GoToBehavior(
            forward_speed=self.config.navigation.max_linear_speed * 0.8,
            turn_speed=self.config.navigation.max_angular_speed * 0.6,
        ))
        self._behavior_manager.register(PatrolBehavior())
        self._behavior_manager.register(ExploreBehavior(
            forward_speed=self.config.navigation.max_linear_speed * 0.7,
            turn_speed=self.config.navigation.max_angular_speed * 0.5,
        ))
        self._behavior_manager.register(AIExploreBehavior())

    def _setup_signal_handlers(self) -> None:
        """Setup SIGINT/SIGTERM handlers for graceful shutdown."""
        def handler(signum, frame):
            self.stop()

        signal.signal(signal.SIGINT, handler)
        signal.signal(signal.SIGTERM, handler)

    def start(self) -> None:
        """
        Initialize ROS2 connections and start processing.

        Call this before run() if you want to do setup before blocking.
        """
        if self._running:
            return

        # Create node
        if self._mock:
            self._node = MockNimbusNode("nimbus")
        else:
            self._node = NimbusNode("nimbus")

        self._node.start()

        # Subscribe to topics
        try:
            from sensor_msgs.msg import LaserScan
            from nav_msgs.msg import Odometry
            from geometry_msgs.msg import Twist

            self._scan_buffer = self._node.subscribe(
                self.config.sensors.lidar_topic,
                LaserScan,
                buffer_size=1
            )

            self._odom_buffer = self._node.subscribe(
                self.config.sensors.odom_topic,
                Odometry,
                buffer_size=1
            )

            self._cmd_publisher = self._node.publisher("/cmd_vel", Twist)

        except ImportError:
            # Running without ROS2 - use mock
            self._scan_buffer = self._node.subscribe("/scan", object, buffer_size=1)
            self._odom_buffer = self._node.subscribe("/odom_raw", object, buffer_size=1)

        self._running = True
        self._shutdown_event.clear()

        # Default to idle behavior only if no behavior is already set
        if not self._behavior_manager.active_behavior:
            self.set_behavior("idle")

    def run(self, on_update: Optional[Callable[[RobotContext], None]] = None) -> None:
        """
        Main control loop (blocking).

        Args:
            on_update: Optional callback called each control cycle
        """
        if not self._running:
            self.start()

        period = 1.0 / self._control_rate

        while self._running and not self._shutdown_event.is_set():
            loop_start = time.time()

            try:
                self._control_step()

                if on_update:
                    on_update(self._context)

            except Exception as e:
                # Don't crash on errors - log and continue
                print(f"Control loop error: {e}")

            # Maintain loop rate
            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                self._shutdown_event.wait(timeout=sleep_time)

        # Cleanup
        self._send_stop()
        if self._node:
            self._node.shutdown()

    def _control_step(self) -> None:
        """Single control loop iteration."""
        # Read sensors
        self._update_sensors()

        # Get velocity from behavior
        velocity = self._behavior_manager.compute(self._context)

        # Apply safety filtering (only for obstacles in forward arc)
        closest = self._context.sensors.closest_obstacle if self._context.sensors else float('inf')
        obstacle_angle = self._context.sensors.obstacle_direction if self._context.sensors else 0.0
        safe_linear, safe_angular = self._safety.limit_velocity(
            velocity.linear, velocity.angular, closest, obstacle_angle
        )

        # Smooth velocity
        smooth_linear, smooth_angular = self._smoother.smooth(safe_linear, safe_angular)

        # Update state based on safety
        if self._safety.is_emergency:
            self._context.set_state(RobotState.EMERGENCY_STOP)
            self._smoother.reset()

        # Store command
        self._context.set_velocity_cmd(Velocity(smooth_linear, smooth_angular))

        # Send to motors
        self._send_velocity(smooth_linear, smooth_angular)

    def _update_sensors(self) -> None:
        """Read and process sensor data."""
        # Default values
        pose = Pose2D()
        velocity = Velocity()
        lidar_ranges = tuple([float('inf')] * 360)
        closest = float('inf')
        closest_angle = 0.0

        # Process odometry
        if self._odom_buffer:
            odom_msg = self._odom_buffer.latest()
            if odom_msg:
                pose, velocity = self._odom_processor.process(odom_msg)

        # Process LIDAR
        if self._scan_buffer:
            scan_msg = self._scan_buffer.latest()
            if scan_msg:
                ranges = np.array(scan_msg.ranges)
                lidar_ranges = tuple(ranges.tolist())
                closest, closest_angle = self._lidar_processor.find_closest(ranges)

        # Create snapshot
        snapshot = self._fusion.update(
            pose=pose,
            velocity=velocity,
            lidar_ranges=lidar_ranges,
            closest_obstacle=closest,
            obstacle_direction=closest_angle
        )

        self._context.update_sensors(snapshot)

    def _send_velocity(self, linear: float, angular: float) -> None:
        """Send velocity command to motors."""
        if self._cmd_publisher:
            msg = create_twist_msg(linear, angular)
            self._cmd_publisher.publish(msg)
            self._last_cmd_time = time.time()

    def _send_stop(self) -> None:
        """Send stop command."""
        self._send_velocity(0.0, 0.0)

    def stop(self) -> None:
        """Request graceful shutdown."""
        self._running = False
        self._shutdown_event.set()

    # --- Public API ---

    @property
    def context(self) -> RobotContext:
        """Get the robot context."""
        return self._context

    @property
    def is_running(self) -> bool:
        """Check if runner is running."""
        return self._running

    def set_behavior(self, name: str) -> bool:
        """
        Set active behavior by name.

        Returns True if successful.
        """
        success = self._behavior_manager.activate(name)
        if success:
            self._context.set_behavior(name)
        return success

    @property
    def current_behavior(self) -> Optional[str]:
        """Name of current behavior."""
        return self._behavior_manager.active_behavior

    @property
    def available_behaviors(self) -> list[str]:
        """List of available behavior names."""
        return self._behavior_manager.available_behaviors

    def goto(self, x: float, y: float, theta: Optional[float] = None) -> bool:
        """
        Navigate to coordinates.

        Args:
            x, y: Target position (meters)
            theta: Optional final orientation (radians)

        Returns:
            True if navigation started
        """
        goto_behavior = self._behavior_manager.get_behavior("goto")
        if goto_behavior and isinstance(goto_behavior, GoToBehavior):
            goto_behavior.set_target(x, y, theta)
            self._context.set_target(Pose2D(x, y, theta or 0.0))
            return self.set_behavior("goto")
        return False

    def patrol(self, waypoints: list[tuple[float, float]], loop: bool = True) -> bool:
        """
        Start patrolling waypoints.

        Args:
            waypoints: List of (x, y) coordinates
            loop: Whether to loop continuously

        Returns:
            True if patrol started
        """
        patrol_behavior = self._behavior_manager.get_behavior("patrol")
        if patrol_behavior and isinstance(patrol_behavior, PatrolBehavior):
            patrol_behavior.set_waypoints(waypoints)
            patrol_behavior._loop = loop
            return self.set_behavior("patrol")
        return False

    def explore(self, memory_name: str = "default") -> bool:
        """
        Start AI exploration with Ollama.

        Args:
            memory_name: Name of memory to use/create

        Returns:
            True if exploration started
        """
        ai_behavior = self._behavior_manager.get_behavior("ai_explore")
        if ai_behavior and isinstance(ai_behavior, AIExploreBehavior):
            ai_behavior.set_memory(memory_name)
            return self.set_behavior("ai_explore")
        return False

    def explore_systematic(self, memory_name: str = "explore") -> bool:
        """
        Start systematic exploration (no LLM required).

        Uses heuristics to prefer unexplored directions.

        Args:
            memory_name: Name of memory to use/create

        Returns:
            True if exploration started
        """
        explore_behavior = self._behavior_manager.get_behavior("explore")
        if explore_behavior and isinstance(explore_behavior, ExploreBehavior):
            explore_behavior.set_memory(memory_name)
            return self.set_behavior("explore")
        return False

    def get_exploration_status(self) -> Optional[dict]:
        """Get current AI exploration status."""
        ai_behavior = self._behavior_manager.get_behavior("ai_explore")
        if ai_behavior and isinstance(ai_behavior, AIExploreBehavior):
            return ai_behavior.get_status()
        return None

    def get_explore_status(self) -> Optional[dict]:
        """Get current systematic exploration status."""
        explore_behavior = self._behavior_manager.get_behavior("explore")
        if explore_behavior and isinstance(explore_behavior, ExploreBehavior):
            return explore_behavior.get_status()
        return None

    def emergency_stop(self) -> None:
        """Trigger emergency stop."""
        self._context.set_state(RobotState.EMERGENCY_STOP)
        self._send_stop()
        self._smoother.reset()

    @property
    def safety_status(self) -> dict:
        """Get current safety status."""
        return self._safety.status


def create_runner(
    config_path: Optional[str] = None,
    mock: bool = False
) -> NimbusRunner:
    """
    Factory function to create a configured runner.

    Args:
        config_path: Optional path to config file
        mock: Use mock node for testing

    Returns:
        Configured NimbusRunner
    """
    from pathlib import Path

    config = NimbusConfig.load(Path(config_path) if config_path else None)
    return NimbusRunner(config=config, mock=mock)
