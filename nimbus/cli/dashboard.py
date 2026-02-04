"""
Live terminal dashboard for Nimbus.

Uses Rich for beautiful, real-time visualization.
"""

import json
import time
from pathlib import Path

from rich.console import Console
from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
import math
import sys
import select
import threading
import queue
from collections import deque
from datetime import datetime


class LogBuffer:
    """
    Thread-safe log buffer using queue.Queue.

    Uses Python's queue.Queue for thread-safe producer/consumer pattern.
    Any thread can append; the main thread drains into display buffer.
    """

    def __init__(self, max_lines: int = 12):
        self._queue = queue.Queue()  # Thread-safe inbox
        self._lines = deque(maxlen=max_lines)  # Display buffer (main thread only)

    def append(self, message: str) -> None:
        """Add message from any thread (non-blocking)."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self._queue.put(f"[dim]{timestamp}[/dim] {message}")

    def _drain(self) -> None:
        """Move queued messages to display buffer (call from main thread)."""
        while True:
            try:
                msg = self._queue.get_nowait()
                self._lines.append(msg)
            except queue.Empty:
                break

    def get_text(self) -> str:
        """Get display text (main thread only)."""
        self._drain()
        if not self._lines:
            return "[dim]No events yet[/dim]"
        return "\n".join(self._lines)

    def get_recent(self, count: int = 20) -> list:
        """Get recent lines as list (main thread only)."""
        self._drain()
        return list(self._lines)[-count:]


class KeyboardHandler:
    """Non-blocking keyboard input handler for Unix systems."""

    def __init__(self):
        self._queue = queue.Queue()
        self._stop = threading.Event()
        self._thread = None
        self._old_settings = None

    def start(self) -> None:
        """Start listening for keyboard input."""
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

    def _listen(self) -> None:
        """Background thread that listens for keypresses."""
        try:
            import termios
            import tty
            self._old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            while not self._stop.is_set():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    self._queue.put(key)
        except Exception:
            pass  # Not a TTY or other issue
        finally:
            if self._old_settings:
                try:
                    import termios
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
                except Exception:
                    pass

    def get_key(self) -> str | None:
        """Get next keypress from queue (non-blocking)."""
        try:
            return self._queue.get_nowait()
        except queue.Empty:
            return None

    def stop(self) -> None:
        """Stop the keyboard listener."""
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=0.5)


class LiveDashboard:
    """
    Real-time terminal dashboard using Rich.

    Layout:
    +------------------+------------------+
    |     SENSORS      |      STATUS      |
    +------------------+------------------+
    |    LIDAR VIEW    |     MAP VIEW     |
    +------------------+------------------+
    |             SHORTCUTS                |
    +--------------------------------------+
    |               LOG                    |
    +--------------------------------------+
    """

    # Key mappings for behavior switching
    KEY_MAPPINGS = {
        'w': 'wander',
        'i': 'idle',
        's': 'simple_wander',
        'g': 'goto',
        'p': 'patrol',
        'x': 'explore',
        'a': 'ai_explore',
        'm': 'motor_test',
    }

    def __init__(self, runner, console: Console = None):
        self.runner = runner
        self.console = console or Console()
        self._layout = self._build_layout()
        self._live = Live(
            self._layout,
            refresh_per_second=5,
            console=self.console,
            screen=True,
        )
        self._log = LogBuffer(max_lines=12)  # Unified log buffer
        self._keyboard = KeyboardHandler()
        self._last_state = None
        self._last_behavior = None
        self._had_lidar = True  # Track LIDAR availability

    def _build_layout(self) -> Layout:
        """Build the dashboard layout."""
        layout = Layout()

        layout.split_column(
            Layout(name="header", size=3),
            Layout(name="top", size=10),
            Layout(name="views", size=14),  # Combined LIDAR + Map row
            Layout(name="shortcuts", size=3),
            Layout(name="logs", size=8),  # Reduced for views row
        )

        layout["top"].split_row(
            Layout(name="sensors"),
            Layout(name="status"),
        )

        # Split views into LIDAR and Map side-by-side
        layout["views"].split_row(
            Layout(name="lidar"),
            Layout(name="map"),
        )

        # Header
        layout["header"].update(Panel(
            Text("NIMBUS DASHBOARD", style="bold cyan", justify="center"),
            style="cyan"
        ))

        return layout

    def __enter__(self):
        self._live.__enter__()
        self._keyboard.start()
        self._log.append("[green]Dashboard started[/green]")
        # Register unified log callback
        try:
            from nimbus.core.xrce.logger import set_log_callback
            set_log_callback(self.log)
        except ImportError:
            pass
        return self

    def __exit__(self, *args):
        # Unregister log callback
        try:
            from nimbus.core.xrce.logger import set_log_callback
            set_log_callback(None)
        except ImportError:
            pass
        self._keyboard.stop()
        self._live.__exit__(*args)

    def update(self, context) -> None:
        """Update dashboard with current robot context."""
        # Process keyboard input
        self._process_keys()

        # Check if we're still waiting for connection
        sensors = context.sensors
        has_data = sensors and (sensors.lidar_ranges or sensors.pose.x != 0 or sensors.pose.y != 0)

        if not has_data and not hasattr(self, '_ever_connected'):
            # Show waiting message instead of empty panes
            self._show_waiting_message()
            self._update_logs()
            return

        if has_data:
            self._ever_connected = True

        # Track state/behavior changes for logging
        self._log_changes(context)

        # Update all panels
        self._update_sensors(context)
        self._update_status(context)
        self._update_lidar(context)
        self._update_map(context)
        self._update_shortcuts()
        self._update_logs()

        # Write state for Claude every second
        now = time.time()
        if not hasattr(self, '_last_claude_write') or now - self._last_claude_write >= 1.0:
            self._write_claude_state(context)
            self._last_claude_write = now

    def _show_waiting_message(self) -> None:
        """Show waiting for connection message."""
        waiting_text = Text()
        waiting_text.append("\n\n")
        waiting_text.append("⏳ Please wait up to 30 seconds for robot to connect...\n\n", style="bold yellow")
        waiting_text.append("The ESP32 needs time to establish the XRCE-DDS connection.\n", style="dim")
        waiting_text.append("If it doesn't connect, try power cycling the robot.", style="dim")

        # Fill all the main panels with the waiting message
        self._layout["sensors"].update(Panel(waiting_text, title="Connecting...", border_style="yellow"))
        self._layout["status"].update(Panel(Text("", justify="center"), title="Status", border_style="dim"))
        self._layout["lidar"].update(Panel(
            Text("\n\n   Waiting for LIDAR...", style="dim", justify="center"),
            title="LIDAR View", border_style="dim"
        ))
        self._layout["map"].update(Panel(
            Text("\n\n   Waiting for data...", style="dim", justify="center"),
            title="Map View", border_style="dim"
        ))
        self._layout["shortcuts"].update(Panel(
            Text("[dim]Q: quit[/dim]", justify="center"),
            title="Shortcuts", border_style="dim"
        ))

    def _update_sensors(self, context) -> None:
        """Update sensors panel."""
        sensors = context.sensors

        table = Table.grid(padding=(0, 2))
        table.add_column(style="cyan", width=15)
        table.add_column(style="green", width=20)

        if sensors:
            table.add_row("Position X:", f"{sensors.pose.x:.3f} m")
            table.add_row("Position Y:", f"{sensors.pose.y:.3f} m")
            table.add_row("Heading:", f"{math.degrees(sensors.pose.theta):.1f} deg")
            table.add_row("Odom Vel:", f"{sensors.velocity.linear:.3f} / {sensors.velocity.angular:.3f}")
            # Show commanded velocity (what we're sending to motors)
            cmd = context.velocity_cmd
            table.add_row("Cmd Vel:", Text(f"{cmd.linear:.3f} / {cmd.angular:.3f}", style="yellow"))
            table.add_row("Closest:", f"{sensors.closest_obstacle:.2f} m")
        else:
            table.add_row("Status:", "No sensor data")

        self._layout["sensors"].update(Panel(table, title="Sensors", border_style="blue"))

    def _update_status(self, context) -> None:
        """Update status panel."""
        table = Table.grid(padding=(0, 2))
        table.add_column(style="cyan", width=15)
        table.add_column(width=20)

        # State with color
        state_colors = {
            "IDLE": "white",
            "NAVIGATING": "green",
            "AVOIDING": "yellow",
            "EMERGENCY_STOP": "bold red",
            "CALIBRATING": "cyan",
        }
        state_name = context.state.name
        state_style = state_colors.get(state_name, "white")
        table.add_row("State:", Text(state_name, style=state_style))

        # Behavior
        behavior = context.current_behavior or "None"
        table.add_row("Behavior:", behavior)

        # Target
        target = context.target
        if target:
            table.add_row("Target:", f"({target.x:.2f}, {target.y:.2f})")
            dist = context.distance_to_target()
            if dist:
                table.add_row("Distance:", f"{dist:.2f} m")
        else:
            table.add_row("Target:", "None")

        # Safety
        safety = self.runner.safety_status
        if safety["is_emergency"]:
            table.add_row("Safety:", Text("EMERGENCY", style="bold red"))
        elif not safety["can_move_forward"]:
            table.add_row("Safety:", Text("CAUTION", style="yellow"))
        else:
            table.add_row("Safety:", Text("OK", style="green"))

        # Show wander speed settings
        wander = self.runner._behavior_manager.get_behavior("wander")
        if wander and hasattr(wander, 'forward_speed'):
            table.add_row("Set Speed:", f"fwd={wander.forward_speed:.3f} turn={wander.turn_speed:.3f}")

        # Show exploration progress if in explore mode
        if behavior == "explore":
            explore_status = self.runner.get_explore_status()
            if explore_status:
                pct = explore_status.get("coverage_percent", 0)
                cells = explore_status.get("cells_explored", 0)
                new_area = explore_status.get("new_area_confidence", 0)
                table.add_row("Explored:", f"{pct:.0f}% ({cells} cells)")
                table.add_row("New Area:", f"{new_area:.0f}%")

        self._layout["status"].update(Panel(table, title="Status", border_style="blue"))

    def _update_lidar(self, context) -> None:
        """Update LIDAR visualization."""
        sensors = context.sensors
        if not sensors or not sensors.lidar_ranges or len(sensors.lidar_ranges) == 0:
            error_msg = Text("⚠ NO LIDAR DATA - Robot cannot navigate!", style="bold red", justify="center")
            self._layout["lidar"].update(Panel(error_msg, title="LIDAR View", border_style="red"))
            return

        # ASCII LIDAR visualization with exploration coloring
        viz = self._render_lidar_ascii(sensors.lidar_ranges, sensors.closest_obstacle, context)
        self._layout["lidar"].update(Panel(viz, title="LIDAR View", border_style="green"))

    def _lidar_to_world(self, index: int, distance: float, pose, deg_per_reading: float) -> tuple:
        """Convert LIDAR polar reading to world coordinates."""
        # LIDAR angle in robot frame (index 0 = back, index 180 = front)
        # Add pi to convert correctly to world frame
        lidar_angle_rad = math.radians(index * deg_per_reading)

        # Convert to world frame using robot heading
        world_angle = pose.theta + math.pi + lidar_angle_rad

        world_x = pose.x + distance * math.cos(world_angle)
        world_y = pose.y + distance * math.sin(world_angle)

        return world_x, world_y

    def _render_lidar_ascii(self, ranges: tuple, closest: float, context) -> Text:
        """
        Render LIDAR data as ASCII polar plot with exploration coloring.

        In explore mode, points in visited cells are colored blue.
        """
        width = 29
        height = 12
        canvas = [[' '] * width for _ in range(height)]
        canvas_styles = [[None] * width for _ in range(height)]

        center_x = width // 2
        center_y = height // 2

        max_display_range = 2.0  # meters
        scale = min(width // 2 - 2, height - 2) / max_display_range

        # Get exploration memory if in explore mode
        explore_memory = None
        pose = context.sensors.pose if context.sensors else None
        if context.current_behavior == "explore" and pose:
            behavior = self.runner._behavior_manager.get_behavior("explore")
            if behavior:
                explore_memory = behavior.memory

        # Plot obstacles
        num_readings = len(ranges)
        degrees_per_reading = 360.0 / num_readings if num_readings > 0 else 1.0
        for i in range(num_readings):
            distance = ranges[i]
            if distance < max_display_range and distance > 0.05:
                # Convert polar to cartesian (index -> actual angle)
                actual_angle_deg = i * degrees_per_reading
                angle_rad = math.radians(actual_angle_deg - 90)  # 0 deg = forward = up
                r = distance * scale

                x = int(center_x + r * math.cos(angle_rad) * 2)  # *2 for aspect ratio
                y = int(center_y - r * math.sin(angle_rad))

                if 0 <= x < width and 0 <= y < height:
                    # Use intensity based on distance
                    if distance < 0.3:
                        canvas[y][x] = '#'  # Very close - danger
                    elif distance < 0.5:
                        canvas[y][x] = '+'  # Close
                    else:
                        canvas[y][x] = '.'  # Normal

                    # Check if this point has been explored
                    if explore_memory and pose:
                        world_x, world_y = self._lidar_to_world(
                            i, distance, pose, degrees_per_reading
                        )
                        if explore_memory.is_visited(world_x, world_y):
                            canvas_styles[y][x] = "blue"

        # Mark robot position
        canvas[center_y][center_x] = 'R'
        canvas_styles[center_y][center_x] = "green"

        # Mark forward direction
        if center_y > 0:
            canvas[center_y - 1][center_x] = '^'
            canvas_styles[center_y - 1][center_x] = "green"

        # Redraw any VERY close obstacles on top of robot marker
        for i in range(num_readings):
            distance = ranges[i]
            if 0.05 < distance < 0.2:  # Very close - draw on top
                actual_angle_deg = i * degrees_per_reading
                angle_rad = math.radians(actual_angle_deg - 90)
                r = distance * scale
                x = int(center_x + r * math.cos(angle_rad) * 2)
                y = int(center_y - r * math.sin(angle_rad))
                if 0 <= x < width and 0 <= y < height:
                    canvas[y][x] = '#'
                    # Keep existing style (blue if explored)

        # Build Rich Text output with colors
        lines = []
        for row, styles in zip(canvas, canvas_styles):
            line = Text()
            for char, style in zip(row, styles):
                line.append(char, style=style)
            lines.append(line)

        lines.append(Text(f"  Closest: {closest:.2f}m  |  Scale: 1 char = {1/scale:.2f}m"))

        # Join lines with newlines
        result = Text()
        for i, line in enumerate(lines):
            result.append_text(line)
            if i < len(lines) - 1:
                result.append("\n")

        return result

    def _update_map(self, context) -> None:
        """Update world map visualization."""
        obstacle_map = self.runner.obstacle_map
        sensors = context.sensors

        if not sensors or obstacle_map.num_obstacles == 0 and obstacle_map.num_visited == 0:
            empty_msg = Text("\n\n   Building map...", style="dim", justify="center")
            self._layout["map"].update(Panel(empty_msg, title="Map View", border_style="dim"))
            return

        # Render the map
        viz = self._render_map_ascii(obstacle_map, sensors.pose)
        self._layout["map"].update(Panel(viz, title="Map View", border_style="green"))

    @staticmethod
    def _heading_arrow(theta_rad: float) -> str:
        """Map heading angle to directional arrow character."""
        arrows = ['→', '↗', '↑', '↖', '←', '↙', '↓', '↘']
        index = round(theta_rad / (math.pi / 4)) % 8
        return arrows[index]

    def _render_map_ascii(self, obstacle_map, robot_pose) -> Text:
        """
        Render accumulated obstacle map as ASCII.

        Symbols:
          ' ' = unknown/unexplored
          '#' = high-confidence obstacle (red)
          '+' = lower-confidence obstacle (yellow)
          '~' = robot trail (cyan)
          directional arrow = robot position (bold green)

        Includes compass rose in top-right corner.
        """
        width = 29
        height = 12
        canvas = [[' '] * width for _ in range(height)]
        canvas_styles = [[None] * width for _ in range(height)]

        # Get map bounds with padding
        min_x, min_y, max_x, max_y = obstacle_map.get_bounds()

        # Add padding (0.5m)
        padding = 0.5
        min_x -= padding
        min_y -= padding
        max_x += padding
        max_y += padding

        # Calculate scale to fit canvas
        # Account for character aspect ratio (chars are ~2x taller than wide)
        world_width = max_x - min_x
        world_height = max_y - min_y

        if world_width < 0.1:
            world_width = 2.0
        if world_height < 0.1:
            world_height = 2.0

        # Scale factors (divide by 2 for width to account for character aspect ratio)
        scale_x = (width - 2) / world_width / 2.0
        scale_y = (height - 2) / world_height

        # Use the smaller scale to preserve aspect ratio
        scale = min(scale_x, scale_y)

        # Calculate center offset
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2

        def world_to_canvas(wx: float, wy: float) -> tuple[int, int]:
            """Convert world coordinates to canvas coordinates."""
            # Center on explored area, apply scale, account for aspect ratio
            cx = int((wx - center_x) * scale * 2 + width / 2)
            cy = int(height / 2 - (wy - center_y) * scale)  # Y is inverted
            return (cx, cy)

        # Draw obstacles
        for cell, confidence in obstacle_map.get_obstacles().items():
            cell_x, cell_y = cell
            wx = (cell_x + 0.5) * obstacle_map.config.cell_size
            wy = (cell_y + 0.5) * obstacle_map.config.cell_size
            cx, cy = world_to_canvas(wx, wy)
            if 0 <= cx < width and 0 <= cy < height:
                if confidence >= 0.6:
                    canvas[cy][cx] = '#'
                    canvas_styles[cy][cx] = "red"
                else:
                    canvas[cy][cx] = '+'
                    canvas_styles[cy][cx] = "yellow"

        # Draw robot trail
        trail = obstacle_map.get_robot_trail(max_points=30)
        for wx, wy in trail[:-1]:  # Skip last point (current position)
            cx, cy = world_to_canvas(wx, wy)
            if 0 <= cx < width and 0 <= cy < height:
                # Only draw trail on free space, not on obstacles
                if canvas[cy][cx] == ' ':
                    canvas[cy][cx] = '~'
                    canvas_styles[cy][cx] = "cyan"

        # Draw robot position with directional arrow
        robot_arrow = '→'
        if robot_pose:
            robot_arrow = self._heading_arrow(robot_pose.theta)
            cx, cy = world_to_canvas(robot_pose.x, robot_pose.y)
            if 0 <= cx < width and 0 <= cy < height:
                canvas[cy][cx] = robot_arrow
                canvas_styles[cy][cx] = "bold green"

        # Draw compass rose in top-right corner
        compass_x = width - 5
        #   N
        # W · E
        #   S
        compass_chars = [
            (compass_x + 2, 0, 'N', 'dim white'),
            (compass_x, 1, 'W', 'dim white'),
            (compass_x + 2, 1, robot_arrow, 'bold green'),
            (compass_x + 4, 1, 'E', 'dim white'),
            (compass_x + 2, 2, 'S', 'dim white'),
        ]
        for cx_c, cy_c, char, style in compass_chars:
            if 0 <= cx_c < width and 0 <= cy_c < height:
                canvas[cy_c][cx_c] = char
                canvas_styles[cy_c][cx_c] = style

        # Build Rich Text output
        lines = []
        for row, styles in zip(canvas, canvas_styles):
            line = Text()
            for char, style in zip(row, styles):
                line.append(char, style=style)
            lines.append(line)

        # Add info line
        obs_count = obstacle_map.num_obstacles
        heading_deg = int(math.degrees(robot_pose.theta)) % 360 if robot_pose else 0
        lines.append(Text(f" #:walls ~:trail  Hdg:{heading_deg}°", style="dim"))

        # Join lines
        result = Text()
        for i, line in enumerate(lines):
            result.append_text(line)
            if i < len(lines) - 1:
                result.append("\n")

        return result

    def _process_keys(self) -> None:
        """Process any pending keypresses."""
        while True:
            key = self._keyboard.get_key()
            if key is None:
                break

            # Debug: log all key presses
            self._log.append(f"[dim]Key: {repr(key)}[/dim]")

            # Handle quit
            if key.lower() == 'q':
                self._log.append("[yellow]Quit requested[/yellow]")
                self.runner.stop()
                return

            # Handle behavior switching
            if key.lower() in self.KEY_MAPPINGS:
                behavior = self.KEY_MAPPINGS[key.lower()]
                if self.runner.set_behavior(behavior):
                    self._log.append(f"[green]Switched to {behavior}[/green]")
                else:
                    self._log.append(f"[red]Failed: {behavior} not available[/red]")

            # Handle emergency stop
            elif key.lower() == 'e':
                self.runner.emergency_stop()
                self._log.append("[bold red]EMERGENCY STOP[/bold red]")

            # Handle speed adjustment
            elif key in ('+', '='):
                self._adjust_wander_speed(1.5)  # Increase 50%
            elif key == '-':
                self._adjust_wander_speed(0.67)  # Decrease ~33%

            # Handle motor test controls (when in motor_test mode)
            elif self.runner.current_behavior == 'motor_test':
                self._handle_motor_test_key(key)

    def _adjust_wander_speed(self, factor: float) -> None:
        """Adjust wander behavior speed by factor."""
        # Try to get the wander behavior
        try:
            wander = self.runner._behavior_manager.get_behavior("wander")
            if wander and hasattr(wander, 'adjust_speed'):
                fwd, turn = wander.adjust_speed(factor)
                self._log.append(f"[cyan]Speed: fwd={fwd:.3f} turn={turn:.3f}[/cyan]")
            else:
                avail = list(self.runner._behavior_manager._behaviors.keys())
                self._log.append(f"[yellow]No wander behavior. Available: {avail}[/yellow]")
        except Exception as e:
            self._log.append(f"[red]Speed error: {e}[/red]")

    def _handle_motor_test_key(self, key: str) -> None:
        """Handle motor test control keys."""
        try:
            motor_test = self.runner._behavior_manager.get_behavior("motor_test")
            if not motor_test:
                return

            # Direction controls: 8/k=fwd, 2/j=back, 4/h=left, 6/l=right, 5/space/0=stop
            if key in ('8', 'k'):
                lin, ang = motor_test.forward()
                self._log.append(f"[green]FORWARD[/green] lin={lin:.2f} ang={ang:.2f}")
            elif key in ('2', 'j'):
                lin, ang = motor_test.backward()
                self._log.append(f"[yellow]BACKWARD[/yellow] lin={lin:.2f} ang={ang:.2f}")
            elif key in ('4', 'h'):
                lin, ang = motor_test.left()
                self._log.append(f"[cyan]LEFT[/cyan] lin={lin:.2f} ang={ang:.2f}")
            elif key in ('6', 'l'):
                lin, ang = motor_test.right()
                self._log.append(f"[magenta]RIGHT[/magenta] lin={lin:.2f} ang={ang:.2f}")
            elif key in ('5', ' ', '0'):
                lin, ang = motor_test.stop_motion()
                self._log.append(f"[red]STOP[/red] lin={lin:.2f} ang={ang:.2f}")
        except Exception as e:
            self._log.append(f"[red]Motor test error: {e}[/red]")

    def _log_changes(self, context) -> None:
        """Log state and behavior changes."""
        # Log state changes
        current_state = context.state.name
        if self._last_state is not None and current_state != self._last_state:
            if current_state == "EMERGENCY_STOP":
                self._log.append(f"[bold red]State: {current_state}[/bold red]")
            elif current_state == "NAVIGATING":
                self._log.append(f"[green]State: {current_state}[/green]")
            else:
                self._log.append(f"State: {current_state}")
        self._last_state = current_state

        # Log behavior changes
        current_behavior = context.current_behavior
        if self._last_behavior is not None and current_behavior != self._last_behavior:
            self._log.append(f"Behavior: {current_behavior}")
        self._last_behavior = current_behavior

        # Log LIDAR availability changes
        sensors = context.sensors
        has_lidar = sensors and sensors.lidar_ranges and len(sensors.lidar_ranges) > 0
        if has_lidar != self._had_lidar:
            if has_lidar:
                self._log.append("[green]LIDAR data restored[/green]")
            else:
                self._log.append("[bold red]LIDAR data lost![/bold red]")
            self._had_lidar = has_lidar

    def _update_shortcuts(self) -> None:
        """Update shortcuts panel showing available key bindings."""
        current = self.runner.current_behavior or ""

        # Show motor test controls when in motor_test mode
        if current == "motor_test":
            shortcuts = [
                "[bold cyan]M:motor_test[/bold cyan]",
                "[white]8/K[/white]:[dim]fwd[/dim]",
                "[white]2/J[/white]:[dim]back[/dim]",
                "[white]4/H[/white]:[dim]left[/dim]",
                "[white]6/L[/white]:[dim]right[/dim]",
                "[white]5/SPACE[/white]:[dim]stop[/dim]",
                "[white]I[/white]:[dim]idle[/dim]",
                "[white]E[/white]:[dim]emergency[/dim]",
                "[white]Q[/white]:[dim]quit[/dim]",
            ]
        else:
            # Build shortcuts display with clear key indicators
            shortcuts = []
            for key, behavior in self.KEY_MAPPINGS.items():
                if behavior == current:
                    shortcuts.append(f"[bold cyan]{key.upper()}:{behavior}[/bold cyan]")
                else:
                    shortcuts.append(f"[white]{key.upper()}[/white]:[dim]{behavior}[/dim]")

            shortcuts.append("[white]+/-[/white]:[dim]speed[/dim]")
            shortcuts.append("[white]E[/white]:[dim]emergency[/dim]")
            shortcuts.append("[white]Q[/white]:[dim]quit[/dim]")

        text = Text.from_markup("  ".join(shortcuts))
        self._layout["shortcuts"].update(Panel(text, title="Shortcuts", border_style="magenta"))

    def _update_logs(self) -> None:
        """Update unified log panel with verbosity indicator."""
        from nimbus.core.xrce.logger import get_log_file_path, get_verbosity

        verbosity = get_verbosity()
        verbosity_names = {1: "minimal", 2: "normal", 3: "debug"}
        verbosity_name = verbosity_names.get(verbosity, "normal")

        text = Text.from_markup(self._log.get_text())
        log_path = get_log_file_path()
        self._layout["logs"].update(Panel(
            text,
            title=f"Log [dim](-v {verbosity}: {verbosity_name})[/dim]",
            subtitle=f"[dim]{log_path}[/dim]",
            border_style="yellow"
        ))

    def log(self, message: str) -> None:
        """Add a message to the log buffer (public API)."""
        self._log.append(message)

    def _write_claude_state(self, context) -> None:
        """Write state for Claude Code to read."""
        sensors = context.sensors
        state = {
            "timestamp": datetime.now().isoformat(),
            "robot": {
                "state": context.state.name,
                "behavior": context.current_behavior,
                "pose": {
                    "x": round(sensors.pose.x, 3),
                    "y": round(sensors.pose.y, 3),
                    "theta": round(sensors.pose.theta, 3)
                } if sensors else None,
                "velocity": {
                    "linear": round(sensors.velocity.linear, 3),
                    "angular": round(sensors.velocity.angular, 3)
                } if sensors else None,
                "closest_obstacle": round(sensors.closest_obstacle, 3) if sensors and sensors.closest_obstacle else None,
                "target": {
                    "x": context.target.x,
                    "y": context.target.y,
                    "theta": context.target.theta
                } if context.target else None
            },
            "logs": self._log.get_recent(20),
            "available_behaviors": list(self.runner.available_behaviors),
            "api_port": 8080
        }

        state_path = Path.home() / ".nimbus" / "claude_state.json"
        state_path.parent.mkdir(parents=True, exist_ok=True)

        # Atomic write to prevent partial reads
        tmp_path = state_path.with_suffix(".tmp")
        tmp_path.write_text(json.dumps(state, indent=2))
        tmp_path.rename(state_path)
