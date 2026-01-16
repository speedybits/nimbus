"""
Live terminal dashboard for Nimbus.

Uses Rich for beautiful, real-time visualization.
"""

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
    """Circular buffer for log messages."""

    def __init__(self, max_lines: int = 8):
        self.lines = deque(maxlen=max_lines)

    def append(self, message: str) -> None:
        """Add a timestamped message to the log."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.lines.append(f"[dim]{timestamp}[/dim] {message}")

    def get_text(self) -> str:
        """Get all log lines as a single string."""
        if not self.lines:
            return "[dim]No events yet[/dim]"
        return "\n".join(self.lines)


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
    |           LIDAR VISUALIZATION       |
    +--------------------------------------+
    |             SHORTCUTS                |
    +--------------------------------------+
    |              LOG OUTPUT              |
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
        self._log = LogBuffer(max_lines=6)
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
            Layout(name="lidar", size=12),
            Layout(name="shortcuts", size=3),
            Layout(name="logs", size=8),
        )

        layout["top"].split_row(
            Layout(name="sensors"),
            Layout(name="status"),
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
        return self

    def __exit__(self, *args):
        self._keyboard.stop()
        self._live.__exit__(*args)

    def update(self, context) -> None:
        """Update dashboard with current robot context."""
        # Process keyboard input
        self._process_keys()

        # Track state/behavior changes for logging
        self._log_changes(context)

        # Update all panels
        self._update_sensors(context)
        self._update_status(context)
        self._update_lidar(context)
        self._update_shortcuts()
        self._update_logs()

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
            table.add_row("Linear Vel:", f"{sensors.velocity.linear:.2f} m/s")
            table.add_row("Angular Vel:", f"{sensors.velocity.angular:.2f} rad/s")
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

        self._layout["status"].update(Panel(table, title="Status", border_style="blue"))

    def _update_lidar(self, context) -> None:
        """Update LIDAR visualization."""
        sensors = context.sensors
        if not sensors or not sensors.lidar_ranges or len(sensors.lidar_ranges) == 0:
            error_msg = Text("⚠ NO LIDAR DATA - Robot cannot navigate!", style="bold red", justify="center")
            self._layout["lidar"].update(Panel(error_msg, title="LIDAR View", border_style="red"))
            return

        # ASCII LIDAR visualization
        viz = self._render_lidar_ascii(sensors.lidar_ranges, sensors.closest_obstacle)
        self._layout["lidar"].update(Panel(viz, title="LIDAR View", border_style="green"))

    def _render_lidar_ascii(self, ranges: tuple, closest: float) -> str:
        """
        Render LIDAR data as ASCII polar plot.

        Uses Unicode block characters for a cleaner look.
        """
        width = 60
        height = 12
        canvas = [[' '] * width for _ in range(height)]

        center_x = width // 2
        center_y = height // 2

        # Draw reference circle
        max_display_range = 2.0  # meters
        scale = min(width // 2 - 2, height - 2) / max_display_range

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

        # Mark robot position (draw first so obstacles can overwrite if very close)
        canvas[center_y][center_x] = 'R'

        # Mark forward direction
        if center_y > 0:
            canvas[center_y - 1][center_x] = '^'

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

        # Build output string with distance info
        lines = [''.join(row) for row in canvas]
        lines.append(f"  Closest: {closest:.2f}m  |  Scale: 1 char = {1/scale:.2f}m")

        return '\n'.join(lines)

    def _process_keys(self) -> None:
        """Process any pending keypresses."""
        while True:
            key = self._keyboard.get_key()
            if key is None:
                break

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

        # Build shortcuts display with clear key indicators
        shortcuts = []
        for key, behavior in self.KEY_MAPPINGS.items():
            if behavior == current:
                shortcuts.append(f"[bold cyan]{key.upper()}:{behavior}[/bold cyan]")
            else:
                shortcuts.append(f"[white]{key.upper()}[/white]:[dim]{behavior}[/dim]")

        shortcuts.append("[white]E[/white]:[dim]emergency[/dim]")
        shortcuts.append("[white]Q[/white]:[dim]quit[/dim]")

        text = Text.from_markup("  ".join(shortcuts))
        self._layout["shortcuts"].update(Panel(text, title="Shortcuts", border_style="magenta"))

    def _update_logs(self) -> None:
        """Update log output panel."""
        text = Text.from_markup(self._log.get_text())
        self._layout["logs"].update(Panel(text, title="Log", border_style="yellow"))

    def log(self, message: str) -> None:
        """Add a message to the log buffer (public API)."""
        self._log.append(message)
