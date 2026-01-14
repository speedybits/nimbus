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


class LiveDashboard:
    """
    Real-time terminal dashboard using Rich.

    Layout:
    +------------------+------------------+
    |     SENSORS      |      STATUS      |
    +------------------+------------------+
    |           LIDAR VISUALIZATION       |
    +--------------------------------------+
    """

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

    def _build_layout(self) -> Layout:
        """Build the dashboard layout."""
        layout = Layout()

        layout.split_column(
            Layout(name="header", size=3),
            Layout(name="top", size=10),
            Layout(name="lidar", size=15),
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
        return self

    def __exit__(self, *args):
        self._live.__exit__(*args)

    def update(self, context) -> None:
        """Update dashboard with current robot context."""
        self._update_sensors(context)
        self._update_status(context)
        self._update_lidar(context)

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
        if not sensors or not sensors.lidar_ranges:
            self._layout["lidar"].update(Panel("No LIDAR data", title="LIDAR View"))
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

        # Plot obstacles (sample every 5 degrees)
        for i in range(0, 360, 5):
            if i < len(ranges):
                distance = ranges[i]
                if distance < max_display_range and distance > 0.05:
                    # Convert polar to cartesian
                    angle_rad = math.radians(i - 90)  # 0 deg = forward = up
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

        # Mark robot position
        canvas[center_y][center_x] = 'R'

        # Mark forward direction
        if center_y > 0:
            canvas[center_y - 1][center_x] = '^'

        # Build output string with distance info
        lines = [''.join(row) for row in canvas]
        lines.append(f"  Closest: {closest:.2f}m  |  Scale: 1 char = {1/scale:.2f}m")

        return '\n'.join(lines)
