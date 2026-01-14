"""
Nimbus CLI - Beautiful command-line interface.

Built with Typer and Rich for an elegant terminal experience.
"""

import typer
from typing import Optional
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.progress import Progress, SpinnerColumn, TextColumn
from pathlib import Path

app = typer.Typer(
    name="nimbus",
    help="Nimbus: Lightweight Robot Control Platform",
    add_completion=True,
    rich_markup_mode="rich",
)

console = Console()


@app.command()
def run(
    behavior: str = typer.Option("idle", help="Initial behavior: idle, wander, patrol, goto"),
    api: bool = typer.Option(True, help="Enable REST/WebSocket API"),
    dashboard: bool = typer.Option(True, help="Show live dashboard"),
    config: Optional[Path] = typer.Option(None, help="Path to config file"),
    mock: bool = typer.Option(False, "--mock", help="Use mock node (no ROS2)"),
):
    """
    Start Nimbus robot controller.

    Examples:
        nimbus run --behavior wander
        nimbus run --behavior idle --no-dashboard
    """
    from nimbus.core.runner import create_runner

    # Banner
    console.print(Panel.fit(
        "[bold blue]Nimbus[/bold blue] Robot Controller",
        subtitle="Press Ctrl+C to stop"
    ))

    # Create runner
    runner = create_runner(config_path=str(config) if config else None, mock=mock)

    # Set initial behavior
    if behavior != "idle":
        if not runner.set_behavior(behavior):
            console.print(f"[yellow]Warning:[/yellow] Unknown behavior '{behavior}', using 'idle'")

    console.print(f"[green]Starting with behavior:[/green] {runner.current_behavior}")
    console.print(f"[green]Available behaviors:[/green] {', '.join(runner.available_behaviors)}")

    # Start API server in background if enabled
    api_thread = None
    if api:
        from nimbus.api.server import start_api_background
        api_thread = start_api_background(runner)
        console.print(f"[green]API server:[/green] http://localhost:8080")

    # Run with or without dashboard
    if dashboard:
        from .dashboard import LiveDashboard
        try:
            with LiveDashboard(runner, console) as dash:
                runner.run(on_update=dash.update)
        except Exception as e:
            console.print(f"[red]Dashboard error:[/red] {e}")
            console.print("[yellow]Running without dashboard...[/yellow]")
            runner.run()
    else:
        runner.run()

    console.print("[bold red]Nimbus stopped.[/bold red]")


@app.command()
def status():
    """Show current robot status."""
    import httpx

    try:
        r = httpx.get("http://localhost:8080/api/status", timeout=2.0)
        data = r.json()

        table = Table(title="Robot Status", show_header=True, header_style="bold cyan")
        table.add_column("Property", style="cyan", width=20)
        table.add_column("Value", style="green")

        table.add_row("State", data.get("state", "UNKNOWN"))
        table.add_row("Behavior", data.get("current_behavior", "None"))

        pose = data.get("pose", {})
        table.add_row("Position", f"({pose.get('x', 0):.2f}, {pose.get('y', 0):.2f})")
        table.add_row("Heading", f"{pose.get('theta', 0):.2f} rad")

        table.add_row("Closest Obstacle", f"{data.get('closest_obstacle', 0):.2f} m")

        vel = data.get("velocity", {})
        table.add_row("Velocity", f"{vel.get('linear', 0):.2f} m/s, {vel.get('angular', 0):.2f} rad/s")

        console.print(table)

    except httpx.ConnectError:
        console.print("[red]Error:[/red] Could not connect to Nimbus. Is it running?")
        raise typer.Exit(1)
    except Exception as e:
        console.print(f"[red]Error:[/red] {e}")
        raise typer.Exit(1)


@app.command()
def stop():
    """Emergency stop - immediately halt all motion."""
    import httpx

    try:
        httpx.post("http://localhost:8080/api/stop", timeout=1.0)
        console.print("[bold red]EMERGENCY STOP ACTIVATED[/bold red]")
    except httpx.ConnectError:
        console.print("[yellow]Warning:[/yellow] Could not reach API. Robot may still be moving.")
        console.print("If robot is still moving, physically power it off.")
    except Exception as e:
        console.print(f"[red]Error:[/red] {e}")


@app.command("goto")
def goto_cmd(
    x: float = typer.Argument(..., help="X coordinate (meters)"),
    y: float = typer.Argument(..., help="Y coordinate (meters)"),
    wait: bool = typer.Option(True, help="Wait for navigation to complete"),
):
    """
    Navigate to coordinates.

    Example:
        nimbus goto 2.0 1.5 --wait
    """
    import httpx
    import time

    console.print(f"[green]Navigating to[/green] ({x}, {y})...")

    try:
        r = httpx.post(
            "http://localhost:8080/api/navigate",
            json={"x": x, "y": y},
            timeout=5.0
        )

        if r.status_code == 200:
            console.print("[green]Navigation started[/green]")

            if wait:
                with Progress(
                    SpinnerColumn(),
                    TextColumn("[progress.description]{task.description}"),
                    console=console,
                ) as progress:
                    task = progress.add_task("Navigating...", total=None)

                    while True:
                        status = httpx.get("http://localhost:8080/api/status", timeout=2.0).json()

                        if status["state"] not in ["NAVIGATING", "AVOIDING"]:
                            break

                        progress.update(task, description=f"State: {status['state']}")
                        time.sleep(0.5)

                console.print("[green]Navigation complete[/green]")
        else:
            console.print(f"[red]Failed:[/red] {r.json().get('error', 'Unknown error')}")

    except httpx.ConnectError:
        console.print("[red]Error:[/red] Could not connect to Nimbus. Is it running?")
        raise typer.Exit(1)


@app.command()
def behaviors():
    """List available behaviors."""
    import httpx

    try:
        r = httpx.get("http://localhost:8080/api/behaviors", timeout=2.0)
        data = r.json()

        table = Table(title="Available Behaviors")
        table.add_column("Name", style="cyan")
        table.add_column("Active", style="green")

        status = httpx.get("http://localhost:8080/api/status", timeout=2.0).json()
        current = status.get("current_behavior", "")

        for name in data.get("behaviors", []):
            active = "Yes" if name == current else ""
            table.add_row(name, active)

        console.print(table)

    except httpx.ConnectError:
        console.print("[red]Error:[/red] Could not connect to Nimbus. Is it running?")
        raise typer.Exit(1)


@app.command()
def behavior(
    name: str = typer.Argument(..., help="Behavior name"),
):
    """Set active behavior."""
    import httpx

    try:
        r = httpx.post(f"http://localhost:8080/api/behavior/{name}", timeout=2.0)

        if r.status_code == 200:
            console.print(f"[green]Behavior set to:[/green] {name}")
        else:
            console.print(f"[red]Failed:[/red] {r.json().get('error', 'Unknown error')}")

    except httpx.ConnectError:
        console.print("[red]Error:[/red] Could not connect to Nimbus. Is it running?")
        raise typer.Exit(1)


@app.command()
def explore(
    memory: str = typer.Option("default", "--memory", "-m", help="Memory name to use"),
    new: bool = typer.Option(False, "--new", help="Start with fresh memory"),
    api: bool = typer.Option(True, help="Enable REST/WebSocket API"),
    dashboard: bool = typer.Option(True, help="Show live dashboard"),
    config: Optional[Path] = typer.Option(None, help="Path to config file"),
    mock: bool = typer.Option(False, "--mock", help="Use mock node (no ROS2)"),
):
    """
    Start AI-driven exploration with Ollama.

    The robot will explore autonomously, using Ollama to decide
    where to go next based on LIDAR data.

    Examples:
        nimbus explore                      # Use default memory
        nimbus explore --memory kitchen     # Load/create kitchen memory
        nimbus explore --new living_room    # Start fresh exploration
    """
    from nimbus.core.runner import create_runner
    from nimbus.ai.memory import ExplorationMemory

    # Clear memory if --new
    if new:
        ExplorationMemory.delete_memory(memory)
        console.print(f"[yellow]Starting fresh memory:[/yellow] {memory}")

    # Banner
    console.print(Panel.fit(
        "[bold blue]Nimbus AI Explorer[/bold blue]",
        subtitle="Powered by Ollama | Press Ctrl+C to stop"
    ))

    # Create runner
    runner = create_runner(config_path=str(config) if config else None, mock=mock)

    # Check Ollama availability
    from nimbus.ai.ollama import OllamaClient
    ollama = OllamaClient()
    if ollama.is_available():
        console.print("[green]Ollama:[/green] Connected")
    else:
        console.print("[yellow]Warning:[/yellow] Ollama not available. Will retry during exploration.")

    # Start exploration
    if not runner.explore(memory):
        console.print("[red]Failed to start AI exploration[/red]")
        raise typer.Exit(1)

    console.print(f"[green]Exploring with memory:[/green] {memory}")
    console.print(f"[green]Available behaviors:[/green] {', '.join(runner.available_behaviors)}")

    # Start API server
    api_thread = None
    if api:
        from nimbus.api.server import start_api_background
        api_thread = start_api_background(runner)
        console.print(f"[green]API server:[/green] http://localhost:8080")

    # Run with or without dashboard
    if dashboard:
        from .dashboard import LiveDashboard
        try:
            with LiveDashboard(runner, console) as dash:
                runner.run(on_update=dash.update)
        except Exception as e:
            console.print(f"[red]Dashboard error:[/red] {e}")
            console.print("[yellow]Running without dashboard...[/yellow]")
            runner.run()
    else:
        runner.run()

    # Save memory on exit
    from nimbus.behaviors.ai_explore import AIExploreBehavior
    ai_behavior = runner._behavior_manager.get_behavior("ai_explore")
    if ai_behavior and isinstance(ai_behavior, AIExploreBehavior):
        ai_behavior.memory.save()
        console.print(f"[green]Memory saved:[/green] {memory} ({len(ai_behavior.memory.visited_cells)} cells)")

    console.print("[bold red]Exploration stopped.[/bold red]")


# Memory subcommand group
memory_app = typer.Typer(help="Manage exploration memories")
app.add_typer(memory_app, name="memory")


@memory_app.command("list")
def memory_list():
    """List saved exploration memories."""
    from nimbus.ai.memory import ExplorationMemory

    memories = ExplorationMemory.list_memories()

    if not memories:
        console.print("[yellow]No saved memories found.[/yellow]")
        console.print("Start exploring with: nimbus explore --memory <name>")
        return

    table = Table(title="Exploration Memories")
    table.add_column("Name", style="cyan")
    table.add_column("Cells", style="green")
    table.add_column("Updated", style="dim")

    for mem in memories:
        updated = mem["updated_at"][:19].replace("T", " ") if mem["updated_at"] != "unknown" else "unknown"
        table.add_row(mem["name"], str(mem["cells"]), updated)

    console.print(table)


@memory_app.command("show")
def memory_show(
    name: str = typer.Argument(..., help="Memory name"),
):
    """Show details of a saved memory."""
    from nimbus.ai.memory import ExplorationMemory
    from pathlib import Path

    storage_dir = Path.home() / ".nimbus" / "memories"
    path = storage_dir / f"{name}.json"

    if not path.exists():
        console.print(f"[red]Memory not found:[/red] {name}")
        raise typer.Exit(1)

    memory = ExplorationMemory.load(path)

    table = Table(title=f"Memory: {name}")
    table.add_column("Property", style="cyan")
    table.add_column("Value", style="green")

    table.add_row("Cells Explored", str(len(memory.visited_cells)))
    table.add_row("Path Points", str(len(memory.path_history)))
    table.add_row("Waypoint Decisions", str(len(memory.waypoint_history)))
    table.add_row("Created", memory.created_at[:19].replace("T", " "))
    table.add_row("Updated", memory.updated_at[:19].replace("T", " "))

    console.print(table)

    # Show recent waypoints
    if memory.waypoint_history:
        console.print("\n[bold]Recent Waypoint Decisions:[/bold]")
        for wp in memory.waypoint_history[-5:]:
            status = "[green]✓[/green]" if wp.reached else "[yellow]○[/yellow]"
            console.print(f"  {status} ({wp.x:.1f}, {wp.y:.1f}): {wp.reason[:50]}")


@memory_app.command("delete")
def memory_delete(
    name: str = typer.Argument(..., help="Memory name"),
    force: bool = typer.Option(False, "--force", "-f", help="Delete without confirmation"),
):
    """Delete a saved memory."""
    from nimbus.ai.memory import ExplorationMemory

    if not force:
        confirm = typer.confirm(f"Delete memory '{name}'?")
        if not confirm:
            raise typer.Abort()

    if ExplorationMemory.delete_memory(name):
        console.print(f"[green]Deleted:[/green] {name}")
    else:
        console.print(f"[red]Memory not found:[/red] {name}")
        raise typer.Exit(1)


@app.command()
def test(
    pattern: str = typer.Argument("", help="Test pattern (pytest style)"),
    verbose: bool = typer.Option(False, "-v", "--verbose", help="Verbose output"),
    regression: bool = typer.Option(False, "--regression", help="Run regression tests only"),
    coverage: bool = typer.Option(False, "--cov", help="Run with coverage"),
):
    """
    Run Nimbus test suite.

    Examples:
        nimbus test                    # All tests
        nimbus test test_vfh           # Specific test
        nimbus test --regression       # Regression tests only
    """
    import subprocess
    import sys

    args = ["pytest"]

    if verbose:
        args.append("-v")

    if coverage:
        args.extend(["--cov=nimbus", "--cov-report=term-missing"])

    if regression:
        args.extend(["-m", "regression"])

    if pattern:
        args.extend(["-k", pattern])

    # Add test directory
    test_dir = Path(__file__).parent.parent / "tests"
    args.append(str(test_dir))

    console.print(f"[cyan]Running:[/cyan] {' '.join(args)}")

    result = subprocess.run(args)
    raise typer.Exit(result.returncode)


@app.command()
def agent(
    action: str = typer.Argument(..., help="Action: start, stop, status"),
    transport: Optional[str] = typer.Option(None, help="Transport mode: serial or wifi"),
):
    """
    Manage Micro-ROS agent.

    Examples:
        nimbus agent start                    # Use configured transport
        nimbus agent start --transport wifi   # Force WiFi mode
        nimbus agent stop
        nimbus agent status
    """
    from nimbus.core.agent import MicroROSAgent
    from nimbus.core.config import NimbusConfig

    config = NimbusConfig.load()
    agent_mgr = MicroROSAgent(config.agent)

    if action == "start":
        mode = transport or config.agent.transport
        with console.status(f"[bold green]Starting Micro-ROS agent ({mode})..."):
            success = agent_mgr.start(transport=transport)
        if success:
            console.print(f"[green]Micro-ROS agent started[/green] ({mode} mode)")
            if mode == "wifi":
                from nimbus.core.network import get_local_ip
                agent_ip = config.agent.agent_ip or get_local_ip()
                console.print(f"  Listening on UDP port {config.agent.agent_port}")
                console.print(f"  Agent IP: {agent_ip}")
            else:
                console.print(f"  Device: {config.agent.device}")
        else:
            console.print("[red]Failed to start Micro-ROS agent[/red]")
            raise typer.Exit(1)

    elif action == "stop":
        with console.status("[bold yellow]Stopping Micro-ROS agent..."):
            agent_mgr.stop()
        console.print("[yellow]Micro-ROS agent stopped[/yellow]")

    elif action == "status":
        status = agent_mgr.status()
        if status["running"]:
            console.print(f"[green]Micro-ROS agent is running[/green]")
            console.print(f"  Transport: {status.get('transport', 'unknown')}")
            console.print(f"  Container: {status.get('container_id', 'unknown')[:12]}")
            if status.get("transport") == "wifi":
                console.print(f"  Agent IP: {status.get('agent_ip', 'unknown')}")
                console.print(f"  UDP Port: {status.get('agent_port', 'unknown')}")
            else:
                console.print(f"  Device: {status.get('device', 'unknown')}")
        else:
            console.print("[yellow]Micro-ROS agent is not running[/yellow]")
            console.print(f"  Configured transport: {config.agent.transport}")

    else:
        console.print(f"[red]Unknown action:[/red] {action}")
        console.print("Valid actions: start, stop, status")
        raise typer.Exit(1)


# WiFi subcommand group
wifi_app = typer.Typer(help="WiFi configuration for Yahboom robots")
app.add_typer(wifi_app, name="wifi")


@wifi_app.command("setup")
def wifi_setup(
    ssid: Optional[str] = typer.Option(None, "--ssid", "-s", help="WiFi network name"),
    password: Optional[str] = typer.Option(None, "--password", "-p", help="WiFi password"),
    port: Optional[str] = typer.Option(None, "--port", help="Serial port (auto-detect if not specified)"),
    agent_ip: Optional[str] = typer.Option(None, "--agent-ip", help="Agent IP (auto-detect if not specified)"),
    agent_port: int = typer.Option(8090, "--agent-port", help="Agent UDP port"),
    domain_id: int = typer.Option(20, "--domain-id", help="ROS2 domain ID"),
    no_reboot: bool = typer.Option(False, "--no-reboot", help="Don't reboot after configuration"),
):
    """
    Configure WiFi on Yahboom robot via USB.

    This wizard sends WiFi credentials and agent settings to the
    ESP32 firmware. Connect the robot via USB before running.

    Examples:
        nimbus wifi setup
        nimbus wifi setup --ssid MyNetwork --agent-ip 192.168.1.100
    """
    from nimbus.core.network import find_serial_ports, get_local_ip
    from nimbus.core.robot_config import (
        RobotConfigurator, WiFiCredentials, UDPAgentConfig, TRANSPORT_WIFI_UDP
    )

    console.print(Panel.fit(
        "[bold blue]Nimbus WiFi Setup Wizard[/bold blue]",
        subtitle="Configure robot for wireless operation"
    ))

    # Step 1: Find serial port
    console.print("\n[bold]Step 1:[/bold] Serial Connection")

    if port:
        selected_port = port
    else:
        ports = find_serial_ports()
        if not ports:
            console.print("[red]No serial devices found.[/red]")
            console.print("Please connect the robot via USB and try again.")
            raise typer.Exit(1)

        console.print(f"Found serial ports: {', '.join(ports)}")
        if len(ports) == 1:
            selected_port = ports[0]
            console.print(f"[green]Using:[/green] {selected_port}")
        else:
            selected_port = typer.prompt("Select port", default=ports[0])

    # Step 2: Get WiFi credentials
    console.print("\n[bold]Step 2:[/bold] WiFi Credentials")

    if not ssid:
        ssid = typer.prompt("WiFi network name (SSID)")
    if not password:
        password = typer.prompt("WiFi password", hide_input=True)

    console.print(f"[green]Network:[/green] {ssid}")

    # Step 3: Get agent address
    console.print("\n[bold]Step 3:[/bold] Agent Configuration")

    if not agent_ip:
        detected_ip = get_local_ip()
        use_detected = typer.confirm(f"Use detected IP address ({detected_ip})?", default=True)
        if use_detected:
            agent_ip = detected_ip
        else:
            agent_ip = typer.prompt("Enter agent IP address")

    console.print(f"[green]Agent IP:[/green] {agent_ip}")
    console.print(f"[green]Agent Port:[/green] {agent_port}")
    console.print(f"[green]ROS Domain ID:[/green] {domain_id}")

    # Step 4: Confirm and apply
    console.print("\n[bold]Step 4:[/bold] Apply Configuration")

    if not typer.confirm("Apply this configuration to the robot?", default=True):
        console.print("[yellow]Configuration cancelled.[/yellow]")
        raise typer.Abort()

    try:
        with console.status("[bold green]Connecting to robot..."):
            configurator = RobotConfigurator(selected_port)

        with console.status("[bold green]Sending WiFi configuration..."):
            configurator.set_wifi(WiFiCredentials(ssid, password))

        with console.status("[bold green]Sending agent configuration..."):
            configurator.set_udp_agent(UDPAgentConfig(agent_ip, agent_port))
            configurator.set_transport(TRANSPORT_WIFI_UDP)
            configurator.set_ros_domain_id(domain_id)

        if not no_reboot:
            with console.status("[bold green]Rebooting robot..."):
                configurator.reboot()

        configurator.close()

        console.print("\n[bold green]WiFi configuration complete![/bold green]")
        console.print("\nNext steps:")
        console.print("  1. Disconnect the USB cable")
        console.print("  2. Power cycle the robot")
        console.print("  3. Wait 10-15 seconds for WiFi connection")
        console.print(f"  4. Run: [cyan]nimbus agent start --transport wifi[/cyan]")

    except Exception as e:
        console.print(f"\n[red]Configuration failed:[/red] {e}")
        console.print("\nTroubleshooting:")
        console.print("  - Check USB cable connection")
        console.print("  - Ensure robot is powered on")
        console.print("  - Try a different USB port")
        raise typer.Exit(1)


@wifi_app.command("status")
def wifi_status(
    port: Optional[str] = typer.Option(None, "--port", help="Serial port"),
):
    """
    Show current WiFi configuration on the robot.

    Robot must be connected via USB.
    """
    from nimbus.core.network import find_serial_ports
    from nimbus.core.robot_config import RobotConfigurator

    # Find port
    if not port:
        ports = find_serial_ports()
        if not ports:
            console.print("[red]No serial devices found.[/red]")
            raise typer.Exit(1)
        port = ports[0]

    try:
        with console.status("[bold green]Reading robot configuration..."):
            configurator = RobotConfigurator(port)
            info = configurator.read_config()
            configurator.close()

        table = Table(title="Robot Configuration")
        table.add_column("Property", style="cyan")
        table.add_column("Value", style="green")

        table.add_row("Firmware Version", info.firmware_version or "unknown")
        table.add_row("WiFi SSID", info.wifi_ssid or "not configured")
        table.add_row("Agent IP", info.agent_ip or "not configured")
        table.add_row("Agent Port", str(info.agent_port) if info.agent_port else "not configured")
        table.add_row("Transport Mode", info.car_type or "unknown")
        table.add_row("ROS Domain ID", str(info.ros_domain_id) if info.ros_domain_id is not None else "unknown")

        console.print(table)

    except Exception as e:
        console.print(f"[red]Failed to read configuration:[/red] {e}")
        raise typer.Exit(1)


@wifi_app.command("test")
def wifi_test(
    timeout: float = typer.Option(5.0, "--timeout", "-t", help="Test timeout in seconds"),
):
    """
    Test WiFi connectivity to robot.

    Checks if the Micro-ROS agent can receive connections
    on the configured UDP port.
    """
    from nimbus.core.config import NimbusConfig
    from nimbus.core.network import get_local_ip

    config = NimbusConfig.load()
    agent_ip = config.agent.agent_ip or get_local_ip()
    agent_port = config.agent.agent_port

    console.print(f"Testing connection to {agent_ip}:{agent_port}...")

    # Check if agent is running
    from nimbus.core.agent import MicroROSAgent
    agent_mgr = MicroROSAgent(config.agent)

    if agent_mgr.is_running():
        console.print("[green]Micro-ROS agent is running[/green]")

        # Try to check ROS2 topics
        import subprocess
        try:
            result = subprocess.run(
                ["ros2", "topic", "list"],
                capture_output=True,
                text=True,
                timeout=timeout
            )
            if result.returncode == 0:
                topics = [t for t in result.stdout.strip().split("\n") if t]
                if "/scan" in topics or "/odom_raw" in topics:
                    console.print("[green]Robot topics detected![/green]")
                    for topic in ["/scan", "/odom_raw", "/cmd_vel"]:
                        status = "[green]✓[/green]" if topic in topics else "[yellow]○[/yellow]"
                        console.print(f"  {status} {topic}")
                else:
                    console.print("[yellow]Agent running but robot topics not yet visible[/yellow]")
                    console.print("The robot may still be connecting to WiFi.")
            else:
                console.print("[yellow]Could not list ROS2 topics[/yellow]")
        except FileNotFoundError:
            console.print("[yellow]ROS2 not found - cannot verify topics[/yellow]")
        except subprocess.TimeoutExpired:
            console.print("[yellow]Timeout waiting for ROS2 topics[/yellow]")
    else:
        console.print("[yellow]Micro-ROS agent is not running[/yellow]")
        console.print(f"Start it with: [cyan]nimbus agent start --transport wifi[/cyan]")


@app.command()
def version():
    """Show Nimbus version."""
    from nimbus import __version__
    console.print(f"Nimbus version [cyan]{__version__}[/cyan]")


def main():
    """Entry point."""
    app()


if __name__ == "__main__":
    main()
