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
    mock: bool = typer.Option(False, "--mock", help="Use mock node (no hardware)"),
    sim: bool = typer.Option(False, "--sim", help="Run in simulation mode (virtual world)"),
    map_file: Optional[Path] = typer.Option(None, "--map", help="Path to map file for simulation"),
    spawn: Optional[str] = typer.Option(None, "--spawn", help="Spawn position as x,y,theta (e.g., '0.5,0.5,0')"),
    discover: bool = typer.Option(False, "--discover", help="Auto-discover ESP32 on network"),
    verbosity: int = typer.Option(1, "-v", "--verbosity", min=1, max=3, help="Log verbosity: 1=minimal, 2=normal, 3=debug"),
):
    """
    Start Nimbus robot controller.

    Examples:
        nimbus run --behavior wander
        nimbus run --behavior idle --no-dashboard
        nimbus run --discover --behavior wander    # Auto-discover ESP32
        nimbus run -v 3                            # Debug verbosity
        nimbus run --sim --behavior wander         # Simulation mode
        nimbus run --sim --map path/to/map.txt     # Custom map
    """
    # Set log verbosity
    from nimbus.core.xrce.logger import set_verbosity
    set_verbosity(verbosity)

    from nimbus.core.runner import create_runner

    # Auto-discover ESP32 if requested
    if discover:
        from nimbus.core.network import scan_for_esp32, get_local_subnet

        subnet_info = get_local_subnet()
        if not subnet_info:
            console.print("[red]Could not detect local network for auto-discovery.[/red]")
            raise typer.Exit(1)

        network, prefix_len, _, _ = subnet_info
        console.print(f"Scanning network [cyan]{network}/{prefix_len}[/cyan] for ESP32...")

        with console.status("[cyan]Discovering devices...") as status:
            devices = scan_for_esp32(timeout=3.0)

        if not devices:
            console.print("[yellow]No devices found yet. The ESP32 will connect when it starts.[/yellow]")
        elif len(devices) == 1:
            console.print(f"[green]Found potential ESP32:[/green] {devices[0]['ip']}")
        else:
            console.print(f"[green]Found {len(devices)} devices on network[/green]")
            for dev in devices:
                console.print(f"  - {dev['ip']} ({dev['latency_ms']:.0f}ms)")

    # Parse spawn position if provided
    spawn_x, spawn_y, spawn_theta = None, None, None
    if spawn:
        try:
            parts = [float(p.strip()) for p in spawn.split(",")]
            if len(parts) >= 2:
                spawn_x, spawn_y = parts[0], parts[1]
            if len(parts) >= 3:
                spawn_theta = parts[2]
        except ValueError:
            console.print(f"[red]Invalid spawn format:[/red] {spawn}")
            console.print("Expected format: x,y or x,y,theta (e.g., '0.5,0.5,0')")
            raise typer.Exit(1)

    # Banner
    console.print(Panel.fit(
        "[bold blue]Nimbus[/bold blue] Robot Controller",
        subtitle="Press Ctrl+C to stop"
    ))

    if sim:
        console.print("[green]Mode:[/green] Simulation")
        if map_file:
            console.print(f"[green]Map:[/green] {map_file}")
        else:
            console.print("[green]Map:[/green] empty_room (default)")
    elif not mock:
        console.print("[green]Mode:[/green] XRCE Agent (waiting for ESP32)")

    # Create runner
    from nimbus.core.config import NimbusConfig

    # Load config and apply simulation settings
    nimbus_config = NimbusConfig.load(config) if config else NimbusConfig.load()
    if sim:
        nimbus_config.simulation.enabled = True
        if map_file:
            nimbus_config.simulation.map_file = str(map_file)
        if spawn_x is not None:
            nimbus_config.simulation.spawn_x = spawn_x
        if spawn_y is not None:
            nimbus_config.simulation.spawn_y = spawn_y
        if spawn_theta is not None:
            nimbus_config.simulation.spawn_theta = spawn_theta

    runner = create_runner(
        config_path=str(config) if config else None,
        mock=mock,
        sim=sim,
    )
    runner.config = nimbus_config  # Apply updated config

    # Set initial behavior
    if behavior != "idle":
        if not runner.set_behavior(behavior):
            console.print(f"[yellow]Warning:[/yellow] Unknown behavior '{behavior}', using 'idle'")

    console.print(f"[green]Starting with behavior:[/green] {runner.current_behavior}")
    console.print(f"[green]Available behaviors:[/green] {', '.join(runner.available_behaviors)}")

    # Auto-enable API when dashboard is active (for Claude integration)
    if dashboard:
        api = True

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
    mock: bool = typer.Option(False, "--mock", help="Use mock node (no hardware)"),
    verbosity: int = typer.Option(1, "-v", "--verbosity", min=1, max=3, help="Log verbosity: 1=minimal, 2=normal, 3=debug"),
):
    """
    Start AI-driven exploration with Ollama.

    The robot will explore autonomously, using Ollama to decide
    where to go next based on LIDAR data.

    Examples:
        nimbus explore                              # Use default memory
        nimbus explore --memory kitchen             # Load/create kitchen memory
        nimbus explore --new living_room            # Start fresh exploration
        nimbus explore -v 3                         # Debug verbosity
    """
    # Set log verbosity
    from nimbus.core.xrce.logger import set_verbosity
    set_verbosity(verbosity)

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
    runner = create_runner(
        config_path=str(config) if config else None,
        mock=mock,
    )

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


# WiFi subcommand group
wifi_app = typer.Typer(help="WiFi configuration for Yahboom robots")
app.add_typer(wifi_app, name="wifi")


@wifi_app.command("setup")
def wifi_setup(
    ssid: Optional[str] = typer.Option(None, "--ssid", "-s", help="WiFi network name"),
    password: Optional[str] = typer.Option(None, "--password", "-p", help="WiFi password"),
    port: Optional[str] = typer.Option(None, "--port", help="Serial port (auto-detect if not specified)"),
    agent_ip: Optional[str] = typer.Option(None, "--agent-ip", help="Agent IP (auto-detect if not specified)"),
    agent_hostname: Optional[str] = typer.Option(None, "--agent-hostname", help="Agent hostname (e.g., myhost.local)"),
    agent_port: int = typer.Option(8090, "--agent-port", help="Agent UDP port"),
    domain_id: int = typer.Option(20, "--domain-id", help="ROS2 domain ID"),
    no_reboot: bool = typer.Option(False, "--no-reboot", help="Don't reboot after configuration"),
    skip_dialout_check: bool = typer.Option(False, "--skip-dialout-check", help="Skip dialout group check"),
):
    """
    Configure WiFi on Yahboom robot via USB.

    This wizard sends WiFi credentials and agent settings to the
    ESP32 firmware. Connect the robot via USB before running.

    Examples:
        nimbus wifi setup
        nimbus wifi setup --ssid MyNetwork --agent-ip 192.168.1.100
        nimbus wifi setup --agent-hostname mycomputer.local
    """
    from nimbus.core.network import (
        find_serial_ports, get_local_ip, check_dialout_group,
        find_serial_ports_delta, resolve_hostname, validate_ip_address
    )
    from nimbus.core.robot_config import (
        RobotConfigurator, WiFiCredentials, UDPAgentConfig, TRANSPORT_WIFI_UDP
    )
    import socket

    console.print(Panel.fit(
        "[bold blue]Nimbus WiFi Setup Wizard[/bold blue]",
        subtitle="Configure robot for wireless operation"
    ))

    # =========================================================================
    # Step 1: Prerequisites Check
    # =========================================================================
    console.print("\n[bold cyan]Step 1/6:[/bold cyan] Prerequisites Check")

    if not skip_dialout_check:
        in_dialout, username = check_dialout_group()
        if in_dialout:
            console.print(f"[green]✓[/green] User '{username}' is in dialout group")
        else:
            console.print(f"[yellow]⚠[/yellow] User '{username}' is NOT in dialout group")
            console.print("  Serial port access may fail without dialout membership.")
            console.print(f"  To fix: [cyan]sudo usermod -aG dialout {username}[/cyan]")
            console.print("  Then log out and back in for changes to take effect.")
            if not typer.confirm("Continue anyway?", default=True):
                raise typer.Exit(1)
    else:
        console.print("[dim]Dialout group check skipped[/dim]")

    # =========================================================================
    # Step 2: USB Connection with Delta Detection
    # =========================================================================
    console.print("\n[bold cyan]Step 2/6:[/bold cyan] USB Connection")

    if port:
        selected_port = port
        console.print(f"[green]Using specified port:[/green] {selected_port}")
    else:
        # Capture ports BEFORE connecting
        ports_before = find_serial_ports()
        if ports_before:
            console.print(f"[dim]Existing serial ports: {', '.join(ports_before)}[/dim]")
        else:
            console.print("[dim]No existing serial ports detected[/dim]")

        console.print("\n[yellow]→[/yellow] Connect the robot via USB cable now...")
        typer.prompt("Press Enter when connected", default="", show_default=False)

        # Capture ports AFTER connecting
        new_ports, all_ports = find_serial_ports_delta(ports_before)

        if not all_ports:
            console.print("[red]No serial devices found.[/red]")
            console.print("\nTroubleshooting:")
            console.print("  - Check USB cable is firmly connected")
            console.print("  - Ensure robot is powered on")
            console.print("  - Try a different USB port")
            if not skip_dialout_check:
                in_dialout, username = check_dialout_group()
                if not in_dialout:
                    console.print(f"  - Add user to dialout group: sudo usermod -aG dialout {username}")
            raise typer.Exit(1)

        if new_ports:
            console.print(f"[green]✓[/green] New device detected: [bold]{', '.join(new_ports)}[/bold]")
            selected_port = new_ports[0]
        elif len(all_ports) == 1:
            selected_port = all_ports[0]
            console.print(f"[green]Using:[/green] {selected_port}")
        else:
            console.print(f"Available ports: {', '.join(all_ports)}")
            selected_port = typer.prompt("Select port", default=all_ports[0])

    console.print(f"[green]Selected port:[/green] {selected_port}")

    # =========================================================================
    # Step 3: WiFi Credentials with Validation
    # =========================================================================
    console.print("\n[bold cyan]Step 3/6:[/bold cyan] WiFi Credentials")

    if not ssid:
        while True:
            ssid = typer.prompt("WiFi network name (SSID)")
            if len(ssid) > 32:
                console.print("[red]SSID too long (max 32 characters)[/red]")
                continue
            if len(ssid) == 0:
                console.print("[red]SSID cannot be empty[/red]")
                continue
            break

    if not password:
        password = typer.prompt("WiFi password", hide_input=True)
        if len(password) < 8:
            console.print("[yellow]⚠[/yellow] Password is less than 8 characters")
            console.print("  Most WPA2 networks require 8+ character passwords.")
            if not typer.confirm("Continue with this password?", default=True):
                password = typer.prompt("WiFi password", hide_input=True)

    console.print(f"[green]Network:[/green] {ssid}")
    console.print(f"[green]Password:[/green] {'*' * len(password)}")

    # =========================================================================
    # Step 4: Agent Connection Method
    # =========================================================================
    console.print("\n[bold cyan]Step 4/6:[/bold cyan] Agent Configuration")

    resolved_ip = None

    if agent_hostname:
        # User provided hostname via CLI
        console.print(f"Using hostname: {agent_hostname}")
        try:
            resolved_ip = resolve_hostname(agent_hostname)
            console.print(f"[green]✓[/green] Resolved to: {resolved_ip}")
        except ValueError as e:
            console.print(f"[red]Cannot resolve hostname:[/red] {e}")
            raise typer.Exit(1)
    elif agent_ip:
        # User provided IP via CLI
        if not validate_ip_address(agent_ip):
            console.print(f"[red]Invalid IP address format:[/red] {agent_ip}")
            raise typer.Exit(1)
        resolved_ip = agent_ip
    else:
        # Interactive selection
        detected_ip = get_local_ip()
        local_hostname = socket.gethostname()

        console.print("How should the robot connect to this computer?")
        console.print(f"  [cyan]1)[/cyan] mDNS hostname: [bold]{local_hostname}.local[/bold] (recommended)")
        console.print(f"  [cyan]2)[/cyan] Fixed IP address: [bold]{detected_ip}[/bold]")
        console.print(f"  [cyan]3)[/cyan] Custom address")

        choice = typer.prompt("Select option", default="1")

        if choice == "1":
            hostname = f"{local_hostname}.local"
            console.print(f"Using mDNS hostname: {hostname}")
            try:
                resolved_ip = resolve_hostname(hostname)
                console.print(f"[green]✓[/green] Resolved to: {resolved_ip}")
            except ValueError as e:
                console.print(f"[yellow]⚠[/yellow] Could not resolve mDNS hostname: {e}")
                console.print("Using detected IP address instead.")
                resolved_ip = detected_ip
        elif choice == "2":
            resolved_ip = detected_ip
        else:
            custom_addr = typer.prompt("Enter IP address or hostname")
            if validate_ip_address(custom_addr):
                resolved_ip = custom_addr
            else:
                try:
                    resolved_ip = resolve_hostname(custom_addr)
                    console.print(f"[green]✓[/green] Resolved to: {resolved_ip}")
                except ValueError as e:
                    console.print(f"[red]Cannot resolve address:[/red] {e}")
                    raise typer.Exit(1)

    console.print(f"[green]Agent IP:[/green] {resolved_ip}")
    console.print(f"[green]Agent Port:[/green] {agent_port}")
    console.print(f"[green]ROS Domain ID:[/green] {domain_id}")

    # =========================================================================
    # Step 5: Configuration Summary
    # =========================================================================
    console.print("\n[bold cyan]Step 5/6:[/bold cyan] Configuration Summary")

    summary_table = Table(show_header=False, box=None, padding=(0, 2))
    summary_table.add_column("Setting", style="cyan")
    summary_table.add_column("Value", style="green")
    summary_table.add_row("Serial Port", selected_port)
    summary_table.add_row("WiFi SSID", ssid)
    summary_table.add_row("WiFi Password", "*" * len(password))
    summary_table.add_row("Agent IP", resolved_ip)
    summary_table.add_row("Agent Port", str(agent_port))
    summary_table.add_row("ROS Domain ID", str(domain_id))
    console.print(summary_table)

    console.print()
    if not typer.confirm("Apply this configuration to the robot?", default=True):
        if typer.confirm("Restart the wizard?", default=True):
            # Recursively call with no arguments to restart
            console.print("\n" + "=" * 50 + "\n")
            wifi_setup()
            return
        console.print("[yellow]Configuration cancelled.[/yellow]")
        raise typer.Abort()

    # =========================================================================
    # Step 6: Apply Configuration with Retry
    # =========================================================================
    console.print("\n[bold cyan]Step 6/6:[/bold cyan] Apply Configuration")

    max_retries = 3
    for attempt in range(1, max_retries + 1):
        try:
            with console.status("[bold green]Connecting to robot..."):
                configurator = RobotConfigurator(selected_port)

            with console.status("[bold green]Sending WiFi configuration..."):
                configurator.set_wifi(WiFiCredentials(ssid, password))

            with console.status("[bold green]Sending agent configuration..."):
                configurator.set_udp_agent(UDPAgentConfig(resolved_ip, agent_port))
                configurator.set_transport(TRANSPORT_WIFI_UDP)
                configurator.set_ros_domain_id(domain_id)

            if not no_reboot:
                with console.status("[bold green]Rebooting robot..."):
                    configurator.reboot()

            configurator.close()

            console.print("\n[bold green]✓ WiFi configuration complete![/bold green]")
            console.print("\nNext steps:")
            console.print("  1. Disconnect the USB cable")
            console.print("  2. Power cycle the robot")
            console.print("  3. Wait 10-15 seconds for WiFi connection")
            console.print("  4. Run: [cyan]nimbus run --behavior wander[/cyan]")
            return

        except Exception as e:
            console.print(f"\n[red]Configuration failed (attempt {attempt}/{max_retries}):[/red] {e}")

            if attempt < max_retries:
                console.print("\nTroubleshooting tips:")
                console.print("  - Ensure USB cable is firmly connected")
                console.print("  - Check robot is powered on")
                console.print("  - Try unplugging and reconnecting USB")
                if typer.confirm("Retry?", default=True):
                    continue
                else:
                    break
            else:
                console.print("\n[red]All retry attempts failed.[/red]")

    console.print("\nTroubleshooting:")
    console.print("  - Check USB cable connection")
    console.print("  - Ensure robot is powered on")
    console.print("  - Try a different USB port")
    console.print("  - Try a different USB cable")
    in_dialout, username = check_dialout_group()
    if not in_dialout:
        console.print(f"  - Add user to dialout: [cyan]sudo usermod -aG dialout {username}[/cyan]")
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


@wifi_app.command("discover")
def wifi_discover(
    timeout: float = typer.Option(3.0, "-t", "--timeout", help="Scan timeout per host in seconds"),
    probe: bool = typer.Option(False, "--probe", "-p", help="Listen for robot traffic (power cycle robot while listening)"),
):
    """
    Discover ESP32 robots on the local network.

    Scans the local subnet for reachable devices by performing a ping sweep.
    Excludes the gateway and local machine from results.

    Use --probe to listen for XRCE-DDS traffic on port 8090. This requires
    power cycling the robot while the command is listening, as the robot
    only sends connection attempts at boot.

    Examples:
        nimbus wifi discover
        nimbus wifi discover --timeout 5
        nimbus wifi discover --probe --timeout 30   # Listen while rebooting robot
    """
    from nimbus.core.network import scan_for_esp32, get_local_subnet, listen_for_robot

    # If probe flag is set, listen for robot traffic
    if probe:
        import socket
        import select
        import time as time_module

        console.print("[cyan]Listening for robot on UDP port 8090...[/cyan]")
        console.print()
        console.print("[bold yellow]>>> Power cycle the robot NOW <<<[/bold yellow]")
        console.print()

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", 8090))
            sock.setblocking(False)
        except OSError as e:
            if e.errno == 98 or "Address already in use" in str(e):
                console.print("[red]Port 8090 is already in use.[/red]")
                console.print("Something else is using this port. Check with:")
                console.print("  [cyan]sudo lsof -i :8090[/cyan]")
            else:
                console.print(f"[red]Could not bind to port 8090:[/red] {e}")
            raise typer.Exit(1)

        start = time_module.time()
        robot_ip = None
        packet_count = 0

        with console.status("") as status:
            while time_module.time() - start < timeout:
                remaining = int(timeout - (time_module.time() - start))
                status.update(f"[cyan]Waiting for packets... {remaining}s remaining[/cyan]")

                readable, _, _ = select.select([sock], [], [], 0.5)
                if readable:
                    try:
                        data, addr = sock.recvfrom(65535)
                        if len(data) > 4:
                            robot_ip = addr[0]
                            packet_count += 1
                            if packet_count >= 3:
                                break
                    except Exception:
                        pass

        sock.close()

        if robot_ip:
            console.print(f"\n[bold green]Robot found![/bold green]")
            console.print(f"  IP Address: [cyan]{robot_ip}[/cyan]")
            console.print(f"  Packets received: {packet_count}")
            console.print(f"\nTo connect:")
            console.print(f"  [cyan]nimbus run --behavior wander[/cyan]")
        else:
            console.print("\n[yellow]No robot traffic detected.[/yellow]")
            console.print("\nMake sure to:")
            console.print("  1. Power cycle the robot AFTER starting this command")
            console.print("  2. Robot is configured with this computer's IP as agent")
            console.print("  3. Try longer timeout: nimbus wifi discover --probe --timeout 30")
        return

    # Get subnet info for display
    subnet_info = get_local_subnet()
    if not subnet_info:
        console.print("[red]Could not detect local network.[/red]")
        console.print("Ensure you're connected to a network and try again.")
        raise typer.Exit(1)

    network, prefix_len, gateway, local_ip = subnet_info
    subnet_str = f"{network}/{prefix_len}"

    console.print(f"Scanning network [cyan]{subnet_str}[/cyan]...")
    console.print(f"[dim]Local IP: {local_ip}, Gateway: {gateway or 'unknown'}[/dim]\n")

    # Perform scan with spinner
    with console.status("[cyan]Scanning for devices...") as status:
        devices = scan_for_esp32(timeout=timeout)

    if not devices:
        console.print("[yellow]No devices found on the network.[/yellow]")
        console.print("\nTroubleshooting:")
        console.print("  1. Ensure the robot is powered on")
        console.print("  2. Check the robot is on the same WiFi network")
        console.print("  3. Try increasing timeout: nimbus wifi discover --timeout 5")
        raise typer.Exit(1)

    # Display results in a table
    table = Table(title="Discovered Devices")
    table.add_column("IP Address", style="cyan")
    table.add_column("Latency", style="green")
    table.add_column("Status", style="dim")

    for device in devices:
        latency_str = f"{device['latency_ms']:.0f}ms"
        table.add_row(device["ip"], latency_str, "Reachable")

    console.print(table)

    # Suggest command based on results
    console.print()
    if len(devices) == 1:
        console.print(f"Found 1 device. To connect:")
        console.print(f"  [cyan]nimbus run --behavior wander[/cyan]")
    else:
        console.print(f"Found {len(devices)} devices. To identify the robot:")
        console.print(f"  [cyan]nimbus wifi discover --probe[/cyan]")


@app.command()
def motor(
    action: str = typer.Argument(..., help="Action: forward, backward, left, right, stop, or velocity"),
    speed: float = typer.Option(0.1, "--speed", "-s", help="Speed for preset actions (m/s or rad/s)"),
    linear: Optional[float] = typer.Option(None, "--linear", "-l", help="Linear velocity for 'velocity' action"),
    angular: Optional[float] = typer.Option(None, "--angular", "-a", help="Angular velocity for 'velocity' action"),
):
    """
    Control motors directly (requires nimbus running in motor_test mode).

    Preset actions use --speed for magnitude. The 'velocity' action
    uses --linear and --angular for direct control.

    Examples:
        nimbus motor forward --speed 0.15     # Move forward
        nimbus motor left --speed 0.5         # Turn left
        nimbus motor stop                     # Stop motors
        nimbus motor velocity -l 0.1 -a 0.2   # Custom velocity
    """
    import httpx

    # Map preset actions to velocities
    presets = {
        "forward": (speed, 0.0),
        "backward": (-speed, 0.0),
        "left": (0.0, speed),
        "right": (0.0, -speed),
        "stop": (0.0, 0.0),
    }

    if action in presets:
        lin, ang = presets[action]
    elif action == "velocity":
        lin = linear if linear is not None else 0.0
        ang = angular if angular is not None else 0.0
    else:
        console.print(f"[red]Unknown action:[/red] {action}")
        console.print("Valid actions: forward, backward, left, right, stop, velocity")
        raise typer.Exit(1)

    try:
        # First check if we're in motor_test mode
        status_r = httpx.get("http://localhost:8080/api/status", timeout=2.0)
        current_behavior = status_r.json().get("current_behavior", "")

        if current_behavior != "motor_test":
            # Try to switch to motor_test mode
            switch_r = httpx.post("http://localhost:8080/api/behavior/motor_test", timeout=2.0)
            if switch_r.status_code != 200:
                console.print("[yellow]Warning:[/yellow] Could not switch to motor_test mode")
                console.print("Use 'nimbus behavior motor_test' first, or dashboard 'M' key")
                raise typer.Exit(1)
            console.print("[green]Switched to motor_test mode[/green]")

        # Send velocity command
        r = httpx.post(
            f"http://localhost:8080/api/motor_test/velocity?linear={lin}&angular={ang}",
            timeout=2.0
        )

        if r.status_code == 200:
            if action == "stop":
                console.print("[red]Motors stopped[/red]")
            else:
                console.print(f"[green]Motor command sent:[/green] linear={lin:.3f}, angular={ang:.3f}")
        else:
            console.print(f"[red]Failed:[/red] {r.json().get('detail', 'Unknown error')}")
            raise typer.Exit(1)

    except httpx.ConnectError:
        console.print("[red]Error:[/red] Could not connect to Nimbus. Is it running?")
        console.print("Start with: [cyan]nimbus run --behavior motor_test[/cyan]")
        raise typer.Exit(1)


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
