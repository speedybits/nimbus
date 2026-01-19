"""Communication assertions with rate limiting and dashboard integration.

This module provides assertion helpers that detect error conditions during
robot communication and route them to the appropriate display:
- Critical errors → Dashboard Log pane (user-visible)
- Protocol details → XRCE Debug pane

Features:
- Rate limiting to prevent spam from repeated failures
- Suppression counting to show missed events
- Dual-pane routing for different severity levels
"""

import time
from typing import Optional, Callable

# Dashboard log callback (routes to Log pane)
_dashboard_log: Optional[Callable[[str], None]] = None

# Rate limiting state
_last_fired: dict[str, float] = {}
_fire_counts: dict[str, int] = {}


def set_dashboard_log(callback: Optional[Callable[[str], None]]) -> None:
    """
    Set the callback for dashboard Log pane messages.

    Args:
        callback: Function that takes a log message string, or None to disable
    """
    global _dashboard_log
    _dashboard_log = callback


def assert_comm(
    assertion_id: str,
    condition: bool,
    message: str,
    severity: str = "yellow",
    cooldown: float = 5.0,
    xrce_detail: Optional[str] = None,
) -> bool:
    """
    Fire assertion if condition is False.

    Routes messages to both the dashboard Log pane (user-visible) and
    XRCE Debug pane (protocol details). Rate-limited to prevent spam.

    Args:
        assertion_id: Unique ID for rate limiting (e.g., "T1", "S3")
        condition: If False, fires the assertion
        message: User-facing message (supports Rich markup)
        severity: Rich style for message ("yellow", "red", "bold red")
        cooldown: Minimum seconds between fires for same assertion_id
        xrce_detail: Optional additional detail for XRCE Debug pane only

    Returns:
        The condition value (True = OK, False = assertion fired)
    """
    if condition:
        return True

    now = time.time()
    last = _last_fired.get(assertion_id, 0)

    # Rate limiting: track but don't fire if within cooldown
    if now - last < cooldown:
        _fire_counts[assertion_id] = _fire_counts.get(assertion_id, 0) + 1
        return False

    # Fire the assertion
    _last_fired[assertion_id] = now
    count = _fire_counts.get(assertion_id, 0) + 1
    _fire_counts[assertion_id] = 0

    # Format message with severity
    formatted = f"[{severity}]{message}[/{severity}]"
    if count > 1:
        formatted += f" [dim](+{count - 1} suppressed)[/dim]"

    # Route to dashboard Log pane (user-visible)
    if _dashboard_log:
        _dashboard_log(formatted)

    # Also log to XRCE Debug pane with optional detail
    from .logger import xrce_log
    xrce_log(formatted)
    if xrce_detail:
        xrce_log(f"  [dim]{xrce_detail}[/dim]")

    return False


def clear_state() -> None:
    """Clear all rate limiting state. Useful for testing."""
    global _last_fired, _fire_counts
    _last_fired = {}
    _fire_counts = {}
