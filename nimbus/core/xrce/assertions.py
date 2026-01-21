"""Communication assertions with rate limiting and dashboard integration.

This module provides assertion helpers that detect error conditions during
robot communication and route them to the unified log display.

Features:
- Rate limiting to prevent spam from repeated failures
- Suppression counting to show missed events
- Verbosity-aware routing through xrce_log()
"""

import time
from typing import Optional

from .logger import xrce_log, LogLevel

# Rate limiting state
_last_fired: dict[str, float] = {}
_fire_counts: dict[str, int] = {}


def assert_comm(
    assertion_id: str,
    condition: bool,
    message: str,
    severity: str = "yellow",
    cooldown: float = 5.0,
    xrce_detail: Optional[str] = None,
    level: LogLevel = LogLevel.NORMAL,
) -> bool:
    """
    Fire assertion if condition is False.

    Routes messages through the unified xrce_log() with verbosity support.
    Rate-limited to prevent spam.

    Args:
        assertion_id: Unique ID for rate limiting (e.g., "T1", "S3")
        condition: If False, fires the assertion
        message: User-facing message (supports Rich markup)
        severity: Rich style for message ("yellow", "red", "bold red")
        cooldown: Minimum seconds between fires for same assertion_id
        xrce_detail: Optional additional detail (shown at DEBUG level)
        level: Log level for this assertion (default: NORMAL)

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

    # Route through unified xrce_log with appropriate level
    xrce_log(formatted, level=level)
    if xrce_detail:
        xrce_log(f"  [dim]{xrce_detail}[/dim]", level=LogLevel.DEBUG)

    return False


def clear_state() -> None:
    """Clear all rate limiting state. Useful for testing."""
    global _last_fired, _fire_counts
    _last_fired = {}
    _fire_counts = {}
