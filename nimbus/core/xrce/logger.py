"""
XRCE debug logging with dashboard integration.

This module provides a simple logging mechanism that can route
debug messages either to stdout (print) or to a dashboard callback.
Also writes to a persistent log file for debugging.

Verbosity levels:
- 1 (CRITICAL): Critical errors, state/behavior changes
- 2 (NORMAL): Level 1 + connection events, topic detection
- 3 (DEBUG): Level 2 + all protocol details (packets, parsing)
"""

import re
from datetime import datetime
from enum import IntEnum
from typing import Callable, Optional

# Global callback for XRCE log messages
_log_callback: Optional[Callable[[str], None]] = None

# Log file path
XRCE_LOG_FILE = "/tmp/nimbus.log"


class LogLevel(IntEnum):
    """Log verbosity levels."""
    CRITICAL = 1  # Critical errors, state/behavior changes
    NORMAL = 2    # Connection events, topic detection
    DEBUG = 3     # All protocol details (packets, parsing)


# Global verbosity setting (default: NORMAL)
_verbosity: LogLevel = LogLevel.NORMAL


def get_verbosity() -> LogLevel:
    """Get the current verbosity level."""
    return _verbosity


def set_verbosity(level: int) -> None:
    """
    Set the verbosity level.

    Args:
        level: Verbosity level (1=critical, 2=normal, 3=debug)
    """
    global _verbosity
    _verbosity = LogLevel(max(1, min(3, level)))


def get_log_file_path() -> str:
    """Get the path to the XRCE log file."""
    return XRCE_LOG_FILE


def set_log_callback(callback: Optional[Callable[[str], None]]) -> None:
    """
    Set the callback for XRCE log messages.

    Args:
        callback: Function that takes a log message string, or None to use print
    """
    global _log_callback
    _log_callback = callback


def _strip_markup(message: str) -> str:
    """Remove Rich markup tags from a message for plain text logging."""
    return re.sub(r'\[/?[^\]]+\]', '', message)


def xrce_log(message: str, level: LogLevel = LogLevel.DEBUG) -> None:
    """
    Log an XRCE debug message.

    If a callback is set (e.g., from the dashboard), the message is sent there.
    Otherwise, it's printed to stdout. Also writes to log file.

    The file always receives all messages regardless of verbosity.
    Dashboard/stdout only receives messages at or below current verbosity level.

    Args:
        message: The debug message to log
        level: Message importance level (default: DEBUG)
    """
    # Always write to log file (strip Rich markup for plain text)
    try:
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        plain_message = _strip_markup(message)
        with open(XRCE_LOG_FILE, "a") as f:
            f.write(f"{timestamp} {plain_message}\n")
    except Exception:
        pass  # Don't let logging errors break the agent

    # Only show in dashboard/stdout if verbosity allows
    if level > _verbosity:
        return

    # Send to callback or print
    if _log_callback is not None:
        _log_callback(message)
    else:
        print(message)
