"""
XRCE debug logging with dashboard integration.

This module provides a simple logging mechanism that can route
debug messages either to stdout (print) or to a dashboard callback.
Also writes to a persistent log file for debugging.
"""

import re
from datetime import datetime
from typing import Callable, Optional

# Global callback for XRCE log messages
_log_callback: Optional[Callable[[str], None]] = None

# Log file path
XRCE_LOG_FILE = "/tmp/nimbus_xrce.log"


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


def xrce_log(message: str) -> None:
    """
    Log an XRCE debug message.

    If a callback is set (e.g., from the dashboard), the message is sent there.
    Otherwise, it's printed to stdout. Also writes to log file.

    Args:
        message: The debug message to log
    """
    # Write to log file (strip Rich markup for plain text)
    try:
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        plain_message = _strip_markup(message)
        with open(XRCE_LOG_FILE, "a") as f:
            f.write(f"{timestamp} {plain_message}\n")
    except Exception:
        pass  # Don't let logging errors break the agent

    # Send to callback or print
    if _log_callback is not None:
        _log_callback(message)
    else:
        print(message)
