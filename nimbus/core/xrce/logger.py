"""
XRCE debug logging with dashboard integration.

This module provides a simple logging mechanism that can route
debug messages either to stdout (print) or to a dashboard callback.
"""

from typing import Callable, Optional

# Global callback for XRCE log messages
_log_callback: Optional[Callable[[str], None]] = None


def set_log_callback(callback: Optional[Callable[[str], None]]) -> None:
    """
    Set the callback for XRCE log messages.

    Args:
        callback: Function that takes a log message string, or None to use print
    """
    global _log_callback
    _log_callback = callback


def xrce_log(message: str) -> None:
    """
    Log an XRCE debug message.

    If a callback is set (e.g., from the dashboard), the message is sent there.
    Otherwise, it's printed to stdout.

    Args:
        message: The debug message to log
    """
    if _log_callback is not None:
        _log_callback(message)
    else:
        print(message)
