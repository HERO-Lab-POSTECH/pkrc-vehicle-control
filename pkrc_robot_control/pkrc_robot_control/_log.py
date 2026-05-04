"""Shared logger dispatch helper.

rclpy's per-call-site severity tracking raises ValueError when the same
source line dispatches multiple severities via getattr (per the
feedback_rclpy_logger_getattr_trap memory). We use explicit if/elif
dispatch and accept None (print fallback for early-init / standalone use).
"""
from typing import Callable


def make_logger(rclpy_logger) -> Callable[[str, str], None]:
    """Return a callable (level: str, msg: str) -> None.

    `rclpy_logger` may be None — fallback is print().
    """
    def _log(level: str, msg: str) -> None:
        if rclpy_logger is None:
            print(msg)
            return
        if level == 'info':
            rclpy_logger.info(msg)
        elif level in ('warn', 'warning'):
            rclpy_logger.warn(msg)
        elif level == 'error':
            rclpy_logger.error(msg)
        elif level == 'debug':
            rclpy_logger.debug(msg)
        elif level == 'fatal':
            rclpy_logger.fatal(msg)
        else:
            rclpy_logger.info(msg)  # unknown level fallback
    return _log
