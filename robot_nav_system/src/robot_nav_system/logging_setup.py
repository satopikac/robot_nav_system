"""Unified logging configuration for the robot navigation system."""

import logging
import sys


_CONFIGURED = False


def setup_logging(level: str = "INFO", name: str = "robot_nav") -> logging.Logger:
    """Configure and return the root logger for the system.

    Safe to call multiple times; only configures handlers once.
    """
    global _CONFIGURED
    logger = logging.getLogger(name)

    if _CONFIGURED:
        return logger

    log_level = getattr(logging, level.upper(), logging.INFO)
    logger.setLevel(log_level)

    fmt = logging.Formatter(
        "[%(asctime)s] [%(name)s.%(module)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    console = logging.StreamHandler(sys.stdout)
    console.setLevel(log_level)
    console.setFormatter(fmt)
    logger.addHandler(console)

    _CONFIGURED = True
    return logger


def get_logger(module_name: str) -> logging.Logger:
    """Get a child logger under the root 'robot_nav' logger."""
    return logging.getLogger(f"robot_nav.{module_name}")
