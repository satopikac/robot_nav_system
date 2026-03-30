"""Navigator factory -- builds the appropriate navigator from config."""
from __future__ import annotations

from ..config import Config
from ..logging_setup import get_logger
from .base import BaseNavigator
from .simulated import SimulatedNavigator

log = get_logger("navigator_factory")


def build_navigator(config: Config) -> BaseNavigator:
    """Build and return the navigator matching the runtime mode."""
    mode = config.get("runtime.mode", "simulation")
    robots = list(config.get("robots.names", ["robot_0"]))
    default_positions = dict(config.get("robots.default_positions", {}))

    if mode == "ros_single":
        from .ros_single import ROSSingleNavigator
        return ROSSingleNavigator(
            move_base_action=config.get("ros.move_base_action", "move_base"),
            frame_id=config.get("ros.frame_id", "map"),
            base_frame=config.get("ros.base_frame", "base_link"),
            goal_timeout=float(config.get("robots.goal_timeout", 300.0)),
        )

    if mode == "ros_multi":
        from .ros_multi import ROSMultiNavigator
        return ROSMultiNavigator(
            robots=robots,
            global_frame=config.get("robots.global_frame", "map"),
            base_frame_suffix=config.get("robots.base_frame_suffix", "base_footprint"),
            goal_timeout=float(config.get("robots.goal_timeout", 300.0)),
            server_timeout=float(config.get("robots.server_timeout", 10.0)),
        )

    # default: simulation
    if mode != "simulation":
        log.warning("Unknown runtime mode '%s', falling back to simulation", mode)
    return SimulatedNavigator(
        robots=robots,
        default_positions=default_positions,
        frame_id=config.get("ros.frame_id", "map"),
    )
