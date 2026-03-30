"""Unified exception hierarchy for the robot navigation system."""


class RobotNavError(Exception):
    """Base exception for the entire robot navigation system."""


class ConfigError(RobotNavError):
    """Invalid or missing configuration."""


class SemanticMapError(RobotNavError):
    """Semantic map loading, parsing, or object matching failures."""


class LLMError(RobotNavError):
    """LLM API call or response parsing failures."""


class TaskPlanningError(RobotNavError):
    """Task planning failed (no valid subtasks, unresolvable targets, etc.)."""


class NavigationError(RobotNavError):
    """Navigation goal execution failures (timeout, abort, server unavailable)."""


class AllocationError(RobotNavError):
    """Task-to-robot allocation failures."""


class StateTransitionError(RobotNavError):
    """Illegal state machine transition attempted."""


class ExplorationError(RobotNavError):
    """explore-lite start/stop failures."""


class MapConversionError(RobotNavError):
    """DMROS to nav-agent format conversion failures."""
