"""Abstract base class for all navigator backends."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple

from ..agent.models import ExecutionRecord, SemanticObject, SubTask


class BaseNavigator(ABC):
    """Common interface implemented by all navigator backends."""

    @abstractmethod
    def execute_single(
        self,
        tasks: List[Tuple[SubTask, SemanticObject]],
    ) -> List[ExecutionRecord]:
        """Execute *tasks* sequentially on one robot. Return execution records."""

    @abstractmethod
    def execute_multi(
        self,
        assignments: Dict[str, List[Tuple[SubTask, SemanticObject]]],
        dependencies: Dict[int, Optional[int]],
    ) -> List[ExecutionRecord]:
        """Execute *assignments* (robot->tasks) in parallel. Return records."""

    @abstractmethod
    def get_robot_positions(self) -> Dict[str, Tuple[float, float]]:
        """Return {robot_name: (x, y)} for all known robots."""

    def cancel_all_goals(self) -> None:
        """Cancel all pending navigation goals. Override in ROS navigators."""
        pass
