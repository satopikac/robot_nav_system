"""Task allocation strategies for multi-robot navigation.

Provides two allocators:
- GreedyAllocator: priority-based greedy nearest-first assignment
- HungarianAllocator: optimal assignment via the Hungarian (KM) algorithm
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple

from ..exceptions import AllocationError
from .models import SemanticObject, SubTask


class BaseAllocator(ABC):
    """Assign subtasks to robots given robot positions and task targets."""

    @abstractmethod
    def allocate(
        self,
        subtasks: List[SubTask],
        targets: List[SemanticObject],
        robot_positions: Dict[str, Tuple[float, float]],
    ) -> Dict[str, List[int]]:
        raise NotImplementedError


def _dist(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


class GreedyAllocator(BaseAllocator):
    """Priority-based greedy assignment: assign each task to the nearest idle robot."""

    def allocate(
        self,
        subtasks: List[SubTask],
        targets: List[SemanticObject],
        robot_positions: Dict[str, Tuple[float, float]],
    ) -> Dict[str, List[int]]:
        assignments: Dict[str, List[int]] = {r: [] for r in robot_positions}

        remaining_indices: List[int] = []
        for idx, st in enumerate(subtasks):
            if st.assigned_robot and st.assigned_robot in robot_positions:
                assignments[st.assigned_robot].append(idx)
            else:
                remaining_indices.append(idx)

        used = {r for r in robot_positions if assignments[r]}
        available = [r for r in robot_positions if r not in used]
        if not available:
            available = list(robot_positions.keys())

        for idx in remaining_indices:
            target_pos = (targets[idx].position[0], targets[idx].position[1])
            best_robot: Optional[str] = None
            best_dist = float("inf")
            for robot in available:
                d = _dist(robot_positions[robot], target_pos)
                if d < best_dist:
                    best_dist = d
                    best_robot = robot

            if best_robot is None:
                raise AllocationError(f"无法为子任务 {idx} 分配机器人")
            assignments[best_robot].append(idx)
            available.remove(best_robot)
            if not available:
                available = list(robot_positions.keys())

        return {r: idxs for r, idxs in assignments.items() if idxs}


class HungarianAllocator(BaseAllocator):
    """Optimal assignment using the Hungarian (KM) algorithm via scipy."""

    def allocate(
        self,
        subtasks: List[SubTask],
        targets: List[SemanticObject],
        robot_positions: Dict[str, Tuple[float, float]],
    ) -> Dict[str, List[int]]:
        try:
            import numpy as np
            from scipy.optimize import linear_sum_assignment
        except ImportError as e:
            raise AllocationError(
                "Hungarian allocator requires numpy and scipy. "
                "Install: pip install numpy scipy"
            ) from e

        assignments: Dict[str, List[int]] = {r: [] for r in robot_positions}
        free_indices: List[int] = []
        for idx, st in enumerate(subtasks):
            if st.assigned_robot and st.assigned_robot in robot_positions:
                assignments[st.assigned_robot].append(idx)
            else:
                free_indices.append(idx)

        if not free_indices:
            return {r: idxs for r, idxs in assignments.items() if idxs}

        robots = list(robot_positions.keys())
        n_r = len(robots)
        n_t = len(free_indices)

        size = max(n_r, n_t)
        cost = np.full((size, size), 1e6)
        for i, robot in enumerate(robots):
            rp = robot_positions[robot]
            for j, tidx in enumerate(free_indices):
                tp = (targets[tidx].position[0], targets[tidx].position[1])
                cost[i, j] = _dist(rp, tp)

        row_idx, col_idx = linear_sum_assignment(cost)
        for ri, ci in zip(row_idx, col_idx):
            if ri < n_r and ci < n_t:
                robot = robots[ri]
                assignments[robot].append(free_indices[ci])

        return {r: idxs for r, idxs in assignments.items() if idxs}
