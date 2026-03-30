"""Data models for the robot navigation system.

Reused from unified_nav_agent with extensions for system state management.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import Dict, List, Optional, Tuple


# --------------- enums ---------------

class NavMode(str, Enum):
    SINGLE = "single"
    MULTI = "multi"


class AllocStrategy(str, Enum):
    GREEDY = "greedy"
    HUNGARIAN = "hungarian"


class RobotTaskStatus(IntEnum):
    PENDING = 0
    READY = 1
    ACTIVE = 2
    SUCCEEDED = 3
    FAILED = 4


# --------------- semantic map ---------------

@dataclass
class SemanticObject:
    obj_id: str
    name: str
    aliases: List[str]
    category: str
    room: str
    position: Tuple[float, float, float]   # (x, y, yaw)
    description: str = ""

    def all_keywords(self) -> List[str]:
        return [self.name, *self.aliases, self.category, self.room]


# --------------- task plan ---------------

@dataclass
class SubTask:
    action: str
    target_name: str
    reason: str = ""
    target_obj_id: Optional[str] = None
    depends_on: Optional[int] = None        # subtask index this depends on
    assigned_robot: Optional[str] = None


@dataclass
class TaskPlan:
    original_instruction: str
    mode: NavMode = NavMode.SINGLE
    subtasks: List[SubTask] = field(default_factory=list)
    alloc_strategy: AllocStrategy = AllocStrategy.GREEDY
    notes: str = ""


# --------------- execution tracking ---------------

@dataclass
class ExecutionRecord:
    step_index: int
    action: str
    target_name: str
    target_obj_id: Optional[str]
    success: bool
    message: str = ""
    robot: Optional[str] = None


@dataclass
class ProgressSnapshot:
    original_instruction: str
    mode: str = "single"
    completed: List[Dict[str, str]] = field(default_factory=list)
    in_progress: Optional[Dict[str, str]] = None
    pending: List[Dict[str, str]] = field(default_factory=list)
