from .llm_client import LLMClient
from .memory import DialogueMemory
from .models import (
    AllocStrategy,
    ExecutionRecord,
    NavMode,
    ProgressSnapshot,
    SemanticObject,
    SubTask,
    TaskPlan,
)
from .task_allocator import BaseAllocator, GreedyAllocator, HungarianAllocator
from .task_manager import TaskManager


def get_navigation_agent_class():
    """Lazy import to avoid circular dependency with perception.semantic_map."""
    from .agent import NavigationAgent
    return NavigationAgent


__all__ = [
    "get_navigation_agent_class",
    "LLMClient",
    "DialogueMemory",
    "AllocStrategy",
    "ExecutionRecord",
    "NavMode",
    "ProgressSnapshot",
    "SemanticObject",
    "SubTask",
    "TaskPlan",
    "BaseAllocator",
    "GreedyAllocator",
    "HungarianAllocator",
    "TaskManager",
]
