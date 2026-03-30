"""Task lifecycle tracking for plan execution."""
from __future__ import annotations

from collections import deque
from typing import Deque, Dict, List, Optional

from .models import (
    ExecutionRecord,
    NavMode,
    ProgressSnapshot,
    SubTask,
    TaskPlan,
)


class TaskManager:
    """Tracks plan execution state for both single and multi-robot modes."""

    def __init__(self) -> None:
        self.original_instruction: str = ""
        self.mode: NavMode = NavMode.SINGLE
        self._pending: Deque[SubTask] = deque()
        self._completed: List[ExecutionRecord] = []
        self._current: Optional[SubTask] = None
        self._current_step_index = 0

    def set_plan(self, plan: TaskPlan) -> None:
        self.clear()
        self.original_instruction = plan.original_instruction
        self.mode = plan.mode
        for subtask in plan.subtasks:
            self._pending.append(subtask)

    def clear(self) -> None:
        self.original_instruction = ""
        self.mode = NavMode.SINGLE
        self._pending.clear()
        self._completed.clear()
        self._current = None
        self._current_step_index = 0

    def has_pending(self) -> bool:
        return bool(self._pending)

    def next_task(self) -> Optional[SubTask]:
        if not self._pending:
            self._current = None
            return None
        self._current = self._pending.popleft()
        self._current_step_index += 1
        return self._current

    def complete_current(self, success: bool, message: str = "", robot: Optional[str] = None) -> None:
        if not self._current:
            return
        self._completed.append(
            ExecutionRecord(
                step_index=self._current_step_index,
                action=self._current.action,
                target_name=self._current.target_name,
                target_obj_id=self._current.target_obj_id,
                success=success,
                message=message,
                robot=robot,
            )
        )
        self._current = None

    def add_records(self, records: List[ExecutionRecord]) -> None:
        """Bulk-add execution records (used after navigator.execute_* calls)."""
        self._completed.extend(records)
        self._pending.clear()
        self._current = None

    def records(self) -> List[ExecutionRecord]:
        return list(self._completed)

    def progress_snapshot(self) -> ProgressSnapshot:
        completed = [
            {
                "step": str(r.step_index),
                "action": r.action,
                "target": r.target_name,
                "result": "success" if r.success else "fail",
                "robot": r.robot or "-",
            }
            for r in self._completed
        ]
        in_progress = None
        if self._current:
            in_progress = {"action": self._current.action, "target": self._current.target_name}
        pending = [{"action": t.action, "target": t.target_name} for t in self._pending]
        return ProgressSnapshot(
            original_instruction=self.original_instruction,
            mode=self.mode.value,
            completed=completed,
            in_progress=in_progress,
            pending=pending,
        )

    def status_text(self, runtime_mode: str = "simulation") -> str:
        progress = self.progress_snapshot()
        lines = [
            f"运行环境: {runtime_mode}",
            f"导航模式: {self.mode.value}",
            f"当前任务: {self.original_instruction or '无'}",
            f"已完成: {len(progress.completed)} 步",
            f"进行中: {progress.in_progress or '无'}",
            f"待执行: {len(progress.pending)} 步",
        ]
        if progress.completed:
            lines.append("  --- 已完成列表 ---")
            for r in progress.completed:
                robot_tag = f"[{r['robot']}]" if r.get("robot", "-") != "-" else ""
                lines.append(f"  {r['step']}. {robot_tag} {r['action']} -> {r['target']} : {r['result']}")
        if progress.pending:
            lines.append("  --- 待执行列表 ---")
            for idx, task in enumerate(progress.pending, start=1):
                lines.append(f"  {idx}. {task['action']} -> {task['target']}")
        return "\n".join(lines)
