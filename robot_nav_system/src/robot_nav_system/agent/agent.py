"""Navigation Agent -- integrates LLM planning, semantic map, allocation, and execution."""
from __future__ import annotations

from typing import TYPE_CHECKING, Dict, List, Optional, Tuple

from ..config import Config
from ..exceptions import AllocationError, LLMError, NavigationError, TaskPlanningError
from ..logging_setup import get_logger
from ..navigation.base import BaseNavigator
from .llm_client import LLMClient

if TYPE_CHECKING:
    from ..perception.semantic_map import SemanticMap
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

log = get_logger("agent")


class NavigationAgent:
    """Main agent: plan -> allocate -> execute -> summarize."""

    def __init__(
        self,
        config: Config,
        semantic_map: SemanticMap,
        llm_client: LLMClient,
        navigator: BaseNavigator,
    ):
        self.config = config
        self.semantic_map = semantic_map
        self.llm = llm_client
        self.navigator = navigator
        self.task_manager = TaskManager()
        self.memory = DialogueMemory(
            max_turns=int(config.get("memory.max_history_turns", 8))
        )
        self.strict_map_only = bool(config.get("planner.strict_map_only", True))
        self.max_subtasks = int(config.get("planner.max_subtasks", 8))
        self.runtime_mode = str(config.get("runtime.mode", "simulation"))
        self.robots: List[str] = list(config.get("robots.names", ["robot_0"]))

    # ---- Planning ----

    def _validate_plan_with_map(self, plan: TaskPlan) -> Tuple[TaskPlan, List[str]]:
        validated: List[SubTask] = []
        dropped: List[str] = []
        for st in plan.subtasks:
            obj, _score = self.semantic_map.match_object(st.target_name)
            if obj:
                st.target_name = obj.name
                st.target_obj_id = obj.obj_id
                validated.append(st)
            else:
                dropped.append(st.target_name)

        if self.strict_map_only:
            plan.subtasks = validated
        else:
            plan.subtasks = validated if validated else plan.subtasks
        return plan, dropped

    def plan_new_task(self, instruction: str) -> TaskPlan:
        if self.semantic_map.is_empty:
            raise TaskPlanningError(
                "语义地图为空，请先启动探索模式发现环境中的物体。"
            )

        progress = ProgressSnapshot(original_instruction=instruction)
        raw_plan = self.llm.plan_tasks(
            instruction=instruction,
            semantic_objects=self.semantic_map.as_prompt_brief(),
            progress=progress,
            recent_dialogue=self.memory.recent(),
            robots=self.robots,
            max_subtasks=self.max_subtasks,
        )
        validated_plan, dropped = self._validate_plan_with_map(raw_plan)
        if dropped:
            log.warning("Targets not in semantic map, dropped: %s", dropped)
        if not validated_plan.subtasks:
            raise TaskPlanningError("无法从指令中规划出可执行子任务（语义地图内）。")
        return validated_plan

    # ---- Execution ----

    def _resolve_targets(self, plan: TaskPlan) -> List[Optional[SemanticObject]]:
        targets: List[Optional[SemanticObject]] = []
        for st in plan.subtasks:
            obj = self.semantic_map.find_by_id(st.target_obj_id or "")
            targets.append(obj)
        return targets

    def _build_dependency_map(self, plan: TaskPlan) -> Dict[int, Optional[int]]:
        deps: Dict[int, Optional[int]] = {}
        for idx, st in enumerate(plan.subtasks):
            deps[idx] = st.depends_on
        return deps

    def _get_allocator(self, strategy: AllocStrategy) -> BaseAllocator:
        if strategy == AllocStrategy.HUNGARIAN:
            return HungarianAllocator()
        return GreedyAllocator()

    def _execute_single(self, plan: TaskPlan) -> List[ExecutionRecord]:
        targets = self._resolve_targets(plan)
        tasks: List[Tuple[SubTask, SemanticObject]] = []
        for st, obj in zip(plan.subtasks, targets):
            if obj is None:
                log.warning("Target not found, skipping: %s", st.target_name)
                continue
            tasks.append((st, obj))
        if not tasks:
            raise NavigationError("没有可执行的子任务")
        return self.navigator.execute_single(tasks)

    def _execute_multi(self, plan: TaskPlan) -> List[ExecutionRecord]:
        targets = self._resolve_targets(plan)

        valid_subtasks: List[SubTask] = []
        valid_targets: List[SemanticObject] = []
        for st, obj in zip(plan.subtasks, targets):
            if obj is None:
                log.warning("Target not found, skipping: %s", st.target_name)
                continue
            valid_subtasks.append(st)
            valid_targets.append(obj)

        if not valid_subtasks:
            raise NavigationError("没有可执行的子任务")

        robot_positions = self.navigator.get_robot_positions()
        if not robot_positions:
            raise NavigationError("无法获取任何机器人的位置信息")

        allocator = self._get_allocator(plan.alloc_strategy)
        allocation = allocator.allocate(valid_subtasks, valid_targets, robot_positions)

        strategy_name = "贪心最近优先" if plan.alloc_strategy == AllocStrategy.GREEDY else "匈牙利最优分配"
        log.info("Allocation strategy: %s", strategy_name)
        for robot, indices in allocation.items():
            task_names = [valid_subtasks[i].target_name for i in indices]
            log.info("  %s -> %s", robot, task_names)

        assignments: Dict[str, List[Tuple[SubTask, SemanticObject]]] = {}
        for robot, indices in allocation.items():
            assignments[robot] = [(valid_subtasks[i], valid_targets[i]) for i in indices]

        dependencies = self._build_dependency_map(plan)
        return self.navigator.execute_multi(assignments, dependencies)

    def execute_plan(self, plan: TaskPlan) -> List[ExecutionRecord]:
        if plan.mode == NavMode.SINGLE:
            log.info("Mode: single | subtasks: %d", len(plan.subtasks))
            return self._execute_single(plan)
        else:
            log.info("Mode: multi | subtasks: %d | strategy: %s",
                     len(plan.subtasks), plan.alloc_strategy.value)
            return self._execute_multi(plan)

    # ---- Full pipeline ----

    def run_instruction(self, instruction: str) -> str:
        self.memory.add_user(instruction)

        plan = self.plan_new_task(instruction)
        self.task_manager.set_plan(plan)
        log.info("Generated %d subtasks | mode: %s", len(plan.subtasks), plan.mode.value)
        if plan.notes:
            log.info("Planner notes: %s", plan.notes)

        records = self.execute_plan(plan)
        self.task_manager.add_records(records)

        progress = self.task_manager.progress_snapshot()
        summary = self.llm.summarize_task(
            original_instruction=instruction,
            records=records,
            progress=progress,
            recent_dialogue=self.memory.recent(),
        )
        self.memory.add_assistant(summary)
        self.task_manager.clear()
        return summary

    def interrupt(self) -> str:
        current_instruction = self.task_manager.original_instruction or "无当前任务"
        self.task_manager.clear()
        msg = self.llm.notify_interrupt(current_instruction)
        self.memory.add_assistant(msg)
        return msg

    def status(self) -> str:
        status_text = self.task_manager.status_text(runtime_mode=self.runtime_mode)
        self.memory.add_assistant(status_text)
        return status_text

    def handle_command(self, text: str) -> str:
        normalized = text.strip().lower()
        if normalized in {"interrupt", "中断"}:
            return self.interrupt()
        if normalized in {"status", "状态"}:
            return self.status()
        if normalized in {"exit", "quit", "结束"}:
            return "exit"
        if normalized in {"help", "帮助"}:
            return self._help_text()

        try:
            return self.run_instruction(text)
        except (TaskPlanningError, LLMError, AllocationError, NavigationError) as e:
            msg = f"任务失败: {e}"
            self.memory.add_assistant(msg)
            self.task_manager.clear()
            return msg
        except Exception as e:
            msg = f"系统异常: {e}"
            self.memory.add_assistant(msg)
            self.task_manager.clear()
            return msg

    @staticmethod
    def _help_text() -> str:
        return (
            "可用命令:\n"
            "  <自然语言指令>  - 输入导航任务（自动判断单机/多机模式）\n"
            "  status / 状态   - 查看当前任务状态\n"
            "  interrupt / 中断 - 中断当前任务\n"
            "  help / 帮助     - 显示此帮助\n"
            "  exit / quit / 结束 - 退出系统\n"
            "\n"
            "  explore / 探索   - 启动自主探索模式\n"
            "  stop / 停止      - 停止探索返回空闲\n"
            "\n"
            "示例指令:\n"
            "  单机: 去咖啡机拿杯咖啡送到沙发\n"
            "  多机: 让三个机器人分别去咖啡机、沙发和电视柜\n"
            "  依赖: 先去咖啡机接咖啡，完成后再送到沙发，同时另一个机器人去拿杯子"
        )
