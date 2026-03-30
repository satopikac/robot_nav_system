"""System Orchestrator -- top-level controller for the navigation system.

Owns the state machine, agent, perception pipeline, and mode controllers.
Handles the complete lifecycle from startup to shutdown.
"""
from __future__ import annotations

import signal
import sys
from pathlib import Path
from typing import Optional

from ..agent.agent import NavigationAgent
from ..agent.llm_client import LLMClient
from ..config import Config
from ..exceptions import (
    ExplorationError,
    RobotNavError,
    StateTransitionError,
)
from ..logging_setup import get_logger, setup_logging
from ..navigation import build_navigator
from ..navigation.base import BaseNavigator
from ..perception import MapConverter, MapWatcher, SemanticMap
from .mode_controller import ExploreController, TaskController
from .state_machine import SystemState, SystemStateMachine

log = get_logger("orchestrator")


class SystemOrchestrator:
    """Top-level system controller.

    In simulation mode: runs a REPL loop.
    In ROS mode: provides ROS services and spins.
    """

    def __init__(self, config: Config):
        self.config = config
        self.sm = SystemStateMachine()

        # Will be initialized in startup()
        self.semantic_map: Optional[SemanticMap] = None
        self.llm: Optional[LLMClient] = None
        self.navigator: Optional[BaseNavigator] = None
        self.agent: Optional[NavigationAgent] = None
        self.converter: Optional[MapConverter] = None
        self.watcher: Optional[MapWatcher] = None
        self.explore_ctrl: Optional[ExploreController] = None
        self.task_ctrl: Optional[TaskController] = None

        self._was_exploring = False
        self._shutting_down = False

    # ---- Lifecycle ----

    def startup(self) -> None:
        """Initialize all subsystems."""
        log.info("Starting robot_nav_system (profile: %s)", self.config.profile_name)

        # 1. Build semantic map
        map_path = Path(self.config.get("semantic_map.path", "data/semantic_map.json"))
        if map_path.exists():
            try:
                self.semantic_map = SemanticMap.from_json(map_path)
                log.info("Loaded semantic map: %d objects", len(self.semantic_map.objects))
            except Exception as e:
                log.warning("Failed to load semantic map, starting empty: %s", e)
                self.semantic_map = SemanticMap.empty(map_path)
        else:
            log.info("No semantic map found, starting empty (exploration will populate)")
            self.semantic_map = SemanticMap.empty(map_path)

        # 2. Build LLM client
        self.llm = LLMClient(self.config)

        # 3. Build navigator
        self.navigator = build_navigator(self.config)

        # 4. Build agent
        self.agent = NavigationAgent(
            self.config, self.semantic_map, self.llm, self.navigator
        )

        # 5. Build perception pipeline
        if self.config.get("dmros.enabled", False):
            self.converter = MapConverter(self.config)
            dmros_output = Path(self.config.get("dmros.output_dir", "output/semantic")) / "semantic_map.json"
            self.watcher = MapWatcher(
                dmros_output_path=dmros_output,
                nav_map_path=map_path,
                converter=self.converter,
                semantic_map=self.semantic_map,
                poll_interval=float(self.config.get("dmros.poll_interval", 3.0)),
            )
            self.watcher.start()

        # 6. Build mode controllers
        self.explore_ctrl = ExploreController(self.config, self.navigator)
        self.task_ctrl = TaskController(self.agent, self.sm)

        # 7. Register state machine callbacks
        self._register_callbacks()

        # 8. Transition to IDLE
        self.sm.transition(SystemState.IDLE)

        # 9. Auto-start exploration if configured
        if self.config.get("runtime.auto_explore_on_startup", False):
            self.request_explore()

        log.info("System ready (state: %s)", self.sm.state.value)

    def shutdown(self) -> None:
        """Graceful shutdown."""
        if self._shutting_down:
            return
        self._shutting_down = True
        log.info("Shutting down...")

        try:
            self.sm.transition(SystemState.SHUTTING_DOWN)
        except StateTransitionError:
            pass

        if self.explore_ctrl and self.explore_ctrl.is_exploring:
            self.explore_ctrl.stop_exploration()

        if self.watcher:
            self.watcher.stop()

        if self.navigator:
            self.navigator.cancel_all_goals()

        log.info("Shutdown complete")

    # ---- Public API ----

    def submit_task(self, instruction: str) -> str:
        """Submit a natural language task. Switches from exploration if needed."""
        if self.sm.state == SystemState.EXPLORING:
            self._was_exploring = True
            self.sm.transition(SystemState.TASK_RECEIVED)
        elif self.sm.state == SystemState.IDLE:
            self._was_exploring = False
            self.sm.transition(SystemState.TASK_RECEIVED)
        else:
            return f"无法接受任务：当前状态为 {self.sm.state.value}"

        try:
            # TASK_RECEIVED -> PLANNING
            self.sm.transition(SystemState.PLANNING)

            # Execute task
            self.sm.transition(SystemState.EXECUTING)
            result = self.task_ctrl.execute_instruction(instruction)

            # EXECUTING -> COMPLETED
            self.sm.transition(SystemState.COMPLETED)

            # Auto-resume exploration or go idle
            if self._was_exploring and self.config.get("runtime.resume_explore_after_task", False):
                self.sm.transition(SystemState.EXPLORING)
            else:
                self.sm.transition(SystemState.IDLE)

            return result

        except RobotNavError as e:
            log.error("Task failed: %s", e)
            try:
                self.sm.transition(SystemState.ERROR)
                self.sm.transition(SystemState.IDLE)
            except StateTransitionError:
                pass
            return f"任务失败: {e}"
        except Exception as e:
            log.exception("Unexpected error during task execution")
            try:
                self.sm.transition(SystemState.ERROR)
                self.sm.transition(SystemState.IDLE)
            except StateTransitionError:
                pass
            return f"系统异常: {e}"

    def request_explore(self) -> bool:
        """Switch to exploration mode."""
        if self.sm.state != SystemState.IDLE:
            log.warning("Cannot explore from state: %s", self.sm.state.value)
            return False
        try:
            self.sm.transition(SystemState.EXPLORING)
            return True
        except StateTransitionError as e:
            log.error("Failed to start exploration: %s", e)
            return False

    def request_idle(self) -> bool:
        """Switch to idle mode (stop exploration)."""
        if self.sm.state != SystemState.EXPLORING:
            return True  # Already idle or in another state
        try:
            self.sm.transition(SystemState.IDLE)
            return True
        except StateTransitionError as e:
            log.error("Failed to stop exploration: %s", e)
            return False

    def get_status(self) -> dict:
        """Return current system status snapshot."""
        status = {
            "state": self.sm.state.value,
            "profile": self.config.profile_name,
            "runtime_mode": self.config.get("runtime.mode"),
            "exploring": self.explore_ctrl.is_exploring if self.explore_ctrl else False,
            "semantic_map_objects": len(self.semantic_map.objects) if self.semantic_map else 0,
            "semantic_map_version": self.semantic_map.version if self.semantic_map else 0,
        }
        if self.agent:
            status["agent_status"] = self.agent.task_manager.status_text(
                runtime_mode=self.config.get("runtime.mode", "simulation")
            )
        return status

    # ---- REPL (for manual_sim mode) ----

    def run_repl(self) -> int:
        """Interactive CLI loop for simulation / manual mode."""
        self._print_banner()

        # Install signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        try:
            while not self._shutting_down:
                try:
                    user_input = input("\n你> ").strip()
                except (KeyboardInterrupt, EOFError):
                    break

                if not user_input:
                    continue

                output = self.handle_command(user_input)
                if output == "exit":
                    break
                print(f"\nAgent> {output}")

        finally:
            self.shutdown()

        return 0

    def handle_command(self, text: str) -> str:
        """Route commands to the appropriate handler."""
        normalized = text.strip().lower()

        # System commands
        if normalized in {"exit", "quit", "结束"}:
            return "exit"
        if normalized in {"help", "帮助"}:
            return self._help_text()
        if normalized in {"status", "状态"}:
            status = self.get_status()
            lines = [f"  {k}: {v}" for k, v in status.items() if k != "agent_status"]
            if "agent_status" in status:
                lines.append(f"\n{status['agent_status']}")
            return "\n".join(lines)
        if normalized in {"explore", "探索"}:
            if self.request_explore():
                return "已进入探索模式"
            return f"无法进入探索模式（当前状态: {self.sm.state.value}）"
        if normalized in {"stop", "停止"}:
            if self.request_idle():
                return "已停止探索，进入空闲状态"
            return f"无法停止（当前状态: {self.sm.state.value}）"
        if normalized in {"interrupt", "中断"}:
            if self.agent:
                return self.agent.interrupt()
            return "无当前任务"

        # Natural language instruction -> task execution
        return self.submit_task(text)

    # ---- Callbacks ----

    def _register_callbacks(self) -> None:
        """Register state machine callbacks for mode switching."""
        self.sm.on_enter(SystemState.EXPLORING, self._on_enter_exploring)
        self.sm.on_exit(SystemState.EXPLORING, self._on_exit_exploring)

    def _on_enter_exploring(self) -> None:
        if self.explore_ctrl:
            try:
                self.explore_ctrl.start_exploration()
                log.info("Exploration started")
            except ExplorationError as e:
                log.error("Failed to start exploration: %s", e)

    def _on_exit_exploring(self) -> None:
        if self.explore_ctrl:
            self.explore_ctrl.stop_exploration()
            log.info("Exploration stopped")

    # ---- Helpers ----

    def _signal_handler(self, signum: int, frame) -> None:
        log.info("Received signal %d, shutting down...", signum)
        self._shutting_down = True

    def _print_banner(self) -> None:
        mode = self.config.get("runtime.mode", "simulation")
        profile = self.config.profile_name
        n_objects = len(self.semantic_map.objects) if self.semantic_map else 0
        robots = self.config.get("robots.names", [])

        print("=" * 50)
        print("  Robot Navigation System")
        print(f"  Profile: {profile}")
        print(f"  Mode:    {mode}")
        print(f"  Robots:  {', '.join(robots)}")
        print(f"  Map:     {n_objects} objects")
        print("=" * 50)
        print("输入自然语言指令，或输入 help 查看帮助。")

    @staticmethod
    def _help_text() -> str:
        return (
            "可用命令:\n"
            "  <自然语言指令>   - 导航任务（自动判断单机/多机）\n"
            "  explore / 探索   - 启动自主探索模式\n"
            "  stop / 停止      - 停止探索，回到空闲\n"
            "  status / 状态    - 查看系统状态\n"
            "  interrupt / 中断 - 中断当前任务\n"
            "  help / 帮助      - 显示此帮助\n"
            "  exit / quit / 结束 - 退出系统\n"
            "\n"
            "示例:\n"
            "  去咖啡机拿杯咖啡送到沙发\n"
            "  让三个机器人分别去咖啡机、沙发和电视柜\n"
            "  先去咖啡机，完成后再送到沙发，同时另一个机器人去拿杯子"
        )
