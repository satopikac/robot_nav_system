"""Mode controllers: ExploreController and TaskController.

ExploreController manages dynamic start/stop of explore-lite.
TaskController wraps the NavigationAgent for task execution.
"""
from __future__ import annotations

import subprocess
import time
from pathlib import Path
from typing import TYPE_CHECKING, List, Optional

from ..config import Config
from ..exceptions import ExplorationError
from ..logging_setup import get_logger

if TYPE_CHECKING:
    from ..agent.agent import NavigationAgent
    from ..navigation.base import BaseNavigator
    from .state_machine import SystemStateMachine

log = get_logger("mode_controller")


class ExploreController:
    """Manages the lifecycle of explore-lite processes.

    In ROS mode, uses subprocess to launch/kill explore-lite launch files.
    In simulation mode, this is a no-op.
    """

    def __init__(self, config: Config, navigator: Optional[BaseNavigator] = None):
        self._config = config
        self._navigator = navigator
        self._use_ros = bool(config.get("runtime.use_ros", False))
        self._exploration_enabled = bool(config.get("exploration.enabled", False))
        self._launch_file = str(config.get("exploration.launch_file", ""))
        self._processes: List[subprocess.Popen] = []
        self._exploring = False

    @property
    def is_exploring(self) -> bool:
        return self._exploring

    def start_exploration(self) -> bool:
        """Launch explore-lite nodes. Returns True on success."""
        if self._exploring:
            log.warning("Exploration already running")
            return True

        if not self._use_ros or not self._exploration_enabled:
            log.info("Exploration not available in this mode (use_ros=%s, enabled=%s)",
                     self._use_ros, self._exploration_enabled)
            self._exploring = True  # Mark as exploring even in sim mode
            return True

        if not self._launch_file:
            raise ExplorationError("No exploration launch file configured")

        try:
            # Try to use roslaunch Python API first
            try:
                import roslaunch
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)

                # Resolve the launch file path
                launch_path = self._resolve_launch_path()
                if not launch_path:
                    raise ExplorationError(f"Launch file not found: {self._launch_file}")

                parent = roslaunch.parent.ROSLaunchParent(uuid, [str(launch_path)])
                parent.start()
                self._processes.append(parent)
                log.info("Started explore-lite via roslaunch API: %s", launch_path)
            except ImportError:
                # Fallback: use subprocess
                launch_path = self._resolve_launch_path()
                if not launch_path:
                    raise ExplorationError(f"Launch file not found: {self._launch_file}")

                proc = subprocess.Popen(
                    ["roslaunch", str(launch_path)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                self._processes.append(proc)
                log.info("Started explore-lite via subprocess: %s (PID=%d)",
                         launch_path, proc.pid)

            self._exploring = True
            return True

        except ExplorationError:
            raise
        except Exception as e:
            raise ExplorationError(f"Failed to start exploration: {e}") from e

    def stop_exploration(self) -> bool:
        """Kill explore-lite and cancel move_base goals. Returns True on success."""
        if not self._exploring:
            return True

        # Cancel all move_base goals first
        if self._navigator:
            try:
                self._navigator.cancel_all_goals()
                log.info("Cancelled all navigation goals")
            except Exception as e:
                log.warning("Failed to cancel goals: %s", e)

        # Kill explore-lite processes
        for proc in self._processes:
            try:
                if hasattr(proc, 'shutdown'):
                    # roslaunch parent
                    proc.shutdown()
                elif hasattr(proc, 'terminate'):
                    # subprocess
                    proc.terminate()
                    proc.wait(timeout=3.0)
            except Exception as e:
                log.warning("Failed to stop explore process: %s", e)
                try:
                    if hasattr(proc, 'kill'):
                        proc.kill()
                except Exception:
                    pass

        self._processes.clear()
        self._exploring = False

        # Brief wait for robots to stop
        time.sleep(0.5)
        log.info("Exploration stopped")
        return True

    def _resolve_launch_path(self) -> Optional[Path]:
        """Try to find the launch file."""
        # Try relative to package launch dir
        pkg_dir = Path(__file__).resolve().parent.parent.parent.parent
        candidates = [
            pkg_dir / "launch" / self._launch_file,
            pkg_dir / self._launch_file,
            Path(self._launch_file),
        ]
        for p in candidates:
            if p.exists():
                return p
        return None


class TaskController:
    """Wraps NavigationAgent for task execution within the state machine."""

    def __init__(self, agent: NavigationAgent, state_machine: SystemStateMachine):
        self._agent = agent
        self._sm = state_machine

    def execute_instruction(self, instruction: str) -> str:
        """Run the full plan->execute->summarize pipeline.

        The caller (Orchestrator) is responsible for state transitions.
        """
        # Ensure semantic map is up-to-date
        self._agent.semantic_map.reload()
        return self._agent.run_instruction(instruction)

    def cancel_current(self) -> str:
        """Cancel the current task."""
        return self._agent.interrupt()
