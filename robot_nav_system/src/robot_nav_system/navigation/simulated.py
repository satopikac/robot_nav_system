"""Simulated navigator -- no ROS required. User inputs success/fail."""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from ..agent.models import ExecutionRecord, SemanticObject, SubTask
from .base import BaseNavigator


class SimulatedNavigator(BaseNavigator):
    """Simulated navigator for testing without ROS."""

    def __init__(
        self,
        robots: List[str],
        default_positions: Dict[str, List[float]],
        frame_id: str = "map",
    ):
        self.robots = robots
        self._positions: Dict[str, Tuple[float, float]] = {
            r: (pos[0], pos[1]) for r, pos in default_positions.items() if r in robots
        }
        self.frame_id = frame_id

    def get_robot_positions(self) -> Dict[str, Tuple[float, float]]:
        return dict(self._positions)

    def execute_single(
        self, tasks: List[Tuple[SubTask, SemanticObject]]
    ) -> List[ExecutionRecord]:
        records: List[ExecutionRecord] = []
        for idx, (subtask, target) in enumerate(tasks):
            x, y, yaw = target.position
            print(f"\n[SIM-SINGLE] 步骤 {idx + 1}/{len(tasks)}: "
                  f"{subtask.action} -> {target.name}")
            print(f"  目标坐标: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
            print(f"  所在房间: {target.room}  |  类别: {target.category}")

            result = self._ask_result()
            records.append(
                ExecutionRecord(
                    step_index=idx + 1,
                    action=subtask.action,
                    target_name=subtask.target_name,
                    target_obj_id=subtask.target_obj_id,
                    success=(result == "success"),
                    message=f"simulated {result}",
                )
            )
        return records

    def execute_multi(
        self,
        assignments: Dict[str, List[Tuple[SubTask, SemanticObject]]],
        dependencies: Dict[int, Optional[int]],
    ) -> List[ExecutionRecord]:
        print("\n╔══════════════════════════════════════╗")
        print("║        多机器人任务分配方案          ║")
        print("╠══════════════════════════════════════╣")
        for robot, task_list in assignments.items():
            for st, obj in task_list:
                dep_str = ""
                if st.depends_on is not None:
                    dep_str = f"  (依赖任务 {st.depends_on})"
                print(f"║  {robot:>8s} -> {obj.name:<10s} "
                      f"({obj.position[0]:6.2f},{obj.position[1]:6.2f}){dep_str}")
        print("╚══════════════════════════════════════╝")

        records: List[ExecutionRecord] = []
        step = 0
        print("\n[SIM-MULTI] 开始模拟并行执行...\n")
        for robot, task_list in assignments.items():
            for subtask, target in task_list:
                step += 1
                x, y, yaw = target.position
                print(f"  [{robot}] {subtask.action} -> {target.name}  "
                      f"(x={x:.2f}, y={y:.2f})")
                result = self._ask_result(prompt_prefix=f"  [{robot}]")
                records.append(
                    ExecutionRecord(
                        step_index=step,
                        action=subtask.action,
                        target_name=subtask.target_name,
                        target_obj_id=subtask.target_obj_id,
                        success=(result == "success"),
                        message=f"simulated {result}",
                        robot=robot,
                    )
                )
        return records

    @staticmethod
    def _ask_result(prompt_prefix: str = " ") -> str:
        while True:
            ans = input(f"{prompt_prefix} 请输入结果 [success/fail]: ").strip().lower()
            if ans in ("success", "fail"):
                return ans
            print(f"{prompt_prefix} 无效输入，请输入 success 或 fail。")
