"""Multi-robot ROS navigator with namespaced move_base and goal cancellation."""
from __future__ import annotations

import threading
from typing import Dict, List, Optional, Tuple

from ..agent.models import ExecutionRecord, SemanticObject, SubTask
from ..logging_setup import get_logger
from .base import BaseNavigator

log = get_logger("ros_multi")


class ROSMultiNavigator(BaseNavigator):
    """Multi-robot ROS navigator using namespaced move_base action servers."""

    def __init__(
        self,
        robots: List[str],
        global_frame: str = "map",
        base_frame_suffix: str = "base_footprint",
        goal_timeout: float = 300.0,
        server_timeout: float = 10.0,
    ):
        import rospy
        import tf2_ros

        self.robots = robots
        self.global_frame = global_frame
        self.base_frame_suffix = base_frame_suffix
        self.goal_timeout = goal_timeout
        self.server_timeout = server_timeout

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Track active action clients for cancellation
        self._active_clients: Dict[str, object] = {}
        self._clients_lock = threading.Lock()

    def get_robot_positions(self) -> Dict[str, Tuple[float, float]]:
        import rospy
        import tf2_ros

        positions: Dict[str, Tuple[float, float]] = {}
        for robot in self.robots:
            base_frame = f"{robot}/{self.base_frame_suffix}"
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.global_frame, base_frame, rospy.Time(0), rospy.Duration(0.5)
                )
                positions[robot] = (
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass
        return positions

    def cancel_all_goals(self) -> None:
        """Cancel all active move_base goals across all robots."""
        with self._clients_lock:
            for robot, client in self._active_clients.items():
                try:
                    client.cancel_all_goals()
                    log.info("Cancelled goals for %s", robot)
                except Exception as e:
                    log.warning("Failed to cancel goals for %s: %s", robot, e)

    def execute_single(
        self, tasks: List[Tuple[SubTask, SemanticObject]]
    ) -> List[ExecutionRecord]:
        robot = self.robots[0]
        return self._execute_robot_tasks(robot, tasks, start_step=1)

    def execute_multi(
        self,
        assignments: Dict[str, List[Tuple[SubTask, SemanticObject]]],
        dependencies: Dict[int, Optional[int]],
    ) -> List[ExecutionRecord]:
        import rospy

        all_records: List[ExecutionRecord] = []
        lock = threading.Lock()
        step_counter = [0]
        completed_indices: Dict[int, threading.Event] = {}

        all_subtask_global_indices: List[int] = []
        for robot, task_list in assignments.items():
            for st, obj in task_list:
                g_idx = id(st)
                completed_indices[g_idx] = threading.Event()
                all_subtask_global_indices.append(g_idx)

        dep_events: Dict[int, threading.Event] = {}
        for g_idx, dep in dependencies.items():
            if dep is not None and dep in completed_indices:
                dep_events[g_idx] = completed_indices[dep]

        def worker(robot: str, task_list: List[Tuple[SubTask, SemanticObject]]) -> None:
            records = self._execute_robot_tasks_with_deps(
                robot, task_list, step_counter, lock, completed_indices, dep_events
            )
            with lock:
                all_records.extend(records)

        threads: List[threading.Thread] = []
        for robot, task_list in assignments.items():
            t = threading.Thread(target=worker, args=(robot, task_list), daemon=True)
            threads.append(t)
            t.start()

        for t in threads:
            t.join()

        rospy.loginfo("=== Multi-robot execution complete ===")
        return all_records

    def _execute_robot_tasks(
        self,
        robot: str,
        tasks: List[Tuple[SubTask, SemanticObject]],
        start_step: int = 1,
    ) -> List[ExecutionRecord]:
        import rospy
        import actionlib
        from tf.transformations import quaternion_from_euler
        from geometry_msgs.msg import PoseStamped
        from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
        from actionlib_msgs.msg import GoalStatus

        ns = f"/{robot}/move_base"
        client = actionlib.SimpleActionClient(ns, MoveBaseAction)
        rospy.loginfo("[%s] Waiting for move_base server...", robot)
        if not client.wait_for_server(rospy.Duration(self.server_timeout)):
            rospy.logerr("[%s] move_base server timeout", robot)
            return [
                ExecutionRecord(
                    step_index=start_step + i,
                    action=st.action,
                    target_name=st.target_name,
                    target_obj_id=st.target_obj_id,
                    success=False,
                    message="SERVER_TIMEOUT",
                    robot=robot,
                )
                for i, (st, _) in enumerate(tasks)
            ]

        with self._clients_lock:
            self._active_clients[robot] = client

        records: List[ExecutionRecord] = []
        for i, (subtask, target) in enumerate(tasks):
            x, y, yaw = target.position
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)

            goal = MoveBaseGoal()
            goal.target_pose = PoseStamped()
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.header.frame_id = self.global_frame
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.x = qx
            goal.target_pose.pose.orientation.y = qy
            goal.target_pose.pose.orientation.z = qz
            goal.target_pose.pose.orientation.w = qw

            rospy.loginfo("[%s] send goal: %s (%.2f, %.2f)", robot, target.name, x, y)
            client.send_goal(goal)
            finished = client.wait_for_result(rospy.Duration(self.goal_timeout))

            if not finished:
                client.cancel_goal()
                success, msg = False, "GOAL_TIMEOUT"
            else:
                state = client.get_state()
                success = (state == GoalStatus.SUCCEEDED)
                msg = "SUCCEEDED" if success else f"STATE_{state}"

            rospy.loginfo("[%s] result: %s", robot, msg)
            records.append(
                ExecutionRecord(
                    step_index=start_step + i,
                    action=subtask.action,
                    target_name=subtask.target_name,
                    target_obj_id=subtask.target_obj_id,
                    success=success,
                    message=msg,
                    robot=robot,
                )
            )

        with self._clients_lock:
            self._active_clients.pop(robot, None)

        return records

    def _execute_robot_tasks_with_deps(
        self,
        robot: str,
        task_list: List[Tuple[SubTask, SemanticObject]],
        step_counter: List[int],
        lock: threading.Lock,
        completed_indices: Dict[int, threading.Event],
        dep_events: Dict[int, threading.Event],
    ) -> List[ExecutionRecord]:
        import rospy
        import actionlib
        from tf.transformations import quaternion_from_euler
        from geometry_msgs.msg import PoseStamped
        from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
        from actionlib_msgs.msg import GoalStatus

        ns = f"/{robot}/move_base"
        client = actionlib.SimpleActionClient(ns, MoveBaseAction)
        if not client.wait_for_server(rospy.Duration(self.server_timeout)):
            return []

        with self._clients_lock:
            self._active_clients[robot] = client

        records: List[ExecutionRecord] = []
        for subtask, target in task_list:
            st_id = id(subtask)

            if st_id in dep_events:
                rospy.loginfo("[%s] Waiting for dependency...", robot)
                dep_events[st_id].wait()

            x, y, yaw = target.position
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)

            goal = MoveBaseGoal()
            goal.target_pose = PoseStamped()
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.header.frame_id = self.global_frame
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.x = qx
            goal.target_pose.pose.orientation.y = qy
            goal.target_pose.pose.orientation.z = qz
            goal.target_pose.pose.orientation.w = qw

            rospy.loginfo("[%s] navigate -> %s", robot, target.name)
            client.send_goal(goal)
            finished = client.wait_for_result(rospy.Duration(self.goal_timeout))

            if not finished:
                client.cancel_goal()
                success, msg = False, "GOAL_TIMEOUT"
            else:
                state = client.get_state()
                success = (state == GoalStatus.SUCCEEDED)
                msg = "SUCCEEDED" if success else f"STATE_{state}"

            with lock:
                step_counter[0] += 1
                step = step_counter[0]

            records.append(
                ExecutionRecord(
                    step_index=step,
                    action=subtask.action,
                    target_name=subtask.target_name,
                    target_obj_id=subtask.target_obj_id,
                    success=success,
                    message=msg,
                    robot=robot,
                )
            )
            if st_id in completed_indices:
                completed_indices[st_id].set()

        with self._clients_lock:
            self._active_clients.pop(robot, None)

        return records
