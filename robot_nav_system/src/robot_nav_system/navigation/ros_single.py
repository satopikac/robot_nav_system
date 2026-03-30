"""Single-robot ROS move_base navigator with goal cancellation support."""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from ..agent.models import ExecutionRecord, SemanticObject, SubTask
from ..logging_setup import get_logger
from .base import BaseNavigator

log = get_logger("ros_single")


class ROSSingleNavigator(BaseNavigator):
    """Single-robot ROS move_base navigator."""

    def __init__(
        self,
        move_base_action: str = "move_base",
        frame_id: str = "map",
        base_frame: str = "base_link",
        goal_timeout: float = 300.0,
    ):
        import rospy
        import actionlib
        import tf
        from move_base_msgs.msg import MoveBaseAction

        self.frame_id = frame_id
        self.base_frame = base_frame
        self.goal_timeout = goal_timeout

        self.tf_listener = tf.TransformListener()
        rospy.loginfo("Waiting for move_base server [%s]...", move_base_action)
        self.client = actionlib.SimpleActionClient(move_base_action, MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base")

    def get_robot_positions(self) -> Dict[str, Tuple[float, float]]:
        import rospy
        try:
            (trans, _rot) = self.tf_listener.lookupTransform(
                self.frame_id, self.base_frame, rospy.Time(0)
            )
            return {"robot": (trans[0], trans[1])}
        except Exception:
            return {}

    def cancel_all_goals(self) -> None:
        """Cancel the current move_base goal."""
        try:
            self.client.cancel_all_goals()
            log.info("Cancelled all move_base goals")
        except Exception as e:
            log.warning("Failed to cancel goals: %s", e)

    def execute_single(
        self, tasks: List[Tuple[SubTask, SemanticObject]]
    ) -> List[ExecutionRecord]:
        import rospy
        import tf
        from geometry_msgs.msg import PoseStamped
        from move_base_msgs.msg import MoveBaseGoal
        from actionlib_msgs.msg import GoalStatus

        records: List[ExecutionRecord] = []
        for idx, (subtask, target) in enumerate(tasks):
            x, y, yaw = target.position
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)

            goal = MoveBaseGoal()
            goal.target_pose = PoseStamped()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.x = q[0]
            goal.target_pose.pose.orientation.y = q[1]
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]

            rospy.loginfo("[Single] Step %d: navigate -> %s (%.2f, %.2f)",
                          idx + 1, target.name, x, y)
            self.client.send_goal(goal)
            finished = self.client.wait_for_result(rospy.Duration(self.goal_timeout))

            if not finished:
                self.client.cancel_goal()
                success, msg = False, "GOAL_TIMEOUT"
            else:
                state = self.client.get_state()
                success = (state == GoalStatus.SUCCEEDED)
                msg = "SUCCEEDED" if success else f"STATE_{state}"

            rospy.loginfo("[Single] Step %d result: %s", idx + 1, msg)
            records.append(
                ExecutionRecord(
                    step_index=idx + 1,
                    action=subtask.action,
                    target_name=subtask.target_name,
                    target_obj_id=subtask.target_obj_id,
                    success=success,
                    message=msg,
                )
            )
        return records

    def execute_multi(
        self,
        assignments: Dict[str, List[Tuple[SubTask, SemanticObject]]],
        dependencies: Dict[int, Optional[int]],
    ) -> List[ExecutionRecord]:
        raise NotImplementedError("ROSSingleNavigator does not support multi-robot execution")
