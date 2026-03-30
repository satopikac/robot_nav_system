#!/usr/bin/env python3
"""System orchestrator ROS node / standalone entry point.

Usage:
  # Manual simulation (no ROS):
  python3 system_orchestrator_node.py --profile manual_sim

  # TurtleBot3 Gazebo (requires ROS):
  rosrun robot_nav_system system_orchestrator_node.py _profile:=turtlebot3_sim

  # Real robot (requires ROS):
  rosrun robot_nav_system system_orchestrator_node.py _profile:=real_robot
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Add src to path for standalone execution
_src_dir = Path(__file__).resolve().parent.parent / "src"
if str(_src_dir) not in sys.path:
    sys.path.insert(0, str(_src_dir))

from robot_nav_system.config import Config
from robot_nav_system.core import SystemOrchestrator
from robot_nav_system.logging_setup import setup_logging


def main() -> int:
    setup_logging("INFO")

    # Try ROS param first, then CLI arg
    profile_name = "manual_sim"
    use_ros = False

    try:
        import rospy
        rospy.init_node("system_orchestrator", anonymous=False)
        profile_name = rospy.get_param("~profile", "turtlebot3_sim")
        use_ros = True
    except (ImportError, Exception):
        # No ROS available, parse CLI args
        parser = argparse.ArgumentParser(description="Robot Navigation System")
        parser.add_argument("--profile", default="manual_sim",
                            help="Deployment profile name")
        parser.add_argument("--profiles-dir", default=None,
                            help="Directory containing profile JSON files")
        args = parser.parse_args()
        profile_name = args.profile

    # Resolve profiles directory
    default_profiles_dir = Path(__file__).resolve().parent.parent / "config" / "profiles"
    profiles_dir = default_profiles_dir

    config = Config.from_profile(profile_name, profiles_dir=profiles_dir)

    orchestrator = SystemOrchestrator(config)
    orchestrator.startup()

    if use_ros:
        # ROS mode: register services and spin
        _setup_ros_services(orchestrator)
        try:
            import rospy
            rospy.loginfo("System orchestrator ready, spinning...")
            rospy.spin()
        except KeyboardInterrupt:
            pass
        finally:
            orchestrator.shutdown()
    else:
        # Non-ROS mode: REPL
        return orchestrator.run_repl()

    return 0


def _setup_ros_services(orchestrator: SystemOrchestrator) -> None:
    """Register ROS services for external interaction."""
    try:
        import rospy
        from std_msgs.msg import String

        def execute_task_cb(req):
            """Handle ExecuteTask service calls."""
            result = orchestrator.submit_task(req.instruction)
            # Return as simple response
            class Resp:
                def __init__(self, success, result):
                    self.success = success
                    self.result = result
            return Resp("任务失败" not in result and "系统异常" not in result, result)

        def switch_mode_cb(req):
            if req.mode in ("explore", "探索"):
                ok = orchestrator.request_explore()
                msg = "已进入探索模式" if ok else "无法进入探索模式"
            elif req.mode in ("idle", "stop", "停止"):
                ok = orchestrator.request_idle()
                msg = "已停止" if ok else "无法停止"
            else:
                ok = False
                msg = f"未知模式: {req.mode}"
            class Resp:
                def __init__(self, success, message):
                    self.success = success
                    self.message = message
            return Resp(ok, msg)

        def get_status_cb(req):
            status = orchestrator.get_status()
            class Resp:
                def __init__(self, state, details):
                    self.state = state
                    self.details = details
            return Resp(status["state"], str(status))

        # Publish state on a topic
        state_pub = rospy.Publisher("~system_state", String, queue_size=1, latch=True)

        def publish_state():
            state_pub.publish(String(data=orchestrator.sm.state.value))

        # Register callbacks to publish state changes
        from robot_nav_system.core.state_machine import SystemState
        for state in SystemState:
            orchestrator.sm.on_enter(state, publish_state)

        rospy.loginfo("ROS services registered (use rostopic/rosservice to interact)")

    except ImportError:
        pass


if __name__ == "__main__":
    sys.exit(main())
