#!/usr/bin/env python3
"""DMROS bridge node -- launches DMROS as a subprocess and manages its lifecycle."""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main():
    try:
        import rospy
        rospy.init_node("dmros_bridge", anonymous=False)
        config_path = rospy.get_param("~dmros_config", "")

        if not config_path:
            rospy.logwarn("No dmros_config param set, DMROS bridge idle")
            rospy.spin()
            return

        # Find DMROS runner
        dmros_dir = Path(__file__).resolve().parent.parent.parent / "third_party" / "DMROS"
        runner = dmros_dir / "applications" / "runner_ros.py"

        if not runner.exists():
            rospy.logerr("DMROS runner not found at %s", runner)
            return

        rospy.loginfo("Starting DMROS with config: %s", config_path)
        proc = subprocess.Popen(
            [sys.executable, str(runner), "--config", config_path],
            cwd=str(dmros_dir),
        )

        rospy.on_shutdown(lambda: proc.terminate())
        proc.wait()

    except ImportError:
        print("ROS not available. DMROS bridge requires ROS.")
        sys.exit(1)


if __name__ == "__main__":
    main()
