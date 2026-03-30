"""Default configuration values for the robot navigation system.

These defaults are merged with the selected deployment profile.
Profile values override these defaults.
"""

from typing import Any, Dict

DEFAULT_CONFIG: Dict[str, Any] = {
    "profile": "manual_sim",

    "runtime": {
        "mode": "simulation",       # simulation | ros_single | ros_multi
        "use_ros": False,
        "auto_explore_on_startup": False,
        "resume_explore_after_task": False,
    },

    "llm": {
        "base_url": "https://api.deepseek.com",
        "api_key": "",
        "model": "deepseek-chat",
        "temperature": 0.2,
        "max_tokens": 1024,
        "timeout": 45,
    },

    "memory": {
        "max_history_turns": 8,
    },

    "planner": {
        "max_subtasks": 8,
        "strict_map_only": True,
    },

    "robots": {
        "names": ["robot_0"],
        "global_frame": "map",
        "base_frame_suffix": "base_footprint",
        "goal_timeout": 300.0,
        "server_timeout": 10.0,
        "default_positions": {
            "robot_0": [0.0, 0.0],
        },
    },

    "ros": {
        "move_base_action": "move_base",
        "frame_id": "map",
        "base_frame": "base_link",
    },

    "dmros": {
        "enabled": False,
        "output_dir": "output/semantic",
        "ros_stream_config": "",
        "poll_interval": 3.0,
        "use_llm_for_metadata": False,
    },

    "exploration": {
        "enabled": False,
        "launch_file": "",
        "planner_frequency": 0.33,
        "min_frontier_size": 0.75,
        "progress_timeout": 30.0,
    },

    "converter": {
        "robot_length": 0.281,
        "robot_width": 0.306,
        "safety_margin": 0.15,
    },

    "semantic_map": {
        "path": "data/semantic_map.json",
        "auto_reload": False,
    },

    "rgbd_topics": {
        "rgb": "/camera/rgb/image_raw",
        "depth": "/camera/depth/image_raw",
        "odom": "/odom",
        "camera_info": "/camera/rgb/camera_info",
    },
}
