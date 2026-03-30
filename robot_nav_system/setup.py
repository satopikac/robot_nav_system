from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["robot_nav_system",
              "robot_nav_system.core",
              "robot_nav_system.agent",
              "robot_nav_system.navigation",
              "robot_nav_system.perception",
              "robot_nav_system.config"],
    package_dir={"": "src"},
)

setup(**d)
