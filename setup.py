from setuptools import setup, find_packages

setup(
    name="robot_nav_system",
    version="0.1.0",
    package_dir={"": "robot_nav_system/src"},
    packages=find_packages(where="robot_nav_system/src"),
    python_requires=">=3.8",
    install_requires=[
        "openai>=1.0.0",
        "numpy>=1.20.0",
        "scipy>=1.7.0",
    ],
)
