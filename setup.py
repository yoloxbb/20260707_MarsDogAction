"""ROS2 setup.py for marsdog_action_executor."""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "marsdog_action_executor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            f"share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        # Launch files
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        # Config files
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        # Emotion/behavior visualization assets
        (
            os.path.join(
                "share", package_name, "config", "emotion_images"
            ),
            glob("config/emotion_images/*"),
        ),
        # Behavior audio assets
        (
            os.path.join("share", package_name, "config", "sounds"),
            glob("config/sounds/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "action_executor_node = marsdog_action_executor.node:main",
            "emotion_display = marsdog_action_executor.emotion_display:main",
        ],
    },
)
