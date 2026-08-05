"""Launch file for the MarsDog Action Executor node.

Usage::

    ros2 launch marsdog_action_executor action_executor.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="marsdog_action_executor",
                executable="action_executor_node",
                name="action_executor_node",
                output="screen",
                parameters=[],
            ),
        ]
    )
