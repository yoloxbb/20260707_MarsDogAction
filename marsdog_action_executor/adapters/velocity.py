"""Shared body-velocity command and ROS2 Twist publisher.

Go2 and Lite3 closed-loop controllers exchange the same body-frame velocity
triple even though their hardware command transports differ.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class TwistCommand:
    """Body-frame velocity command (vx, vy, vyaw)."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0

    @property
    def is_zero(self) -> bool:
        return (
            self.linear_x == 0.0
            and self.linear_y == 0.0
            and self.angular_z == 0.0
        )


class Ros2TwistPublisher:
    """Publish :class:`TwistCommand` on a ROS2 Twist topic."""

    def __init__(
        self,
        node: Any,
        topic: str = "/cmd_vel",
        qos_depth: int = 10,
        label: str = "Chassis",
    ) -> None:
        from geometry_msgs.msg import Twist

        self._message_type = Twist
        self._node = node
        self._topic = topic
        self._publisher = node.create_publisher(Twist, topic, qos_depth)
        self._label = str(label)
        self._last_logged_command: TwistCommand | None = None

    def __call__(self, command: TwistCommand) -> None:
        message = self._message_type()
        message.linear.x = float(command.linear_x)
        message.linear.y = float(command.linear_y)
        message.linear.z = 0.0
        message.angular.x = 0.0
        message.angular.y = 0.0
        message.angular.z = float(command.angular_z)
        self._publisher.publish(message)
        self._log_command_transition(command)

    def _log_command_transition(self, command: TwistCommand) -> None:
        if command == self._last_logged_command:
            return
        self._last_logged_command = command

        matched = self._publisher.get_subscription_count()
        detail = (
            f"{self._label} Twist -> {self._topic}: "
            f"linear=({command.linear_x:.3f}, {command.linear_y:.3f}), "
            f"angular_z={command.angular_z:.3f}, "
            f"matched_subscribers={matched}"
        )
        if not command.is_zero and matched == 0:
            self._node.get_logger().warning(
                detail + "; command has no matched chassis subscriber"
            )
        else:
            self._node.get_logger().info(detail)
