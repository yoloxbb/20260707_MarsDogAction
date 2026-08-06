"""AGV motion adapter backed by ROS2 ``geometry_msgs/msg/Twist``.

The core adapter is ROS-independent: callers provide a ``publish_twist``
callback, which makes motion timing and safety behaviour unit-testable without
ROS2. ``Ros2TwistPublisher`` is the thin runtime bridge to ``/cmd_vel``.
"""

from __future__ import annotations

import logging
import math
import random
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class TwistCommand:
    """Ground-vehicle velocity command."""

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
    """Convert :class:`TwistCommand` values to ROS2 Twist messages."""

    def __init__(self, node: Any, topic: str = "/cmd_vel", qos_depth: int = 10) -> None:
        from geometry_msgs.msg import Twist

        self._message_type = Twist
        self._node = node
        self._topic = topic
        self._publisher = node.create_publisher(Twist, topic, qos_depth)
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
        """Log segment changes without printing every rate-controlled sample."""
        if command == self._last_logged_command:
            return
        self._last_logged_command = command

        matched = self._publisher.get_subscription_count()
        detail = (
            f"AGV Twist -> {self._topic}: "
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


class AgvMotionAdapter:
    """Execute exact ``ACT_*`` IDs as configured Twist motion groups.

    A zero Twist is always published when a motion group finishes, fails, or is
    canceled. Velocity values are clamped to configured limits before being
    published.
    """

    def __init__(
        self,
        publish_twist: Callable[[TwistCommand], None],
        motion_groups: Mapping[str, list[Mapping[str, Any]]],
        action_motion_groups: Mapping[str, str],
        *,
        publish_rate_hz: float = 10.0,
        max_linear_x: float = 0.25,
        max_linear_y: float = 0.0,
        max_angular_z: float = 1.2,
        stop_publish_count: int = 3,
        should_stop: Callable[[], bool] | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        rate_hz = float(publish_rate_hz)
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be finite and > 0")
        self._publish_twist = publish_twist
        self._motion_groups: dict[str, Any] = {}
        for name, segments in motion_groups.items():
            if isinstance(segments, dict):
                # pick_random-style group stored as-is
                self._motion_groups[name] = dict(segments)
            else:
                self._motion_groups[name] = [dict(s) for s in segments]
        self._action_motion_groups = dict(action_motion_groups)
        self._period_sec = 1.0 / rate_hz
        self._max_linear_x = abs(float(max_linear_x))
        self._max_linear_y = abs(float(max_linear_y))
        self._max_angular_z = abs(float(max_angular_z))
        self._stop_publish_count = max(1, int(stop_publish_count))
        self._should_stop = should_stop or (lambda: False)
        self._monotonic = monotonic
        self._sleep = sleep
        self._stop_requested = threading.Event()
        self._execution_lock = threading.Lock()

    @property
    def mapped_actions(self) -> set[str]:
        return set(self._action_motion_groups)

    @property
    def motion_groups(self) -> set[str]:
        return set(self._motion_groups)

    def execute_step(
        self,
        unit_config: Mapping[str, Any],
        ctx: Any = None,
        duration: float | None = None,
    ) -> bool:
        """Publish a configured motion group at the configured rate."""
        if getattr(ctx, "motion_state", "active") == "stationary":
            return self.hold_position(duration)

        del duration  # Group segment durations are authoritative.

        unit_id = str(unit_config.get("unit_id", ""))
        group_name = self._action_motion_groups.get(unit_id)
        if group_name is None:
            logger.error("No AGV motion group is mapped for %s", unit_id)
            self._publish_stop()
            return False

        if unit_id == "ACT_SYSTEM_EMERGENCY_STOP":
            self.emergency_stop()
            return True

        return self.execute_group(group_name, ctx)

    def execute_group(
        self,
        group_name: str,
        ctx: Any = None,
    ) -> bool:
        """Execute a named motion group for behavior-stage mobility.

        Supports two group formats:

        - **List**: traditional fixed-segment sequence.
        - **Dict with ``type: pick_random``**: randomly selects one candidate
          direction and a random duration in *[min, max]* on every invocation.
        """
        if getattr(ctx, "motion_state", "active") == "stationary":
            self.hold_position()
            return True

        if group_name not in self._motion_groups:
            logger.error("Unknown AGV motion group: %s", group_name)
            self._publish_stop()
            return False
        if group_name == "stop":
            self.emergency_stop()
            return True

        speed_scale = self._positive_scale(
            getattr(ctx, "speed_scale", 1.0), "speed_scale",
        )
        duration_scale = self._positive_scale(
            getattr(ctx, "duration_scale", 1.0), "duration_scale",
        )

        group_config = self._motion_groups[group_name]

        completed = True
        with self._execution_lock:
            self._stop_requested.clear()
            try:
                if isinstance(group_config, dict) and group_config.get("type") == "pick_random":
                    completed = self._execute_pick_random(
                        group_config, speed_scale, duration_scale,
                    )
                else:
                    for segment in group_config:
                        command = self._command_from_segment(segment, speed_scale)
                        seg_dur = float(segment.get("duration_sec", 0.0)) * duration_scale
                        if not self._run_segment(command, seg_dur):
                            completed = False
                            break
            except Exception:
                logger.exception("AGV motion group %s failed", group_name)
                completed = False
            finally:
                self._publish_stop()

        return completed

    def _execute_pick_random(
        self,
        config: dict[str, Any],
        speed_scale: float,
        duration_scale: float,
    ) -> bool:
        """Execute a ``pick_random`` motion group.

        Randomly selects one candidate direction and assigns a random
        duration between *duration_min_sec* and *duration_max_sec*.
        """
        candidates = config.get("candidates", [])
        if not candidates:
            logger.error("pick_random group has no candidates")
            return False

        segment = dict(random.choice(candidates))
        dur_min = float(config.get("duration_min_sec", 1.0))
        dur_max = float(config.get("duration_max_sec", 3.0))
        segment["duration_sec"] = random.uniform(dur_min, dur_max) * duration_scale

        command = self._command_from_segment(segment, speed_scale)
        logger.debug(
            "pick_random: linear=(%.2f,%.2f) angular=%.2f  duration=%.2fs",
            command.linear_x, command.linear_y, command.angular_z,
            segment["duration_sec"],
        )
        return self._run_segment(command, segment["duration_sec"])

    def cancel_step(self, step: Any = None) -> None:
        """Stop the active group and publish zero velocity."""
        del step
        self._stop_requested.set()
        self._publish_stop()

    def hold_position(self, duration_sec: float | None = None) -> bool:
        """Force zero velocity, optionally reaffirming it for a stage duration."""
        self._stop_requested.set()
        self._publish_stop()
        try:
            duration = float(duration_sec or 0.0)
        except (TypeError, ValueError):
            duration = 0.0
        if not math.isfinite(duration) or duration <= 0.0:
            return True

        deadline = self._monotonic() + duration
        stop = TwistCommand()
        try:
            while self._monotonic() < deadline:
                if self._should_stop():
                    return False
                self._publish_twist(stop)
                remaining = deadline - self._monotonic()
                self._sleep(min(self._period_sec, max(0.0, remaining)))
            return True
        finally:
            self._publish_stop()

    def emergency_stop(self) -> None:
        """Immediately request stop and publish redundant zero commands."""
        self._stop_requested.set()
        self._publish_stop()

    def _run_segment(self, command: TwistCommand, duration_sec: float) -> bool:
        if duration_sec <= 0.0:
            return True

        deadline = self._monotonic() + duration_sec
        while self._monotonic() < deadline:
            if self._stop_requested.is_set() or self._should_stop():
                return False
            self._publish_twist(command)
            remaining = deadline - self._monotonic()
            self._sleep(min(self._period_sec, max(0.0, remaining)))
        return True

    def _command_from_segment(
        self,
        segment: Mapping[str, Any],
        speed_scale: float,
    ) -> TwistCommand:
        return TwistCommand(
            linear_x=self._clamp(
                float(segment.get("linear_x", 0.0)) * speed_scale,
                self._max_linear_x,
            ),
            linear_y=self._clamp(
                float(segment.get("linear_y", 0.0)) * speed_scale,
                self._max_linear_y,
            ),
            angular_z=self._clamp(
                float(segment.get("angular_z", 0.0)) * speed_scale,
                self._max_angular_z,
            ),
        )

    def _publish_stop(self) -> None:
        stop = TwistCommand()
        for _ in range(self._stop_publish_count):
            try:
                self._publish_twist(stop)
            except Exception as exc:
                logger.warning(
                    "Failed to publish AGV zero-velocity command: %s",
                    exc,
                )
                break

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return max(-limit, min(limit, value))

    @staticmethod
    def _positive_scale(value: Any, name: str) -> float:
        try:
            scale = float(value)
        except (TypeError, ValueError):
            logger.warning("Invalid AGV %s=%r; using 1.0", name, value)
            return 1.0
        if not math.isfinite(scale) or scale <= 0.0:
            logger.warning("Invalid AGV %s=%r; using 1.0", name, value)
            return 1.0
        return scale
