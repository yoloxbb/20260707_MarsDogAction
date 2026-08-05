"""Wake-source orientation adapter backed by the Nav2 ``Spin`` action.

``WakeOrientationAdapter`` is ROS-independent and owns the audio-angle
calibration/validation contract. ``Ros2Nav2SpinClient`` is the runtime bridge
to ``nav2_msgs/action/Spin``.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Callable, Mapping

logger = logging.getLogger(__name__)


class WakeOrientationAdapter:
    """Turn the chassis toward ``wake_angle_deg`` from ``base_link``."""

    ACTION_ID = "ACT_INTERACT_RESPOND_CALL"

    def __init__(
        self,
        spin_relative: Callable[[float, float, float], bool],
        *,
        angle_zero_offset_deg: float = 0.0,
        angle_direction_sign: float = 1.0,
        angle_deadband_deg: float = 5.0,
        required_frame_id: str = "base_link",
        result_timeout_sec: float = 15.0,
        time_allowance_sec: float = 12.0,
    ) -> None:
        self._spin_relative = spin_relative
        self._angle_zero_offset_deg = self._finite_float(
            angle_zero_offset_deg,
            "angle_zero_offset_deg",
        )
        direction_sign = self._finite_float(
            angle_direction_sign,
            "angle_direction_sign",
        )
        if direction_sign == 0.0:
            raise ValueError("angle_direction_sign must not be zero")
        self._angle_direction_sign = math.copysign(1.0, direction_sign)
        self._angle_deadband_deg = self._finite_float(
            angle_deadband_deg,
            "angle_deadband_deg",
        )
        if not 0.0 <= self._angle_deadband_deg < 180.0:
            raise ValueError("angle_deadband_deg must be within [0, 180)")
        self._required_frame_id = str(required_frame_id).strip()
        if not self._required_frame_id:
            raise ValueError("required_frame_id must be non-empty")
        self._result_timeout_sec = self._positive_float(
            result_timeout_sec,
            "result_timeout_sec",
        )
        self._time_allowance_sec = self._positive_float(
            time_allowance_sec,
            "time_allowance_sec",
        )

    def execute_step(
        self,
        unit_config: Mapping[str, Any],
        ctx: Any,
        duration: float | None = None,
    ) -> bool:
        """Validate and convert a wake angle, then invoke Nav2 Spin."""
        del duration
        unit_id = str(unit_config.get("unit_id", ""))
        if unit_id != self.ACTION_ID:
            logger.error("Wake orientation cannot execute action %s", unit_id)
            return False

        if not bool(getattr(ctx, "use_wake_angle", False)):
            logger.error(
                "%s requires params_json.use_wake_angle=true",
                self.ACTION_ID,
            )
            return False

        wake_angle_deg = getattr(ctx, "wake_angle_deg", None)
        if wake_angle_deg is None:
            logger.error(
                "%s requires finite params_json.wake_angle_deg",
                self.ACTION_ID,
            )
            return False
        try:
            raw_angle_deg = float(wake_angle_deg)
        except (TypeError, ValueError):
            logger.error("Invalid wake_angle_deg=%r", wake_angle_deg)
            return False
        if not math.isfinite(raw_angle_deg):
            logger.error("Invalid wake_angle_deg=%r", wake_angle_deg)
            return False

        frame_id = str(getattr(ctx, "wake_frame_id", "") or "").strip()
        if frame_id != self._required_frame_id:
            logger.error(
                "Wake angle frame %r is unsupported; expected %r",
                frame_id,
                self._required_frame_id,
            )
            return False

        calibrated_deg = self._angle_direction_sign * (
            raw_angle_deg - self._angle_zero_offset_deg
        )
        relative_deg = self._normalise_degrees(calibrated_deg)
        if abs(relative_deg) <= self._angle_deadband_deg:
            logger.info(
                "Wake source already ahead: raw=%.2fdeg, "
                "calibrated=%.2fdeg, deadband=%.2fdeg",
                raw_angle_deg,
                relative_deg,
                self._angle_deadband_deg,
            )
            return True

        target_yaw_rad = math.radians(relative_deg)
        logger.info(
            "Orienting to wake source: raw=%.2fdeg, calibrated=%.2fdeg, "
            "target_yaw=%.4frad, frame=%s",
            raw_angle_deg,
            relative_deg,
            target_yaw_rad,
            frame_id,
        )
        return bool(
            self._spin_relative(
                target_yaw_rad,
                self._result_timeout_sec,
                self._time_allowance_sec,
            )
        )

    def cancel_step(self, step: Any = None) -> None:
        del step
        cancel_spin = getattr(self._spin_relative, "cancel_spin", None)
        if callable(cancel_spin):
            cancel_spin()

    def emergency_stop(self) -> None:
        self.cancel_step()

    @staticmethod
    def _normalise_degrees(value: float) -> float:
        """Return the shortest signed angle in ``[-180, 180)``."""
        return (value + 180.0) % 360.0 - 180.0

    @staticmethod
    def _finite_float(value: Any, name: str) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be finite") from exc
        if not math.isfinite(result):
            raise ValueError(f"{name} must be finite")
        return result

    @classmethod
    def _positive_float(cls, value: Any, name: str) -> float:
        result = cls._finite_float(value, name)
        if result <= 0.0:
            raise ValueError(f"{name} must be > 0")
        return result


class Ros2Nav2SpinClient:
    """Synchronous facade over the asynchronous Nav2 ``Spin`` action."""

    def __init__(
        self,
        node: Any,
        *,
        action_name: str = "/spin",
        server_timeout_sec: float = 5.0,
        should_stop: Callable[[], bool] | None = None,
    ) -> None:
        from nav2_msgs.action import Spin
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup

        self._node = node
        self._action_type = Spin
        self._server_timeout_sec = float(server_timeout_sec)
        if (
            not math.isfinite(self._server_timeout_sec)
            or self._server_timeout_sec <= 0.0
        ):
            raise ValueError("server_timeout_sec must be finite and > 0")
        self._should_stop = should_stop or (lambda: False)
        self._client = ActionClient(
            node,
            Spin,
            action_name,
            callback_group=ReentrantCallbackGroup(),
        )
        self._action_name = action_name
        self._cancel_requested = threading.Event()
        self._goal_lock = threading.Lock()
        self._active_goal_handle: Any = None
        self._last_feedback_log = 0.0

    def __call__(
        self,
        target_yaw_rad: float,
        result_timeout_sec: float,
        time_allowance_sec: float,
    ) -> bool:
        from action_msgs.msg import GoalStatus

        self._cancel_requested.clear()
        if not self._client.wait_for_server(
            timeout_sec=self._server_timeout_sec
        ):
            self._node.get_logger().error(
                f"Nav2 Spin server unavailable: {self._action_name}"
            )
            return False

        goal = self._action_type.Goal()
        goal.target_yaw = float(target_yaw_rad)
        whole_seconds = int(time_allowance_sec)
        goal.time_allowance.sec = whole_seconds
        goal.time_allowance.nanosec = int(
            (float(time_allowance_sec) - whole_seconds) * 1_000_000_000
        )

        self._node.get_logger().info(
            f"Nav2 Spin goal: target_yaw={goal.target_yaw:.4f}rad "
            f"({math.degrees(goal.target_yaw):.2f}deg), "
            f"time_allowance={time_allowance_sec:.1f}s"
        )
        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback,
        )
        send_future.add_done_callback(
            self._cancel_late_goal_if_requested
        )
        if not self._wait_future(send_future, self._server_timeout_sec):
            self._node.get_logger().error(
                "Nav2 Spin goal was not accepted in time"
            )
            return False

        try:
            goal_handle = send_future.result()
        except Exception as exc:
            self._node.get_logger().error(
                f"Nav2 Spin goal send failed: {exc}"
            )
            return False
        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().error("Nav2 Spin goal was rejected")
            return False

        with self._goal_lock:
            self._active_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, result_timeout_sec):
            self.cancel_spin()
            self._node.get_logger().error(
                "Nav2 Spin timed out after "
                f"{float(result_timeout_sec):.1f}s"
            )
            return False

        try:
            wrapped_result = result_future.result()
        except Exception as exc:
            with self._goal_lock:
                self._active_goal_handle = None
            self._node.get_logger().error(
                f"Nav2 Spin result failed: {exc}"
            )
            return False
        with self._goal_lock:
            self._active_goal_handle = None

        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info("Nav2 Spin succeeded")
            return True

        self._node.get_logger().error(
            f"Nav2 Spin failed: status={wrapped_result.status}"
        )
        return False

    def cancel_spin(self) -> None:
        """Request cancellation of the active Spin goal, if any."""
        self._cancel_requested.set()
        with self._goal_lock:
            goal_handle = self._active_goal_handle
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self._node.get_logger().warning(
                    f"Failed to cancel Nav2 Spin goal: {exc}"
                )

    def _wait_future(self, future: Any, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while time.monotonic() < deadline:
            if future.done():
                return True
            if self._cancel_requested.is_set() or self._should_stop():
                self.cancel_spin()
                return False
            time.sleep(0.05)
        return future.done()

    def _on_feedback(self, feedback_message: Any) -> None:
        now = time.monotonic()
        if now - self._last_feedback_log < 1.0:
            return
        self._last_feedback_log = now
        traveled = float(
            feedback_message.feedback.angular_distance_traveled
        )
        self._node.get_logger().info(
            "Nav2 Spin feedback: "
            f"traveled={traveled:.4f}rad "
            f"({math.degrees(traveled):.2f}deg)"
        )

    def _cancel_late_goal_if_requested(self, send_future: Any) -> None:
        if not self._cancel_requested.is_set() and not self._should_stop():
            return
        try:
            goal_handle = send_future.result()
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
        except Exception as exc:
            self._node.get_logger().warning(
                f"Failed to cancel late Nav2 Spin goal: {exc}"
            )
