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
    """Calibrate a raw microphone-array angle into base-relative yaw."""

    ACTION_ID = "ACT_INTERACT_RESPOND_CALL"

    def __init__(
        self,
        spin_relative: Callable[[float, float, float], bool],
        *,
        angle_zero_offset_deg: float = 90.0,
        angle_direction_sign: float = -1.0,
        angle_deadband_deg: float = 5.0,
        required_frame_id: str = "microphone_array",
        result_timeout_sec: float = 15.0,
        time_allowance_sec: float = 12.0,
        linear_array_back_search_enabled: bool = False,
        visual_confirm_timeout_sec: float = 1.5,
        visual_min_confidence: float = 0.6,
        visual_max_age_ms: float = 800.0,
        should_stop: Callable[[], bool] | None = None,
        prepare_motion: Callable[[], bool] | None = None,
        finish_motion: Callable[[], Any] | None = None,
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
        self._linear_array_back_search_enabled = bool(
            linear_array_back_search_enabled
        )
        self._visual_confirm_timeout_sec = self._positive_float(
            visual_confirm_timeout_sec,
            "visual_confirm_timeout_sec",
        )
        self._visual_min_confidence = min(
            1.0,
            max(
                0.0,
                self._finite_float(
                    visual_min_confidence,
                    "visual_min_confidence",
                ),
            ),
        )
        self._visual_max_age_ms = self._positive_float(
            visual_max_age_ms,
            "visual_max_age_ms",
        )
        self._should_stop = should_stop or (lambda: False)
        self._prepare_motion = prepare_motion
        self._finish_motion = finish_motion
        self._cancel_requested = threading.Event()
        self._visual_condition = threading.Condition()
        self._human_revision = 0

    def execute_step(
        self,
        unit_config: Mapping[str, Any],
        ctx: Any,
        duration: float | None = None,
    ) -> bool:
        """Validate and convert a wake angle, then invoke Nav2 Spin."""
        unit_id = str(unit_config.get("unit_id", ""))
        if unit_id != self.ACTION_ID:
            logger.error("Wake orientation cannot execute action %s", unit_id)
            return False

        step_deadline: float | None = None
        if duration is not None:
            try:
                step_budget = float(duration)
            except (TypeError, ValueError):
                logger.error("Invalid wake orientation duration=%r", duration)
                return False
            if not math.isfinite(step_budget) or step_budget <= 0.0:
                logger.error("Wake orientation budget is exhausted")
                return False
            step_deadline = time.monotonic() + step_budget

        self._cancel_requested.clear()

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
        within_deadband = abs(relative_deg) <= self._angle_deadband_deg
        if within_deadband:
            logger.info(
                "Wake source already ahead: raw=%.2fdeg, "
                "calibrated=%.2fdeg, deadband=%.2fdeg",
                raw_angle_deg,
                relative_deg,
                self._angle_deadband_deg,
            )
        else:
            logger.info(
                "Orienting to wake source: raw=%.2fdeg, calibrated=%.2fdeg, "
                "target_yaw=%.4frad, frame=%s",
                raw_angle_deg,
                relative_deg,
                math.radians(relative_deg),
                frame_id,
            )
        if within_deadband and not self._linear_array_back_search_enabled:
            return True

        # Nav2 turns the dog through the ordinary /cmd_vel topic, so from here
        # on an external publisher owns the chassis.  On Lite3 that only works
        # once the backend hands control over (Vision Mode); without this hook
        # the Spin goal is accepted and streamed at full speed while the dog,
        # still on the joystick, drops every sample and the unit dies on
        # behavior_goal_timeout.  Measured on hardware 2026-09-20: /cmd_vel
        # carried a constant -0.6 rad/s for the whole 8 s window while
        # /leg_odom2 yaw never moved and /robot_status motion_state stayed 0.
        # Same injection contract as navigation_adapter and the UWB follow
        # adapter.
        prepare = self._prepare_motion
        if callable(prepare) and not bool(prepare()):
            logger.error("Chassis rejected wake-orientation motion preflight")
            return False
        try:
            return self._orient_and_search(
                relative_deg,
                within_deadband,
                step_deadline,
            )
        finally:
            finish = self._finish_motion
            if callable(finish):
                finish()

    def _orient_and_search(
        self,
        relative_deg: float,
        within_deadband: bool,
        step_deadline: float | None,
    ) -> bool:
        """Turn to the calibrated heading while the chassis hook is held."""
        # Snapshot before the turn: the visual confirmation is about a human
        # that becomes trackable *after* the dog has moved, so a fresh
        # update_visual arriving during the Spin must count as new.
        human_revision = self._current_human_revision()
        if not within_deadband:
            if not self._spin_with_deadline(
                math.radians(relative_deg),
                step_deadline,
            ):
                return False

        if not self._linear_array_back_search_enabled:
            return True
        if self._wait_for_fresh_human(
            human_revision,
            self._remaining_step_sec(step_deadline),
        ):
            logger.info("Wake orientation confirmed a fresh visual human")
            return True
        if self._stopping() or self._deadline_expired(step_deadline):
            return False

        rear_deg = self._mirrored_rear_degrees(relative_deg)
        rear_delta_deg = self._normalise_degrees(rear_deg - relative_deg)
        if abs(rear_delta_deg) <= self._angle_deadband_deg:
            logger.info(
                "Linear-array rear candidate overlaps the first heading: "
                "front=%.2fdeg rear=%.2fdeg",
                relative_deg,
                rear_deg,
            )
            return True

        logger.info(
            "No visual human at first wake heading; checking mirrored rear "
            "candidate: first=%.2fdeg rear=%.2fdeg delta=%.2fdeg",
            relative_deg,
            rear_deg,
            rear_delta_deg,
        )
        human_revision = self._current_human_revision()
        if not self._spin_with_deadline(
            math.radians(rear_delta_deg),
            step_deadline,
        ):
            return False
        if self._wait_for_fresh_human(
            human_revision,
            self._remaining_step_sec(step_deadline),
        ):
            logger.info("Wake rear search confirmed a fresh visual human")
        else:
            logger.info("Wake rear search completed without a visual human")
        return not (
            self._stopping() or self._deadline_expired(step_deadline)
        )

    def update_visual(self, payload: Mapping[str, Any]) -> None:
        """Record a fresh, trackable human without selecting or locking it."""
        target = payload.get("active_target")
        if not isinstance(target, Mapping):
            return
        try:
            confidence = float(target.get("confidence", 0.0))
            age_ms = float(target.get("last_seen_age_ms", float("inf")))
        except (TypeError, ValueError):
            return
        if (
            not math.isfinite(confidence)
            or not math.isfinite(age_ms)
            or str(target.get("target_type", "")) != "human"
            or confidence < self._visual_min_confidence
            or not 0.0 <= age_ms <= self._visual_max_age_ms
            or str(target.get("tracking_state", "")) != "tracking"
        ):
            return
        with self._visual_condition:
            self._human_revision += 1
            self._visual_condition.notify_all()

    def cancel_step(self, step: Any = None) -> None:
        del step
        self._cancel_requested.set()
        with self._visual_condition:
            self._visual_condition.notify_all()
        cancel_spin = getattr(self._spin_relative, "cancel_spin", None)
        if callable(cancel_spin):
            cancel_spin()

    def emergency_stop(self) -> None:
        self.cancel_step()

    def _current_human_revision(self) -> int:
        with self._visual_condition:
            return self._human_revision

    def _wait_for_fresh_human(
        self,
        after_revision: int,
        remaining_step_sec: float | None = None,
    ) -> bool:
        timeout = self._visual_confirm_timeout_sec
        if remaining_step_sec is not None:
            timeout = min(timeout, remaining_step_sec)
        if timeout <= 0.0:
            return False
        deadline = time.monotonic() + timeout
        with self._visual_condition:
            while self._human_revision <= after_revision:
                if self._stopping():
                    return False
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._visual_condition.wait(timeout=min(0.05, remaining))
            return True

    def _spin_with_deadline(
        self,
        target_yaw_rad: float,
        step_deadline: float | None,
    ) -> bool:
        remaining = self._remaining_step_sec(step_deadline)
        if remaining is not None and remaining <= 0.0:
            return False
        result_timeout = self._result_timeout_sec
        time_allowance = self._time_allowance_sec
        if remaining is not None:
            result_timeout = min(result_timeout, remaining)
            time_allowance = min(time_allowance, remaining)
        succeeded = bool(
            self._spin_relative(
                target_yaw_rad,
                result_timeout,
                time_allowance,
            )
        )
        return succeeded and not self._deadline_expired(step_deadline)

    @staticmethod
    def _remaining_step_sec(step_deadline: float | None) -> float | None:
        if step_deadline is None:
            return None
        return max(0.0, step_deadline - time.monotonic())

    @classmethod
    def _deadline_expired(cls, step_deadline: float | None) -> bool:
        remaining = cls._remaining_step_sec(step_deadline)
        return remaining is not None and remaining <= 0.0

    def _stopping(self) -> bool:
        return self._cancel_requested.is_set() or bool(self._should_stop())

    @classmethod
    def _mirrored_rear_degrees(cls, front_deg: float) -> float:
        if abs(front_deg) < 1e-9:
            return -180.0
        rear = math.copysign(180.0 - abs(front_deg), front_deg)
        return cls._normalise_degrees(rear)

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
        try:
            total_timeout = float(result_timeout_sec)
        except (TypeError, ValueError):
            total_timeout = 0.0
        if not math.isfinite(total_timeout) or total_timeout <= 0.0:
            self._node.get_logger().error(
                "Nav2 Spin has no remaining time budget"
            )
            return False
        deadline = time.monotonic() + total_timeout
        server_wait = min(
            self._server_timeout_sec,
            max(0.0, deadline - time.monotonic()),
        )
        if server_wait <= 0.0:
            return False
        if not self._client.wait_for_server(
            timeout_sec=server_wait
        ):
            self._node.get_logger().error(
                f"Nav2 Spin server unavailable: {self._action_name}"
            )
            return False

        goal = self._action_type.Goal()
        goal.target_yaw = float(target_yaw_rad)
        try:
            requested_allowance = float(time_allowance_sec)
        except (TypeError, ValueError):
            requested_allowance = 0.0
        if (
            not math.isfinite(requested_allowance)
            or requested_allowance <= 0.0
        ):
            self._node.get_logger().error(
                "Nav2 Spin time allowance must be finite and positive"
            )
            return False
        remaining = max(0.0, deadline - time.monotonic())
        effective_time_allowance = min(requested_allowance, remaining)
        if effective_time_allowance <= 0.0:
            return False
        whole_seconds = int(effective_time_allowance)
        goal.time_allowance.sec = whole_seconds
        goal.time_allowance.nanosec = int(
            (effective_time_allowance - whole_seconds) * 1_000_000_000
        )

        self._node.get_logger().info(
            f"Nav2 Spin goal: target_yaw={goal.target_yaw:.4f}rad "
            f"({math.degrees(goal.target_yaw):.2f}deg), "
            f"time_allowance={effective_time_allowance:.1f}s"
        )
        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback,
        )
        send_future.add_done_callback(
            self._cancel_late_goal_if_requested
        )
        acceptance_wait = min(
            self._server_timeout_sec,
            max(0.0, deadline - time.monotonic()),
        )
        if not self._wait_future(send_future, acceptance_wait):
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
        result_wait = max(0.0, deadline - time.monotonic())
        if not self._wait_future(result_future, result_wait):
            self.cancel_spin()
            self._node.get_logger().error(
                "Nav2 Spin timed out after "
                f"{total_timeout:.1f}s total"
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
