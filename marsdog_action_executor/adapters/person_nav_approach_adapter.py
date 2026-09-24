"""One-shot person localization followed by one Nav2 goal.

The Vision service owns bbox/depth synchronization.  This adapter never
drives toward a bounding box. Explicit approach commands bind one fresh
human track from the visual event stream before requesting localization.
"""

from __future__ import annotations

import json
import math
import threading
import time
import uuid
from typing import Any, Mapping

from .target_approach_adapter import TargetApproachResult
from .velocity import TwistCommand


class PersonNavApproachAdapter:
    ACTION_ID = "ACT_INTERACT_APPROACH_VOICE_CALLER"
    OWNER_ACTION_BEHAVIORS = {
        "ACT_INTERACT_APPROACH_OWNER": "come_to_owner",
        "ACT_INTERACT_APPROACH_OWNER_CLOSER": "approach_owner",
        "ACT_INTERACT_RETURN_OWNER": "return_to_owner",
    }
    MIN_STAND_OFF_M = 1.5
    TARGET_MAX_AGE_SEC = 0.35
    VISUAL_EVENT_MAX_AGE_SEC = 0.5
    TARGET_ACQUIRE_TIMEOUT_SEC = 2.0

    def __init__(self, transport: Any, publish_stop: Any, *,
                 should_stop=None, chassis_backend=None,
                 target_acquire_timeout_sec: float = TARGET_ACQUIRE_TIMEOUT_SEC):
        self._transport = transport
        self._publish_stop = publish_stop
        self._should_stop = should_stop or (lambda: False)
        self._chassis_backend = chassis_backend
        self._cancel = threading.Event()
        self._execution_lock = threading.Lock()
        self._visual_lock = threading.Lock()
        self._latest_visual: tuple[dict[str, Any], float] | None = None
        self._target_acquire_timeout_sec = float(target_acquire_timeout_sec)
        if (not math.isfinite(self._target_acquire_timeout_sec)
                or self._target_acquire_timeout_sec <= 0):
            raise ValueError("target acquisition timeout must be finite and positive")

    def update_visual(self, payload: Mapping[str, Any]) -> None:
        """Cache the latest event; invalid packets revoke an earlier target."""
        with self._visual_lock:
            self._latest_visual = (dict(payload), time.monotonic()) if payload else None

    def _current_human_target(self) -> dict[str, Any] | None:
        with self._visual_lock:
            snapshot = self._latest_visual
        if snapshot is None:
            return None
        payload, received_at = snapshot
        elapsed_sec = time.monotonic() - received_at
        if elapsed_sec > self.VISUAL_EVENT_MAX_AGE_SEC:
            return None
        target = payload.get("active_target")
        if not isinstance(target, Mapping):
            return None
        epoch = str(target.get("vision_epoch", ""))
        target_id = str(target.get("target_id", ""))
        if (target.get("target_type") != "human"
                or target.get("tracking_state") != "tracking"
                or not epoch or epoch != str(payload.get("vision_epoch", ""))
                or not target_id.startswith(epoch + ":human:")
                or not target_id[len(epoch) + len(":human:"):]):
            return None
        try:
            age_ms = float(target.get("last_seen_age_ms", -1))
        except (TypeError, ValueError):
            return None
        if (not math.isfinite(age_ms) or age_ms < 0
                or age_ms + elapsed_sec * 1000 > self.TARGET_MAX_AGE_SEC * 1000):
            return None
        return dict(target)

    def _acquire_human_target(self, deadline: float) -> dict[str, Any] | None:
        acquire_deadline = min(deadline, time.monotonic() + self._target_acquire_timeout_sec)
        while time.monotonic() < acquire_deadline:
            if self._cancel.is_set() or self._should_stop():
                return None
            target = self._current_human_target()
            if target is not None:
                return target
            time.sleep(min(0.02, max(0.0, acquire_deadline - time.monotonic())))
        return None

    @property
    def recovery_required(self) -> bool:
        return bool(getattr(self._transport, "recovery_required", False))

    def cancel_task(self) -> None:
        self._cancel.set()
        self._transport.cancel()

    def emergency_stop(self) -> None:
        self.cancel_task()
        self._stop()

    def _stop(self) -> None:
        for _ in range(3):
            self._publish_stop(TwistCommand())

    def _feedback(self, ctx: Any, phase: str, progress: float) -> None:
        reporter = getattr(ctx, "report_runtime_feedback", None)
        if callable(reporter):
            reporter(
                progress,
                str(getattr(ctx, "current_unit", "") or self.ACTION_ID),
                phase,
                True,
            )

    def _finish(self, ctx: Any, success: bool, reason: str, *,
                canceled: bool = False, timed_out: bool = False,
                navigation_required: bool | None = None) -> TargetApproachResult:
        self._stop()
        target = getattr(ctx, "target", None)
        metadata = {
            "state": "arrived" if success else "canceled" if canceled else "failed",
            "reason": reason,
            "navigation_required": navigation_required,
            "recovery_required": self.recovery_required,
            "target_id": str(target.get("target_id", "")) if isinstance(target, Mapping) else "",
            "target_identity": str(target.get("identity", "")) if isinstance(target, Mapping) else "",
        }
        ctx.metadata["target_approach"] = metadata
        self._feedback(ctx, metadata["state"], 1.0 if success else 0.0)
        return TargetApproachResult(success, reason, metadata, canceled, timed_out)

    def execute_task(self, unit_config: Mapping[str, Any], ctx: Any,
                     timeout_sec: float) -> TargetApproachResult:
        unit_id = str(unit_config.get("unit_id", ""))
        if unit_id != self.ACTION_ID and unit_id not in self.OWNER_ACTION_BEHAVIORS:
            return self._finish(ctx, False, "unsupported_action")
        target = getattr(ctx, "target", None)
        params = getattr(ctx, "params", {})
        if unit_id == self.ACTION_ID:
            if not isinstance(target, Mapping) or target.get("target_type") != "human":
                return self._finish(ctx, False, "human_target_required")
            role = str(params.get("speaker_role", ""))
            speaker_id = str(params.get("speaker_id", ""))
            if (not params.get("wake_id") or not params.get("interaction_id")
                    or params.get("speaker_status") != "matched"
                    or not ((role == "owner" and speaker_id == "owner")
                            or (role == "family" and speaker_id in {
                                "family_member_1", "family_member_2",
                                "family_member_3", "family_member_4"}))):
                return self._finish(ctx, False, "wake_identity_not_authorized")
        elif (getattr(ctx, "resolved_behavior_name", "")
                != self.OWNER_ACTION_BEHAVIORS[unit_id]):
            return self._finish(ctx, False, "owner_target_not_authorized")
        if params.get("strict_target_lock") is not True or params.get("allow_target_switch") is not False:
            return self._finish(ctx, False, "target_lock_required")
        try:
            stand_off = float(params.get("stand_off_distance_m", 1.5))
            budget = float(timeout_sec)
        except (TypeError, ValueError):
            return self._finish(ctx, False, "invalid_approach_budget")
        if not math.isfinite(stand_off) or stand_off < self.MIN_STAND_OFF_M:
            return self._finish(ctx, False, "unsafe_stand_off_distance")
        if not math.isfinite(budget) or budget <= 0:
            return self._finish(ctx, False, "invalid_approach_budget")

        with self._execution_lock:
            if self.recovery_required:
                return self._finish(ctx, False, "operator_recovery_required")
            self._cancel.clear()
            deadline = time.monotonic() + budget
            if self._should_stop():
                return self._finish(ctx, False, "canceled_before_localization", canceled=True)
            if unit_id != self.ACTION_ID:
                self._feedback(ctx, "acquiring_human_target", 0.05)
                target = self._acquire_human_target(deadline)
                if target is None:
                    if self._cancel.is_set() or self._should_stop():
                        return self._finish(ctx, False, "canceled_before_localization", canceled=True)
                    return self._finish(ctx, False, "visual_target_unavailable")
                ctx.target = target
                ctx.metadata["target_resolution"] = "action_visual_event"
            epoch = str(target.get("vision_epoch", ""))
            target_id = str(target.get("target_id", ""))
            if not epoch or not target_id.startswith(epoch + ":human:"):
                return self._finish(ctx, False, "invalid_target_identity")
            self._feedback(ctx, "locating_person", 0.1)
            located, reason = self._transport.locate(
                target_id, stand_off, deadline, self._cancel
            )
            if self._cancel.is_set() or self._should_stop():
                return self._finish(ctx, False, "canceled_during_localization", canceled=True)
            if located is None:
                return self._finish(ctx, False, reason)
            if located.get("target_id") != target_id or located.get("ok") is not True:
                return self._finish(ctx, False, "localization_target_mismatch")
            if located.get("status") != 0 or not self._valid_person_point(located.get("person_point")):
                return self._finish(ctx, False, "invalid_localization_success")
            if located.get("navigation_required") is False:
                return self._finish(ctx, True, "already_within_stand_off", navigation_required=False)
            if located.get("navigation_required") is not True:
                return self._finish(ctx, False, "navigation_required_missing")
            pose = located.get("navigation_goal")
            if not self._transport.valid_pose(pose):
                return self._finish(ctx, False, "invalid_navigation_goal")
            if time.monotonic() >= deadline:
                return self._finish(ctx, False, "approach_budget_expired", timed_out=True)
            if self._cancel.is_set() or self._should_stop():
                return self._finish(ctx, False, "canceled_before_navigation", canceled=True)
            self._feedback(ctx, "navigating_to_person", 0.5)
            prepare = getattr(self._chassis_backend, "prepare_navigation", None)
            finish = getattr(self._chassis_backend, "finish_navigation", None)
            try:
                prepared = not callable(prepare) or bool(prepare())
            except Exception as exc:
                self._transport.recovery_required = True
                return self._finish(
                    ctx, False,
                    "chassis_preflight_exception:%s:operator_recovery_required" % type(exc).__name__,
                )
            if not prepared:
                return self._finish(ctx, False, "chassis_navigation_preflight_failed")
            try:
                if self._cancel.is_set() or self._should_stop():
                    success, canceled, nav_reason = (
                        False, True, "canceled_before_navigation"
                    )
                else:
                    success, canceled, nav_reason = self._transport.navigate(
                        pose, deadline, self._cancel
                    )
            except Exception as exc:
                self._transport.recovery_required = True
                success, canceled, nav_reason = (
                    False, False,
                    "navigation_exception:%s:operator_recovery_required" % type(exc).__name__,
                )
            finally:
                try:
                    settled = not callable(finish) or finish() is not False
                except Exception:
                    settled = False
            if not settled:
                self._transport.recovery_required = True
                success, canceled, nav_reason = (
                    False, False,
                    "chassis_navigation_settle_failed:operator_recovery_required",
                )
            if self._cancel.is_set() or self._should_stop():
                canceled = canceled or not self.recovery_required
                success = False
            return self._finish(
                ctx, success, nav_reason, canceled=canceled,
                timed_out=nav_reason == "navigation_timeout",
                navigation_required=True,
            )

    @staticmethod
    def _valid_person_point(value: Any) -> bool:
        if not isinstance(value, Mapping):
            return False
        header = value.get("header")
        point = value.get("point")
        if (not isinstance(header, Mapping) or header.get("frame_id") != "map"
                or not isinstance(point, Mapping)):
            return False
        try:
            coords = [float(point[key]) for key in ("x", "y", "z")]
        except (KeyError, TypeError, ValueError):
            return False
        return all(math.isfinite(coord) for coord in coords)


class Ros2PersonApproachTransport:
    """ROS futures are drained through real Nav2 terminal results."""

    def __init__(self, node: Any, *, vision_service: str = "/perception/vision/task",
                 nav_action: str = "/navigate_to_pose", service_timeout_sec: float = 30.0,
                 server_timeout_sec: float = 10.0):
        from marsdog_vision_interaction.srv import VisionTask
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup

        self._node = node
        self._vision_type = VisionTask
        self._nav_type = NavigateToPose
        self._service_timeout = float(service_timeout_sec)
        self._server_timeout = float(server_timeout_sec)
        if (not math.isfinite(self._service_timeout) or self._service_timeout <= 0
                or not math.isfinite(self._server_timeout) or self._server_timeout <= 0):
            raise ValueError("person approach service and server timeouts must be finite and positive")
        self._vision = node.create_client(
            VisionTask, vision_service, callback_group=ReentrantCallbackGroup()
        )
        self._nav = ActionClient(
            node, NavigateToPose, nav_action, callback_group=ReentrantCallbackGroup()
        )
        self._lock = threading.Lock()
        self._goal = None
        self._cancel_sent = False
        self.recovery_required = False

    def cancel(self) -> None:
        with self._lock:
            goal = self._goal
            if goal is None or self._cancel_sent:
                return
            self._cancel_sent = True
        goal.cancel_goal_async()

    @staticmethod
    def _wait(future: Any, deadline: float, cancel: threading.Event) -> bool:
        while not future.done():
            if cancel.is_set() or time.monotonic() >= deadline:
                return False
            time.sleep(0.02)
        return True

    def locate(self, target_id: str, stand_off: float, deadline: float,
               cancel: threading.Event) -> tuple[dict | None, str]:
        service_deadline = min(deadline, time.monotonic() + self._service_timeout)
        while time.monotonic() < service_deadline and not cancel.is_set():
            if self._vision.wait_for_service(timeout_sec=min(
                0.1, max(0.0, service_deadline - time.monotonic())
            )):
                break
        else:
            return None, "canceled_before_localization" if cancel.is_set() else "vision_service_unavailable"
        if cancel.is_set():
            return None, "canceled_before_localization"
        request = self._vision_type.Request()
        request.task_id = "person-locate-" + uuid.uuid4().hex
        request.task_type = "locate_person_once"
        request.params_json = json.dumps({
            "target_id": target_id, "stand_off_distance": stand_off,
        })
        future = self._vision.call_async(request)
        response_deadline = min(deadline, time.monotonic() + self._service_timeout)
        if not self._wait(future, response_deadline, cancel):
            remove_pending = getattr(self._vision, "remove_pending_request", None)
            if callable(remove_pending):
                try:
                    remove_pending(future)
                except Exception:
                    pass
            future.cancel()
            return None, "vision_localization_timeout"
        try:
            response = future.result()
            result = json.loads(response.result_json or "{}")
        except Exception as exc:
            return None, "vision_localization_error:%s" % exc
        if not isinstance(result, dict):
            return None, "vision_localization_invalid_response"
        if not response.success:
            return None, str(result.get("error_code") or response.error_message or "vision_localization_failed")
        return result, "ok"

    @staticmethod
    def valid_pose(pose: Any) -> bool:
        if not isinstance(pose, Mapping):
            return False
        header = pose.get("header")
        p = pose.get("pose")
        if not isinstance(header, Mapping) or header.get("frame_id") != "map" or not isinstance(p, Mapping):
            return False
        position = p.get("position")
        orientation = p.get("orientation")
        try:
            values = [float(position[k]) for k in ("x", "y", "z")]
            values += [float(orientation[k]) for k in ("x", "y", "z", "w")]
        except (TypeError, ValueError, KeyError):
            return False
        norm_squared = sum(v * v for v in values[3:])
        return (all(math.isfinite(value) for value in values)
                and 0.9 <= norm_squared <= 1.1)

    def navigate(self, pose: Mapping[str, Any], deadline: float,
                 cancel: threading.Event) -> tuple[bool, bool, str]:
        from action_msgs.msg import GoalStatus
        from rclpy import ok as ros_ok

        server_wait = min(self._server_timeout, max(0.0, deadline - time.monotonic()))
        if server_wait <= 0 or not self._nav.wait_for_server(timeout_sec=server_wait):
            return False, False, "nav2_server_unavailable"
        if cancel.is_set():
            return False, True, "canceled_before_navigation"
        goal = self._nav_type.Goal()
        goal.pose.header.frame_id = pose["header"]["frame_id"]
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        for key in ("x", "y", "z"):
            setattr(goal.pose.pose.position, key, float(pose["pose"]["position"][key]))
        for key in ("x", "y", "z", "w"):
            setattr(goal.pose.pose.orientation, key, float(pose["pose"]["orientation"][key]))
        sent = self._nav.send_goal_async(goal)
        # A timeout after dispatch has uncertain acceptance.  Latch recovery
        # and cancel any late accepted handle before accepting another Goal.
        accept_deadline = min(deadline, time.monotonic() + self._server_timeout)
        while not sent.done() and ros_ok() and time.monotonic() < accept_deadline:
            time.sleep(0.02)
        if not sent.done():
            self.recovery_required = True
            sent.add_done_callback(self._cancel_late_accepted_goal)
            return False, False, "nav2_acceptance_unknown:operator_recovery_required"
        try:
            handle = sent.result()
        except Exception as exc:
            return False, False, "nav2_send_error:%s" % exc
        if handle is None or not handle.accepted:
            return False, False, "nav2_goal_rejected"
        with self._lock:
            self._goal = handle
            self._cancel_sent = False
        result_future = handle.get_result_async()
        timed_out = False
        cancel_deadline = None
        try:
            while not result_future.done() and ros_ok():
                if cancel.is_set() or time.monotonic() >= deadline:
                    timed_out = not cancel.is_set()
                    self.cancel()
                    if cancel_deadline is None:
                        cancel_deadline = time.monotonic() + self._server_timeout
                if cancel_deadline is not None and time.monotonic() >= cancel_deadline:
                    self.recovery_required = True
                    return False, False, "nav2_terminal_unknown:operator_recovery_required"
                time.sleep(0.02)
            if not result_future.done():
                self.recovery_required = True
                return False, False, "nav2_result_unknown_after_shutdown:operator_recovery_required"
            wrapped = result_future.result()
            if wrapped is None:
                return False, False, "nav2_empty_result"
            status = wrapped.status
            if timed_out:
                return False, False, "navigation_timeout"
            if cancel.is_set() or status == GoalStatus.STATUS_CANCELED:
                return False, True, "navigation_canceled"
            if status == GoalStatus.STATUS_SUCCEEDED:
                return True, False, "navigation_succeeded"
            return False, False, "navigation_status_%s" % status
        finally:
            if result_future.done():
                with self._lock:
                    self._goal = None
                    self._cancel_sent = False
            else:
                with self._lock:
                    # Permit an emergency-stop retry while recovery is latched.
                    self._cancel_sent = False

    def _cancel_late_accepted_goal(self, future: Any) -> None:
        try:
            handle = future.result()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
                handle.get_result_async()
        except Exception as exc:
            self._node.get_logger().error("Late Nav2 goal cancel failed: %s" % exc)
