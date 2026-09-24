"""Unitree Go2 built-in UTrack owner-follow adapter.

The Go2 firmware exposes UTrack as a request/response DDS API on
``/api/uwbswitch/request`` and ``/api/uwbswitch/response``.  Action only owns
the session switch; the robot-side UTrack service owns ranging and locomotion.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping

from .velocity import TwistCommand

logger = logging.getLogger(__name__)


UTRACK_SWITCH_SET_API_ID = 1001


@dataclass(frozen=True)
class UtrackSwitchResult:
    """Result of one correlated UTrack switch request."""

    success: bool
    reason: str = ""
    code: int | None = None


class Ros2Go2UtrackClient:
    """Correlated ROS 2 client for the Go2 ``uwbswitch`` DDS service."""

    def __init__(
        self,
        node: Any,
        *,
        request_topic: str = "/api/uwbswitch/request",
        response_topic: str = "/api/uwbswitch/response",
        state_topic: str = "/uwbstate",
        response_timeout_sec: float = 2.0,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        try:
            from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
            from unitree_api.msg import Request, Response
            from unitree_go.msg import UwbState
        except ImportError as exc:  # pragma: no cover - board dependency
            raise RuntimeError(
                "Go2 UTrack messages are unavailable; source the unitree_ros2 "
                "workspace before starting chassis_type:=go2"
            ) from exc

        timeout = float(response_timeout_sec)
        if timeout <= 0.0:
            raise ValueError("response_timeout_sec must be > 0")

        self._node = node
        self._request_type = Request
        self._request_topic = str(request_topic)
        self._response_topic = str(response_topic)
        self._state_topic = str(state_topic)
        self._response_timeout_sec = timeout
        self._monotonic = monotonic
        self._condition = threading.Condition()
        self._pending_ids: set[int] = set()
        self._responses: dict[int, tuple[int, int, str]] = {}
        self._next_request_id = max(1, time.time_ns() & 0x7FFFFFFFFFFFFFFF)
        self._last_state: dict[str, Any] = {}

        # switch_set() may wait inside another ROS callback.  A distinct
        # callback group lets the MultiThreadedExecutor deliver the response.
        callback_group = MutuallyExclusiveCallbackGroup()
        self._publisher = node.create_publisher(Request, self._request_topic, 10)
        self._response_subscription = node.create_subscription(
            Response,
            self._response_topic,
            self._on_response,
            10,
            callback_group=callback_group,
        )
        self._state_subscription = node.create_subscription(
            UwbState,
            self._state_topic,
            self._on_state,
            10,
            callback_group=callback_group,
        )

    @property
    def request_topic(self) -> str:
        return self._request_topic

    @property
    def response_topic(self) -> str:
        return self._response_topic

    @property
    def state_topic(self) -> str:
        return self._state_topic

    @property
    def matched_subscribers(self) -> int:
        return int(self._publisher.get_subscription_count())

    @property
    def last_state(self) -> dict[str, Any]:
        with self._condition:
            return dict(self._last_state)

    def switch_set(
        self,
        enable: bool,
        timeout_sec: float | None = None,
    ) -> UtrackSwitchResult:
        matched = self.matched_subscribers
        if matched <= 0:
            return UtrackSwitchResult(False, "uwb_switch_no_subscriber")

        with self._condition:
            request_id = self._next_request_id
            self._next_request_id = (
                1 if request_id >= 0x7FFFFFFFFFFFFFFF else request_id + 1
            )
            self._pending_ids.add(request_id)

        message = self._request_type()
        message.header.identity.id = int(request_id)
        message.header.identity.api_id = UTRACK_SWITCH_SET_API_ID
        message.header.policy.noreply = False
        message.parameter = json.dumps(
            {"enable": 1 if enable else 0}, separators=(",", ":")
        )
        try:
            self._publisher.publish(message)
        except Exception as exc:
            with self._condition:
                self._pending_ids.discard(request_id)
            return UtrackSwitchResult(
                False, f"uwb_switch_publish_failed:{exc}"
            )
        self._node.get_logger().info(
            "Go2 UTrack -> %s: request_id=%d enable=%d "
            "matched_subscribers=%d"
            % (self._request_topic, request_id, int(enable), matched)
        )

        timeout = (
            self._response_timeout_sec
            if timeout_sec is None else max(0.0, float(timeout_sec))
        )
        deadline = self._monotonic() + timeout
        with self._condition:
            while request_id not in self._responses:
                remaining = deadline - self._monotonic()
                if remaining <= 0.0:
                    self._pending_ids.discard(request_id)
                    return UtrackSwitchResult(False, "uwb_switch_response_timeout")
                self._condition.wait(timeout=remaining)
            api_id, code, data = self._responses.pop(request_id)
            self._pending_ids.discard(request_id)

        if api_id != UTRACK_SWITCH_SET_API_ID:
            return UtrackSwitchResult(
                False,
                f"uwb_switch_api_mismatch:{api_id}",
                code,
            )
        if code != 0:
            return UtrackSwitchResult(
                False,
                f"uwb_switch_rejected:{code}:{data}",
                code,
            )
        return UtrackSwitchResult(True, code=code)

    def _on_response(self, message: Any) -> None:
        try:
            request_id = int(message.header.identity.id)
            api_id = int(message.header.identity.api_id)
            code = int(message.header.status.code)
            data = str(message.data)
        except (AttributeError, TypeError, ValueError):
            self._node.get_logger().warning(
                "Ignored malformed Go2 UTrack response"
            )
            return
        with self._condition:
            # The response Topic is shared by every UTrack client.  Never let
            # another client's response satisfy this process's pending call.
            if request_id not in self._pending_ids:
                return
            self._responses[request_id] = (api_id, code, data)
            self._condition.notify_all()

    def _on_state(self, message: Any) -> None:
        with self._condition:
            self._last_state = {
                "received_monotonic": self._monotonic(),
                "enabled_from_app": bool(
                    getattr(message, "enabled_from_app", False)
                ),
                "distance_est": float(getattr(message, "distance_est", 0.0)),
                "yaw_est": float(getattr(message, "yaw_est", 0.0)),
                "error_state": int(getattr(message, "error_state", 0)),
            }


class Go2UtrackFollowAdapter:
    """Manage one session-scoped robot-side UTrack follow switch."""

    def __init__(
        self,
        *,
        enabled: bool,
        switch_set: Callable[[bool, float | None], UtrackSwitchResult],
        response_timeout_sec: float = 2.0,
        stop_publish_count: int = 3,
        publish_twist: Callable[[TwistCommand], None] | None = None,
        should_stop: Callable[[], bool] | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._enabled = bool(enabled)
        self._switch_set = switch_set
        self._response_timeout_sec = max(0.01, float(response_timeout_sec))
        self._stop_publish_count = max(1, int(stop_publish_count))
        self._publish_twist = publish_twist or (lambda command: None)
        self._should_stop = should_stop or (lambda: False)
        self._monotonic = monotonic
        self._sleep = sleep
        self._lock = threading.RLock()
        self._cancel_requested = threading.Event()
        self._interaction_id = ""
        self._desired_follow = False
        self._session_controlled = False
        self._suspended = False
        self._switch_enabled = False
        # A timed-out/rejected enable request can still have reached the
        # robot.  Keep this uncertainty until a disable is acknowledged.
        self._switch_may_be_enabled = False
        self._last_error = ""

    @property
    def active(self) -> bool:
        with self._lock:
            return self._switch_enabled

    @property
    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    def update_control(self, payload: Mapping[str, Any]) -> bool:
        enabled = bool(payload.get("enabled", False))
        mode = str(payload.get("mode", "face_body_centering"))
        interaction_id = str(payload.get("interaction_id", "")).strip()
        if enabled and mode in ("follow", "follow_owner"):
            with self._lock:
                self._desired_follow = True
                self._session_controlled = True
                if interaction_id:
                    self._interaction_id = interaction_id
                suspended = self._suspended
            return True if suspended else self.start()

        with self._lock:
            current_id = self._interaction_id
        if not interaction_id or not current_id or interaction_id == current_id:
            with self._lock:
                self._desired_follow = False
                self._session_controlled = False
                self._interaction_id = ""
            return self._disable("attention_follow_disabled")
        return True

    def execute_step(
        self,
        unit_config: Mapping[str, Any],
        ctx: Any = None,
        duration: float | None = None,
    ) -> bool:
        del unit_config
        self._cancel_requested.clear()
        interaction_id = str(getattr(ctx, "interaction_id", "") or "").strip()
        with self._lock:
            if interaction_id:
                self._interaction_id = interaction_id
            standalone = not self._session_controlled
            if standalone:
                self._desired_follow = True

        if not self.start():
            raise RuntimeError(self.last_error or "uwb_switch_enable_failed")
        if not standalone:
            return True

        deadline = self._monotonic() + max(0.0, float(duration or 0.0))
        while self._monotonic() < deadline:
            if self._cancel_requested.is_set() or self._should_stop():
                self.cancel_step()
                return False
            self._sleep(min(0.05, deadline - self._monotonic()))
        with self._lock:
            self._desired_follow = False
            self._interaction_id = ""
        return self._disable("standalone_follow_completed")

    def start(self) -> bool:
        with self._lock:
            if not self._enabled:
                self._last_error = "uwb_follow_disabled"
                self._publish_stop_locked()
                return False
            if self._switch_enabled:
                return True
            if self._switch_may_be_enabled:
                if not self._disable_locked("recover_unknown_switch_state"):
                    return False
            self._switch_may_be_enabled = True
            result = self._call_switch_locked(True)
            if not result.success:
                enable_error = result.reason or "uwb_switch_enable_failed"
                # The request may have reached the robot even when its
                # response was lost.  Compensate with an explicit disable.
                cleanup_ok = self._disable_locked("enable_failed_cleanup")
                self._last_error = (
                    enable_error
                    if cleanup_ok else f"{enable_error};{self._last_error}"
                )
                logger.error("Go2 UTrack enable failed: %s", self._last_error)
                self._publish_stop_locked()
                return False
            self._switch_enabled = True
            self._last_error = ""
            logger.info("Go2 built-in UTrack follow enabled")
            return True

    def poll_health(self) -> bool:
        with self._lock:
            return not self._desired_follow or self._switch_enabled or self._suspended

    def cancel_step(self, step: Any = None) -> None:
        del step
        self._cancel_requested.set()
        with self._lock:
            self._desired_follow = False
            self._session_controlled = False
            self._interaction_id = ""
        self._disable("canceled")

    def emergency_stop(self) -> None:
        self._cancel_requested.set()
        with self._lock:
            self._desired_follow = False
            self._session_controlled = False
            self._interaction_id = ""
        self._disable("emergency_stop")

    def suspend(self) -> None:
        with self._lock:
            self._suspended = True
        self._disable("foreground_behavior")

    def resume(self) -> bool:
        with self._lock:
            self._suspended = False
            desired = self._desired_follow
        return self.start() if desired else True

    def stop(self, reason: str = "stopped") -> None:
        with self._lock:
            self._desired_follow = False
            self._session_controlled = False
            self._interaction_id = ""
        self._disable(reason)

    def _disable(self, reason: str) -> bool:
        with self._lock:
            return self._disable_locked(reason)

    def _disable_locked(self, reason: str) -> bool:
        should_disable = self._switch_enabled or self._switch_may_be_enabled
        was_enabled = self._switch_enabled
        self._switch_enabled = False
        result = (
            self._call_switch_locked(False)
            if should_disable else UtrackSwitchResult(True)
        )
        self._publish_stop_locked()
        if not result.success:
            # Preserve the uncertain state so a later stop/emergency-stop can
            # retry the disable instead of assuming the robot is already off.
            self._switch_may_be_enabled = should_disable
            self._last_error = result.reason or "uwb_switch_disable_failed"
            logger.error("Go2 UTrack disable failed: %s", self._last_error)
            return False
        self._switch_may_be_enabled = False
        if was_enabled:
            logger.info("Go2 built-in UTrack follow disabled: %s", reason)
        self._last_error = ""
        return True

    def _call_switch_locked(self, enable: bool) -> UtrackSwitchResult:
        try:
            return self._switch_set(enable, self._response_timeout_sec)
        except Exception as exc:
            return UtrackSwitchResult(
                False, f"uwb_switch_transport_error:{exc}"
            )

    def _publish_stop_locked(self) -> None:
        # UTrack owns robot-side motion while enabled.  After disabling it,
        # redundant StopMove commands close the fail-safe boundary.
        for _ in range(self._stop_publish_count):
            try:
                self._publish_twist(TwistCommand())
            except Exception as exc:
                logger.warning("Failed to publish Go2 UTrack stop: %s", exc)
