"""ROS2 client adapter for the go2_uwb_behavior follow chain.

The follow feature no longer belongs to this repository's process pipeline.
The upstream package ``go2_uwb_behavior`` owns the whole loop -- UWB input,
planning and the /cmd_vel output -- and exposes it as:

    /go2/follow_uwb     action   FollowUwb
    /go2/random_roam    action   RandomRoam      (uwb_roam_adapter.py)
    /go2/set_behavior   service  SetBehavior     (accepts IDLE=0 / STOP=2)

Launching ``uwb_aoa_pkg`` or ``local_follow.launch.py`` ourselves is now
forbidden (docs/上层对接与部署.md §4.1): they publish /cmd_vel as well, and
would fight the chain, which publishes /cmd_vel at 20 Hz from startup whether
or not it is following anything.  So this adapter starts no process at all --
it is a pure client.

That same 20 Hz stream is why Lite3 motion ownership has to change hands
explicitly.  ``Lite3ChassisBackend`` counts the chain node as a competing
owner and fails twist actions with ``lite3_control_conflict``; the follow
adapter is the single caller allowed to keep it running, and says so by
calling ``prepare_navigation(allow_uwb_chain=True)``.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Callable, Mapping

logger = logging.getLogger(__name__)

# FollowUwb result codes, spelled out so logs name the failure instead of
# printing a bare integer.
FOLLOW_RESULT_CODES: dict[int, str] = {
    0: "SUCCESS",
    1: "CANCELED",
    2: "PREEMPTED_BY_MODE",
    3: "NOT_READY",
    4: "TIMEOUT",
    5: "INPUT_TIMEOUT",
    6: "STOP_UNCONFIRMED",
}

FOLLOW_FEEDBACK_STATES: dict[int, str] = {
    0: "STARTING",
    1: "FOLLOWING",
    2: "HOLDING",
    3: "INPUT_PAUSED",
    4: "STOPPING",
}

SET_BEHAVIOR_IDLE = 0
SET_BEHAVIOR_STOP = 2

# An explicit FollowUwb timeout must sit in this range; 0 is reserved for
# "follow until canceled" and is only correct for a background session.
MIN_FOLLOW_TIMEOUT_SEC = 5.0
MAX_FOLLOW_TIMEOUT_SEC = 3600.0

_POLL_INTERVAL_SEC = 0.05

# Why a wait ended, so callers can tell our own budget expiring apart from an
# external cancel -- they mean opposite things for the Stage result.
_WAIT_DONE = "done"
_WAIT_CANCELED = "canceled"
_WAIT_TIMEOUT = "timeout"


class Ros2FollowUwbClient:
    """Synchronous facade over the asynchronous ``/go2/follow_uwb`` action.

    Modelled on ``Ros2Nav2SpinClient`` in ``wake_orientation_adapter``: the
    same polling wait loop, the same late-goal cancel callback, and the same
    assumption that the executor runs on a MultiThreadedExecutor so these
    futures can complete while the behavior callback blocks.
    """

    def __init__(
        self,
        node: Any,
        *,
        action_name: str = "/go2/follow_uwb",
        server_timeout_sec: float = 5.0,
        should_stop: Callable[[], bool] | None = None,
    ) -> None:
        from go2_uwb_behavior.action import FollowUwb
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup

        self._node = node
        self._action_type = FollowUwb
        self._action_name = str(action_name)
        self._server_timeout_sec = float(server_timeout_sec)
        if (
            not math.isfinite(self._server_timeout_sec)
            or self._server_timeout_sec <= 0.0
        ):
            raise ValueError("server_timeout_sec must be finite and > 0")
        self._should_stop = should_stop or (lambda: False)
        self._client = ActionClient(
            node,
            FollowUwb,
            self._action_name,
            callback_group=ReentrantCallbackGroup(),
        )
        self._cancel_requested = threading.Event()
        self._goal_lock = threading.Lock()
        self._active_goal_handle: Any = None
        self._result_future: Any = None
        self._last_feedback: dict[str, float] | None = None
        self._last_feedback_at: float | None = None
        self._last_state_name = ""
        self._last_feedback_log = 0.0

    @property
    def last_feedback(self) -> Mapping[str, float] | None:
        with self._goal_lock:
            return self._last_feedback

    @property
    def last_state_name(self) -> str:
        with self._goal_lock:
            return self._last_state_name

    @property
    def last_feedback_at(self) -> float | None:
        with self._goal_lock:
            return self._last_feedback_at

    def has_active_goal(self) -> bool:
        with self._goal_lock:
            return self._active_goal_handle is not None

    def wait_for_server(self, timeout_sec: float) -> bool:
        if timeout_sec <= 0.0:
            return False
        return bool(self._client.wait_for_server(timeout_sec=timeout_sec))

    def start_follow(
        self, timeout_sec: float, *, deadline: float
    ) -> tuple[Any | None, str]:
        """Send one follow goal and wait for it to be accepted.

        Returns ``(goal_handle, "")`` or ``(None, reason)``.
        """
        with self._goal_lock:
            self._last_feedback = None
            self._last_feedback_at = None
            self._last_state_name = ""

        goal = self._action_type.Goal()
        goal.timeout_sec = float(timeout_sec)

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        send_future.add_done_callback(self._cancel_late_goal_if_requested)

        acceptance_wait = min(
            self._server_timeout_sec, max(0.0, deadline - time.monotonic())
        )
        if self._wait_future(send_future, acceptance_wait) != _WAIT_DONE:
            # Acceptance is unknown. The late-goal callback will cancel if it
            # arrives, but the caller must retain chassis ownership.
            self._cancel_requested.set()
            return None, "uwb_follow_goal_acceptance_unknown"

        try:
            goal_handle = send_future.result()
        except Exception as exc:
            return None, f"uwb_follow_goal_send_failed:{exc}"
        if goal_handle is None or not goal_handle.accepted:
            return None, "uwb_follow_goal_rejected"

        with self._goal_lock:
            self._active_goal_handle = goal_handle
            # One future for the whole lifetime of the goal: polling it is how
            # poll_health notices a background session ending on its own.
            self._result_future = goal_handle.get_result_async()
        return goal_handle, ""

    def reset_cancel(self) -> None:
        self._cancel_requested.clear()

    def wait_for_result(self, deadline: float) -> tuple[Any, str]:
        """Wait for the terminal result.

        Returns ``(wrapped_result, "")``, or ``(None, _WAIT_CANCELED |
        _WAIT_TIMEOUT)`` when the wait was cut short.
        """
        with self._goal_lock:
            result_future = self._result_future
        if result_future is None:
            return None, "uwb_follow_goal_missing"
        status = self._wait_future(
            result_future, max(0.0, deadline - time.monotonic())
        )
        if status != _WAIT_DONE:
            return None, status
        try:
            wrapped = result_future.result()
        except Exception as exc:
            with self._goal_lock:
                self._clear_goal_locked()
            return None, f"uwb_follow_result_failed:{exc}"
        with self._goal_lock:
            self._clear_goal_locked()
        return wrapped, ""

    def poll_result(self) -> tuple[Any, str]:
        """Non-blocking check for a background session's terminal result."""
        with self._goal_lock:
            result_future = self._result_future
            if self._active_goal_handle is None or result_future is None:
                return None, "uwb_follow_no_active_goal"
            if not result_future.done():
                return None, ""
            self._clear_goal_locked()
        try:
            return result_future.result(), ""
        except Exception as exc:
            return None, f"uwb_follow_result_failed:{exc}"

    def cancel(self) -> None:
        """Request cancellation of the active goal, if any."""
        self._cancel_requested.set()
        with self._goal_lock:
            goal_handle = self._active_goal_handle
        if goal_handle is None:
            return
        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:
            logger.warning("Failed to cancel follow goal: %s", exc)

    def take_goal_handle(self) -> Any:
        """Take the active handle out of this client without cancelling it."""
        with self._goal_lock:
            goal_handle = self._active_goal_handle
            self._clear_goal_locked()
        return goal_handle

    def _clear_goal_locked(self) -> None:
        self._active_goal_handle = None
        self._result_future = None

    def _wait_future(self, future: Any, timeout_sec: float) -> str:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while time.monotonic() < deadline:
            if future.done():
                return _WAIT_DONE
            if self._cancel_requested.is_set() or self._should_stop():
                self.cancel()
                return _WAIT_CANCELED
            time.sleep(_POLL_INTERVAL_SEC)
        return _WAIT_DONE if future.done() else _WAIT_TIMEOUT

    def _on_feedback(self, feedback_message: Any) -> None:
        feedback = feedback_message.feedback
        state = int(getattr(feedback, "state", -1))
        reading = {
            "state": float(state),
            "distance": float(getattr(feedback, "distance", 0.0)),
            "heading": float(getattr(feedback, "heading", 0.0)),
            "elapsed_sec": float(getattr(feedback, "elapsed_sec", 0.0)),
        }
        with self._goal_lock:
            self._last_feedback = reading
            self._last_feedback_at = time.monotonic()
            self._last_state_name = FOLLOW_FEEDBACK_STATES.get(
                state, f"UNKNOWN({state})"
            )
        now = time.monotonic()
        if now - self._last_feedback_log < 1.0:
            return
        self._last_feedback_log = now
        self._node.get_logger().info(
            "UWB follow feedback: "
            f"state={self._last_state_name}, "
            f"distance={reading['distance']:.2f}m, "
            f"heading={math.degrees(reading['heading']):.1f}deg"
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
                f"Failed to cancel late UWB follow goal: {exc}"
            )


class Ros2SetBehaviorClient:
    """Synchronous facade over the ``/go2/set_behavior`` mode service.

    Only IDLE and STOP are accepted by the server; FOLLOW and ROAM are
    rejected.  Following is engaged by the action goal instead.
    """

    def __init__(
        self,
        node: Any,
        *,
        service_name: str = "/go2/set_behavior",
        server_timeout_sec: float = 5.0,
    ) -> None:
        from go2_uwb_behavior.srv import SetBehavior
        from rclpy.callback_groups import ReentrantCallbackGroup

        self._node = node
        self._service_type = SetBehavior
        self._service_name = str(service_name)
        self._server_timeout_sec = float(server_timeout_sec)
        if (
            not math.isfinite(self._server_timeout_sec)
            or self._server_timeout_sec <= 0.0
        ):
            raise ValueError("server_timeout_sec must be finite and > 0")
        self._client = node.create_client(
            SetBehavior,
            self._service_name,
            callback_group=ReentrantCallbackGroup(),
        )

    def wait_for_server(self, timeout_sec: float) -> bool:
        if timeout_sec <= 0.0:
            return False
        return bool(self._client.wait_for_service(timeout_sec=timeout_sec))

    def set_mode(self, mode: int) -> tuple[bool, str]:
        """Call the mode service.  Returns ``(accepted, message)``."""
        request = self._service_type.Request()
        request.mode = int(mode)
        try:
            future = self._client.call_async(request)
        except Exception as exc:
            return False, f"uwb_set_behavior_call_failed:{exc}"
        deadline = time.monotonic() + self._server_timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                break
            time.sleep(_POLL_INTERVAL_SEC)
        if not future.done():
            return False, "uwb_set_behavior_timeout"
        try:
            response = future.result()
        except Exception as exc:
            return False, f"uwb_set_behavior_failed:{exc}"
        if response is None:
            return False, "uwb_set_behavior_empty_response"
        if not bool(response.accepted):
            return False, f"uwb_set_behavior_rejected:{response.message}"
        return True, str(response.message)


class UwbFollowActionAdapter:
    """Own one go2_uwb_behavior follow session for the Lite3 chassis.

    The public surface matches the legacy process adapter so ``ros_node`` can
    swap implementations without touching its call sites.
    """

    def __init__(
        self,
        *,
        enabled: bool,
        node: Any = None,
        follow_action: str = "/go2/follow_uwb",
        set_behavior_service: str = "/go2/set_behavior",
        server_timeout_sec: float = 5.0,
        ready_timeout_sec: float = 4.0,
        prepare_motion: Callable[..., bool] | None = None,
        finish_motion: Callable[[], bool | None] | None = None,
        should_stop: Callable[[], bool] | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        auto_resume: bool = False,
        follow_client_factory: Callable[..., Any] | None = None,
        behavior_client_factory: Callable[..., Any] | None = None,
    ) -> None:
        self._enabled = bool(enabled)
        self._node = node
        self._auto_resume = bool(auto_resume)
        self._follow_action = str(follow_action)
        self._set_behavior_service = str(set_behavior_service)
        self._server_timeout_sec = self._positive_float(
            server_timeout_sec, "server_timeout_sec"
        )
        self._ready_timeout_sec = self._positive_float(
            ready_timeout_sec, "ready_timeout_sec"
        )
        self._prepare_motion = prepare_motion or (lambda **_: True)
        self._finish_motion = finish_motion or (lambda: None)
        self._should_stop = should_stop or (lambda: False)
        self._monotonic = monotonic
        self._follow_client_factory = (
            follow_client_factory or Ros2FollowUwbClient
        )
        self._behavior_client_factory = (
            behavior_client_factory or Ros2SetBehaviorClient
        )

        self._lock = threading.RLock()
        self._cancel_requested = threading.Event()
        self._desired_follow = False
        self._session_controlled = False
        self._suspended = False
        # Set only after this adapter has taken chassis motion ownership, so an
        # idle stop cannot run Lite3's mode-release/settle handshake.
        self._motion_prepared = False
        self._interaction_id = ""
        self._active = False
        self._last_error = ""
        self.recovery_required = False
        self._long_goal_active = False
        self._started_at = 0.0
        self._feedback_timeout_sec = 3.0
        self._follow_client: Any = None
        self._behavior_client: Any = None

        if self._enabled:
            self._last_error = self._build_clients()

    # ── Public surface shared with the legacy process adapter ──────────

    @property
    def active(self) -> bool:
        with self._lock:
            return bool(self._active)

    @property
    def session_desired(self) -> bool:
        """True while the attention session still asks for following.

        Unlike ``active`` this survives a ``suspend()``: a session parked
        behind a foreground behaviour is still the thing that owns the
        chassis, so the goal gate has to keep rejecting lower-priority
        arrivals for as long as it lasts.
        """
        with self._lock:
            return bool(self._desired_follow)

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
                if self._suspended:
                    return True
            # A background session has no Stage budget, so it genuinely wants
            # the protocol's "follow until canceled" sentinel.
            return self.start(timeout_sec=0.0)

        with self._lock:
            current_id = self._interaction_id
        if not interaction_id or not current_id or interaction_id == current_id:
            with self._lock:
                self._desired_follow = False
                self._session_controlled = False
                self._interaction_id = ""
            self._stop_locked("attention_follow_disabled")
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

        budget = self._resolve_follow_budget(duration)
        if budget is None:
            self._refuse(
                "uwb_follow_no_time_budget:"
                f"ACT_INTERACT_FOLLOW_OWNER.duration={duration!r}"
            )
            return False

        # The preflight runs before the goal so the chassis is already in
        # Vision Mode -- and confirmed free of competing owners -- when the
        # chain starts publishing /cmd_vel.
        if not self.start(timeout_sec=budget):
            raise RuntimeError(self.last_error or "uwb_follow_start_failed")

        completed = self._run_until_deadline(
            self._monotonic() + budget, cancel_on_budget=standalone
        )
        if standalone:
            with self._lock:
                self._desired_follow = False
                self._interaction_id = ""
            self._stop_locked(
                "standalone_follow_completed" if completed else "follow_interrupted"
            )
        return completed

    def start(self, *, timeout_sec: float | None = None) -> bool:
        """Ensure one follow goal is in flight.

        ``timeout_sec`` is the goal's own deadline: ``0.0`` for a background
        session that follows until canceled, or an already-clamped Stage
        budget.  ``None`` means the background sentinel.
        """
        with self._lock:
            if not self._enabled:
                self._last_error = "uwb_follow_disabled"
                return False
            if self._follow_client is None or self._behavior_client is None:
                return False
            if self._active:
                return True
            self._cancel_requested.clear()
            reset_cancel = getattr(self._follow_client, "reset_cancel", None)
            if callable(reset_cancel):
                reset_cancel()

        goal_timeout = 0.0 if timeout_sec is None else float(timeout_sec)
        deadline = self._monotonic() + self._ready_timeout_sec
        follow_client = self._follow_client
        behavior_client = self._behavior_client

        if not follow_client.wait_for_server(
            min(self._server_timeout_sec, self._ready_timeout_sec)
        ):
            self._refuse(
                f"uwb_follow_action_unavailable:{self._follow_action}"
            )
            return False

        # The mode service is the only way to stop the chain, so refuse to hand
        # it /cmd_vel at all if that lever is missing.
        if not behavior_client.wait_for_server(
            max(0.0, deadline - self._monotonic())
        ):
            self._refuse(
                f"uwb_set_behavior_unavailable:{self._set_behavior_service}"
            )
            return False

        # Clear any STOP latch left by an earlier emergency stop before asking
        # the chain to follow again.
        accepted, message = behavior_client.set_mode(SET_BEHAVIOR_IDLE)
        if not accepted:
            self._refuse(f"uwb_set_behavior_idle_rejected:{message}")
            return False

        if not self._prepare_motion(allow_uwb_chain=True):
            self._refuse("uwb_motion_preflight_failed")
            return False
        with self._lock:
            self._motion_prepared = True

        if self._cancel_requested.is_set() or self._should_stop():
            self._refuse("uwb_follow_canceled_before_send")
            self._release_motion()
            return False

        goal_handle, reason = follow_client.start_follow(
            goal_timeout, deadline=deadline
        )
        if goal_handle is None:
            with self._lock:
                self._last_error = reason
            logger.error("UWB follow goal failed to start: %s", reason)
            if reason == "uwb_follow_goal_acceptance_unknown":
                self.recovery_required = True
                behavior_client.set_mode(SET_BEHAVIOR_STOP)
            else:
                self._release_motion()
            return False

        with self._lock:
            self._active = True
            self._last_error = ""
            self._started_at = self._monotonic()
        logger.info(
            "UWB follow goal accepted: action=%s timeout_sec=%.1f",
            self._follow_action,
            goal_timeout,
        )
        return True

    def poll_health(self) -> bool:
        """Reap a background session that ended on its own.

        Called from the attention-tracking tick, which ignores the return
        value; the point is to notice a chain that left FOLLOW mode and give
        the chassis back rather than leaving vision mode armed.
        """
        with self._lock:
            if not self._active:
                return True
            follow_client = self._follow_client
        if follow_client is None:
            return True

        if self._long_goal_active:
            feedback_at = getattr(follow_client, "last_feedback_at", None)
            reference = feedback_at if feedback_at is not None else self._started_at
            if self._monotonic() - reference > self._feedback_timeout_sec:
                with self._lock:
                    self._last_error = "uwb_follow_feedback_stale"
                return False

        wrapped, reason = follow_client.poll_result()
        if wrapped is None and not reason:
            return True  # still running
        if reason:
            with self._lock:
                self.recovery_required = True
                self._last_error = f"{reason}:operator_recovery_required"
            return False

        code = -1
        message = ""
        if wrapped is not None and wrapped.result is not None:
            code = int(wrapped.result.code)
            message = str(wrapped.result.message)
        if code != 0:
            with self._lock:
                self._last_error = (
                    "uwb_follow_ended:"
                    f"{FOLLOW_RESULT_CODES.get(code, code)}:{message}"
                )
                if code == 6:
                    self.recovery_required = True
            logger.error(
                "UWB follow session ended without success: code=%s message=%s",
                FOLLOW_RESULT_CODES.get(code, code),
                message or reason,
            )
            self._stop_locked("follow_failed")
            return False
        self._stop_locked("follow_completed")
        return True

    def cancel_step(self, step: Any = None) -> None:
        del step
        self._cancel_requested.set()
        with self._lock:
            self._desired_follow = False
            self._session_controlled = False
            self._interaction_id = ""
        self._stop_locked("canceled")

    def request_cancel(self) -> None:
        """Request the inner cancellation; ownership stays until Result."""
        self._cancel_requested.set()
        client = self._follow_client
        if client is not None:
            client.cancel()

    def brake_ready(self) -> bool:
        client = self._behavior_client
        return bool(client is not None and client.wait_for_server(self._ready_timeout_sec))

    def arm_chain(self) -> bool:
        """Release a confirmed STOP latch before a new UWB Action Goal."""
        client = self._behavior_client
        if self.recovery_required or client is None:
            self._last_error = "uwb_set_behavior_idle_unavailable"
            return False
        accepted, message = client.set_mode(SET_BEHAVIOR_IDLE)
        if not accepted:
            self._last_error = f"uwb_set_behavior_idle_failed:{message}"
        return bool(accepted)

    def brake_chain(self) -> bool:
        client = self._behavior_client
        if client is None:
            return False
        accepted, message = client.set_mode(SET_BEHAVIOR_STOP)
        if not accepted:
            self.recovery_required = True
            self._last_error = f"uwb_set_behavior_stop_failed:{message}"
        return bool(accepted)

    def stop_confirmed(self, reason: str = "stopped") -> bool:
        """Brake and wait for the inner Action's real terminal Result."""
        with self._lock:
            if not self._active:
                return not self.recovery_required
            client = self._follow_client
            behavior = self._behavior_client
        if client is None or behavior is None:
            self.recovery_required = True
            self._last_error = "uwb_follow_stop_client_missing:operator_recovery_required"
            return False
        self.request_cancel()
        stop_ok = True
        accepted, message = behavior.set_mode(SET_BEHAVIOR_STOP)
        if not accepted:
            logger.error("UWB STOP unconfirmed: %s", message)
            self.recovery_required = True
            self._last_error = "uwb_follow_stop_unconfirmed:operator_recovery_required"
            stop_ok = False
        deadline = self._monotonic() + max(5.0, self._server_timeout_sec)
        while self._monotonic() < deadline:
            wrapped, error = client.poll_result()
            if wrapped is not None:
                if getattr(wrapped, "status", 5) not in (4, 5, 6):
                    break
                code = int(getattr(getattr(wrapped, "result", None), "code", -1))
                stopped = self._stop_locked(reason)
                if code == 6:
                    self.recovery_required = True
                    self._last_error = "uwb_follow_STOP_UNCONFIRMED:operator_recovery_required"
                    return False
                if code not in (0, 1) and not (
                    code == 2 and self._cancel_requested.is_set()
                ):
                    self._last_error = f"uwb_follow_terminal:{FOLLOW_RESULT_CODES.get(code, code)}"
                    return False
                return stopped and stop_ok
            if error:
                break
            time.sleep(_POLL_INTERVAL_SEC)
        self.recovery_required = True
        self._last_error = "uwb_follow_terminal_unknown:operator_recovery_required"
        return False

    def emergency_stop(self) -> None:
        if self._long_goal_active:
            self.request_cancel()
        else:
            self._cancel_requested.set()
        with self._lock:
            self._desired_follow = False
            self._session_controlled = False
            self._interaction_id = ""
        with self._lock:
            active = self._active
            behavior = self._behavior_client
        if active and self._long_goal_active:
            if behavior is None:
                self.recovery_required = True
                self._last_error = "uwb_follow_stop_client_missing:operator_recovery_required"
            else:
                accepted, message = behavior.set_mode(SET_BEHAVIOR_STOP)
                if not accepted:
                    self.recovery_required = True
                    self._last_error = f"uwb_follow_stop_unconfirmed:{message}"
        elif not self._long_goal_active:
            self._stop_locked("emergency_stop")

    def suspend(self) -> None:
        """Yield /cmd_vel while another foreground behavior executes."""
        with self._lock:
            self._suspended = True
        self._stop_locked("foreground_behavior")

    def resume(self) -> bool:
        """Release the chassis after a foreground behavior finishes.

        With ``auto_resume`` the still-desired session restarts at once.
        Without it -- the default -- the interruption simply ends: following
        again requires a fresh session message or a fresh ``follow_owner``
        goal, so the robot never starts walking again on its own.

        ``_suspended`` is cleared either way.  Leaving it set would trip the
        ``if self._suspended: return True`` short-circuit in
        ``update_control`` forever, closing the only re-arm path.
        """
        with self._lock:
            self._suspended = False
            desired = self._desired_follow
            auto_resume = self._auto_resume
        if not (desired and auto_resume):
            return True
        return self.start(timeout_sec=0.0)

    def stop(self, reason: str = "stopped") -> None:
        with self._lock:
            self._desired_follow = False
            self._session_controlled = False
            self._interaction_id = ""
        self._stop_locked(reason)

    # ── Internals ─────────────────────────────────────────────────────

    @staticmethod
    def _positive_float(value: Any, name: str) -> float:
        result = float(value)
        if not math.isfinite(result) or result <= 0.0:
            raise ValueError(f"{name} must be finite and > 0")
        return result

    def _build_clients(self) -> str:
        """Build both clients, or return why they could not be built.

        The message packages live in the go2_uwb_behavior workspace, so a
        missing workspace is a deployment state to report, not a crash: this
        package is deliberately built without that dependency.
        """
        if self._node is None:
            return "uwb_clients_unavailable:no_node"
        try:
            self._follow_client = self._follow_client_factory(
                self._node,
                action_name=self._follow_action,
                server_timeout_sec=self._server_timeout_sec,
                should_stop=self._should_stop,
            )
            self._behavior_client = self._behavior_client_factory(
                self._node,
                service_name=self._set_behavior_service,
                server_timeout_sec=self._server_timeout_sec,
            )
        except Exception as exc:
            self._follow_client = None
            self._behavior_client = None
            return f"uwb_clients_unavailable:{type(exc).__name__}:{exc}"
        return ""

    def _resolve_follow_budget(self, duration: float | None) -> float | None:
        """Map a Stage budget onto the protocol's explicit-timeout range.

        Returns ``None`` when there is no usable budget, which the caller
        treats as a hard failure: degrading to 0 here would silently ask the
        chain to follow until canceled, the opposite of a bounded Stage.
        """
        try:
            budget = float(duration)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            return None
        if not math.isfinite(budget) or budget <= 0.0:
            return None
        return min(MAX_FOLLOW_TIMEOUT_SEC, max(MIN_FOLLOW_TIMEOUT_SEC, budget))

    def _run_until_deadline(self, deadline: float, *, cancel_on_budget: bool) -> bool:
        """Block on the follow result.

        Returns True when the session ran out its budget and was stopped by us
        -- a bounded session doing exactly what it was asked -- and False on
        every other ending: an external cancel, or a terminal result that is
        not SUCCESS.
        """
        with self._lock:
            follow_client = self._follow_client
        if follow_client is None:
            self._refuse("uwb_follow_client_missing")
            return False

        wrapped, reason = follow_client.wait_for_result(deadline)
        if wrapped is None:
            if reason == _WAIT_TIMEOUT:
                # Our budget expired.  For a Stage that is the session
                # completing, so cancel what is left of the goal and report
                # success; for a background session the goal must survive.
                if cancel_on_budget:
                    follow_client.cancel()
                return True
            if reason == _WAIT_CANCELED:
                return False
            self._refuse(reason)
            return False

        code = -1
        message = ""
        if wrapped.result is not None:
            code = int(wrapped.result.code)
            message = str(wrapped.result.message)
        if code == 0:
            return True
        with self._lock:
            self._last_error = (
                "uwb_follow_failed:"
                f"{FOLLOW_RESULT_CODES.get(code, code)}:{message}"
            )
        logger.error(
            "UWB follow ended: code=%s message=%s",
            FOLLOW_RESULT_CODES.get(code, code),
            message,
        )
        return False

    def _stop_locked(self, reason: str) -> bool:
        """Stop the chain, then release the chassis -- in that order.

        The chain publishes /cmd_vel at 20 Hz, so releasing motion ownership
        first would leave a window where a stale non-zero command still lands
        on a chassis nobody is guarding.

        The brake is only sent for a session we actually engaged.  Calling the
        service when nothing was started would block for ``server_timeout_sec``
        waiting on a server that may not be running -- and the node-shutdown
        path calls ``emergency_stop()`` unconditionally.
        """
        with self._lock:
            was_active = self._active
            self._active = False
            follow_client = self._follow_client
            behavior_client = self._behavior_client
            was_motion_prepared = self._motion_prepared
            self._motion_prepared = False

        goal_handle = follow_client.take_goal_handle() if follow_client else None

        if was_active and behavior_client is not None:
            accepted, message = behavior_client.set_mode(SET_BEHAVIOR_STOP)
            if not accepted:
                logger.warning("UWB set_behavior STOP not accepted: %s", message)
                self.recovery_required = True
                self._last_error = "uwb_follow_stop_unconfirmed:operator_recovery_required"

        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                logger.warning("Failed to cancel UWB follow goal: %s", exc)

        if was_motion_prepared:
            try:
                if self._finish_motion() is False:
                    self.recovery_required = True
                    self._last_error = "uwb_follow_settle_failed:operator_recovery_required"
            except Exception as exc:
                self.recovery_required = True
                self._last_error = "uwb_follow_settle_failed:operator_recovery_required"
                logger.warning("Failed to finish UWB chassis motion: %s", exc)

        if was_active:
            logger.info("UWB follow session stopped: %s", reason)
        return not self.recovery_required

    def _release_motion(self) -> None:
        with self._lock:
            was_prepared = self._motion_prepared
            self._motion_prepared = False
        if was_prepared:
            try:
                if self._finish_motion() is False:
                    self.recovery_required = True
                    self._last_error = "uwb_follow_settle_failed:operator_recovery_required"
            except Exception as exc:
                self.recovery_required = True
                self._last_error = "uwb_follow_settle_failed:operator_recovery_required"
                logger.warning("Failed to finish UWB chassis motion: %s", exc)

    def _refuse(self, reason: str) -> None:
        with self._lock:
            self._last_error = reason
        logger.error("UWB follow refused: %s", reason)
