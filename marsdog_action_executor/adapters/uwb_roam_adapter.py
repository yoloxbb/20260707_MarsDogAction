"""Single RandomRoam goal; terminal Result is the motion ownership boundary."""
from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any


logger = logging.getLogger(__name__)
ROAM_STATES = {
    0: "PREPARING", 1: "SELECTING_GOAL", 2: "NAVIGATING",
    3: "RETURNING", 4: "STOPPING", 5: "RETRYING",
}


class UwbRoamAdapter:
    def __init__(self, *, node=None, backend, should_stop=lambda: False,
                 action_name="/go2/random_roam", client=None, goal_factory=None,
                 brake_ready=lambda: True, brake_chain=lambda: True,
                 arm_chain=lambda: True):
        self._backend = backend
        self._should_stop = should_stop
        self._cancel = threading.Event()
        self._closed = threading.Event()
        self._handle = None
        self._lock = threading.Lock()
        self.active = False
        self.recovery_required = False
        self.last_error = ""
        self._client = client
        self._goal_factory = goal_factory
        self._node = node
        self._action_name = action_name
        self._brake_ready = brake_ready
        self._brake_chain = brake_chain
        self._arm_chain = arm_chain

    def cancel_step(self, step=None):
        self._cancel.set()

    emergency_stop = cancel_step

    def close(self):
        """Request cancellation before the ROS executor stops processing futures."""
        self._cancel.set()
        if self._handle is not None:
            try:
                self._handle.cancel_goal_async()
            except Exception:
                pass
        if self.active:
            try:
                if self._brake_chain() is False:
                    self.recovery_required = True
                    self.last_error = "uwb_roam_stop_unconfirmed:operator_recovery_required"
            except Exception:
                self.recovery_required = True
                self.last_error = "uwb_roam_stop_unconfirmed:operator_recovery_required"
            emergency_stop = getattr(self._backend, "emergency_stop", None)
            if callable(emergency_stop):
                emergency_stop()
        self._closed.set()

    def execute_step(self, unit_config, ctx=None, duration=None):
        if not self._lock.acquire(blocking=False):
            self.last_error = "uwb_roam_busy"
            return False
        prepared = False
        unresolved = False
        try:
            self.last_error = ""
            if self._closed.is_set():
                self.last_error = "uwb_roam_closed"
                return False
            self._cancel.clear()
            budget = float(duration or 0)
            if not math.isfinite(budget) or budget < 10:
                self.last_error = "uwb_roam_insufficient_budget"
                return False
            deadline = time.monotonic() + budget
            if self._client is None:
                from go2_uwb_behavior.action import RandomRoam
                from rclpy.action import ActionClient
                from rclpy.callback_groups import ReentrantCallbackGroup
                self._client = ActionClient(self._node, RandomRoam, self._action_name,
                                           callback_group=ReentrantCallbackGroup())
                self._goal_factory = RandomRoam.Goal
            if not self._client.wait_for_server(timeout_sec=min(3.0, budget)):
                self.last_error = "uwb_roam_server_unavailable"
                return False
            if not self._brake_ready():
                self.last_error = "uwb_roam_brake_unavailable"
                return False
            if self._stopped(ctx):
                self.last_error = "uwb_roam_canceled"
                return False
            prepared = True
            if not self._backend.prepare_navigation(allow_uwb_chain=True):
                self.last_error = self._backend.last_error or "uwb_roam_preflight_failed"
                return False
            remaining = deadline - time.monotonic()
            if remaining < 7 or self._stopped(ctx):
                self.last_error = "uwb_roam_interrupted_before_send"
                return False
            # Follow cancellation leaves the UWB controller in STOP. It must
            # acknowledge IDLE before RandomRoam can accept a new goal.
            if not self._arm_chain():
                self.last_error = "uwb_roam_idle_unconfirmed"
                return False
            if self._stopped(ctx):
                self.last_error = "uwb_roam_interrupted_before_send"
                return False
            goal = self._goal_factory()
            goal.random_seed = 0
            goal.timeout_sec = min(30.0, remaining - 2.0)
            goal.min_radius = 0.5
            goal.max_radius = 2.0
            self.active = True
            unresolved = True
            # Per-goal closure keeps late feedback separate from subsequent goals.
            feedback_snapshot = {}
            feedback_log = {"time": float("-inf"), "state": None}

            def on_feedback(message):
                feedback = message.feedback
                state = ROAM_STATES.get(int(feedback.state), f"UNKNOWN({feedback.state})")
                snapshot = {
                    "state": state,
                    "distance_remaining": float(feedback.distance_remaining),
                    "owner_distance": float(feedback.owner_distance),
                    "elapsed_sec": float(feedback.elapsed_sec),
                }
                feedback_snapshot.update(snapshot)
                now = time.monotonic()
                if state == feedback_log["state"] and now - feedback_log["time"] < 1.0:
                    return
                feedback_log.update(time=now, state=state)
                self._info(
                    f"UWB roam feedback: state={state}, "
                    f"distance_remaining={snapshot['distance_remaining']:.3f}m, "
                    f"owner_distance={snapshot['owner_distance']:.3f}m, "
                    f"elapsed={snapshot['elapsed_sec']:.1f}s"
                )

            self._info(
                f"UWB roam goal: action={self._action_name}, "
                f"timeout={goal.timeout_sec:.1f}s, radius="
                f"[{goal.min_radius:.2f}, {goal.max_radius:.2f}]m"
            )
            send = self._client.send_goal_async(goal, feedback_callback=on_feedback)
            # Keep ownership even if acceptance is late. Never abandon a send
            # future: an accepted late goal must be canceled and drained.
            interrupted = False
            while not send.done():
                if self._closed.is_set():
                    raise RuntimeError("uwb_roam_shutdown_with_pending_goal")
                interrupted |= self._stopped(ctx) or time.monotonic() >= deadline
                time.sleep(0.02)
            handle = send.result()
            if handle is None or not handle.accepted:
                unresolved = False
                self.last_error = "uwb_roam_rejected"
                return False
            self._handle = handle
            result = handle.get_result_async()
            cancel_sent = False
            while True:
                if self._closed.is_set():
                    raise RuntimeError("uwb_roam_shutdown_with_pending_result")
                interrupted |= self._stopped(ctx) or time.monotonic() >= deadline
                if interrupted and not cancel_sent:
                    try:
                        handle.cancel_goal_async()
                        cancel_sent = True
                    except Exception:
                        # Retry while retaining ownership of the actual goal.
                        pass
                if result.done():
                    break
                time.sleep(0.02)
            wrapped = result.result()
            unresolved = False
            self._handle = None
            if ctx is not None:
                ctx.metadata["uwb_roam_last_feedback"] = dict(feedback_snapshot)
                ctx.metadata["uwb_roam_terminal_status"] = int(wrapped.status)
            self._info(
                f"UWB roam result: status={wrapped.status}, code={wrapped.result.code}, "
                f"message={wrapped.result.message}, last_feedback={feedback_snapshot or 'none'}"
            )
            if interrupted:
                self.last_error = (
                    "uwb_roam_canceled" if wrapped.status == 5
                    else f"uwb_roam_terminal_after_cancel:status={wrapped.status},"
                         f"code={wrapped.result.code}"
                )
                return False
            if wrapped.status != 4 or wrapped.result.code != 0:
                self.last_error = (f"uwb_roam_failed:status={wrapped.status},"
                                   f"code={wrapped.result.code},message={wrapped.result.message}")
                return False
            if ctx is not None:
                ctx.metadata["uwb_roam_result"] = {
                    "code": int(wrapped.result.code),
                    "owner_distance": float(wrapped.result.owner_distance),
                    "min_clearance": float(wrapped.result.min_clearance),
                }
            return True
        except Exception:
            if unresolved:
                # A transport error is not evidence that the chassis stopped.
                # Keep ownership latched and refuse subsequent foreground work.
                self.recovery_required = True
                self.last_error = "uwb_roam_terminal_unknown:operator_recovery_required"
            raise
        finally:
            # Runs only after rejected acceptance or a real Result, never on
            # cancel acknowledgement. No pose may start before this settles.
            if prepared and not unresolved:
                if self._backend.finish_navigation() is False:
                    self.last_error = self._backend.last_error or "uwb_roam_settle_failed"
                    self.recovery_required = True
                    self.active = False
                    self._lock.release()
                    raise RuntimeError(self.last_error)
            if not unresolved:
                self.active = False
                self._lock.release()

    def _info(self, message: str):
        if self._node is not None:
            self._node.get_logger().info(message)
        else:
            logger.info(message)

    def _stopped(self, ctx: Any):
        return (self._cancel.is_set() or self._should_stop()
                or bool(getattr(ctx, "cancel_requested", False)))
