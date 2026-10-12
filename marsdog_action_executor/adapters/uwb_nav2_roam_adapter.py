"""UWB generates a fixed target; Action owns the one Nav2 execution."""
from __future__ import annotations

import logging
import math
import threading
import time
import uuid

from .nav2_goal_transport import Ros2Nav2GoalTransport


logger = logging.getLogger(__name__)


def _pose_dict(message):
    return {
        "header": {"frame_id": message.header.frame_id,
                   "stamp": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9},
        "pose": {
            "position": {k: float(getattr(message.pose.position, k)) for k in ("x", "y", "z")},
            "orientation": {k: float(getattr(message.pose.orientation, k)) for k in ("x", "y", "z", "w")},
        },
    }


class Ros2UwbNavigationGoalProvider:
    def __init__(self, node, service_name="/go2/generate_navigation_goal"):
        from go2_uwb_behavior.srv import GenerateNavigationGoal
        from rclpy.callback_groups import ReentrantCallbackGroup

        self._node = node
        self._type = GenerateNavigationGoal
        self._client = node.create_client(
            GenerateNavigationGoal, service_name, callback_group=ReentrantCallbackGroup(),
        )

    def generate(self, deadline, cancel):
        while time.monotonic() < deadline and not cancel.is_set():
            if self._client.wait_for_service(timeout_sec=min(.1, max(0., deadline - time.monotonic()))):
                break
        else:
            return None, "uwb_target_canceled" if cancel.is_set() else "uwb_target_service_unavailable"
        if cancel.is_set():
            return None, "uwb_target_canceled"
        request = self._type.Request()
        request.request_id = "roam-target-" + uuid.uuid4().hex
        request.random_seed, request.min_radius, request.max_radius = 0, .5, 2.
        future = self._client.call_async(request)
        while not future.done():
            if cancel.is_set() or time.monotonic() >= deadline:
                # This service generates data only. A late response cannot move
                # the robot, and is never promoted into a new navigation goal.
                self._client.remove_pending_request(future)
                future.cancel()
                return None, "uwb_target_canceled" if cancel.is_set() else "uwb_target_timeout"
            time.sleep(.02)
        response = future.result()
        if response is None or response.request_id != request.request_id:
            return None, "uwb_target_response_mismatch"
        if not response.success or response.code != 0:
            return None, f"uwb_target_failed:code={response.code},message={response.message}"
        goal, center = _pose_dict(response.navigation_goal), _pose_dict(response.center_pose)
        now = self._node.get_clock().now().nanoseconds * 1e-9
        for pose in (goal, center):
            if not Ros2Nav2GoalTransport.valid_pose(pose):
                return None, "uwb_target_invalid_map_pose"
            age = now - pose["header"]["stamp"]
            if not math.isfinite(age) or not -.1 <= age <= 2.:
                return None, "uwb_target_stale"
        gp, cp = goal["pose"]["position"], center["pose"]["position"]
        if not .5 - 1e-6 <= math.hypot(gp["x"] - cp["x"], gp["y"] - cp["y"]) <= 2. + 1e-6:
            return None, "uwb_target_outside_requested_annulus"
        return {"request_id": response.request_id, "navigation_goal": goal, "center_pose": center}, "ok"


class UwbNav2RoamAdapter:
    def __init__(self, *, node=None, backend, should_stop=lambda: False,
                 service_name="/go2/generate_navigation_goal", nav_action="/navigate_to_pose",
                 provider=None, navigator=None, arm_chain=lambda: True,
                 brake_chain=lambda: True):
        self._node, self._backend = node, backend
        self._provider, self._navigator = provider, navigator
        self._service_name, self._nav_action = service_name, nav_action
        self._should_stop = should_stop
        self._arm_chain, self._brake_chain = arm_chain, brake_chain
        self._cancel, self._closed = threading.Event(), threading.Event()
        self._lock = threading.Lock()
        self.active = False
        self.recovery_required = False
        self.last_error = ""

    def cancel_step(self, step=None):
        self._cancel.set()
        if self._navigator is not None:
            try:
                self._navigator.cancel()
            except Exception:
                self.recovery_required = True

    emergency_stop = cancel_step

    def close(self):
        self._closed.set()
        self.cancel_step()

    def execute_step(self, unit_config, ctx=None, duration=None):
        if not self._lock.acquire(blocking=False):
            self.last_error = "uwb_roam_busy"
            return False
        prepared = False
        try:
            if self.recovery_required:
                self.last_error = "uwb_roam_terminal_unknown:operator_recovery_required"
                return False
            self.last_error = ""
            self._cancel.clear()
            budget = float(duration or 0.)
            if not math.isfinite(budget) or budget < 10.:
                self.last_error = "uwb_roam_insufficient_budget"
                return False
            deadline = time.monotonic() + budget
            if self._stopped(ctx):
                self.last_error = "uwb_roam_canceled"
                return False
            if self._provider is None:
                self._provider = Ros2UwbNavigationGoalProvider(self._node, self._service_name)
            if self._navigator is None:
                self._navigator = Ros2Nav2GoalTransport(self._node, nav_action=self._nav_action)
            if not self._arm_chain():
                self.last_error = "uwb_roam_idle_unconfirmed"
                return False
            prepared = bool(self._backend.prepare_navigation(allow_uwb_chain=True))
            if not prepared:
                self.last_error = self._backend.last_error or "uwb_roam_preflight_failed"
                return False
            self.active = True
            # Cancellation/lease checks are also observed while requesting a
            # point, through the same Event passed to the provider and Nav2.
            monitor_done = threading.Event()

            def monitor():
                while not monitor_done.wait(.02):
                    if self._stopped(ctx) or time.monotonic() >= deadline:
                        self._cancel.set()
                        return

            thread = threading.Thread(target=monitor, name="uwb-nav2-cancel", daemon=True)
            thread.start()
            try:
                target, reason = self._provider.generate(min(deadline - 3., time.monotonic() + 5.), self._cancel)
                if target is None:
                    self.last_error = reason
                    self._brake_chain()
                    return False
                if self._stopped(ctx):
                    self.last_error = "uwb_roam_canceled"
                    return False
                if ctx is not None:
                    ctx.metadata["uwb_roam_target"] = target
                center = target["center_pose"]["pose"]["position"]
                guard_error = []

                def feedback(message):
                    current = message.feedback.current_pose
                    if current.header.frame_id != "map":
                        guard_error.append("uwb_nav2_feedback_frame_invalid")
                        self._cancel.set()
                    else:
                        x, y = float(current.pose.position.x), float(current.pose.position.y)
                        if (not math.isfinite(x) or not math.isfinite(y) or
                                math.hypot(x - center["x"], y - center["y"]) > 6.):
                            guard_error.append("uwb_nav2_geofence_stop")
                            self._cancel.set()

                self._info(f"UWB target -> Nav2: request_id={target['request_id']}, goal={target['navigation_goal']}")
                try:
                    success, canceled, reason = self._navigator.navigate(
                        target["navigation_goal"], min(deadline - 2., time.monotonic() + 30.),
                        self._cancel, feedback_callback=feedback,
                    )
                except Exception:
                    self.recovery_required = True
                    raise
                self.recovery_required |= bool(getattr(self._navigator, "recovery_required", False))
                terminal_status = getattr(self._navigator, "last_terminal_status", None)
                if ctx is not None and terminal_status is not None:
                    ctx.metadata["uwb_roam_terminal_status"] = terminal_status
                self._info(f"UWB Nav2 result: success={success}, canceled={canceled}, reason={reason}")
                if not success or canceled or self._stopped(ctx) or guard_error:
                    self.last_error = (guard_error[-1] if guard_error else
                                       "uwb_roam_canceled" if canceled or self._stopped(ctx) else reason)
                    return False
                if ctx is not None:
                    ctx.metadata["uwb_roam_result"] = {"code": 0, "execution_mode": "nav2"}
                return True
            finally:
                monitor_done.set()
                thread.join(timeout=1.)
        except Exception as exc:
            if not self.last_error:
                self.last_error = f"uwb_nav2_error:{type(exc).__name__}:{exc}"
            return False
        finally:
            try:
                if prepared and not self.recovery_required:
                    try:
                        settled = self._backend.finish_navigation() is not False
                    except Exception as exc:
                        self.recovery_required = True
                        self.last_error = f"uwb_roam_settle_error:{type(exc).__name__}"
                        raise RuntimeError(self.last_error) from exc
                    if not settled:
                        self.recovery_required = True
                        self.last_error = self._backend.last_error or "uwb_roam_settle_failed"
                        raise RuntimeError(self.last_error)
                if not self.recovery_required:
                    self.active = False
            finally:
                self._lock.release()

    def _stopped(self, ctx):
        return (self._closed.is_set() or self._cancel.is_set() or self._should_stop()
                or bool(getattr(ctx, "cancel_requested", False)))

    def _info(self, message):
        (self._node.get_logger().info if self._node is not None else logger.info)(message)
