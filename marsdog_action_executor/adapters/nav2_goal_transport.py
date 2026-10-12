"""One Nav2 goal with cancellation and real-result ownership."""
from __future__ import annotations

import math
import threading
import time
from typing import Any, Mapping


class Ros2Nav2GoalTransport:
    def __init__(self, node: Any, *, nav_action="/navigate_to_pose", server_timeout_sec=10.0,
                 behavior_tree="", _action_type=None):
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup

        self._node = node
        self._nav_type = _action_type or NavigateToPose
        self._behavior_tree = str(behavior_tree)
        self._server_timeout = float(server_timeout_sec)
        if not math.isfinite(self._server_timeout) or self._server_timeout <= 0:
            raise ValueError("Nav2 server timeout must be finite and positive")
        self._nav = ActionClient(node, self._nav_type, nav_action,
                                 callback_group=ReentrantCallbackGroup())
        self._lock = threading.Lock()
        self._goal = None
        self._result_future = None
        self._pending_send = None
        self._cancel_sent = False
        self.recovery_required = False
        self.last_terminal_status = None

    def cancel(self) -> None:
        with self._lock:
            goal = self._goal
            if goal is None or self._cancel_sent:
                return
            self._cancel_sent = True
        try:
            goal.cancel_goal_async()
        except Exception:
            with self._lock:
                self._cancel_sent = False
            raise

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
                 cancel: threading.Event, *, feedback_callback=None) -> tuple[bool, bool, str]:
        from action_msgs.msg import GoalStatus
        from rclpy import ok as ros_ok

        if self.recovery_required:
            return False, False, "nav2_recovery_required:operator_recovery_required"
        self.last_terminal_status = None
        server_wait = min(self._server_timeout, max(0.0, deadline - time.monotonic()))
        if server_wait <= 0 or not self._nav.wait_for_server(timeout_sec=server_wait):
            return False, False, "nav2_server_unavailable"
        if cancel.is_set():
            return False, True, "canceled_before_navigation"
        goal = self._make_goal(pose, deadline)
        sent = (self._nav.send_goal_async(goal, feedback_callback=feedback_callback)
                if feedback_callback is not None else self._nav.send_goal_async(goal))
        with self._lock:
            self._pending_send = sent
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
            self.recovery_required = True
            return False, False, "nav2_send_error:%s:operator_recovery_required" % exc
        if handle is None:
            self.recovery_required = True
            return False, False, "nav2_empty_acceptance:operator_recovery_required"
        with self._lock:
            self._pending_send = None
        if not handle.accepted:
            return False, False, "nav2_goal_rejected"
        with self._lock:
            self._goal = handle
            self._cancel_sent = False
        try:
            result_future = handle.get_result_async()
        except Exception as exc:
            self.recovery_required = True
            self.cancel()
            return False, False, "nav2_result_error:%s:operator_recovery_required" % exc
        with self._lock:
            self._result_future = result_future
        timed_out = False
        cancel_deadline = None
        terminal_confirmed = False
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
            try:
                wrapped = result_future.result()
            except Exception as exc:
                self.recovery_required = True
                return False, False, "nav2_result_error:%s:operator_recovery_required" % exc
            if wrapped is None:
                self.recovery_required = True
                return False, False, "nav2_empty_result:operator_recovery_required"
            status = wrapped.status
            if status not in (4, 5, 6):
                self.recovery_required = True
                return False, False, "nav2_nonterminal_result:operator_recovery_required"
            terminal_confirmed = True
            self.last_terminal_status = status
            return self._terminal_outcome(status, timed_out, cancel)
        finally:
            if terminal_confirmed:
                with self._lock:
                    self._goal = None
                    self._result_future = None
                    self._cancel_sent = False
            else:
                with self._lock:
                    # Permit an emergency-stop retry while recovery is latched.
                    self._cancel_sent = False

    def _terminal_outcome(self, status: int, timed_out: bool, cancel: Any) -> tuple[bool, bool, str]:
        if timed_out:
            return False, False, "navigation_timeout"
        if cancel.is_set() or status == 5:
            return False, True, "navigation_canceled"
        if status == 4:
            return True, False, "navigation_succeeded"
        return False, False, "navigation_status_%s" % status

    def _make_goal(self, pose: Mapping[str, Any], deadline: float) -> Any:
        goal = self._nav_type.Goal()
        goal.behavior_tree = getattr(self, "_behavior_tree", "")
        goal.pose.header.frame_id = pose["header"]["frame_id"]
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        for key in ("x", "y", "z"):
            setattr(goal.pose.pose.position, key, float(pose["pose"]["position"][key]))
        for key in ("x", "y", "z", "w"):
            setattr(goal.pose.pose.orientation, key, float(pose["pose"]["orientation"][key]))
        return goal

    def _cancel_late_accepted_goal(self, future: Any) -> None:
        try:
            handle = future.result()
            with self._lock:
                self._pending_send = None
            if handle is not None and handle.accepted:
                with self._lock:
                    self._goal = handle
                    self._cancel_sent = False
                try:
                    result_future = handle.get_result_async()
                    with self._lock:
                        self._result_future = result_future
                    result_future.add_done_callback(self._drain_late_result)
                finally:
                    # Even a broken result transport must cancel a late
                    # accepted goal; retain the handle under the recovery lock.
                    self.cancel()
        except Exception as exc:
            self._node.get_logger().error("Late Nav2 goal cancel failed: %s" % exc)

    def _drain_late_result(self, future: Any) -> None:
        """Retain a late accepted goal until its real, correlated Result."""
        try:
            wrapped = future.result()
            if wrapped is None or wrapped.status not in (4, 5, 6):
                return
            with self._lock:
                if self._result_future is future:
                    self._goal = None
                    self._result_future = None
                    self._cancel_sent = False
            # Recovery stays latched: the chassis was not settled by the
            # foreground task and needs explicit operator recovery.
        except Exception as exc:
            self._node.get_logger().error("Late Nav2 result failed: %s" % exc)


class Ros2Nav2SpinTransport(Ros2Nav2GoalTransport):
    """Counted relative rotations using the same real-Result ownership.

    Nav2 Spin accumulates yaw across wraparound. Keep 6π as 6π; normalizing
    the requested angle would turn a three-turn goal into a zero-angle goal.
    """

    def __init__(self, node: Any, *, action_name="/spin", server_timeout_sec=5.0):
        from nav2_msgs.action import Spin
        super().__init__(node, nav_action=action_name,
                         server_timeout_sec=server_timeout_sec, _action_type=Spin)

    def spin(self, target_yaw: float, deadline: float,
             cancel: threading.Event, *, time_allowance_sec: float | None = None) -> tuple[bool, bool, str]:
        if not math.isfinite(target_yaw) or target_yaw == 0:
            return False, False, "invalid_spin_angle"
        return self.navigate({"target_yaw": target_yaw, "time_allowance_sec": time_allowance_sec}, deadline, cancel)

    def _terminal_outcome(self, status: int, timed_out: bool, cancel: Any) -> tuple[bool, bool, str]:
        # Cancel/time limits request cancellation; only the associated terminal
        # Result decides whether Spin succeeded, canceled or failed.
        if status == 4:
            return True, False, "spin_succeeded"
        if status == 5:
            return False, True, "spin_canceled"
        return False, False, "spin_status_%s" % status

    def _make_goal(self, command: Mapping[str, Any], deadline: float) -> Any:
        goal = self._nav_type.Goal()
        goal.target_yaw = float(command["target_yaw"])
        remaining = max(0.0, deadline - time.monotonic())
        if command.get("time_allowance_sec") is not None:
            remaining = min(remaining, float(command["time_allowance_sec"]))
        goal.time_allowance.sec = int(remaining)
        goal.time_allowance.nanosec = int((remaining - int(remaining)) * 1_000_000_000)
        self._node.get_logger().info(
            "Nav2 counted Spin: target_yaw=%.6frad, allowance=%.2fs" %
            (goal.target_yaw, remaining))
        return goal
