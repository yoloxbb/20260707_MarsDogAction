"""Nav2 waypoint and behavior-stage mobility adapters.

``Ros2Nav2Client`` owns the ROS2 ``NavigateToPose`` Action client.
``BehaviorMobilityAdapter`` keeps semantic waypoint and stage-motion routing
ROS-independent so it can be validated and unit-tested without Nav2.
"""

from __future__ import annotations

import logging
import math
import random
import threading
import time
from typing import Any, Callable, Mapping

# Sentinel returned by waypoint_for_behavior for waypoint_nav's reserved
# live-map random target.
_RANDOM_WAYPOINT_SENTINEL = "__random__"

logger = logging.getLogger(__name__)


class BehaviorMobilityAdapter:
    """Navigate routed behaviors, then execute their platform action stages."""

    def __init__(
        self,
        navigate_waypoint: Callable[[str, Mapping[str, Any], float], bool],
        motion_adapter: Any,
        waypoints: Mapping[str, Mapping[str, Any]],
        behavior_routes: Mapping[str, Mapping[str, Any]],
        stage_actions: list[str],
        *,
        navigate_fixed_place: Callable[..., bool] | None = None,
        fixed_places: Mapping[str, str] | None = None,
        result_timeout_sec: float = 300.0,
        random_navigation_behaviors: list[str] | None = None,
        random_navigation_place: str | None = None,
        random_navigation_result_timeout_sec: float | None = None,
        random_navigation_in_place_probability: Mapping[str, float] | None = None,
        random_navigation_fixed_pool: list[str] | None = None,
    ) -> None:
        self._navigate_waypoint = navigate_waypoint
        self._navigate_fixed_place = navigate_fixed_place
        self._fixed_places = {
            str(name): str(place)
            for name, place in (fixed_places or {}).items()
        }
        self._motion_adapter = motion_adapter
        self._waypoints = {
            name: dict(config) for name, config in waypoints.items()
        }
        self._behavior_routes = {
            name: {
                **dict(config),
                "stages": list(config.get("stages", [])),
            }
            for name, config in behavior_routes.items()
        }
        self._stage_actions = set(stage_actions)
        self._last_error = ""
        self._result_timeout_sec = float(result_timeout_sec)
        if (
            not math.isfinite(self._result_timeout_sec)
            or self._result_timeout_sec <= 0.0
        ):
            raise ValueError("result_timeout_sec must be finite and > 0")

        # ── waypoint_nav live-map random navigation ─────────────────
        self._random_navigation_behaviors: set[str] = set(
            random_navigation_behaviors or []
        )
        self._random_navigation_place = str(
            random_navigation_place or ""
        ).strip()
        timeout = (
            self._result_timeout_sec
            if random_navigation_result_timeout_sec is None
            else float(random_navigation_result_timeout_sec)
        )
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError(
                "random_navigation_result_timeout_sec must be finite and > 0"
            )
        self._random_navigation_result_timeout_sec = timeout

        # Per-behavior probability of staying in place (skipping the random
        # navigation) instead of walking around.  Only emotion-expression
        # ``express*Alone`` behaviors should be present here.
        self._random_navigation_in_place: dict[str, float] = {
            name: float(prob)
            for name, prob in (
                random_navigation_in_place_probability or {}
            ).items()
        }

        # Temporary override: while non-empty, each random navigation picks
        # one of these fixed waypoints (resolved through ``fixed_places``)
        # instead of asking waypoint_nav for a live-map random target.
        self._random_navigation_fixed_pool: list[str] = [
            str(name) for name in (random_navigation_fixed_pool or [])
        ]

    @property
    def routed_behaviors(self) -> set[str]:
        return set(self._behavior_routes)

    @property
    def last_error(self) -> str:
        return self._last_error

    def waypoint_for_behavior(self, behavior_name: str) -> str | None:
        """Return the waypoint name for a behavior, or the random sentinel.

        Returns:
            * A waypoint name (``str``) for fixed-waypoint behaviors.
            * ``_RANDOM_WAYPOINT_SENTINEL`` for behaviors which request the
              reserved waypoint_nav random target.
            * ``None`` if the behavior has no navigation routing.
        """
        if behavior_name in self._random_navigation_behaviors:
            return _RANDOM_WAYPOINT_SENTINEL
        route = self._behavior_routes.get(behavior_name)
        if route is None:
            return None
        return str(route["waypoint"])

    def handles(self, behavior_name: str, stage_id: str) -> bool:
        route = self._behavior_routes.get(behavior_name)
        if route is None:
            return False
        return stage_id in route["stages"]

    def navigate_for_behavior(
        self,
        behavior_name: str,
        *,
        timeout_sec: float | None = None,
        task_id: str | None = None,
        status_callback: Callable[[Any], None] | None = None,
    ) -> bool:
        self._last_error = ""
        waypoint_name = self.waypoint_for_behavior(behavior_name)
        if waypoint_name is None:
            return True

        effective_timeout = self._result_timeout_sec
        if timeout_sec is not None and math.isfinite(float(timeout_sec)):
            if float(timeout_sec) > 0.0:
                effective_timeout = min(effective_timeout, float(timeout_sec))

        if waypoint_name is _RANDOM_WAYPOINT_SENTINEL:
            in_place_prob = self._random_navigation_in_place.get(behavior_name)
            if in_place_prob is not None and random.random() < in_place_prob:
                logger.info(
                    "Behavior %s stays in place (skipped random navigation, "
                    "probability=%.2f)",
                    behavior_name,
                    in_place_prob,
                )
                return True
            if self._navigate_fixed_place is None:
                logger.error(
                    "Random navigation behavior %s requires waypoint_nav",
                    behavior_name,
                )
                return False
            random_target = self._pick_random_navigation_target(behavior_name)
            if not random_target:
                logger.error(
                    "Random navigation behavior %s has no configured "
                    "waypoint_nav random target",
                    behavior_name,
                )
                return False
            if not task_id:
                logger.error(
                    "Random navigation behavior %s requires a registered "
                    "task_id",
                    behavior_name,
                )
                return False
            logger.info(
                "Navigating behavior %s through waypoint_nav random target %s",
                behavior_name,
                random_target,
            )
            return self._navigate_fixed_with_chassis(
                task_id,
                random_target,
                min(
                    effective_timeout,
                    self._random_navigation_result_timeout_sec,
                ),
                status_callback,
            )

        return self._navigate_named_waypoint(
            waypoint_name,
            behavior_name=behavior_name,
            task_id=task_id,
            timeout_sec=effective_timeout,
            status_callback=status_callback,
        )

    def _pick_random_navigation_target(self, behavior_name: str) -> str:
        """Resolve the waypoint_nav place for a random-navigation behavior.

        When the temporary ``random_navigation_fixed_pool`` override is
        non-empty it wins: one of those fixed waypoints is picked at random on
        every call and resolved through the same ``places`` mapping the fixed
        routes use, so the chosen target is a known point.  Otherwise the
        reserved live-map random target is returned unchanged.
        """
        if self._random_navigation_fixed_pool:
            waypoint_name = random.choice(self._random_navigation_fixed_pool)
            place = self._fixed_places.get(waypoint_name, waypoint_name)
            logger.info(
                "Behavior %s draws fixed random-pool waypoint %s -> place %s",
                behavior_name,
                waypoint_name,
                place,
            )
            return place
        return self._random_navigation_place

    def _navigate_named_waypoint(
        self,
        waypoint_name: str,
        *,
        behavior_name: str,
        task_id: str | None,
        timeout_sec: float,
        status_callback: Callable[[Any], None] | None,
    ) -> bool:
        waypoint = self._waypoints.get(waypoint_name)
        if waypoint is None:
            logger.error("Unknown registered waypoint %s", waypoint_name)
            return False
        logger.info(
            "Navigating behavior %s to waypoint %s (%s)",
            behavior_name,
            waypoint_name,
            waypoint.get("label", ""),
        )
        if self._navigate_fixed_place is not None:
            if not task_id:
                logger.error(
                    "Fixed waypoint %s requires a registered task_id",
                    waypoint_name,
                )
                return False
            place = self._fixed_places.get(waypoint_name)
            if not place:
                logger.error(
                    "Fixed waypoint %s has no waypoint_nav place",
                    waypoint_name,
                )
                return False
            return self._navigate_fixed_with_chassis(
                task_id,
                place,
                timeout_sec,
                status_callback,
            )
        # Backward-compatible injection path used by ROS-independent tests and
        # integrators which have not enabled the shared point-name service.
        return self._navigate_with_chassis(
            waypoint_name, waypoint, timeout_sec
        )

    def _navigate_fixed_with_chassis(
        self,
        task_id: str,
        place: str,
        timeout_sec: float,
        status_callback: Callable[[Any], None] | None,
    ) -> bool:
        """Run platform hooks around a named ``waypoint_nav`` task."""
        prepare = getattr(self._motion_adapter, "prepare_navigation", None)
        finish = getattr(self._motion_adapter, "finish_navigation", None)
        if callable(prepare) and not bool(prepare()):
            logger.error("Chassis rejected navigation preflight")
            self._last_error = self._motion_failure_reason(
                "chassis_navigation_preflight_failed"
            )
            return False
        navigation_ok = False
        finish_ok = True
        try:
            navigation_ok = bool(
                self._navigate_fixed_place(
                    task_id,
                    place,
                    timeout_sec,
                    status_callback,
                )
            )
        finally:
            if callable(finish):
                finish_result = finish()
                finish_ok = finish_result is not False
        if navigation_ok and not finish_ok:
            logger.error("Chassis did not settle after waypoint navigation")
            self._last_error = self._motion_failure_reason(
                "chassis_navigation_settle_failed"
            )
        return navigation_ok and finish_ok

    def _navigate_with_chassis(
        self,
        waypoint_name: str,
        waypoint: Mapping[str, Any],
        timeout_sec: float,
    ) -> bool:
        """Run optional platform mode hooks around an external Nav2 goal."""
        prepare = getattr(self._motion_adapter, "prepare_navigation", None)
        finish = getattr(self._motion_adapter, "finish_navigation", None)
        if callable(prepare) and not bool(prepare()):
            logger.error("Chassis rejected navigation preflight")
            self._last_error = self._motion_failure_reason(
                "chassis_navigation_preflight_failed"
            )
            return False
        navigation_ok = False
        finish_ok = True
        try:
            navigation_ok = bool(
                self._navigate_waypoint(
                    waypoint_name,
                    waypoint,
                    timeout_sec,
                )
            )
        finally:
            if callable(finish):
                finish_result = finish()
                finish_ok = finish_result is not False
        if navigation_ok and not finish_ok:
            logger.error("Chassis did not settle after Nav2 navigation")
            self._last_error = self._motion_failure_reason(
                "chassis_navigation_settle_failed"
            )
        return navigation_ok and finish_ok

    def _motion_failure_reason(self, fallback: str) -> str:
        return str(
            getattr(self._motion_adapter, "last_error", "") or fallback
        ).strip()

    def execute_step(
        self,
        unit_config: Mapping[str, Any],
        ctx: Any,
        duration: float | None = None,
    ) -> bool:
        """Execute the platform action configured for the current stage."""
        behavior_name = str(getattr(ctx, "resolved_behavior_name", ""))
        stage_id = str(getattr(ctx, "current_stage", ""))
        if behavior_name in self._random_navigation_behaviors:
            if stage_id != "navigation":
                return False
            return self.hold_position(duration)
        route = self._behavior_routes.get(behavior_name)
        if route is None or stage_id not in route["stages"]:
            return False
        if getattr(ctx, "motion_state", "active") == "stationary":
            return self.hold_position(duration)
        unit_id = str(unit_config.get("unit_id", ""))
        if unit_id not in self._stage_actions:
            logger.error(
                "No navigation-stage action for %s/%s/%s",
                behavior_name,
                stage_id,
                unit_id,
            )
            return False
        return bool(self._motion_adapter.execute_step(unit_config, ctx, duration))

    def hold_position(self, duration_sec: float | None = None) -> bool:
        """Keep the chassis stationary while retaining the semantic stage."""
        hold_position = getattr(self._motion_adapter, "hold_position", None)
        if callable(hold_position):
            return bool(hold_position(duration_sec))
        self._motion_adapter.cancel_step()
        return True

    def cancel_step(self, step: Any = None) -> None:
        del step
        navigators = [self._navigate_fixed_place, self._navigate_waypoint]
        seen: set[int] = set()
        for navigator in navigators:
            owner = getattr(navigator, "__self__", navigator)
            if owner is None or id(owner) in seen:
                continue
            seen.add(id(owner))
            cancel_navigation = getattr(owner, "cancel_navigation", None)
            if callable(cancel_navigation):
                cancel_navigation()
        self._motion_adapter.cancel_step()

    def emergency_stop(self) -> None:
        self.cancel_step()


class Ros2Nav2Client:
    """Synchronous facade over an asynchronous Nav2 NavigateToPose client.

    The action-executor node runs in a ``MultiThreadedExecutor``. This class
    waits on futures from one executor thread while the other executor thread
    services Nav2 action responses and feedback.
    """

    def __init__(
        self,
        node: Any,
        *,
        action_name: str = "/navigate_to_pose",
        frame_id: str = "map",
        server_timeout_sec: float = 10.0,
        should_stop: Callable[[], bool] | None = None,
    ) -> None:
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup

        self._node = node
        self._action_type = NavigateToPose
        self._frame_id = frame_id
        self._server_timeout_sec = float(server_timeout_sec)
        if (
            not math.isfinite(self._server_timeout_sec)
            or self._server_timeout_sec <= 0.0
        ):
            raise ValueError("server_timeout_sec must be finite and > 0")
        self._should_stop = should_stop or (lambda: False)
        self._client = ActionClient(
            node,
            NavigateToPose,
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
        waypoint_name: str,
        waypoint: Mapping[str, Any],
        timeout_sec: float,
    ) -> bool:
        from action_msgs.msg import GoalStatus

        self._cancel_requested.clear()
        try:
            total_timeout = float(timeout_sec)
        except (TypeError, ValueError):
            total_timeout = 0.0
        if not math.isfinite(total_timeout) or total_timeout <= 0.0:
            self._node.get_logger().error(
                f"Nav2 goal {waypoint_name} has no remaining time budget"
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
                f"Nav2 server unavailable: {self._action_name}"
            )
            return False

        goal = self._action_type.Goal()
        goal.pose.header.frame_id = str(
            waypoint.get("frame_id", self._frame_id)
        )
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(waypoint["x"])
        goal.pose.pose.position.y = float(waypoint["y"])

        orientation_z = float(waypoint["orientation_z"])
        orientation_w = float(waypoint["orientation_w"])
        norm = math.hypot(orientation_z, orientation_w)
        goal.pose.pose.orientation.z = orientation_z / norm
        goal.pose.pose.orientation.w = orientation_w / norm

        self._node.get_logger().info(
            f"Nav2 goal {waypoint_name}: "
            f"x={goal.pose.pose.position.x:.3f}, "
            f"y={goal.pose.pose.position.y:.3f}, "
            f"z={goal.pose.pose.orientation.z:.3f}, "
            f"w={goal.pose.pose.orientation.w:.3f}"
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
                f"Nav2 goal {waypoint_name} was not accepted in time"
            )
            return False

        try:
            goal_handle = send_future.result()
        except Exception as exc:
            self._node.get_logger().error(
                f"Nav2 goal {waypoint_name} send failed: {exc}"
            )
            return False
        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().error(
                f"Nav2 goal {waypoint_name} was rejected"
            )
            return False

        with self._goal_lock:
            self._active_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_wait = max(0.0, deadline - time.monotonic())
        if not self._wait_future(result_future, result_wait):
            self.cancel_navigation()
            self._node.get_logger().error(
                f"Nav2 goal {waypoint_name} timed out after "
                f"{total_timeout:.1f}s total"
            )
            return False

        try:
            wrapped_result = result_future.result()
        except Exception as exc:
            with self._goal_lock:
                self._active_goal_handle = None
            self._node.get_logger().error(
                f"Nav2 goal {waypoint_name} result failed: {exc}"
            )
            return False
        with self._goal_lock:
            self._active_goal_handle = None

        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info(
                f"Nav2 goal {waypoint_name} succeeded"
            )
            return True

        self._node.get_logger().error(
            f"Nav2 goal {waypoint_name} failed: "
            f"status={wrapped_result.status}"
        )
        return False

    def cancel_navigation(self) -> None:
        """Request cancellation of the active Nav2 goal, if any."""
        self._cancel_requested.set()
        with self._goal_lock:
            goal_handle = self._active_goal_handle
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self._node.get_logger().warning(
                    f"Failed to cancel Nav2 goal: {exc}"
                )

    def _wait_future(self, future: Any, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while time.monotonic() < deadline:
            if future.done():
                return True
            if self._cancel_requested.is_set() or self._should_stop():
                self.cancel_navigation()
                return False
            time.sleep(0.05)
        return future.done()

    def _on_feedback(self, feedback_message: Any) -> None:
        now = time.monotonic()
        if now - self._last_feedback_log < 2.0:
            return
        self._last_feedback_log = now
        feedback = feedback_message.feedback
        self._node.get_logger().info(
            "Nav2 feedback: "
            f"remaining={float(feedback.distance_remaining):.2f}m, "
            f"recoveries={int(feedback.number_of_recoveries)}"
        )

    def _cancel_late_goal_if_requested(self, send_future: Any) -> None:
        """Cancel a goal accepted after the caller already stopped waiting."""
        if not self._cancel_requested.is_set() and not self._should_stop():
            return
        try:
            goal_handle = send_future.result()
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
        except Exception as exc:
            self._node.get_logger().warning(
                f"Failed to cancel late Nav2 goal: {exc}"
            )
