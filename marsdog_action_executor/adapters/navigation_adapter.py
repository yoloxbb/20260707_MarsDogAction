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

# Sentinel returned by waypoint_for_behavior when the behavior should navigate
# to a randomly generated pose rather than a fixed waypoint.
_RANDOM_WAYPOINT_SENTINEL = "__random__"

logger = logging.getLogger(__name__)


class BehaviorMobilityAdapter:
    """Navigate routed behaviors, then execute their existing stages as Twist."""

    def __init__(
        self,
        navigate_waypoint: Callable[[str, Mapping[str, Any], float], bool],
        motion_adapter: Any,
        waypoints: Mapping[str, Mapping[str, Any]],
        behavior_routes: Mapping[str, Mapping[str, Any]],
        action_motion_groups: Mapping[str, str],
        *,
        result_timeout_sec: float = 300.0,
        random_navigation_behaviors: list[str] | None = None,
        random_navigation_bounds: Mapping[str, float] | None = None,
        random_navigation_region: Mapping[str, Any] | None = None,
        random_navigation_exclude_waypoints: list[str] | None = None,
    ) -> None:
        self._navigate_waypoint = navigate_waypoint
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
        self._action_motion_groups = dict(action_motion_groups)
        self._result_timeout_sec = float(result_timeout_sec)
        if (
            not math.isfinite(self._result_timeout_sec)
            or self._result_timeout_sec <= 0.0
        ):
            raise ValueError("result_timeout_sec must be finite and > 0")

        # ── Random navigation support ──────────────────────────────────
        self._random_navigation_behaviors: set[str] = set(
            random_navigation_behaviors or []
        )
        # Parse polygon region (new) or fall back to bounds (legacy).
        region = dict(random_navigation_region or {})
        polygon_raw = region.get("polygon")
        if polygon_raw and isinstance(polygon_raw, list) and len(polygon_raw) >= 3:
            self._random_region_polygon: list[tuple[float, float]] = [
                (float(pt[0]), float(pt[1])) for pt in polygon_raw
            ]
        else:
            # Legacy bounds mode: construct a rectangular polygon.
            bnd = dict(random_navigation_bounds or {})
            x_min = float(bnd.get("x_min", -32.4))
            x_max = float(bnd.get("x_max", 9.2))
            y_min = float(bnd.get("y_min", -10.0))
            y_max = float(bnd.get("y_max", 22.0))
            self._random_region_polygon = [
                (x_min, y_min),
                (x_max, y_min),
                (x_max, y_max),
                (x_min, y_max),
            ]
        # Build set of waypoint coords to exclude from random generation.
        exclude_names = set(random_navigation_exclude_waypoints or [])
        self._excluded_coords: set[tuple[float, float]] = set()
        for wpt_name in exclude_names:
            wpt = self._waypoints.get(wpt_name)
            if wpt is not None:
                self._excluded_coords.add(
                    (float(wpt["x"]), float(wpt["y"]))
                )

    @property
    def routed_behaviors(self) -> set[str]:
        return set(self._behavior_routes)

    def waypoint_for_behavior(self, behavior_name: str) -> str | None:
        """Return the waypoint name for a behavior, or the random sentinel.

        Returns:
            * A waypoint name (``str``) for fixed-waypoint behaviors.
            * ``_RANDOM_WAYPOINT_SENTINEL`` for random-navigation behaviors.
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
    ) -> bool:
        waypoint_name = self.waypoint_for_behavior(behavior_name)
        if waypoint_name is None:
            return True

        effective_timeout = self._result_timeout_sec
        if timeout_sec is not None and math.isfinite(float(timeout_sec)):
            if float(timeout_sec) > 0.0:
                effective_timeout = min(effective_timeout, float(timeout_sec))

        if waypoint_name is _RANDOM_WAYPOINT_SENTINEL:
            waypoint = self._generate_random_pose()
            logger.info(
                "Navigating behavior %s to random pose: "
                "x=%.3f, y=%.3f, oz=%.3f, ow=%.3f",
                behavior_name,
                waypoint["x"],
                waypoint["y"],
                waypoint["orientation_z"],
                waypoint["orientation_w"],
            )
            return bool(
                self._navigate_waypoint(
                    "_random_",
                    waypoint,
                    effective_timeout,
                )
            )

        waypoint = self._waypoints[waypoint_name]
        logger.info(
            "Navigating behavior %s to waypoint %s (%s)",
            behavior_name,
            waypoint_name,
            waypoint.get("label", ""),
        )
        return bool(
            self._navigate_waypoint(
                waypoint_name,
                waypoint,
                effective_timeout,
            )
        )

    # ── Random pose generation ────────────────────────────────────────

    @staticmethod
    def _triangle_area(
        a: tuple[float, float],
        b: tuple[float, float],
        c: tuple[float, float],
    ) -> float:
        """Signed area of triangle (a, b, c); positive if CCW."""
        return 0.5 * abs(
            (b[0] - a[0]) * (c[1] - a[1])
            - (c[0] - a[0]) * (b[1] - a[1])
        )

    @staticmethod
    def _sample_triangle(
        a: tuple[float, float],
        b: tuple[float, float],
        c: tuple[float, float],
    ) -> tuple[float, float]:
        """Uniformly sample a point inside triangle (a, b, c)."""
        r1 = random.random()
        r2 = random.random()
        if r1 + r2 > 1.0:
            r1 = 1.0 - r1
            r2 = 1.0 - r2
        return (
            a[0] + r1 * (b[0] - a[0]) + r2 * (c[0] - a[0]),
            a[1] + r1 * (b[1] - a[1]) + r2 * (c[1] - a[1]),
        )

    @staticmethod
    def _point_in_polygon(
        x: float,
        y: float,
        poly: list[tuple[float, float]],
    ) -> bool:
        """Ray-casting test: is (x, y) inside the polygon?"""
        inside = False
        n = len(poly)
        j = n - 1
        for i in range(n):
            xi, yi = poly[i]
            xj, yj = poly[j]
            if ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / (yj - yi) + xi
            ):
                inside = not inside
            j = i
        return inside

    def _generate_random_pose(self) -> dict[str, Any]:
        """Generate a random navigable pose inside the configured polygon.

        Uses triangle-area-weighted sampling for uniform distribution over
        a convex polygon.  Falls back to the polygon centroid on failure.

        Returns:
            A dict with keys ``x``, ``y``, ``orientation_z``,
            ``orientation_w``, ``frame_id``, and ``label``.
        """
        poly = self._random_region_polygon
        n = len(poly)

        if n < 3:
            logger.error("Random region polygon has fewer than 3 vertices.")
            return {
                "x": 0.0, "y": 0.0,
                "orientation_z": 0.0, "orientation_w": 1.0,
                "frame_id": "map", "label": "random (fallback)",
            }

        # ── Triangulate from vertex 0 ────────────────────────────────
        # For convex polygon [v0, v1, …, v_{n-1}], triangles are
        # (v0, v_i, v_{i+1}) for i=1,…,n-2.
        triangles: list[
            tuple[
                tuple[float, float],
                tuple[float, float],
                tuple[float, float],
            ]
        ] = []
        areas: list[float] = []
        v0 = poly[0]
        for i in range(1, n - 1):
            tri = (v0, poly[i], poly[i + 1])
            triangles.append(tri)
            areas.append(self._triangle_area(*tri))

        total_area = sum(areas)
        if total_area <= 0.0:
            logger.error("Random region polygon has zero area.")
            return {
                "x": 0.0, "y": 0.0,
                "orientation_z": 0.0, "orientation_w": 1.0,
                "frame_id": "map", "label": "random (fallback)",
            }

        # ── Rejection sampling against excluded waypoints ────────────
        max_attempts = 200
        for _ in range(max_attempts):
            # Pick triangle proportional to its area.
            pick = random.uniform(0.0, total_area)
            cumulative = 0.0
            chosen_tri = triangles[0]
            for tri, area in zip(triangles, areas):
                cumulative += area
                if pick <= cumulative:
                    chosen_tri = tri
                    break

            x, y = self._sample_triangle(*chosen_tri)

            # Safety net: verify the point is actually inside the polygon.
            if not self._point_in_polygon(x, y, poly):
                continue

            # Exclude points too close to internal-need waypoints.
            if self._excluded_coords:
                too_close = any(
                    math.hypot(x - ex, y - ey) < 0.5
                    for ex, ey in self._excluded_coords
                )
                if too_close:
                    continue

            yaw = random.uniform(-math.pi, math.pi)
            half_yaw = yaw * 0.5
            return {
                "x": x,
                "y": y,
                "orientation_z": math.sin(half_yaw),
                "orientation_w": math.cos(half_yaw),
                "frame_id": "map",
                "label": "random",
            }

        # ── Fallback: polygon centroid ───────────────────────────────
        logger.warning(
            "Could not generate a random pose inside polygon after "
            "%d attempts; using centroid.",
            max_attempts,
        )
        cx = sum(v[0] for v in poly) / n
        cy = sum(v[1] for v in poly) / n
        return {
            "x": cx,
            "y": cy,
            "orientation_z": 0.0,
            "orientation_w": 1.0,
            "frame_id": "map",
            "label": "random (fallback)",
        }

    def execute_step(
        self,
        unit_config: Mapping[str, Any],
        ctx: Any,
        duration: float | None = None,
    ) -> bool:
        """Execute the Twist proxy configured for the current existing stage."""
        behavior_name = str(getattr(ctx, "resolved_behavior_name", ""))
        stage_id = str(getattr(ctx, "current_stage", ""))
        route = self._behavior_routes.get(behavior_name)
        if route is None or stage_id not in route["stages"]:
            return False
        if getattr(ctx, "motion_state", "active") == "stationary":
            return self.hold_position(duration)
        del duration
        unit_id = str(unit_config.get("unit_id", ""))
        group_name = self._action_motion_groups.get(unit_id)
        if group_name is None:
            logger.error(
                "No navigation-stage motion group for %s/%s/%s",
                behavior_name,
                stage_id,
                unit_id,
            )
            return False
        return bool(self._motion_adapter.execute_group(group_name, ctx))

    def hold_position(self, duration_sec: float | None = None) -> bool:
        """Keep the chassis stationary while retaining the semantic stage."""
        hold_position = getattr(self._motion_adapter, "hold_position", None)
        if callable(hold_position):
            return bool(hold_position(duration_sec))
        self._motion_adapter.cancel_step()
        return True

    def cancel_step(self, step: Any = None) -> None:
        del step
        cancel_navigation = getattr(
            self._navigate_waypoint,
            "cancel_navigation",
            None,
        )
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
        if not self._client.wait_for_server(
            timeout_sec=self._server_timeout_sec
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
        if not self._wait_future(send_future, self._server_timeout_sec):
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
        if not self._wait_future(result_future, timeout_sec):
            self.cancel_navigation()
            self._node.get_logger().error(
                f"Nav2 goal {waypoint_name} timed out after "
                f"{timeout_sec:.1f}s"
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
