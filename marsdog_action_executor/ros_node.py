"""ROS2 Action Server node for the MarsDog action executor.

Provides the ``/execute_behavior`` Action Server and optional debug topics.

Architecture (v2)::

    ROS2 Action goal
        │
        ▼
    GoalParser         ← params_json → ExecutionContext
        │
        ▼
    BehaviorResolver   ← exact behavior-name validation
        │
        ▼
    StageExecutor      ← filter candidates → select → execute units / stage
        │
        ▼
    ResultEvaluator    ← behavior-level success condition

Key interfaces:
  - Action Server: ``/execute_behavior`` (marsdog_interfaces/action/ExecuteBehavior)
  - Debug topics:   ``/debug/execute_behavior/goal|feedback|result``

Run via::

    ros2 run marsdog_action_executor action_executor_node
"""

from __future__ import annotations

import json
import logging
import math
import os
import sys
import threading
import time
import uuid
from pathlib import Path

from .adapters.velocity import TwistCommand
from .adapters.go2_sport_backend import (
    Go2ChassisBackend,
    Ros2Go2SportPublisher,
)
from .adapters.lite3_backend import (
    Lite3ChassisBackend,
    Ros2Lite3StatusSubscriber,
    make_ros2_lite3_backend,
)
from .adapters.attention_tracking_controller import (
    AttentionTrackingController,
)
from .adapters.navigation_adapter import (
    BehaviorMobilityAdapter,
    Ros2Nav2Client,
)
from .adapters.waypoint_nav_adapter import (
    Ros2WaypointNavClient,
    WaypointNavStatus,
)
from .adapters.wake_orientation_adapter import (
    Ros2Nav2SpinClient,
    WakeOrientationAdapter,
)
from .adapters.target_approach_adapter import TargetApproachAdapter
from .adapters.person_nav_approach_adapter import (
    PersonNavApproachAdapter,
    Ros2PersonApproachTransport,
)
from .adapters.visual_target_approach_adapter import (
    VisualTargetApproachAdapter,
)
from .adapters.stationary_expression_adapter import (
    StationaryExpressionAdapter,
)
from .adapters.uwb_follow_adapter import UwbFollowAdapter
from .adapters.uwb_follow_action_adapter import UwbFollowActionAdapter
from .adapters.uwb_roam_adapter import UwbRoamAdapter
from .behavior_resolver import BehaviorResolver
from .config_loader import ConfigLoader
from .debug_publishers import DebugPublishers
from .eligibility_checker import EligibilityChecker
from .execution_context import ExecutionContext, parse_params_json_object
from .goal_parser import GoalParser
from .interrupt_manager import InterruptManager
from .models import BehaviorGoal, ExecutionFeedback, ExecutionResult
from .posture_manager import PostureManager
from .result_evaluator import ResultEvaluator
from .ros2_compat import HAS_ROS2, get_execute_behavior_action, get_action_source
from .sound_player import BehaviorSoundController
from .stage_executor import StageExecutor, StageResult
from .stationary_policy import (
    INPLACE_WITH_HUMAN_BEHAVIORS,
    validate_inplace_plan,
)

logger = logging.getLogger(__name__)
_ExecuteBehavior = get_execute_behavior_action()


def _normalise_goal_timeout(value: object) -> float:
    """Return a finite budget; zero means no outer deadline."""
    try:
        timeout = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(timeout):
        return 0.0
    return max(0.0, timeout)


def _outer_runtime_deadline(started: float, timeout_sec: float) -> float | None:
    """Keep the outer Goal alive when its requested budget is non-positive."""
    return started + timeout_sec if timeout_sec > 0.0 else None


def _navigation_cancel_won(cancel_requested: bool, waypoint_outcome) -> bool:
    """Return whether a cancel request is confirmed by the navigation truth.

    For fixed waypoint navigation, an acknowledged cancel is not enough.  The
    original task must reach INTERRUPTED.  A competing SUCCEEDED or FAILED
    terminal remains authoritative.  ``None`` is retained for the direct Nav2
    path and for cancellation before a waypoint task was dispatched.
    """
    if not cancel_requested:
        return False
    if waypoint_outcome is None:
        return True
    return bool(
        waypoint_outcome.terminal_confirmed
        and waypoint_outcome.state == "INTERRUPTED"
    )


def _follow_is_driving(adapter) -> bool:
    """Whether a UWB controller currently owns chassis motion."""
    return adapter is not None and bool(adapter.active)


def _dispatch_visual_event(
    raw_data,
    *,
    attention_controller=None,
    target_approach_adapter=None,
    visual_target_approach_adapter=None,
    wake_orientation_adapter=None,
    person_nav_approach_adapter=None,
) -> bool:
    """Decode one visual event and fail the motion path closed.

    A malformed/non-object message is still safety-relevant while the target
    approach loop is moving.  Feeding an empty snapshot into the adapter
    invalidates its active observation and publishes the redundant zero Twist
    before this function returns.
    """
    try:
        payload = json.loads(raw_data)
    except (json.JSONDecodeError, TypeError):
        payload = None
    if not isinstance(payload, dict):
        if target_approach_adapter is not None:
            target_approach_adapter.update_visual({})
        if visual_target_approach_adapter is not None:
            visual_target_approach_adapter.update_visual({})
        if wake_orientation_adapter is not None:
            wake_orientation_adapter.update_visual({})
        if person_nav_approach_adapter is not None:
            person_nav_approach_adapter.update_visual({})
        return False
    if attention_controller is not None:
        attention_controller.update_visual(payload)
    if target_approach_adapter is not None:
        target_approach_adapter.update_visual(payload)
    if visual_target_approach_adapter is not None:
        visual_target_approach_adapter.update_visual(payload)
    if wake_orientation_adapter is not None:
        wake_orientation_adapter.update_visual(payload)
    if person_nav_approach_adapter is not None:
        person_nav_approach_adapter.update_visual(payload)
    return True


def _dispatch_object_detection(
    raw_data,
    *,
    visual_target_approach_adapter=None,
) -> bool:
    """Decode one object-detection v2 packet for the active leased stream."""
    try:
        payload = json.loads(raw_data)
    except (json.JSONDecodeError, TypeError):
        return False
    if not isinstance(payload, dict):
        return False
    if visual_target_approach_adapter is None:
        return True
    return bool(visual_target_approach_adapter.update_object_detection(payload))


def _debug_goal_from_request(
    goal_request,
    *,
    timestamp: float | None = None,
) -> tuple[BehaviorGoal, bool]:
    """Build a debug Goal without trusting the ROS ``params_json`` field.

    The boolean reports whether the field was a valid JSON object.  Invalid
    input is represented as an empty parameter mapping so even a direct test
    harness call to ``_on_accepted`` cannot raise while publishing debug data.
    The execution path still parses the original field and returns
    ``invalid_params``.
    """
    params = parse_params_json_object(
        getattr(goal_request, "params_json", "{}")
    )
    params_valid = params is not None
    goal_id = getattr(goal_request, "goal_id", "")
    return (
        BehaviorGoal(
            goal_id=goal_id,
            behavior_id=getattr(goal_request, "behavior_id", goal_id),
            behavior_name=getattr(goal_request, "behavior_name", ""),
            priority_level=getattr(goal_request, "priority_level", 0),
            params=dict(params) if params_valid else {},
            timeout_sec=getattr(goal_request, "timeout_sec", 60.0),
            timestamp=time.time() if timestamp is None else timestamp,
        ),
        params_valid,
    )


def _shutdown_rclpy_if_ok(
    rclpy_module,
    rcl_error_type: type[BaseException],
) -> bool:
    """Shut down one ROS context once, tolerating only the shutdown race.

    Returns ``True`` only when this call performed the shutdown.  A signal
    handler may close the default context between ``ok()`` and ``shutdown()``;
    that specific ``RCLError`` is an idempotent success path.  Other exception
    types deliberately propagate.
    """
    try:
        if not rclpy_module.ok():
            return False
        rclpy_module.shutdown()
    except rcl_error_type:
        return False
    return True

# ═══════════════════════════════════════════════════════════════════════════════
# ROS2-dependent class
# ═══════════════════════════════════════════════════════════════════════════════

if HAS_ROS2:
    import rclpy
    from rclpy._rclpy_pybind11 import RCLError
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.utilities import get_rmw_implementation_identifier

    try:
        from marsdog_vision_interaction.srv import VisionTask
    except ImportError:
        VisionTask = None  # type: ignore[assignment,misc]

    class _VisionObjectDetectionLeaseClient:
        """Bounded synchronous facade over the asynchronous VisionTask service."""

        def __init__(
            self,
            node,
            *,
            service_name: str,
            timeout_sec: float,
            confidence: float,
            callback_group,
        ) -> None:
            self._node = node
            self._timeout_sec = max(0.1, float(timeout_sec))
            self._confidence = min(1.0, max(0.0, float(confidence)))
            self._client = (
                node.create_client(
                    VisionTask,
                    service_name,
                    callback_group=callback_group,
                )
                if VisionTask is not None
                else None
            )

        @property
        def available(self) -> bool:
            return self._client is not None

        def start(
            self,
            session_id: str,
            labels: list[str],
            rate_hz: float,
            lease_sec: float,
        ) -> tuple[bool, str]:
            return self._call({
                "enabled": True,
                "session_id": session_id,
                "target_labels": labels,
                "rate_hz": float(rate_hz),
                "confidence": self._confidence,
                "lease_sec": float(lease_sec),
            })

        def stop(self, session_id: str) -> None:
            ok, reason = self._call({
                "enabled": False,
                "session_id": session_id,
            })
            if not ok:
                self._node.get_logger().warning(
                    "Object detection stream stop failed: "
                    f"session={session_id} reason={reason}"
                )

        def _call(self, params: dict) -> tuple[bool, str]:
            client = self._client
            if client is None:
                return False, "vision_task_interface_unavailable"
            if not client.wait_for_service(timeout_sec=self._timeout_sec):
                return False, "vision_task_service_unavailable"
            request = VisionTask.Request()
            request.task_id = f"action-object-{uuid.uuid4().hex}"
            request.task_type = "set_object_detection"
            request.params_json = json.dumps(params, ensure_ascii=False)
            future = client.call_async(request)
            completed = threading.Event()
            future.add_done_callback(lambda _future: completed.set())
            if not completed.wait(self._timeout_sec):
                future.cancel()
                return False, "vision_task_service_timeout"
            try:
                response = future.result()
            except Exception as exc:
                return False, f"vision_task_service_error:{exc}"
            if response is None:
                return False, "vision_task_empty_response"
            try:
                result = json.loads(response.result_json or "{}")
            except (json.JSONDecodeError, TypeError):
                result = {}
            ok = bool(response.success) and bool(result.get("ok", False))
            reason = str(
                result.get("error")
                or response.error_message
                or ("" if ok else "vision_task_rejected")
            )
            return ok, reason

    class ActionExecutorNode(Node):
        """ROS2 node wrapping the v2 execution pipeline.

        Action Server: ``/execute_behavior``
        Debug Topics:  ``/debug/execute_behavior/goal|feedback|result``
        """

        def __init__(self) -> None:
            super().__init__("action_executor_node")
            self.get_logger().info(
                "ROS transport: "
                f"domain_id={os.environ.get('ROS_DOMAIN_ID', '0')}, "
                f"rmw={get_rmw_implementation_identifier()}, "
                f"localhost_only={os.environ.get('ROS_LOCALHOST_ONLY', '0')}"
            )

            # ── Load configuration ──────────────────────────────────────
            config_dir = self._resolve_config_dir()
            self.get_logger().info(f"Loading configs from: {config_dir}")
            self._config = ConfigLoader(config_dir)
            self._config.load_all()
            self.get_logger().info(
                f"Strict behavior-tree contract loaded: "
                f"{len(self._config.behavior_tree_templates)} behaviors, "
                f"{len(self._config.action_catalog)} actions"
            )

            # ── Build v2 pipeline ───────────────────────────────────────
            self._goal_parser = GoalParser()
            self._behavior_resolver = BehaviorResolver(
                canonical_behaviors=self._config.get_behavior_names(),
            )
            self._eligibility = EligibilityChecker(
                action_catalog=self._config.action_catalog,
            )
            self._posture = PostureManager()
            self._interrupt = InterruptManager()

            # ── Select the Go2 or Lite3 chassis backend ────────────────
            go2_config = self._config.go2_sport_config
            go2_limits = go2_config.get("limits", {})
            lite3_config = self._config.lite3_action_config
            lite3_limits = lite3_config.get("limits", {})
            self.declare_parameter("recharge_result_energy_value", 100.0)
            self.declare_parameter("chassis_type", "go2")
            self.declare_parameter(
                "go2_enabled", bool(go2_config.get("enabled", False))
            )
            self.declare_parameter(
                "go2_request_topic",
                str(go2_config.get("request_topic", "/api/sport/request")),
            )
            self.declare_parameter(
                "go2_publish_rate_hz",
                float(go2_config.get("publish_rate_hz", 10.0)),
            )
            self.declare_parameter(
                "go2_max_linear_x",
                float(go2_limits.get("max_linear_x", 0.30)),
            )
            self.declare_parameter(
                "go2_min_linear_x",
                float(go2_limits.get("min_linear_x", 0.0)),
            )
            self.declare_parameter(
                "go2_max_linear_y",
                float(go2_limits.get("max_linear_y", 0.20)),
            )
            self.declare_parameter(
                "go2_max_angular_z",
                float(go2_limits.get("max_angular_z", 1.20)),
            )
            self.declare_parameter(
                "lite3_enabled", bool(lite3_config.get("enabled", False))
            )
            self.declare_parameter(
                "lite3_simple_cmd_topic",
                str(lite3_config.get("simple_cmd_topic", "/simple_cmd")),
            )
            self.declare_parameter(
                "lite3_status_topic",
                str(lite3_config.get("status_topic", "/robot_status")),
            )
            self.declare_parameter(
                "lite3_cmd_vel_topic",
                str(lite3_config.get("cmd_vel_topic", "/cmd_vel")),
            )
            self.declare_parameter(
                "lite3_allow_proxies",
                bool(lite3_config.get("allow_proxies", False)),
            )
            self.declare_parameter(
                "lite3_allow_unverified",
                bool(lite3_config.get("allow_unverified", False)),
            )
            self.declare_parameter(
                "lite3_publish_rate_hz",
                float(lite3_config.get("publish_rate_hz", 20.0)),
            )
            self.declare_parameter(
                "lite3_max_linear_x",
                float(lite3_limits.get("max_linear_x", 0.20)),
            )
            self.declare_parameter(
                "lite3_max_linear_y",
                float(lite3_limits.get("max_linear_y", 0.15)),
            )
            self.declare_parameter(
                "lite3_max_angular_z",
                float(lite3_limits.get("max_angular_z", 0.80)),
            )

            self._twist_publisher = None
            self._go2_publisher: Ros2Go2SportPublisher | None = None
            self._lite3_command_publisher = None
            self._lite3_status_subscriber: Ros2Lite3StatusSubscriber | None = None
            self._chassis_backend: (
                Go2ChassisBackend | Lite3ChassisBackend | None
            ) = None
            controller_adapters = {}
            chassis_type = str(
                self.get_parameter("chassis_type").value
            ).strip().lower()
            if chassis_type not in {"go2", "lite3"}:
                raise RuntimeError(
                    f"Unsupported chassis_type={chassis_type!r}; "
                    "expected 'go2' or 'lite3'"
                )

            rate_parameter = {
                "go2": "go2_publish_rate_hz",
                "lite3": "lite3_publish_rate_hz",
            }[chassis_type]
            linear_parameter = {
                "go2": "go2_max_linear_x",
                "lite3": "lite3_max_linear_x",
            }[chassis_type]
            angular_parameter = {
                "go2": "go2_max_angular_z",
                "lite3": "lite3_max_angular_z",
            }[chassis_type]
            chassis_publish_rate = float(
                self.get_parameter(rate_parameter).value
            )
            chassis_max_linear = float(
                self.get_parameter(linear_parameter).value
            )
            chassis_max_angular = float(
                self.get_parameter(angular_parameter).value
            )
            backend_config = {
                "go2": go2_config,
                "lite3": lite3_config,
            }[chassis_type]
            chassis_stop_count = int(backend_config.get("stop_publish_count", 3))

            if (
                chassis_type == "go2"
                and bool(self.get_parameter("go2_enabled").value)
            ):
                self._go2_publisher = Ros2Go2SportPublisher(
                    self,
                    str(self.get_parameter("go2_request_topic").value),
                )
                self._chassis_backend = Go2ChassisBackend(
                    request_publisher=self._go2_publisher,
                    action_sequences=self._config.get_go2_action_sequences(),
                    publish_rate_hz=chassis_publish_rate,
                    max_linear_x=chassis_max_linear,
                    min_linear_x=float(
                        self.get_parameter("go2_min_linear_x").value
                    ),
                    max_linear_y=float(
                        self.get_parameter("go2_max_linear_y").value
                    ),
                    max_angular_z=chassis_max_angular,
                    stop_publish_count=chassis_stop_count,
                    should_stop=lambda: self._interrupt.cancel_requested,
                )
                self._twist_publisher = self._chassis_backend.twist_publisher
                controller_adapters["go2"] = self._chassis_backend
                self.get_logger().info(
                    "Go2 SportMode backend enabled: "
                    f"topic={self.get_parameter('go2_request_topic').value}, "
                    f"rate={chassis_publish_rate}Hz, "
                    f"mapped_actions={len(self._chassis_backend.mapped_actions)}"
                )
            elif (
                chassis_type == "lite3"
                and bool(self.get_parameter("lite3_enabled").value)
            ):
                (
                    self._chassis_backend,
                    self._lite3_command_publisher,
                    _lite3_twist_publisher,
                ) = make_ros2_lite3_backend(
                    self,
                    simple_cmd_topic=str(
                        self.get_parameter("lite3_simple_cmd_topic").value
                    ),
                    cmd_vel_topic=str(
                        self.get_parameter("lite3_cmd_vel_topic").value
                    ),
                    action_plans=self._config.get_lite3_action_plans(),
                    # The follow adapter is built further down in this same
                    # branch, so the lambda has to defer the lookup: the
                    # checker runs on the first ownership preflight, long
                    # after __init__.  It tells a *driving* follow from an
                    # idle one, which is what lets a wake turn or Nav2 take
                    # /cmd_vel back between follow goals.
                    uwb_driving=lambda: _follow_is_driving(
                        getattr(self, "_uwb_follow_adapter", None)
                    ) or bool(getattr(getattr(self, "_uwb_roam_adapter", None), "active", False)),
                    allow_proxies=bool(
                        self.get_parameter("lite3_allow_proxies").value
                    ),
                    allow_unverified=bool(
                        self.get_parameter("lite3_allow_unverified").value
                    ),
                    accepted_unverified_actions=list(
                        lite3_config.get("accepted_unverified_actions", [])
                    ),
                    status_timeout_sec=float(
                        lite3_config.get("status_timeout_sec", 1.0)
                    ),
                    min_battery_percent=float(
                        lite3_config.get("min_battery_percent", 25.0)
                    ),
                    minimum_backward_clearance_m=float(
                        lite3_config.get(
                            "minimum_backward_clearance_m", 0.60
                        )
                    ),
                    mode_settle_sec=float(
                        lite3_config.get("mode_settle_sec", 0.10)
                    ),
                    navigation_settle_timeout_sec=float(
                        lite3_config.get(
                            "navigation_settle_timeout_sec", 2.0
                        )
                    ),
                    navigation_stable_samples=int(
                        lite3_config.get("navigation_stable_samples", 5)
                    ),
                    posture_recovery_timeout_sec=float(
                        lite3_config.get("posture_recovery_timeout_sec", 6.0)
                    ),
                    publish_rate_hz=chassis_publish_rate,
                    max_linear_x=chassis_max_linear,
                    max_linear_y=float(
                        self.get_parameter("lite3_max_linear_y").value
                    ),
                    max_angular_z=chassis_max_angular,
                    stop_publish_count=chassis_stop_count,
                    command_repeat=int(lite3_config.get("command_repeat", 1)),
                    command_repeat_interval_sec=float(
                        lite3_config.get("command_repeat_interval_sec", 0.05)
                    ),
                    should_stop=lambda: self._interrupt.cancel_requested,
                )
                self._lite3_status_subscriber = Ros2Lite3StatusSubscriber(
                    self,
                    self._chassis_backend.update_status,
                    str(self.get_parameter("lite3_status_topic").value),
                )
                self._twist_publisher = self._chassis_backend.twist_publisher
                controller_adapters["lite3"] = self._chassis_backend
                self.get_logger().info(
                    "Lite3 chassis backend enabled: "
                    f"cmd_vel={self.get_parameter('lite3_cmd_vel_topic').value}, "
                    f"simple_cmd={self.get_parameter('lite3_simple_cmd_topic').value}, "
                    f"status={self.get_parameter('lite3_status_topic').value}, "
                    f"mapped_actions={len(self._chassis_backend.mapped_actions)}, "
                    f"enabled_actions={len(self._chassis_backend.enabled_actions)}, "
                    f"navigation_settle="
                    f"{lite3_config.get('navigation_stable_samples', 5)} samples/"
                    f"{lite3_config.get('navigation_settle_timeout_sec', 2.0)}s"
                )
            else:
                enabled_parameter = {
                    "go2": "go2_enabled",
                    "lite3": "lite3_enabled",
                }[chassis_type]
                self.get_logger().info(
                    f"{chassis_type.upper()} chassis backend disabled; set "
                    f"{enabled_parameter}:=true"
                )

            self._controller_routes = self._config.get_controller_routes(
                chassis_type
            )

            # Always install the fail-closed stationary route. With a chassis
            # backend it publishes redundant stop commands; in simulation the
            # publisher is a no-op while cancel/timing semantics stay equal.
            self._stationary_expression_adapter = StationaryExpressionAdapter(
                publish_twist=self._twist_publisher,
                publish_rate_hz=chassis_publish_rate,
                stop_publish_count=chassis_stop_count,
                should_stop=lambda: self._interrupt.cancel_requested,
            )
            controller_adapters["stationary_expression"] = (
                self._stationary_expression_adapter
            )

            # ── Session-scoped UWB owner following ───────────────────
            uwb_config = self._config.uwb_follow_config
            self.declare_parameter(
                "uwb_follow_enabled", bool(uwb_config.get("enabled", True))
            )
            self.declare_parameter(
                "uwb_follow_setup_script",
                str(uwb_config.get("setup_script", "")),
            )
            self.declare_parameter(
                "uwb_follow_serial_device",
                str(uwb_config.get("serial_device", "")),
            )
            self.declare_parameter(
                "uwb_follow_cmd_vel_topic",
                str(uwb_config.get("cmd_vel_topic", "/cmd_vel")),
            )
            self.declare_parameter(
                "uwb_follow_target_fob_id",
                str(uwb_config.get("target_fob_id", "")),
            )
            # Lite3 follows through the go2_uwb_behavior Action interface;
            # Go2 uses its external UWB process pipeline.
            self._uwb_follow_adapter: (
                UwbFollowAdapter | UwbFollowActionAdapter | None
            ) = None
            if (
                bool(self.get_parameter("uwb_follow_enabled").value)
                and chassis_type in ("go2", "lite3")
                and self._twist_publisher is not None
            ):
                finish_motion = (
                    self._chassis_backend.finish_navigation
                    if chassis_type == "lite3"
                    and self._chassis_backend is not None
                    else None
                )
                if chassis_type == "lite3":
                    # Lite3 follows through the go2_uwb_behavior chain instead
                    # of spawning the pipeline itself: that package now owns
                    # /cmd_vel, and a second publisher would fight it.  See
                    # adapters/uwb_follow_action_adapter.py.
                    ros_interface = uwb_config.get("ros_interface") or {}
                    self._uwb_roam_adapter = UwbRoamAdapter(
                        node=self, backend=self._chassis_backend,
                        should_stop=lambda: self._interrupt.cancel_requested,
                        brake_ready=lambda: self._uwb_follow_adapter.brake_ready(),
                        brake_chain=lambda: self._uwb_follow_adapter.brake_chain(),
                        arm_chain=lambda: self._uwb_follow_adapter.arm_chain(),
                    )
                    controller_adapters["uwb_roam"] = self._uwb_roam_adapter
                    self._uwb_follow_adapter = UwbFollowActionAdapter(
                        enabled=True,
                        node=self,
                        follow_action=str(
                            ros_interface.get(
                                "follow_action", "/go2/follow_uwb"
                            )
                        ),
                        set_behavior_service=str(
                            ros_interface.get(
                                "set_behavior_service", "/go2/set_behavior"
                            )
                        ),
                        server_timeout_sec=float(
                            ros_interface.get("server_timeout_sec", 5.0)
                        ),
                        ready_timeout_sec=float(
                            ros_interface.get("ready_timeout_sec", 4.0)
                        ),
                        prepare_motion=(
                            self._chassis_backend.prepare_navigation
                            if self._chassis_backend is not None
                            else None
                        ),
                        finish_motion=finish_motion,
                        should_stop=lambda: self._interrupt.cancel_requested,
                        auto_resume=False,
                    )
                    self.get_logger().info(
                        "go2_uwb_behavior follow client ready: "
                        f"action="
                        f"{ros_interface.get('follow_action', '/go2/follow_uwb')}, "
                        f"service="
                        f"{ros_interface.get('set_behavior_service', '/go2/set_behavior')}"
                    )
                else:
                    self._uwb_follow_adapter = UwbFollowAdapter(
                        enabled=True,
                        setup_script=str(
                            self.get_parameter("uwb_follow_setup_script").value
                        ),
                        serial_device=str(
                            self.get_parameter("uwb_follow_serial_device").value
                        ),
                        cmd_vel_topic=str(
                            self.get_parameter("uwb_follow_cmd_vel_topic").value
                        ),
                        aoa_package=str(
                            uwb_config.get("aoa_package", "uwb_aoa_pkg")
                        ),
                        aoa_executable=str(
                            uwb_config.get(
                                "aoa_executable", "libAoa_robot_example"
                            )
                        ),
                        target_fob_id=str(
                            self.get_parameter("uwb_follow_target_fob_id").value
                        ),
                        follow_package=str(
                            uwb_config.get(
                                "follow_package", "go2_uwb_local_follow"
                            )
                        ),
                        follow_launch_file=str(
                            uwb_config.get(
                                "follow_launch_file", "local_follow.launch.py"
                            )
                        ),
                        startup_grace_sec=float(
                            uwb_config.get("startup_grace_sec", 1.0)
                        ),
                        shutdown_timeout_sec=float(
                            uwb_config.get("shutdown_timeout_sec", 3.0)
                        ),
                        target_timeout_sec=float(
                            uwb_config.get("target_timeout_sec", 3.0)
                        ),
                        stop_publish_count=int(
                            uwb_config.get("stop_publish_count", 3)
                        ),
                        publish_twist=self._twist_publisher,
                        prepare_motion=None,
                        finish_motion=finish_motion,
                        should_stop=lambda: self._interrupt.cancel_requested,
                        auto_resume=False,
                    )
                    self.get_logger().info(
                        "external UWB follow adapter ready: "
                        "serial="
                        f"{self.get_parameter('uwb_follow_serial_device').value}, "
                        "fob_id="
                        f"{self.get_parameter('uwb_follow_target_fob_id').value}, "
                        "cmd_vel="
                        f"{self.get_parameter('uwb_follow_cmd_vel_topic').value}"
                    )
                    from geometry_msgs.msg import PointStamped
                    self.create_subscription(
                        PointStamped,
                        str(uwb_config.get("target_topic", "/uwb/target_point")),
                        lambda msg: self._uwb_follow_adapter.update_target(
                            float(msg.point.x), float(msg.point.y)
                        ),
                        10,
                    )
                    from diagnostic_msgs.msg import DiagnosticArray
                    self.create_subscription(
                        DiagnosticArray,
                        str(uwb_config.get(
                            "diagnostics_topic",
                            "/go2_uwb_local_follow/follow_diagnostics",
                        )),
                        self._on_uwb_follow_diagnostics,
                        10,
                    )
                controller_adapters["uwb_follow"] = self._uwb_follow_adapter
            elif bool(self.get_parameter("uwb_follow_enabled").value):
                self.get_logger().info(
                    "UWB follow adapter inactive; enable the selected chassis backend"
                )

            # ── Dynamic wake-source orientation through Nav2 Spin ─────
            wake_config = self._config.wake_orientation_config
            self.declare_parameter(
                "wake_orientation_enabled",
                bool(wake_config.get("enabled", True)),
            )
            self.declare_parameter(
                "wake_spin_action_name",
                str(wake_config.get("action_name", "/spin")),
            )
            self.declare_parameter(
                "wake_spin_server_timeout_sec",
                float(wake_config.get("server_timeout_sec", 5.0)),
            )
            self.declare_parameter(
                "wake_spin_result_timeout_sec",
                float(wake_config.get("result_timeout_sec", 15.0)),
            )
            self.declare_parameter(
                "wake_spin_time_allowance_sec",
                float(wake_config.get("time_allowance_sec", 12.0)),
            )
            self.declare_parameter(
                "wake_angle_zero_offset_deg",
                float(wake_config.get("angle_zero_offset_deg", 90.0)),
            )
            self.declare_parameter(
                "wake_angle_direction_sign",
                float(wake_config.get("angle_direction_sign", -1.0)),
            )
            self.declare_parameter(
                "wake_angle_deadband_deg",
                float(wake_config.get("angle_deadband_deg", 5.0)),
            )
            self.declare_parameter(
                "wake_angle_frame_id",
                str(
                    wake_config.get(
                        "required_frame_id",
                        "microphone_array",
                    )
                ),
            )
            self.declare_parameter(
                "wake_linear_array_back_search_enabled",
                bool(
                    wake_config.get(
                        "linear_array_back_search_enabled", False
                    )
                ),
            )
            self.declare_parameter(
                "wake_visual_confirm_timeout_sec",
                float(wake_config.get("visual_confirm_timeout_sec", 1.5)),
            )
            self.declare_parameter(
                "wake_visual_min_confidence",
                float(wake_config.get("visual_min_confidence", 0.60)),
            )
            self.declare_parameter(
                "wake_visual_max_age_ms",
                float(wake_config.get("visual_max_age_ms", 800.0)),
            )

            self._spin_client: Ros2Nav2SpinClient | None = None
            self._wake_orientation_adapter: (
                WakeOrientationAdapter | None
            ) = None
            wake_enabled = bool(
                self.get_parameter("wake_orientation_enabled").value
            )
            if wake_enabled and self._chassis_backend is not None:
                self._spin_client = Ros2Nav2SpinClient(
                    self,
                    action_name=str(
                        self.get_parameter("wake_spin_action_name").value
                    ),
                    server_timeout_sec=float(
                        self.get_parameter(
                            "wake_spin_server_timeout_sec"
                        ).value
                    ),
                    should_stop=lambda: self._interrupt.cancel_requested,
                )
                self._wake_orientation_adapter = WakeOrientationAdapter(
                    spin_relative=self._spin_client,
                    angle_zero_offset_deg=float(
                        self.get_parameter(
                            "wake_angle_zero_offset_deg"
                        ).value
                    ),
                    angle_direction_sign=float(
                        self.get_parameter(
                            "wake_angle_direction_sign"
                        ).value
                    ),
                    angle_deadband_deg=float(
                        self.get_parameter(
                            "wake_angle_deadband_deg"
                        ).value
                    ),
                    required_frame_id=str(
                        self.get_parameter("wake_angle_frame_id").value
                    ),
                    result_timeout_sec=float(
                        self.get_parameter(
                            "wake_spin_result_timeout_sec"
                        ).value
                    ),
                    time_allowance_sec=float(
                        self.get_parameter(
                            "wake_spin_time_allowance_sec"
                        ).value
                    ),
                    linear_array_back_search_enabled=bool(
                        self.get_parameter(
                            "wake_linear_array_back_search_enabled"
                        ).value
                    ),
                    visual_confirm_timeout_sec=float(
                        self.get_parameter(
                            "wake_visual_confirm_timeout_sec"
                        ).value
                    ),
                    visual_min_confidence=float(
                        self.get_parameter(
                            "wake_visual_min_confidence"
                        ).value
                    ),
                    visual_max_age_ms=float(
                        self.get_parameter("wake_visual_max_age_ms").value
                    ),
                    should_stop=lambda: self._interrupt.cancel_requested,
                    # Same injection contract as navigation_adapter and the
                    # UWB follow adapter: Nav2 turns the dog over the ordinary
                    # /cmd_vel topic, and on Lite3 that only works once the
                    # backend hands control over (Vision Mode).  Without this
                    # the Spin goal streams at full speed while the dog, still
                    # on the joystick, drops every sample.  getattr keeps the
                    # hook optional for backends that drive Nav2 themselves.
                    prepare_motion=getattr(
                        self._chassis_backend,
                        "prepare_navigation",
                        None,
                    ),
                    finish_motion=getattr(
                        self._chassis_backend,
                        "finish_navigation",
                        None,
                    ),
                )
                controller_adapters[
                    "wake_orientation"
                ] = self._wake_orientation_adapter
                self.get_logger().info(
                    "Wake orientation adapter enabled: "
                    f"action={self.get_parameter('wake_spin_action_name').value}, "
                    f"frame={self.get_parameter('wake_angle_frame_id').value}, "
                    f"zero_offset="
                    f"{self.get_parameter('wake_angle_zero_offset_deg').value}deg, "
                    f"direction_sign="
                    f"{self.get_parameter('wake_angle_direction_sign').value}, "
                    f"deadband="
                    f"{self.get_parameter('wake_angle_deadband_deg').value}deg, "
                    f"linear_back_search="
                    f"{self.get_parameter('wake_linear_array_back_search_enabled').value}"
                )
            elif wake_enabled:
                self.get_logger().info(
                    "Wake orientation adapter inactive; "
                    "enable the selected chassis backend"
                )
            else:
                self.get_logger().info(
                    "Wake orientation adapter disabled"
                )

            # ── Optional Nav2 semantic-waypoint adapter ───────────────
            navigation_config = self._config.navigation_config
            self.declare_parameter(
                "navigation_enabled",
                bool(navigation_config.get("enabled", False)),
            )
            self.declare_parameter(
                "navigation_action_name",
                str(
                    navigation_config.get(
                        "action_name",
                        "/navigate_to_pose",
                    )
                ),
            )
            self.declare_parameter(
                "navigation_frame_id",
                str(navigation_config.get("frame_id", "map")),
            )
            self.declare_parameter(
                "navigation_server_timeout_sec",
                float(
                    navigation_config.get("server_timeout_sec", 10.0)
                ),
            )
            self.declare_parameter(
                "navigation_result_timeout_sec",
                float(
                    navigation_config.get("result_timeout_sec", 300.0)
                ),
            )
            waypoint_nav_config = navigation_config.get("waypoint_nav", {})
            self.declare_parameter(
                "waypoint_nav_service_name",
                str(waypoint_nav_config.get("service_name", "/waypoint_nav/task")),
            )
            self.declare_parameter(
                "waypoint_nav_status_topic",
                str(waypoint_nav_config.get("status_topic", "/waypoint_nav/status")),
            )
            self.declare_parameter(
                "waypoint_nav_protocol_version",
                str(waypoint_nav_config.get("protocol_version", "1.0")),
            )
            self.declare_parameter(
                "waypoint_nav_client_id",
                str(waypoint_nav_config.get("client_id", "marsdog_action_executor")),
            )
            self.declare_parameter(
                "waypoint_nav_service_timeout_sec",
                float(waypoint_nav_config.get("service_timeout_sec", 5.0)),
            )
            self.declare_parameter(
                "waypoint_nav_query_timeout_sec",
                float(waypoint_nav_config.get("query_timeout_sec", 2.0)),
            )
            self.declare_parameter(
                "waypoint_nav_cancel_confirmation_timeout_sec",
                float(
                    waypoint_nav_config.get(
                        "cancel_confirmation_timeout_sec", 5.0
                    )
                ),
            )
            self.declare_parameter(
                "waypoint_nav_terminal_retention_sec",
                float(waypoint_nav_config.get("terminal_retention_sec", 86400.0)),
            )
            self.declare_parameter(
                "navigation_preempt_lock_wait_sec",
                float(waypoint_nav_config.get("preempt_lock_wait_sec", 8.0)),
            )

            self._nav2_client: Ros2Nav2Client | None = None
            self._waypoint_nav_client: Ros2WaypointNavClient | None = None
            self._mobility_adapter: BehaviorMobilityAdapter | None = None
            if self.get_parameter("navigation_enabled").value:
                if self._chassis_backend is None:
                    raise RuntimeError(
                        "navigation_enabled requires the selected chassis "
                        "backend to be enabled"
                    )
                self._nav2_client = Ros2Nav2Client(
                    self,
                    action_name=str(
                        self.get_parameter(
                            "navigation_action_name"
                        ).value
                    ),
                    frame_id=str(
                        self.get_parameter("navigation_frame_id").value
                    ),
                    server_timeout_sec=float(
                        self.get_parameter(
                            "navigation_server_timeout_sec"
                        ).value
                    ),
                    should_stop=lambda: self._interrupt.cancel_requested,
                )
                self._waypoint_nav_client = Ros2WaypointNavClient(
                    self,
                    service_name=str(
                        self.get_parameter("waypoint_nav_service_name").value
                    ),
                    status_topic=str(
                        self.get_parameter("waypoint_nav_status_topic").value
                    ),
                    protocol_version=str(
                        self.get_parameter("waypoint_nav_protocol_version").value
                    ),
                    client_id=str(
                        self.get_parameter("waypoint_nav_client_id").value
                    ),
                    service_timeout_sec=float(
                        self.get_parameter("waypoint_nav_service_timeout_sec").value
                    ),
                    query_timeout_sec=float(
                        self.get_parameter("waypoint_nav_query_timeout_sec").value
                    ),
                    cancel_confirmation_timeout_sec=float(
                        self.get_parameter(
                            "waypoint_nav_cancel_confirmation_timeout_sec"
                        ).value
                    ),
                    terminal_retention_sec=float(
                        self.get_parameter(
                            "waypoint_nav_terminal_retention_sec"
                        ).value
                    ),
                    should_stop=lambda: self._interrupt.cancel_requested,
                )
                self._mobility_adapter = BehaviorMobilityAdapter(
                    navigate_waypoint=self._nav2_client,
                    navigate_fixed_place=self._waypoint_nav_client,
                    fixed_places=waypoint_nav_config["places"],
                    motion_adapter=self._chassis_backend,
                    waypoints=navigation_config["waypoints"],
                    behavior_routes=navigation_config["behavior_routes"],
                    stage_actions=navigation_config["stage_actions"],
                    result_timeout_sec=float(
                        self.get_parameter(
                            "navigation_result_timeout_sec"
                        ).value
                    ),
                    random_navigation_behaviors=navigation_config.get(
                        "random_navigation_behaviors", []
                    ),
                    random_navigation_place=navigation_config.get(
                        "random_navigation_place"
                    ),
                    random_navigation_result_timeout_sec=navigation_config.get(
                        "random_navigation_result_timeout_sec"
                    ),
                    random_navigation_in_place_probability=(
                        navigation_config.get(
                            "random_navigation_in_place_probability", {}
                        )
                    ),
                    random_navigation_fixed_pool=navigation_config.get(
                        "random_navigation_fixed_pool"
                    ),
                )
                controller_adapters[
                    "behavior_mobility"
                ] = self._mobility_adapter
                self.get_logger().info(
                    "Navigation adapter enabled: "
                    f"fixed_service="
                    f"{self.get_parameter('waypoint_nav_service_name').value}, "
                    f"random_place="
                    f"{self._mobility_adapter._random_navigation_place}, "
                    f"random_pool="
                    f"{','.join(self._mobility_adapter._random_navigation_fixed_pool) or '<server>'},"
                    f"frame={self.get_parameter('navigation_frame_id').value}, "
                    f"waypoints={len(navigation_config['waypoints'])}, "
                    f"routed_behaviors="
                    f"{len(self._mobility_adapter.routed_behaviors)}, "
                    f"random_nav_behaviors="
                    f"{len(self._mobility_adapter._random_navigation_behaviors)}, "
                    f"mapped_stage_actions="
                    f"{len(navigation_config['stage_actions'])}"
                )
            else:
                self.get_logger().info(
                    "Navigation adapter disabled; set "
                    "navigation_enabled:=true with a chassis backend enabled"
                )

            # ── Closed-loop approach to the selected voice caller ───────
            target_defaults = {
                "target_approach_enabled": True,
                "target_approach_publish_rate_hz": 10.0,
                "target_approach_min_confidence": 0.60,
                "target_approach_visual_timeout_sec": 0.8,
                "target_approach_acquire_timeout_sec": 3.0,
                "target_approach_lost_timeout_sec": 1.0,
                "target_approach_timeout_sec": 20.0,
                "target_approach_stop_distance_m": 1.20,
                "target_approach_minimum_safe_distance_m": 0.80,
                "target_approach_distance_deadband_m": 0.15,
                "target_approach_distance_hysteresis_m": 0.10,
                "target_approach_arrival_hold_sec": 0.5,
                "target_approach_linear_gain": 0.50,
                "target_approach_angular_gain": 0.80,
                "target_approach_max_linear_x": 0.15,
                "target_approach_max_angular_z": 0.30,
                "target_approach_max_linear_accel": 0.20,
                "target_approach_max_angular_accel": 0.30,
                "target_approach_heading_deadband": 0.06,
                "target_approach_max_heading_error": 0.25,
                "target_approach_allow_bbox_distance_fallback": False,
                "target_approach_demo_target_height": 0.68,
                "target_approach_demo_height_deadband": 0.05,
                "target_approach_bbox_height_hysteresis": 0.03,
                "target_approach_reacquire_min_consecutive_frames": 3,
            }
            for parameter_name, default_value in target_defaults.items():
                self.declare_parameter(parameter_name, default_value)

            self._target_approach_adapter: TargetApproachAdapter | None = None
            if (
                bool(self.get_parameter("target_approach_enabled").value)
                and self._twist_publisher is not None
            ):
                target_max_linear = min(
                    chassis_max_linear,
                    float(
                        self.get_parameter(
                            "target_approach_max_linear_x"
                        ).value
                    ),
                )
                target_max_angular = min(
                    chassis_max_angular,
                    float(
                        self.get_parameter(
                            "target_approach_max_angular_z"
                        ).value
                    ),
                )
                self._target_approach_adapter = TargetApproachAdapter(
                    publish_twist=self._twist_publisher,
                    publish_rate_hz=float(
                        self.get_parameter(
                            "target_approach_publish_rate_hz"
                        ).value
                    ),
                    min_confidence=float(
                        self.get_parameter(
                            "target_approach_min_confidence"
                        ).value
                    ),
                    visual_timeout_sec=float(
                        self.get_parameter(
                            "target_approach_visual_timeout_sec"
                        ).value
                    ),
                    acquire_timeout_sec=float(
                        self.get_parameter(
                            "target_approach_acquire_timeout_sec"
                        ).value
                    ),
                    lost_timeout_sec=float(
                        self.get_parameter(
                            "target_approach_lost_timeout_sec"
                        ).value
                    ),
                    approach_timeout_sec=float(
                        self.get_parameter(
                            "target_approach_timeout_sec"
                        ).value
                    ),
                    stop_distance_m=float(
                        self.get_parameter(
                            "target_approach_stop_distance_m"
                        ).value
                    ),
                    minimum_safe_distance_m=float(
                        self.get_parameter(
                            "target_approach_minimum_safe_distance_m"
                        ).value
                    ),
                    distance_deadband_m=float(
                        self.get_parameter(
                            "target_approach_distance_deadband_m"
                        ).value
                    ),
                    distance_hysteresis_m=float(
                        self.get_parameter(
                            "target_approach_distance_hysteresis_m"
                        ).value
                    ),
                    arrival_hold_sec=float(
                        self.get_parameter(
                            "target_approach_arrival_hold_sec"
                        ).value
                    ),
                    linear_gain=float(
                        self.get_parameter(
                            "target_approach_linear_gain"
                        ).value
                    ),
                    angular_gain=float(
                        self.get_parameter(
                            "target_approach_angular_gain"
                        ).value
                    ),
                    max_linear_x=target_max_linear,
                    max_angular_z=target_max_angular,
                    max_linear_accel=float(
                        self.get_parameter(
                            "target_approach_max_linear_accel"
                        ).value
                    ),
                    max_angular_accel=float(
                        self.get_parameter(
                            "target_approach_max_angular_accel"
                        ).value
                    ),
                    heading_deadband=float(
                        self.get_parameter(
                            "target_approach_heading_deadband"
                        ).value
                    ),
                    max_heading_error=float(
                        self.get_parameter(
                            "target_approach_max_heading_error"
                        ).value
                    ),
                    allow_bbox_distance_fallback=bool(
                        self.get_parameter(
                            "target_approach_allow_bbox_distance_fallback"
                        ).value
                    ),
                    demo_target_height=float(
                        self.get_parameter(
                            "target_approach_demo_target_height"
                        ).value
                    ),
                    demo_height_deadband=float(
                        self.get_parameter(
                            "target_approach_demo_height_deadband"
                        ).value
                    ),
                    bbox_height_hysteresis=float(
                        self.get_parameter(
                            "target_approach_bbox_height_hysteresis"
                        ).value
                    ),
                    reacquire_min_consecutive_frames=int(
                        self.get_parameter(
                            "target_approach_reacquire_min_consecutive_frames"
                        ).value
                    ),
                    stop_publish_count=chassis_stop_count,
                    should_stop=lambda: self._interrupt.cancel_requested,
                )
                controller_adapters[
                    "target_approach"
                ] = self._target_approach_adapter
                self.get_logger().info(
                    "Target approach adapter enabled: "
                    f"max_linear={target_max_linear:.3f}m/s, "
                    f"max_angular={target_max_angular:.3f}rad/s, "
                    "topic=/perception/visual_event, "
                    "target_key=vision_epoch+target_id, "
                    f"bbox_demo={self.get_parameter('target_approach_allow_bbox_distance_fallback').value}"
                )
            elif bool(self.get_parameter("target_approach_enabled").value):
                self.get_logger().info(
                    "Target approach adapter inactive; enable the selected "
                    "chassis backend"
                )

            # Voice caller approach uses one Vision/SLAM result and one Nav2
            # goal.  The old visual velocity adapter remains unrouted.
            self._person_nav_approach_adapter = None
            if self._twist_publisher is not None and VisionTask is not None:
                self._person_nav_approach_adapter = PersonNavApproachAdapter(
                    Ros2PersonApproachTransport(
                        self,
                        nav_action=str(self.get_parameter("navigation_action_name").value),
                    ),
                    self._twist_publisher,
                    should_stop=lambda: self._interrupt.cancel_requested,
                    chassis_backend=self._chassis_backend,
                )
                controller_adapters["person_nav_approach"] = (
                    self._person_nav_approach_adapter
                )
            else:
                self.get_logger().warning(
                    "Voice caller Nav2 approach unavailable: chassis or VisionTask missing"
                )

            # ── Generic selected visual-target approach ────────────────
            visual_target_config = self._config.visual_target_approach_config
            object_stream_config = visual_target_config.get(
                "object_detection_stream", {}
            )
            self._vision_object_callback_group = ReentrantCallbackGroup()
            self._vision_object_client = _VisionObjectDetectionLeaseClient(
                self,
                service_name=str(
                    object_stream_config.get(
                        "service_name", "/perception/vision/task"
                    )
                ),
                timeout_sec=float(
                    object_stream_config.get("service_timeout_sec", 1.0)
                ),
                confidence=float(
                    object_stream_config.get("confidence", 0.5)
                ),
                callback_group=self._vision_object_callback_group,
            )
            self._visual_target_approach_adapter: (
                VisualTargetApproachAdapter | None
            ) = None
            visual_behavior_policies = dict(
                visual_target_config["behavior_policies"]
            )
            if chassis_type == "go2":
                visual_behavior_policies.update(
                    visual_target_config.get(
                        "go2_owner_approach_policies", {}
                    )
                )
            elif chassis_type == "lite3":
                visual_behavior_policies.update(
                    visual_target_config.get(
                        "lite3_owner_approach_policies", {}
                    )
                )
            if (
                bool(visual_target_config.get("enabled", True))
                and self._twist_publisher is not None
            ):
                visual_target_max_linear = min(
                    chassis_max_linear,
                    float(visual_target_config["max_linear_x"]),
                )
                visual_target_max_angular = min(
                    chassis_max_angular,
                    float(visual_target_config["max_angular_z"]),
                )
                self._visual_target_approach_adapter = (
                    VisualTargetApproachAdapter(
                        publish_twist=self._twist_publisher,
                        behavior_policies=visual_behavior_policies,
                        publish_rate_hz=float(
                            visual_target_config["publish_rate_hz"]
                        ),
                        min_confidence=float(
                            visual_target_config["min_confidence"]
                        ),
                        visual_timeout_sec=float(
                            visual_target_config["visual_timeout_sec"]
                        ),
                        acquire_timeout_sec=float(
                            visual_target_config["acquire_timeout_sec"]
                        ),
                        lost_timeout_sec=float(
                            visual_target_config["lost_timeout_sec"]
                        ),
                        approach_timeout_sec=float(
                            visual_target_config["approach_timeout_sec"]
                        ),
                        minimum_stop_distance_m=float(
                            visual_target_config[
                                "minimum_stop_distance_m"
                            ]
                        ),
                        minimum_safe_distance_m=float(
                            visual_target_config[
                                "minimum_safe_distance_m"
                            ]
                        ),
                        distance_deadband_m=float(
                            visual_target_config["distance_deadband_m"]
                        ),
                        distance_hysteresis_m=float(
                            visual_target_config["distance_hysteresis_m"]
                        ),
                        arrival_hold_sec=float(
                            visual_target_config["arrival_hold_sec"]
                        ),
                        linear_gain=float(
                            visual_target_config["linear_gain"]
                        ),
                        angular_gain=float(
                            visual_target_config["angular_gain"]
                        ),
                        max_linear_x=visual_target_max_linear,
                        max_angular_z=visual_target_max_angular,
                        max_linear_accel=float(
                            visual_target_config["max_linear_accel"]
                        ),
                        max_angular_accel=float(
                            visual_target_config["max_angular_accel"]
                        ),
                        heading_deadband=float(
                            visual_target_config["heading_deadband"]
                        ),
                        max_heading_error=float(
                            visual_target_config["max_heading_error"]
                        ),
                        allow_bbox_distance_fallback=bool(
                            visual_target_config[
                                "allow_bbox_distance_fallback"
                            ]
                        ),
                        bbox_target_height=float(
                            visual_target_config["bbox_target_height"]
                        ),
                        bbox_height_deadband=float(
                            visual_target_config["bbox_height_deadband"]
                        ),
                        bbox_height_hysteresis=float(
                            visual_target_config["bbox_height_hysteresis"]
                        ),
                        reacquire_min_consecutive_frames=int(
                            visual_target_config[
                                "reacquire_min_consecutive_frames"
                            ]
                        ),
                        stop_publish_count=chassis_stop_count,
                        should_stop=lambda: self._interrupt.cancel_requested,
                        require_object_stream=bool(
                            object_stream_config.get("required", True)
                        ),
                        start_object_detection=(
                            self._vision_object_client.start
                            if self._vision_object_client.available
                            else None
                        ),
                        stop_object_detection=(
                            self._vision_object_client.stop
                            if self._vision_object_client.available
                            else None
                        ),
                        object_detection_rate_hz=float(
                            object_stream_config.get("rate_hz", 4.0)
                        ),
                        object_detection_lease_margin_sec=float(
                            object_stream_config.get(
                                "lease_margin_sec", 2.0
                            )
                        ),
                    )
                )
                controller_adapters["visual_target_approach"] = (
                    self._visual_target_approach_adapter
                )
                self.get_logger().info(
                    "Visual target approach enabled: "
                    f"behaviors={len(visual_behavior_policies)}, "
                    f"max_linear={visual_target_max_linear:.3f}m/s, "
                    "human_range_mode=bbox_height, "
                    "nonhuman_range_mode=metric, "
                    "object_stream=session_v2"
                )
            elif bool(visual_target_config.get("enabled", True)):
                self.get_logger().info(
                    "Visual target approach inactive; enable the selected "
                    "chassis backend"
                )

            self._stage_executor = StageExecutor(
                eligibility_checker=self._eligibility,
                posture_manager=self._posture,
                interrupt_manager=self._interrupt,
                action_catalog=self._config.action_catalog,
                controller_routes=self._controller_routes,
                controller_adapters=controller_adapters,
            )
            self._result_evaluator = ResultEvaluator()
            self._behavior_execution_lock = threading.Lock()
            self._current_priority: int = 99  # lower = more urgent
            self._goal_reservation_lock = threading.Lock()
            self._reserved_goal_id: str | None = None
            self._long_goal_id: str | None = None
            self._lease_lock = threading.Lock()
            self._lease_seen: dict[tuple[str, str], float] = {}
            self.declare_parameter("long_goal_lease_timeout_sec", 2.0)
            self.declare_parameter("long_goal_lease_grace_sec", 3.0)
            from std_msgs.msg import String
            self.create_subscription(
                String, "/behavior/goal_lease", self._on_goal_lease, 10,
            )

            # ── Background session attention tracking ────────────────
            self._behavior_sounds = BehaviorSoundController(
                self._config.sound_config,
                self._config.config_dir,
            )

            self.declare_parameter("attention_tracking_enabled", True)
            self.declare_parameter("attention_tracking_gain", 0.8)
            self.declare_parameter("attention_tracking_deadband", 0.08)
            self.declare_parameter("attention_tracking_activation_deadband", 0.14)
            self.declare_parameter("attention_tracking_smoothing_alpha", 0.15)
            self.declare_parameter("attention_tracking_max_angular_z", 0.25)
            self.declare_parameter("attention_tracking_max_angular_accel", 0.25)
            self.declare_parameter("attention_tracking_visual_timeout_sec", 0.8)
            self.declare_parameter("attention_tracking_direction_sign", -1.0)
            self.declare_parameter("follow_target_height", 0.68)
            self.declare_parameter("follow_height_deadband", 0.05)
            self.declare_parameter("follow_activation_deadband", 0.10)
            self.declare_parameter("follow_linear_gain", 1.0)
            self.declare_parameter("follow_max_linear_x", 0.25)
            self.declare_parameter("follow_max_linear_accel", 0.50)
            self.declare_parameter("follow_max_heading_error", 0.30)
            self._attention_controller: AttentionTrackingController | None = None
            if (
                bool(self.get_parameter("attention_tracking_enabled").value)
                and self._twist_publisher is not None
            ):
                self._attention_controller = AttentionTrackingController(
                    gain=float(self.get_parameter("attention_tracking_gain").value),
                    deadband=float(
                        self.get_parameter("attention_tracking_deadband").value
                    ),
                    activation_deadband=float(
                        self.get_parameter(
                            "attention_tracking_activation_deadband"
                        ).value
                    ),
                    smoothing_alpha=float(
                        self.get_parameter(
                            "attention_tracking_smoothing_alpha"
                        ).value
                    ),
                    max_angular_z=float(
                        self.get_parameter(
                            "attention_tracking_max_angular_z"
                        ).value
                    ),
                    max_angular_accel=float(
                        self.get_parameter(
                            "attention_tracking_max_angular_accel"
                        ).value
                    ),
                    visual_timeout_sec=float(
                        self.get_parameter(
                            "attention_tracking_visual_timeout_sec"
                        ).value
                    ),
                    wake_angle_zero_offset_deg=float(
                        self.get_parameter(
                            "wake_angle_zero_offset_deg"
                        ).value
                    ),
                    wake_angle_direction_sign=float(
                        self.get_parameter(
                            "wake_angle_direction_sign"
                        ).value
                    ),
                    wake_angle_deadband_deg=float(
                        self.get_parameter(
                            "wake_angle_deadband_deg"
                        ).value
                    ),
                    direction_sign=float(
                        self.get_parameter(
                            "attention_tracking_direction_sign"
                        ).value
                    ),
                    follow_target_height=float(
                        self.get_parameter("follow_target_height").value
                    ),
                    follow_height_deadband=float(
                        self.get_parameter("follow_height_deadband").value
                    ),
                    follow_activation_deadband=float(
                        self.get_parameter("follow_activation_deadband").value
                    ),
                    follow_linear_gain=float(
                        self.get_parameter("follow_linear_gain").value
                    ),
                    follow_max_linear_x=float(
                        self.get_parameter("follow_max_linear_x").value
                    ),
                    follow_max_linear_accel=float(
                        self.get_parameter("follow_max_linear_accel").value
                    ),
                    follow_max_heading_error=float(
                        self.get_parameter("follow_max_heading_error").value
                    ),
                )
                self.get_logger().info(
                    "Session attention tracking ready: "
                    "/behavior/attention_tracking + /perception/visual_event"
                )
            if (
                self._attention_controller is not None
                or self._uwb_follow_adapter is not None
                or self._target_approach_adapter is not None
                or self._person_nav_approach_adapter is not None
                or self._visual_target_approach_adapter is not None
                or self._wake_orientation_adapter is not None
            ):
                self._setup_attention_tracking_io()

            # ── Exact behavior-tree contract ─────────────────────────────
            self._acceptable_behaviors = self._config.get_behavior_names()
            self.get_logger().info(
                f"Accepted behavior-tree names: "
                f"{len(self._acceptable_behaviors)}"
            )

            # ── Debug publishers ────────────────────────────────────────
            self._debug = DebugPublishers(self, enable_legacy=False)

            # ── Action Server ───────────────────────────────────────────
            self._action_callback_group = ReentrantCallbackGroup()
            action_source = get_action_source()
            if _ExecuteBehavior is None:
                self.get_logger().error(
                    "ExecuteBehavior action message not found. "
                    "Ensure marsdog_interfaces is installed and built. "
                    "Action Server will NOT start."
                )
            else:
                self._action_server = ActionServer(
                    self,
                    _ExecuteBehavior,
                    "/execute_behavior",
                    execute_callback=self._on_execute,
                    goal_callback=self._on_goal,
                    cancel_callback=self._on_cancel,
                    handle_accepted_callback=self._on_accepted,
                    callback_group=self._action_callback_group,
                )
                self.get_logger().info(
                    f"Action Server /execute_behavior started "
                    f"(action from {action_source})."
                )

            self.get_logger().info(
                "ActionExecutorNode (v2) started — sole /execute_behavior server. "
                "Ensure no other marsdog_action_executor instance is running."
            )

        # ── Config path resolution ─────────────────────────────────────

        @staticmethod
        def _resolve_config_dir() -> Path:
            """Find the config directory: ROS2 share dir → local fallback."""
            try:
                from ament_index_python.packages import get_package_share_directory
                share = Path(get_package_share_directory("marsdog_action_executor"))
                cfg = share / "config"
                if cfg.exists():
                    return cfg
            except Exception:
                pass
            # Fallback: local source tree
            local = Path(__file__).resolve().parent.parent / "config"
            if local.exists():
                return local
            return Path("config")

        # ── Action Server callbacks ─────────────────────────────────────

        def _on_goal(self, goal_request) -> GoalResponse:
            """Accept only exact names from behavior_tree_actions.yaml.

            Priority-based preemption: a higher-priority goal interrupts
            the currently-running behavior (voice commands > emotions).
            """
            name = goal_request.behavior_name
            gid = goal_request.goal_id
            new_priority = int(getattr(goal_request, "priority_level", 0))

            params = parse_params_json_object(
                getattr(goal_request, "params_json", "{}")
            )
            if params is None:
                self.get_logger().warning(
                    f"Rejected goal: invalid_params goal_id={gid} -> {name}; "
                    "params_json must be a valid JSON object"
                )
                return GoalResponse.REJECT
            try:
                timeout_value = float(getattr(goal_request, "timeout_sec", 60.0))
            except (TypeError, ValueError):
                return GoalResponse.REJECT
            if not math.isfinite(timeout_value):
                return GoalResponse.REJECT

            if (name != "emergency_stop" and any(
                getattr(adapter, "recovery_required", False)
                for adapter in (
                    getattr(self, "_uwb_roam_adapter", None),
                    self._uwb_follow_adapter,
                    getattr(self, "_person_nav_approach_adapter", None),
                )
            )):
                self.get_logger().error("Rejected goal: controller terminal unknown; recovery required")
                return GoalResponse.REJECT

            if name in self._acceptable_behaviors:
                if name != "emergency_stop":
                    with self._goal_reservation_lock:
                        busy = self._reserved_goal_id is not None
                    if busy or self._behavior_execution_lock.locked():
                        # The previous Result, not CancelGoal acceptance, is
                        # the only point where motion ownership changes.
                        self.get_logger().warning(
                            f"Rejected goal while previous Result is pending: {gid} -> {name}"
                        )
                        return GoalResponse.REJECT
                    with self._goal_reservation_lock:
                        if self._reserved_goal_id is not None:
                            return GoalResponse.REJECT
                        self._reserved_goal_id = str(gid)
                if name == "emergency_stop":
                    self._interrupt.request_cancel()
                    self._behavior_sounds.stop()
                    if getattr(self, "_uwb_roam_adapter", None) is not None:
                        self._uwb_roam_adapter.emergency_stop()
                    if self._uwb_follow_adapter is not None:
                        self._uwb_follow_adapter.emergency_stop()
                    if self._wake_orientation_adapter is not None:
                        self._wake_orientation_adapter.emergency_stop()
                    if self._target_approach_adapter is not None:
                        self._target_approach_adapter.emergency_stop()
                    if self._person_nav_approach_adapter is not None:
                        self._person_nav_approach_adapter.emergency_stop()
                    if self._visual_target_approach_adapter is not None:
                        self._visual_target_approach_adapter.emergency_stop()
                    if self._mobility_adapter is not None:
                        self._mobility_adapter.emergency_stop()
                    if self._chassis_backend is not None:
                        self._chassis_backend.emergency_stop()
                    self._stationary_expression_adapter.emergency_stop()
                self.get_logger().info(
                    f"Accepted goal: {gid} -> {name} "
                    f"(priority={new_priority})"
                )
                return GoalResponse.ACCEPT

            from difflib import get_close_matches
            close = get_close_matches(
                name,
                sorted(self._acceptable_behaviors),
                n=3,
                cutoff=0.5,
            )
            hint = f" Did you mean: {close}?" if close else ""
            self.get_logger().warning(
                f"Rejected goal: unsupported_behavior={name!r} "
                f"goal_id={gid}. "
                f"Accepted: {len(self._acceptable_behaviors)} strict "
                f"behavior-tree names.{hint}"
            )
            return GoalResponse.REJECT

        def _on_goal_lease(self, message) -> None:
            try:
                payload = json.loads(message.data)
            except (TypeError, ValueError):
                return
            if not isinstance(payload, dict) or payload.get("schema_version") != 1:
                return
            gid = payload.get("goal_id")
            bid = payload.get("behavior_id")
            if (not isinstance(gid, str) or not gid
                    or not isinstance(bid, str) or not bid):
                return
            with self._goal_reservation_lock:
                if gid != self._reserved_goal_id:
                    return
            with self._lease_lock:
                self._lease_seen[(gid, bid)] = time.monotonic()

        def _on_uwb_follow_diagnostics(self, message) -> None:
            adapter = self._uwb_follow_adapter
            if not isinstance(adapter, UwbFollowAdapter):
                return
            for status in message.status:
                if "UWB follow controller" in status.name:
                    adapter.update_controller_state(status.message)

        def _lease_expired(self, gid: str, bid: str, started: float) -> bool:
            with self._lease_lock:
                renewed = self._lease_seen.get((gid, bid))
            grace = float(self.get_parameter("long_goal_lease_grace_sec").value)
            timeout = float(self.get_parameter("long_goal_lease_timeout_sec").value)
            return (time.monotonic() - (renewed if renewed is not None else started)
                    > (timeout if renewed is not None else grace))

        def _setup_attention_tracking_io(self) -> None:
            from std_msgs.msg import String
            from rclpy.qos import (
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )

            reliable = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            visual_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
            self.create_subscription(
                String,
                "/behavior/attention_tracking",
                self._on_attention_control,
                reliable,
            )
            self.create_subscription(
                String,
                "/perception/visual_event",
                self._on_visual_event,
                visual_qos,
            )
            object_stream_config = (
                self._config.visual_target_approach_config.get(
                    "object_detection_stream", {}
                )
            )
            self.create_subscription(
                String,
                str(
                    object_stream_config.get(
                        "topic", "/perception/vision/object_detections"
                    )
                ),
                self._on_object_detection,
                visual_qos,
                callback_group=self._vision_object_callback_group,
            )
            self.create_timer(0.1, self._tick_attention_tracking)

        def _on_attention_control(self, message) -> None:
            try:
                payload = json.loads(message.data)
            except (json.JSONDecodeError, TypeError):
                self.get_logger().warning("Invalid attention control JSON")
                return
            if not isinstance(payload, dict):
                self.get_logger().warning("Invalid attention control JSON object")
                return
            if self._attention_controller is not None:
                self._attention_controller.update_control(payload)
            # /behavior/attention_tracking belongs to the voice session.
            # UWB follow is owned only by its ExecuteBehavior Goal.
            if (
                (
                    self._attention_controller is None
                    or not self._attention_controller.enabled
                )
                and self._twist_publisher is not None
            ):
                if self._behavior_execution_lock.acquire(blocking=False):
                    try:
                        self._twist_publisher(TwistCommand())
                    finally:
                        self._behavior_execution_lock.release()

        def _on_visual_event(self, message) -> None:
            if not _dispatch_visual_event(
                message.data,
                attention_controller=self._attention_controller,
                target_approach_adapter=self._target_approach_adapter,
                visual_target_approach_adapter=(
                    self._visual_target_approach_adapter
                ),
                wake_orientation_adapter=self._wake_orientation_adapter,
                person_nav_approach_adapter=self._person_nav_approach_adapter,
            ):
                self.get_logger().warning(
                    "Invalid visual event JSON; target approach stopped"
                )

        def _on_object_detection(self, message) -> None:
            if not _dispatch_object_detection(
                message.data,
                visual_target_approach_adapter=(
                    self._visual_target_approach_adapter
                ),
            ):
                # Foreign sessions and idle packets are expected and are not
                # warnings. Malformed packets are still safely ignored because
                # they cannot update the active movement observation.
                return

        # Retain the private hook for older local tests/integration harnesses.
        def _on_attention_visual(self, message) -> None:
            self._on_visual_event(message)

        def _tick_attention_tracking(self) -> None:
            if (
                self._attention_controller is None
                or self._twist_publisher is None
            ):
                return
            if getattr(getattr(self, "_uwb_roam_adapter", None), "active", False):
                return
            # Hold the same ownership mutex used by foreground behaviors across
            # both command calculation and publication.  This closes the
            # check-then-publish race where an old attention command could land
            # after a target-approach behavior had already acquired /cmd_vel.
            if not self._behavior_execution_lock.acquire(blocking=False):
                return
            try:
                command = self._attention_controller.command(suspended=False)
                if command is not None:
                    self._twist_publisher(command)
            finally:
                self._behavior_execution_lock.release()

        def _on_cancel(self, goal_handle) -> CancelResponse:
            """Accept all cancel requests."""
            self.get_logger().info(
                f"Cancel requested: {goal_handle.request.goal_id}"
            )
            if (self._long_goal_id is not None
                    and str(goal_handle.request.goal_id) != self._long_goal_id):
                return CancelResponse.REJECT
            self._interrupt.request_cancel()
            self._behavior_sounds.stop()
            if (self._long_goal_id is not None
                    and goal_handle.request.behavior_name == "play_alone"
                    and getattr(self, "_uwb_roam_adapter", None) is not None):
                self._uwb_roam_adapter.cancel_step()
            if (
                self._uwb_follow_adapter is not None
                and goal_handle.request.behavior_name == "follow_owner"
            ):
                if self._long_goal_id is not None:
                    self._uwb_follow_adapter.request_cancel()
                else:
                    self._uwb_follow_adapter.cancel_step()
            if self._wake_orientation_adapter is not None:
                self._wake_orientation_adapter.cancel_step()
            if self._target_approach_adapter is not None:
                self._target_approach_adapter.cancel_task()
            if self._person_nav_approach_adapter is not None:
                self._person_nav_approach_adapter.cancel_task()
            if self._visual_target_approach_adapter is not None:
                self._visual_target_approach_adapter.cancel_task()
            if self._mobility_adapter is not None:
                self._mobility_adapter.cancel_step()
            if (self._chassis_backend is not None
                    and not (self._long_goal_id is not None
                             and goal_handle.request.behavior_name in
                             ("play_alone", "follow_owner"))):
                self._chassis_backend.cancel_step()
            elif (self._chassis_backend is not None
                  and self._long_goal_id is not None
                  and goal_handle.request.behavior_name == "play_alone"
                  and not getattr(self._uwb_roam_adapter, "active", False)):
                self._chassis_backend.cancel_step()
            self._stationary_expression_adapter.cancel_step()
            return CancelResponse.ACCEPT

        def _on_accepted(self, goal_handle) -> None:
            """Kick off async execution when goal is accepted."""
            goal_handle.execute()

            g = goal_handle.request
            goal, params_valid = _debug_goal_from_request(g)
            if not params_valid:
                # Defensive path for direct callback harnesses or a future
                # transport regression.  Normal ROS flow rejects this in
                # _on_goal before _on_accepted can be invoked.
                self.get_logger().error(
                    f"Accepted callback received invalid params_json: "
                    f"goal_id={goal.goal_id}; recorded params as an empty object"
                )
            self._debug.publish_goal(goal)

        async def _on_execute(self, goal_handle):
            """Execute behavior; emergency stop bypasses serialization.

            When preempting a lower-priority goal, waits for the configured
            lock budget (8s by default) for
            the running goal to release the lock after receiving the
            cancel signal.
            """
            request = goal_handle.request
            if request.behavior_name == "emergency_stop":
                return self._execute_emergency_stop(goal_handle)

            if not self._behavior_execution_lock.acquire(blocking=False):
                # Preemption path: wait for old goal to finish cancelling
                lock_wait_sec = float(
                    self.get_parameter(
                        "navigation_preempt_lock_wait_sec"
                    ).value
                )
                deadline = time.monotonic() + lock_wait_sec
                acquired = False
                while time.monotonic() < deadline:
                    if self._behavior_execution_lock.acquire(blocking=False):
                        acquired = True
                        break
                    time.sleep(0.05)
                if not acquired:
                    goal_handle.abort()
                    return _make_result(
                        request.goal_id,
                        getattr(request, "behavior_id", request.goal_id),
                        request.behavior_name,
                        status="FAILED",
                        result="failed",
                        reason=(
                            "executor_busy: previous task has no confirmed "
                            f"terminal within {lock_wait_sec:.1f}s"
                        ),
                        reward=-1.0,
                    )
            try:
                self._current_priority = int(
                    getattr(request, "priority_level", 0)
                )
                return await self._execute_behavior(goal_handle)
            finally:
                self._behavior_sounds.stop()
                if request.behavior_name in INPLACE_WITH_HUMAN_BEHAVIORS:
                    self._stationary_expression_adapter.cancel_step()
                    if self._chassis_backend is not None:
                        self._chassis_backend.hold_position()
                self._behavior_execution_lock.release()
                with self._goal_reservation_lock:
                    if self._reserved_goal_id == str(request.goal_id):
                        self._reserved_goal_id = None
                with self._lease_lock:
                    self._lease_seen.pop((
                        str(request.goal_id),
                        str(getattr(request, "behavior_id", "") or request.goal_id),
                    ), None)

        async def _execute_behavior(self, goal_handle):
            """Async execution callback — v2 pipeline."""
            goal_req = goal_handle.request
            gid = goal_req.goal_id
            requested_name = goal_req.behavior_name
            behavior_id = getattr(goal_req, "behavior_id", gid)
            runtime_timeout_sec = _normalise_goal_timeout(
                getattr(goal_req, "timeout_sec", 60.0)
            )
            behavior_started_wall = time.time()
            behavior_started_monotonic = time.monotonic()

            self.get_logger().info(
                f"[{gid}] Executing: {requested_name}"
            )

            # ── Step 1: Parse ──────────────────────────────────────────
            ctx = self._goal_parser.parse_from_ros_goal(goal_req)
            ctx.runtime_deadline_monotonic = _outer_runtime_deadline(
                behavior_started_monotonic, runtime_timeout_sec,
            )
            if not ctx.is_valid:
                self.get_logger().error(
                    f"[{gid}] Invalid params: {ctx.error_reason}"
                )
                goal_handle.abort()
                return _make_result(
                    gid, behavior_id, requested_name,
                    status="FAILED", result="failed",
                    reason=ctx.error_reason or "invalid_params",
                    reward=-1.0,
                )

            # ── Step 2: Validate the exact behavior-tree name ───────────
            ctx = self._behavior_resolver.resolve(ctx)
            if not ctx.is_valid:
                self.get_logger().error(
                    f"[{gid}] Unresolvable behavior: {ctx.error_reason}"
                )
                goal_handle.abort()
                return _make_result(
                    gid, behavior_id, requested_name,
                    resolved_name=ctx.resolved_behavior_name,
                    status="FAILED", result="failed",
                    reason=ctx.error_reason or "unsupported_behavior",
                    reward=-1.0,
                )

            canonical = ctx.resolved_behavior_name
            self.get_logger().info(f"[{gid}] Validated: {canonical}")
            ctx.runtime_feedback = lambda progress, action, message, safe: (
                self._publish_feedback(
                    goal_handle,
                    gid,
                    behavior_id,
                    canonical,
                    progress,
                    str(ctx.current_stage or "target_approach"),
                    action,
                    safe,
                    message,
                )
            )

            # ── Step 3: Get template & stages ──────────────────────────
            template = self._config.get_behavior_template(canonical)
            if template is None:
                self.get_logger().error(
                    f"[{gid}] No template for: {canonical}"
                )
                goal_handle.abort()
                return _make_result(
                    gid, behavior_id, requested_name,
                    resolved_name=canonical,
                    status="FAILED", result="failed",
                    reason=f"no_template: {canonical}",
                    reward=-1.0,
                )

            stages = template.get("stages", [])
            if not stages:
                self.get_logger().error(
                    f"[{gid}] Behavior {canonical} has no stages"
                )
                goal_handle.abort()
                return _make_result(
                    gid, behavior_id, requested_name,
                    resolved_name=canonical,
                    status="FAILED", result="failed",
                    reason=f"no_stages: {canonical}",
                    reward=-1.0,
                )

            if canonical in ("follow_owner", "play_alone") and runtime_timeout_sec <= 0:
                return self._execute_long_behavior(
                    goal_handle, ctx, stages, behavior_started_wall,
                )

            success_condition = template.get("success_condition")
            # Dict format → extract type field (e.g. {type: action_completed, ...})
            if isinstance(success_condition, dict):
                success_condition = success_condition.get("type")

            self.get_logger().info(
                f"[{gid}] Plan: {canonical} -> {len(stages)} stages"
            )

            inplace_check = validate_inplace_plan(
                canonical,
                ctx.params,
                stages,
                self._controller_routes,
                self._config.navigation_config,
            )
            if not inplace_check.valid:
                reason = inplace_check.reason
                self._stationary_expression_adapter.cancel_step()
                if self._chassis_backend is not None:
                    self._chassis_backend.hold_position()
                self.get_logger().error(
                    f"[{gid}] In-place plan rejected: {reason}"
                )
                goal_handle.abort()
                self._debug.publish_result(ExecutionResult(
                    goal_id=gid,
                    behavior_id=behavior_id,
                    behavior_name=canonical,
                    status="FAILED",
                    result="failed",
                    reason=reason,
                    reward=-1.0,
                ))
                return _make_result(
                    gid, behavior_id, requested_name,
                    resolved_name=canonical,
                    status="FAILED", result="failed",
                    reason=reason, reward=-1.0,
                )

            # ── Step 4: Navigate semantic waypoint, when routed ─────────
            stage_results: dict[str, bool] = {}

            # Reset interrupt state
            self._interrupt.reset()
            self._posture.reset()
            if goal_handle.is_cancel_requested:
                # Do not let a cancel accepted while this Goal was waiting for
                # the execution lock be erased by the per-goal reset above.
                self._interrupt.request_cancel()

            if (
                self._mobility_adapter is not None
                and self._mobility_adapter.waypoint_for_behavior(canonical)
                is not None
            ):
                waypoint_name = (
                    self._mobility_adapter.waypoint_for_behavior(canonical)
                )
                # Random behaviors request waypoint_nav's reserved live-map
                # target and use their own shorter timeout; semantic routes
                # keep the full fixed-waypoint budget.
                is_random_nav = (
                    canonical
                    in self._mobility_adapter._random_navigation_behaviors
                )
                if is_random_nav:
                    nav_timeout = float(
                        self._config.navigation_config.get(
                            "random_navigation_result_timeout_sec"
                        )
                        or self._config.navigation_config.get(
                            "result_timeout_sec", 300.0
                        )
                    )
                else:
                    nav_timeout = float(
                        self._config.navigation_config.get(
                            "result_timeout_sec", 300.0
                        )
                    )
                remaining = ctx.remaining_runtime_sec(time.monotonic())
                if remaining is not None:
                    nav_timeout = min(nav_timeout, remaining)
                self.get_logger().info(
                    f"[{gid}] Navigation: {canonical} -> "
                    f"waypoint {waypoint_name} "
                    f"(timeout={nav_timeout:.0f}s)"
                )
                waypoint_task_id = f"action:{gid}:waypoint"

                def _waypoint_feedback(status: WaypointNavStatus) -> None:
                    progress_by_code = {
                        "QUEUED": 0.02,
                        "DISPATCHED": 0.05,
                        "NAV2_ACCEPTED": 0.10,
                        "RECOVERY_REQUIRED": 0.05,
                        "NAV2_SUCCEEDED": 0.20,
                    }
                    self._publish_feedback(
                        goal_handle,
                        gid,
                        behavior_id,
                        canonical,
                        progress_by_code.get(status.code, 0.10),
                        "waypoint_navigation",
                        f"WAYPOINT_NAV/{status.code}",
                        (
                            status.safe_to_interrupt
                            if not status.terminal
                            else False
                        ),
                        (
                            f"waypoint task={status.task_id} "
                            f"state={status.state} code={status.code}: "
                            f"{status.message}"
                        ),
                    )

                navigation_ok = (
                    nav_timeout > 0.0
                    and not goal_handle.is_cancel_requested
                    and self._mobility_adapter.navigate_for_behavior(
                        canonical,
                        timeout_sec=nav_timeout,
                        task_id=waypoint_task_id,
                        status_callback=_waypoint_feedback,
                    )
                )
                if not navigation_ok:
                    cancel_requested = (
                        goal_handle.is_cancel_requested
                        or self._interrupt.cancel_requested
                    )
                    waypoint_outcome = (
                        self._waypoint_nav_client.last_outcome
                        if (
                            self._waypoint_nav_client is not None
                            and self._waypoint_nav_client.last_outcome.task_id
                            == waypoint_task_id
                        )
                        else None
                    )
                    mobility_error = str(
                        getattr(
                            self._mobility_adapter,
                            "last_error",
                            "",
                        )
                        or ""
                    ).strip()
                    canceled = _navigation_cancel_won(
                        cancel_requested,
                        waypoint_outcome,
                    )
                    timed_out = (
                        not canceled
                        and (
                            ctx.remaining_runtime_sec(time.monotonic()) == 0.0
                            or (
                                waypoint_outcome is not None
                                and waypoint_outcome.code == "TIMEOUT"
                            )
                        )
                    )
                    status = (
                        "CANCELED" if canceled
                        else "TIMEOUT" if timed_out
                        else "FAILED"
                    )
                    result_text = "canceled" if canceled else "failed"
                    if timed_out:
                        reason = "behavior_goal_timeout"
                    elif mobility_error:
                        reason = mobility_error
                    elif waypoint_outcome is not None:
                        reason = (
                            f"waypoint_navigation_{result_text}: "
                            f"task_id={waypoint_outcome.task_id}, "
                            f"state={waypoint_outcome.state}, "
                            f"code={waypoint_outcome.code}, "
                            f"terminal_confirmed="
                            f"{waypoint_outcome.terminal_confirmed}, "
                            f"message={waypoint_outcome.message}"
                        )
                    else:
                        reason = (
                            f"navigation_{result_text}: "
                            f"waypoint={waypoint_name}"
                        )
                    self.get_logger().error(f"[{gid}] {reason}")
                    if canceled:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    self._debug.publish_result(
                        ExecutionResult(
                            goal_id=gid,
                            behavior_id=behavior_id,
                            behavior_name=canonical,
                            status=status,
                            result=result_text,
                            reason=reason,
                            duration_sec=(
                                time.time() - behavior_started_wall
                            ),
                            reward=-1.0,
                        )
                    )
                    return _make_result(
                        gid,
                        behavior_id,
                        requested_name,
                        resolved_name=canonical,
                        status=status,
                        result=result_text,
                        reason=reason,
                        reward=-1.0,
                    )

            # ── Step 5: Execute existing behavior-tree stages ───────────
            # For target-bound behaviors, sound is part of the interaction and
            # must not start before target_approach succeeds. Behaviors without
            # an approach Stage still start audio with their first Stage.
            sound_started = self._behavior_sounds.apply_control_behavior(
                canonical
            )

            for i, stage_cfg in enumerate(stages):
                stage_id = (
                    stage_cfg.get("stage_id")
                    or stage_cfg.get("stage_name")
                    or f"stage_{i}"
                )

                # ── Cancel check ────────────────────────────────────
                if goal_handle.is_cancel_requested:
                    ctx.cancel_requested = True
                    self._interrupt.request_cancel()
                    self.get_logger().info(f"[{gid}] Cancel — stopping at stage {stage_id}")
                    break

                # ── Timeout check ────────────────────────────────────
                remaining = ctx.remaining_runtime_sec(time.monotonic())
                if remaining is not None and remaining <= 0.0:
                    elapsed = time.time() - behavior_started_wall
                    self.get_logger().warning(
                        f"[{gid}] Timeout after {elapsed:.1f}s "
                        f"(limit: {runtime_timeout_sec:.1f}s)"
                    )
                    ctx.metadata["unit_failure_state"] = "timeout"
                    ctx.metadata["unit_failure_reason"] = (
                        "behavior_goal_timeout"
                    )
                    stage_results[stage_id] = False
                    break

                # ── Execute stage ────────────────────────────────────
                self.get_logger().info(
                    f"[{gid}] Stage {i+1}/{len(stages)}: {stage_id}"
                )
                if not sound_started and stage_id != "target_approach":
                    sound_path = self._behavior_sounds.play_for(canonical)
                    sound_started = True
                    if sound_path is not None:
                        self.get_logger().info(
                            f"[{gid}] Behavior sound started: {canonical} -> "
                            f"{sound_path.name}"
                        )
                self._interrupt.apply_to_context(ctx)

                # Stage-scoped audio. Started after the context refresh so a
                # cancel that landed earlier this iteration starts nothing new,
                # and scoped to this call so the failure-policy break below
                # cannot leak the stream into the rest of the behavior.
                # target_approach needs no guard here: no stage mapping ever
                # targets it, unlike the behavior-level start above.
                with self._behavior_sounds.stage_sound(
                    canonical, stage_id
                ) as stage_sound_path:
                    if stage_sound_path is not None:
                        self.get_logger().info(
                            f"[{gid}] Stage sound started: {stage_id} -> "
                            f"{stage_sound_path.name}"
                        )
                    result = self._stage_executor.execute_stage(stage_cfg, ctx)

                stage_results[stage_id] = result.success
                self._interrupt.apply_to_context(ctx)

                # ── Publish feedback ─────────────────────────────────
                progress = round((i + 1) / len(stages), 4)
                self._publish_feedback(
                    goal_handle, gid, behavior_id, canonical,
                    progress, stage_id, result.unit_id,
                    self._interrupt.safe_to_interrupt,
                    f"Stage {i+1}/{len(stages)}: {result.unit_id} — "
                    f"{'OK' if result.success else result.message}",
                )

                # ── Handle stage failure ─────────────────────────────
                if not result.success:
                    failure_policy = stage_cfg.get("failure_policy", "abort")
                    required = stage_cfg.get("required", True)
                    self.get_logger().warning(
                        f"[{gid}] Stage {stage_id} failed: {result.message} "
                        f"(policy={failure_policy}, required={required})"
                    )
                    if required and failure_policy == "abort":
                        break
                    # skip_stage → continue to next stage

            # ── Step 6: Evaluate result ────────────────────────────────
            behavior_result = self._result_evaluator.evaluate(
                ctx, stage_results, success_condition,
            )

            duration = time.time() - behavior_started_wall
            self.get_logger().info(
                f"[{gid}] Result: {behavior_result.status} "
                f"(success={behavior_result.success}, "
                f"stages={behavior_result.completed_stages}, "
                f"units={behavior_result.executed_units}, "
                f"duration={duration:.1f}s)"
            )

            if behavior_result.success:
                goal_handle.succeed()
            elif ctx.cancel_requested and behavior_result.status != "controller_error":
                goal_handle.canceled()
            else:
                goal_handle.abort()

            result_text = (
                "completed"
                if behavior_result.success
                else "canceled"
                if behavior_result.status == "canceled"
                else "failed"
            )

            # ── Metadata for behaviors that report internal state ──────
            metadata: dict[str, object] = {}
            target_approach_metadata = ctx.metadata.get("target_approach")
            if isinstance(target_approach_metadata, dict):
                metadata["target_approach"] = dict(
                    target_approach_metadata
                )
            visual_target_metadata = ctx.metadata.get(
                "visual_target_approach"
            )
            if isinstance(visual_target_metadata, dict):
                metadata["visual_target_approach"] = dict(
                    visual_target_metadata
                )
            lite3_action_metadata = ctx.metadata.get("lite3_action")
            if isinstance(lite3_action_metadata, dict):
                metadata["lite3_action"] = dict(lite3_action_metadata)
            lite3_action_history = ctx.metadata.get("lite3_actions")
            if isinstance(lite3_action_history, list):
                metadata["lite3_actions"] = [
                    dict(item)
                    for item in lite3_action_history
                    if isinstance(item, dict)
                ]
            if (
                behavior_result.success
                and canonical in ("restInPlace", "recharge")
            ):
                battery_level = self._read_battery_level()
                metadata["energyValue"] = battery_level
                if canonical == "recharge" and isinstance(
                    lite3_action_metadata, dict
                ):
                    metadata["charging_completed"] = (
                        lite3_action_metadata.get("semantic_effect")
                        != "simulated"
                    )
                self.get_logger().info(
                    f"[{gid}] metadata: energyValue={battery_level}"
                )
            metadata_json = json.dumps(metadata) if metadata else "{}"

            exec_result = ExecutionResult(
                goal_id=gid,
                behavior_id=behavior_id,
                behavior_name=canonical,
                status=behavior_result.status.upper(),
                result=result_text,
                reason=(
                    str(ctx.metadata.get("unit_failure_reason"))
                    if not behavior_result.success
                    and ctx.metadata.get("unit_failure_reason")
                    else behavior_result.message or behavior_result.reason
                ),
                duration_sec=duration,
                reward=behavior_result.reward,
                emotion_delta_json="{}",
                need_delta_json="{}",
                metadata_json=metadata_json,
            )
            self._debug.publish_result(exec_result)

            return _make_result(
                gid, behavior_id, requested_name,
                resolved_name=canonical,
                status=behavior_result.status.upper(),
                result=result_text,
                reason=(
                    str(ctx.metadata.get("unit_failure_reason"))
                    if not behavior_result.success
                    and ctx.metadata.get("unit_failure_reason")
                    else behavior_result.message or behavior_result.reason
                ),
                reward=behavior_result.reward,
                metadata=metadata,
            )

        def _execute_long_behavior(
            self, goal_handle, ctx: ExecutionContext, stages: list[dict],
            started_wall: float,
        ):
            """Run one leased Goal until cancellation or a real failure."""
            gid = str(goal_handle.request.goal_id)
            behavior_id = str(getattr(goal_handle.request, "behavior_id", "") or gid)
            name = ctx.resolved_behavior_name
            started = time.monotonic()
            cycle = 0
            cycles_completed = 0
            state = {"stage": "starting", "action": "", "safe": True}
            lease_lost = threading.Event()
            ticker_done = threading.Event()
            self._long_goal_id = gid
            self._interrupt.reset()
            self._posture.reset()
            if goal_handle.is_cancel_requested:
                self._interrupt.request_cancel()

            def canceled() -> bool:
                return bool(goal_handle.is_cancel_requested or self._interrupt.cancel_requested)

            def tick() -> None:
                while not ticker_done.wait(0.5):
                    if self._lease_expired(gid, behavior_id, started):
                        lease_lost.set()
                        if name == "play_alone":
                            roam_adapter = getattr(self, "_uwb_roam_adapter", None)
                            if roam_adapter is not None:
                                roam_adapter.cancel_step()
                            self._stationary_expression_adapter.cancel_step()
                            if (self._chassis_backend is not None
                                    and not getattr(roam_adapter, "active", False)):
                                self._chassis_backend.cancel_step()
                        elif self._uwb_follow_adapter is not None:
                            self._uwb_follow_adapter.request_cancel()
                    self._publish_feedback(
                        goal_handle, gid, behavior_id, name, 0.0,
                        state["stage"], state["action"],
                        bool(state["safe"] and not canceled() and not lease_lost.is_set()),
                        f"cycle={cycle} stage={state['stage']} "
                        f"lease={'expired' if lease_lost.is_set() else 'active'}",
                    )

            thread = threading.Thread(target=tick, name=f"goal-feedback-{gid}", daemon=True)
            thread.start()
            status = "FAILED"
            reason = "long_goal_failed"
            try:
                if name == "follow_owner":
                    adapter = self._uwb_follow_adapter
                    if adapter is None:
                        reason = "uwb_follow_unavailable"
                    elif canceled():
                        status, reason = "CANCELED", "canceled_before_start"
                    else:
                        if isinstance(adapter, UwbFollowActionAdapter):
                            adapter._long_goal_active = True
                        state.update(stage="following", action="ACT_INTERACT_FOLLOW_OWNER")
                        if isinstance(adapter, UwbFollowActionAdapter):
                            started_ok = adapter.start(timeout_sec=0.0)
                        else:
                            started_ok = adapter.start(wait_for_grace=True)
                            if started_ok:
                                adapter.enable_target_monitor()
                        if not started_ok:
                            reason = adapter.last_error or "uwb_follow_start_failed"
                            if canceled() and not adapter.recovery_required:
                                status, reason = "CANCELED", "canceled_during_startup"
                        else:
                            while True:
                                if canceled():
                                    status, reason = "CANCELED", "canceled"
                                    break
                                if lease_lost.is_set():
                                    reason = "upstream_goal_lease_expired"
                                    break
                                if not adapter.poll_health() or not adapter.active:
                                    reason = adapter.last_error or "uwb_follow_ended"
                                    break
                                time.sleep(0.1)
                        state.update(stage="stopping", safe=False)
                        if not adapter.stop_confirmed(reason):
                            status = "FAILED"
                            reason = adapter.last_error or "uwb_follow_terminal_unknown"
                else:
                    roam = getattr(self, "_uwb_roam_adapter", None)
                    if canceled():
                        status, reason = "CANCELED", "canceled_before_start"
                    elif roam is None:
                        reason = "uwb_roam_unavailable"
                    else:
                        while True:
                            if canceled():
                                status, reason = "CANCELED", "canceled"
                                break
                            if lease_lost.is_set():
                                reason = "upstream_goal_lease_expired"
                                break
                            cycle += 1
                            ctx.completed_stages.clear()
                            ctx.metadata.pop("uwb_roam_terminal_status", None)
                            for stage in stages:
                                stage_id = str(stage.get("stage_id") or "stage")
                                state.update(stage=stage_id, action="", safe=True)
                                if canceled() or lease_lost.is_set():
                                    break
                                result = self._stage_executor.execute_stage(stage, ctx)
                                state["action"] = result.unit_id
                                if not result.success:
                                    reason = str(ctx.metadata.get("unit_failure_reason") or result.message)
                                    break
                                self._publish_feedback(
                                    goal_handle, gid, behavior_id, name, 0.0,
                                    stage_id, result.unit_id, True,
                                    f"cycle={cycle} stage={stage_id} complete",
                                )
                            else:
                                cycles_completed += 1
                                state.update(stage="cycle_complete", action="", safe=True)
                                continue
                            if canceled():
                                if ctx.metadata.get("uwb_roam_terminal_status") in (4, 6):
                                    status = "FAILED"
                                else:
                                    status, reason = "CANCELED", "canceled"
                            elif lease_lost.is_set():
                                reason = "upstream_goal_lease_expired"
                            break
                    state.update(stage="stopping", safe=False)
                    terminal_unknown = bool(roam is not None and roam.active)
                    if roam is not None and roam.active:
                        # The adapter owns the inner goal until its real Result.
                        # A transport failure latches recovery_required and
                        # prevents a replacement Goal from taking /cmd_vel.
                        roam.cancel_step()
                        reason = "uwb_roam_terminal_unknown:operator_recovery_required"
                        status = "FAILED"
                    if self._chassis_backend is not None:
                        self._chassis_backend.emergency_stop()
                        if (not terminal_unknown
                                and self._chassis_backend.finish_navigation() is False):
                            status = "FAILED"
                            reason = (self._chassis_backend.last_error
                                      or "play_alone_stop_unconfirmed")
                            if roam is not None:
                                roam.recovery_required = True
            except Exception as exc:
                reason = f"long_goal_exception:{type(exc).__name__}:{exc}"
                self.get_logger().error(reason)
                if name == "follow_owner" and self._uwb_follow_adapter is not None:
                    self._uwb_follow_adapter.stop_confirmed(reason)
                if name == "play_alone" and self._chassis_backend is not None:
                    self._chassis_backend.emergency_stop()
            finally:
                ticker_done.set()
                thread.join(timeout=1.0)
                if name == "follow_owner" and isinstance(
                    self._uwb_follow_adapter, UwbFollowActionAdapter
                ):
                    self._uwb_follow_adapter._long_goal_active = False
                self._long_goal_id = None

            if status == "CANCELED":
                goal_handle.canceled()
            else:
                goal_handle.abort()
            result = _make_result(
                gid, behavior_id, name, status=status,
                result="canceled" if status == "CANCELED" else "failed",
                reason=reason, reward=-1.0,
                metadata={"cycles_completed": cycles_completed},
            )
            self._debug.publish_result(ExecutionResult(
                goal_id=gid, behavior_id=behavior_id, behavior_name=name,
                status=status,
                result="canceled" if status == "CANCELED" else "failed",
                reason=reason, duration_sec=time.time() - started_wall,
                reward=-1.0,
            ))
            return result

        def _execute_emergency_stop(self, goal_handle):
            """Publish zero velocity immediately without waiting for another goal."""
            self._behavior_sounds.stop()
            roam = getattr(self, "_uwb_roam_adapter", None)
            if roam is not None:
                roam.emergency_stop()
            self._stationary_expression_adapter.emergency_stop()
            goal_req = goal_handle.request
            gid = goal_req.goal_id
            behavior_id = getattr(goal_req, "behavior_id", gid)
            reason = "selected chassis backend disabled; no motion stop outlet"
            if self._uwb_follow_adapter is not None:
                self._uwb_follow_adapter.emergency_stop()
                reason = "UWB follow stopped and chassis stop published"
            if self._wake_orientation_adapter is not None:
                self._wake_orientation_adapter.emergency_stop()
                reason = "Nav2 Spin canceled"
            if self._target_approach_adapter is not None:
                self._target_approach_adapter.emergency_stop()
                reason = "target approach canceled and chassis stop published"
            if self._person_nav_approach_adapter is not None:
                self._person_nav_approach_adapter.emergency_stop()
                reason = "Nav2 caller approach cancel requested and chassis stop published"
            if self._visual_target_approach_adapter is not None:
                self._visual_target_approach_adapter.emergency_stop()
                reason = (
                    "visual target approach canceled and chassis stop "
                    "published"
                )
            if self._mobility_adapter is not None:
                self._mobility_adapter.emergency_stop()
                reason = "Nav2 canceled and chassis stop published"
            if self._chassis_backend is not None:
                self._chassis_backend.emergency_stop()
                if self._mobility_adapter is None:
                    reason = "chassis stop published"

            exec_result = ExecutionResult(
                goal_id=gid,
                behavior_id=behavior_id,
                behavior_name="emergency_stop",
                status="SUCCESS",
                result="completed",
                reason=reason,
                reward=1.0,
            )
            self._debug.publish_result(exec_result)
            goal_handle.succeed()
            return _make_result(
                gid,
                behavior_id,
                "emergency_stop",
                status="SUCCESS",
                result="completed",
                reason=reason,
                reward=1.0,
            )

        # ── Helpers ────────────────────────────────────────────────────

        def _read_battery_level(self) -> int:
            """Return the configured/BMS-backed actual battery percentage.

            Until a BMS subscriber is connected, the explicit launch
            parameter implements the agreed contract default of 100%.
            """
            value = float(
                self.get_parameter("recharge_result_energy_value").value
            )
            return int(min(100.0, max(0.0, value)))

        def _publish_feedback(
            self, goal_handle, gid: str, behavior_id: str,
            behavior_name: str, progress: float, stage: str,
            action: str, safe: bool, message: str,
        ) -> None:
            """Publish feedback via Action + debug topic."""
            if _ExecuteBehavior:
                fb = _ExecuteBehavior.Feedback()
                fb.goal_id = gid
                fb.behavior_id = behavior_id
                fb.behavior_name = behavior_name
                fb.status = "RUNNING"
                fb.progress = progress
                fb.safe_to_interrupt = safe
                fb.current_action = action
                fb.message = message
                goal_handle.publish_feedback(fb)

            debug_fb = ExecutionFeedback(
                goal_id=gid,
                behavior_id=behavior_id,
                behavior_name=behavior_name,
                status="RUNNING",
                progress=progress,
                current_stage=stage,
                current_action=action,
                safe_to_interrupt=safe,
                message=message,
            )
            self._debug.publish_feedback(debug_fb)


# ═══════════════════════════════════════════════════════════════════════════════
# Result helper
# ═══════════════════════════════════════════════════════════════════════════════

def _make_result(
    goal_id: str,
    behavior_id: str,
    behavior_name: str,
    resolved_name: str = "",
    status: str = "SUCCESS",
    result: str = "completed",
    reason: str = "",
    reward: float = 1.0,
    metadata: dict | None = None,
) -> object:
    """Build an ExecuteBehavior.Result with explicit field assignment.

    Uses attribute assignment rather than constructor kwargs to avoid
    silent field drops that can occur with ROS2 action message types.
    """
    if _ExecuteBehavior is None:
        return ExecutionResult(
            goal_id=goal_id,
            behavior_id=behavior_id,
            behavior_name=str(resolved_name or behavior_name),
            status=str(status),
            result=str(result),
            reason=str(reason),
            reward=float(reward),
            emotion_delta_json="{}",
            need_delta_json="{}",
            metadata_json=json.dumps(metadata) if metadata else "{}",
        )
    r = _ExecuteBehavior.Result()
    r.goal_id = goal_id
    r.behavior_id = behavior_id
    r.behavior_name = str(resolved_name or behavior_name)
    r.status = str(status)
    r.result = str(result)
    r.reason = str(reason)
    r.reward = float(reward)
    r.emotion_delta_json = "{}"
    r.need_delta_json = "{}"
    r.metadata_json = json.dumps(metadata) if metadata else "{}"
    return r


# ═══════════════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════════════


def main(args: list[str] | None = None) -> None:
    """Entry point for the ROS2 node.

    Before running, source ROS2 and the colcon workspace::

        source /opt/ros/humble/setup.bash
        source ~/ros2_ws/install/setup.bash
        ros2 run marsdog_action_executor action_executor_node
    """
    if not HAS_ROS2:
        print(
            "ROS2 (rclpy) is not available. "
            "Make sure you've sourced ROS2 setup.bash, "
            "then run again with:\n"
            "  ros2 run marsdog_action_executor action_executor_node\n"
            "\n"
            "For a no-ROS2 demo, use:\n"
            "  uv run python -m marsdog_action_executor.standalone_demo"
        )
        sys.exit(1)

    rclpy.init(args=args)
    node = ActionExecutorNode()
    # Long Goal workers and synchronous service waits need spare ROS callback
    # threads for inner Action results, service replies and lease renewals.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._behavior_sounds.stop()
        # Let the Action worker drain an accepted inner cancellation while
        # ROS callbacks can still deliver its real Result. A hard process kill
        # skips this path; the external supervisor must stop the chassis then.
        node._interrupt.request_cancel()
        roam = getattr(node, "_uwb_roam_adapter", None)
        if roam is not None:
            roam.cancel_step()
        if node._uwb_follow_adapter is not None:
            node._uwb_follow_adapter.request_cancel()
        deadline = time.monotonic() + 7.0
        while (rclpy.ok() and node._behavior_execution_lock.locked()
               and time.monotonic() < deadline):
            executor.spin_once(timeout_sec=0.1)
        if roam is not None:
            roam.close()
        if node._uwb_follow_adapter is not None and rclpy.ok():
            node._uwb_follow_adapter.emergency_stop()
        if rclpy.ok():
            node._stationary_expression_adapter.emergency_stop()
        if node._mobility_adapter is not None and rclpy.ok():
            node._mobility_adapter.emergency_stop()
        if node._wake_orientation_adapter is not None and rclpy.ok():
            node._wake_orientation_adapter.emergency_stop()
        if node._target_approach_adapter is not None and rclpy.ok():
            node._target_approach_adapter.emergency_stop()
        if node._person_nav_approach_adapter is not None and rclpy.ok():
            node._person_nav_approach_adapter.emergency_stop()
        if node._visual_target_approach_adapter is not None and rclpy.ok():
            node._visual_target_approach_adapter.emergency_stop()
        if node._chassis_backend is not None and rclpy.ok():
            node._chassis_backend.emergency_stop()
        executor.shutdown()
        node.destroy_node()
        _shutdown_rclpy_if_ok(rclpy, RCLError)


if __name__ == "__main__":
    main()
