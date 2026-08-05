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
import os
import sys
import threading
import time
from pathlib import Path

from .adapters.agv_adapter import (
    AgvMotionAdapter,
    Ros2TwistPublisher,
    TwistCommand,
)
from .adapters.attention_tracking_controller import (
    AttentionTrackingController,
)
from .adapters.navigation_adapter import (
    BehaviorMobilityAdapter,
    Ros2Nav2Client,
)
from .adapters.wake_orientation_adapter import (
    Ros2Nav2SpinClient,
    WakeOrientationAdapter,
)
from .behavior_resolver import BehaviorResolver
from .config_loader import ConfigLoader
from .debug_publishers import DebugPublishers
from .eligibility_checker import EligibilityChecker
from .execution_context import ExecutionContext
from .goal_parser import GoalParser
from .interrupt_manager import InterruptManager
from .models import BehaviorGoal, ExecutionFeedback, ExecutionResult
from .posture_manager import PostureManager
from .result_evaluator import ResultEvaluator
from .ros2_compat import HAS_ROS2, get_execute_behavior_action, get_action_source
from .stage_executor import StageExecutor, StageResult

logger = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════════════════════
# ROS2-dependent class
# ═══════════════════════════════════════════════════════════════════════════════

if HAS_ROS2:
    import rclpy
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.utilities import get_rmw_implementation_identifier

    _ExecuteBehavior = get_execute_behavior_action()

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

            # ── Optional AGV /cmd_vel adapter ──────────────────────────
            agv_config = self._config.agv_motion_config
            limits = agv_config.get("limits", {})
            self.declare_parameter(
                "agv_enabled",
                bool(agv_config.get("enabled", False)),
            )
            self.declare_parameter(
                "agv_cmd_vel_topic",
                str(agv_config.get("cmd_vel_topic", "/cmd_vel")),
            )
            self.declare_parameter(
                "agv_publish_rate_hz",
                float(agv_config.get("publish_rate_hz", 10.0)),
            )
            self.declare_parameter(
                "agv_max_linear_x",
                float(limits.get("max_linear_x", 0.25)),
            )
            self.declare_parameter(
                "agv_max_linear_y",
                float(limits.get("max_linear_y", 0.0)),
            )
            self.declare_parameter(
                "agv_max_angular_z",
                float(limits.get("max_angular_z", 1.2)),
            )
            self.declare_parameter("recharge_result_energy_value", 100.0)

            self._agv_adapter: AgvMotionAdapter | None = None
            self._twist_publisher: Ros2TwistPublisher | None = None
            controller_adapters = {}
            if self.get_parameter("agv_enabled").value:
                cmd_vel_topic = str(
                    self.get_parameter("agv_cmd_vel_topic").value
                )
                twist_publisher = Ros2TwistPublisher(self, cmd_vel_topic)
                self._twist_publisher = twist_publisher
                self._agv_adapter = AgvMotionAdapter(
                    publish_twist=twist_publisher,
                    motion_groups=agv_config["motion_groups"],
                    action_motion_groups=agv_config["action_motion_groups"],
                    publish_rate_hz=float(
                        self.get_parameter("agv_publish_rate_hz").value
                    ),
                    max_linear_x=float(
                        self.get_parameter("agv_max_linear_x").value
                    ),
                    max_linear_y=float(
                        self.get_parameter("agv_max_linear_y").value
                    ),
                    max_angular_z=float(
                        self.get_parameter("agv_max_angular_z").value
                    ),
                    stop_publish_count=int(
                        agv_config.get("stop_publish_count", 3)
                    ),
                    should_stop=lambda: self._interrupt.cancel_requested,
                )
                controller_adapters["agv"] = self._agv_adapter
                self.get_logger().info(
                    f"AGV adapter enabled: topic={cmd_vel_topic}, "
                    f"rate={self.get_parameter('agv_publish_rate_hz').value}Hz, "
                    f"mapped_actions={len(self._agv_adapter.mapped_actions)}"
                )
            else:
                self.get_logger().info(
                    "AGV adapter disabled; set agv_enabled:=true to publish /cmd_vel"
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
                float(wake_config.get("angle_zero_offset_deg", 0.0)),
            )
            self.declare_parameter(
                "wake_angle_direction_sign",
                float(wake_config.get("angle_direction_sign", 1.0)),
            )
            self.declare_parameter(
                "wake_angle_deadband_deg",
                float(wake_config.get("angle_deadband_deg", 5.0)),
            )
            self.declare_parameter(
                "wake_angle_frame_id",
                str(wake_config.get("required_frame_id", "base_link")),
            )

            self._spin_client: Ros2Nav2SpinClient | None = None
            self._wake_orientation_adapter: (
                WakeOrientationAdapter | None
            ) = None
            wake_enabled = bool(
                self.get_parameter("wake_orientation_enabled").value
            )
            if wake_enabled and self._agv_adapter is not None:
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
                    f"{self.get_parameter('wake_angle_deadband_deg').value}deg"
                )
            elif wake_enabled:
                self.get_logger().info(
                    "Wake orientation adapter inactive; "
                    "set agv_enabled:=true"
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

            self._nav2_client: Ros2Nav2Client | None = None
            self._mobility_adapter: BehaviorMobilityAdapter | None = None
            if self.get_parameter("navigation_enabled").value:
                if self._agv_adapter is None:
                    raise RuntimeError(
                        "navigation_enabled requires agv_enabled=true "
                        "because routed behavior stages use /cmd_vel"
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
                self._mobility_adapter = BehaviorMobilityAdapter(
                    navigate_waypoint=self._nav2_client,
                    motion_adapter=self._agv_adapter,
                    waypoints=navigation_config["waypoints"],
                    behavior_routes=navigation_config["behavior_routes"],
                    action_motion_groups=navigation_config[
                        "action_motion_groups"
                    ],
                    result_timeout_sec=float(
                        self.get_parameter(
                            "navigation_result_timeout_sec"
                        ).value
                    ),
                )
                controller_adapters[
                    "behavior_mobility"
                ] = self._mobility_adapter
                self.get_logger().info(
                    "Navigation adapter enabled: "
                    f"action={self.get_parameter('navigation_action_name').value}, "
                    f"frame={self.get_parameter('navigation_frame_id').value}, "
                    f"waypoints={len(navigation_config['waypoints'])}, "
                    f"routed_behaviors="
                    f"{len(self._mobility_adapter.routed_behaviors)}, "
                    f"mapped_stage_actions="
                    f"{len(navigation_config['action_motion_groups'])}"
                )
            else:
                self.get_logger().info(
                    "Navigation adapter disabled; set "
                    "navigation_enabled:=true with agv_enabled:=true"
                )

            self._stage_executor = StageExecutor(
                eligibility_checker=self._eligibility,
                posture_manager=self._posture,
                interrupt_manager=self._interrupt,
                action_catalog=self._config.action_catalog,
                controller_routes=self._config.get_controller_routes(),
                controller_adapters=controller_adapters,
            )
            self._result_evaluator = ResultEvaluator()
            self._behavior_execution_lock = threading.Lock()
            self._current_priority: int = 99  # lower = more urgent

            # ── Background session attention tracking ────────────────
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
            self.declare_parameter("follow_linear_gain", 0.55)
            self.declare_parameter("follow_max_linear_x", 0.22)
            self.declare_parameter("follow_max_linear_accel", 0.20)
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
                self._setup_attention_tracking_io()
                self.get_logger().info(
                    "Session attention tracking ready: "
                    "/behavior/attention_tracking + /perception/visual_event"
                )

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

            if name in self._acceptable_behaviors:
                if name != "emergency_stop" and self._behavior_execution_lock.locked():
                    if new_priority < self._current_priority:
                        self.get_logger().info(
                            f"Preempting priority={self._current_priority} "
                            f"goal with priority={new_priority} -> {name}"
                        )
                        self._interrupt.request_cancel()
                        if self._wake_orientation_adapter is not None:
                            self._wake_orientation_adapter.cancel_step()
                        if self._mobility_adapter is not None:
                            self._mobility_adapter.cancel_step()
                        if self._agv_adapter is not None:
                            self._agv_adapter.cancel_step()
                        # fall through to ACCEPT — _on_execute will wait
                        # for the old goal's lock to be released
                    else:
                        self.get_logger().warning(
                            f"Rejected goal while higher-priority "
                            f"({self._current_priority}) is running: "
                            f"{gid} -> {name} (new={new_priority})"
                        )
                        return GoalResponse.REJECT
                if name == "emergency_stop":
                    self._interrupt.request_cancel()
                    if self._wake_orientation_adapter is not None:
                        self._wake_orientation_adapter.emergency_stop()
                    if self._mobility_adapter is not None:
                        self._mobility_adapter.emergency_stop()
                    if self._agv_adapter is not None:
                        self._agv_adapter.emergency_stop()
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
                self._on_attention_visual,
                visual_qos,
            )
            self.create_timer(0.1, self._tick_attention_tracking)

        def _on_attention_control(self, message) -> None:
            if self._attention_controller is None:
                return
            try:
                payload = json.loads(message.data)
            except (json.JSONDecodeError, TypeError):
                self.get_logger().warning("Invalid attention control JSON")
                return
            self._attention_controller.update_control(payload)
            if (
                not self._attention_controller.enabled
                and not self._behavior_execution_lock.locked()
                and self._twist_publisher is not None
            ):
                self._twist_publisher(TwistCommand())

        def _on_attention_visual(self, message) -> None:
            if self._attention_controller is None:
                return
            try:
                payload = json.loads(message.data)
            except (json.JSONDecodeError, TypeError):
                return
            self._attention_controller.update_visual(payload)

        def _tick_attention_tracking(self) -> None:
            if (
                self._attention_controller is None
                or self._twist_publisher is None
            ):
                return
            command = self._attention_controller.command(
                suspended=self._behavior_execution_lock.locked(),
            )
            if command is not None:
                self._twist_publisher(command)

        def _on_cancel(self, goal_handle) -> CancelResponse:
            """Accept all cancel requests."""
            self.get_logger().info(
                f"Cancel requested: {goal_handle.request.goal_id}"
            )
            self._interrupt.request_cancel()
            if self._wake_orientation_adapter is not None:
                self._wake_orientation_adapter.cancel_step()
            if self._mobility_adapter is not None:
                self._mobility_adapter.cancel_step()
            if self._agv_adapter is not None:
                self._agv_adapter.cancel_step()
            return CancelResponse.ACCEPT

        def _on_accepted(self, goal_handle) -> None:
            """Kick off async execution when goal is accepted."""
            goal_handle.execute()

            g = goal_handle.request
            goal = BehaviorGoal(
                goal_id=g.goal_id,
                behavior_id=getattr(g, "behavior_id", g.goal_id),
                behavior_name=g.behavior_name,
                priority_level=getattr(g, "priority_level", 0),
                params=json.loads(getattr(g, "params_json", "{}") or "{}"),
                timeout_sec=getattr(g, "timeout_sec", 60.0),
                timestamp=time.time(),
            )
            self._debug.publish_goal(goal)

        async def _on_execute(self, goal_handle):
            """Execute behavior; emergency stop bypasses serialization.

            When preempting a lower-priority goal, waits up to 3s for
            the running goal to release the lock after receiving the
            cancel signal.
            """
            request = goal_handle.request
            if request.behavior_name == "emergency_stop":
                return self._execute_emergency_stop(goal_handle)

            if not self._behavior_execution_lock.acquire(blocking=False):
                # Preemption path: wait for old goal to finish cancelling
                deadline = time.time() + 3.0
                acquired = False
                while time.time() < deadline:
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
                        reason="executor_busy",
                        reward=-1.0,
                    )
            try:
                self._current_priority = int(
                    getattr(request, "priority_level", 0)
                )
                return await self._execute_behavior(goal_handle)
            finally:
                self._behavior_execution_lock.release()

        async def _execute_behavior(self, goal_handle):
            """Async execution callback — v2 pipeline."""
            goal_req = goal_handle.request
            gid = goal_req.goal_id
            requested_name = goal_req.behavior_name
            behavior_id = getattr(goal_req, "behavior_id", gid)
            timeout_sec = getattr(goal_req, "timeout_sec", 60.0)

            self.get_logger().info(
                f"[{gid}] Executing: {requested_name}"
            )

            # ── Step 1: Parse ──────────────────────────────────────────
            ctx = self._goal_parser.parse_from_ros_goal(goal_req)
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

            success_condition = template.get("success_condition")
            # Dict format → extract type field (e.g. {type: action_completed, ...})
            if isinstance(success_condition, dict):
                success_condition = success_condition.get("type")

            self.get_logger().info(
                f"[{gid}] Plan: {canonical} -> {len(stages)} stages"
            )

            # ── Step 4: Navigate semantic waypoint, when routed ─────────
            stage_results: dict[str, bool] = {}

            # Reset interrupt state
            self._interrupt.reset()
            self._posture.reset()

            if (
                self._mobility_adapter is not None
                and self._mobility_adapter.waypoint_for_behavior(canonical)
                is not None
            ):
                waypoint_name = (
                    self._mobility_adapter.waypoint_for_behavior(canonical)
                )
                # Use Nav2 result timeout rather than behavior timeout;
                # navigation can take much longer than a single action.
                nav_timeout = float(
                    self._config.navigation_config.get(
                        "result_timeout_sec", 300.0
                    )
                )
                self.get_logger().info(
                    f"[{gid}] Navigation: {canonical} -> "
                    f"waypoint {waypoint_name} "
                    f"(timeout={nav_timeout:.0f}s)"
                )
                nav_start = time.time()
                navigation_ok = (
                    self._mobility_adapter.navigate_for_behavior(
                        canonical,
                        timeout_sec=nav_timeout,
                    )
                )
                if not navigation_ok:
                    canceled = (
                        goal_handle.is_cancel_requested
                        or self._interrupt.cancel_requested
                    )
                    status = "CANCELED" if canceled else "FAILED"
                    result_text = "canceled" if canceled else "failed"
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
                            duration_sec=time.time() - nav_start,
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
            start_time = time.time()  # stage timeout starts after navigation
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
                elapsed = time.time() - start_time
                if elapsed > timeout_sec:
                    self.get_logger().warning(
                        f"[{gid}] Timeout after {elapsed:.1f}s "
                        f"(limit: {timeout_sec:.1f}s)"
                    )
                    break

                # ── Execute stage ────────────────────────────────────
                self.get_logger().info(
                    f"[{gid}] Stage {i+1}/{len(stages)}: {stage_id}"
                )
                self._interrupt.apply_to_context(ctx)

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

            duration = time.time() - start_time
            self.get_logger().info(
                f"[{gid}] Result: {behavior_result.status} "
                f"(success={behavior_result.success}, "
                f"stages={behavior_result.completed_stages}, "
                f"units={behavior_result.executed_units}, "
                f"duration={duration:.1f}s)"
            )

            if behavior_result.success:
                goal_handle.succeed()
            elif ctx.cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()

            # ── Metadata for behaviors that report internal state ──────
            metadata: dict[str, object] = {}
            if (
                behavior_result.success
                and canonical in ("restInPlace", "recharge")
            ):
                battery_level = self._read_battery_level()
                metadata["energyValue"] = battery_level
                self.get_logger().info(
                    f"[{gid}] metadata: energyValue={battery_level}"
                )
            metadata_json = json.dumps(metadata) if metadata else "{}"

            exec_result = ExecutionResult(
                goal_id=gid,
                behavior_id=behavior_id,
                behavior_name=canonical,
                status=behavior_result.status.upper(),
                result="completed" if behavior_result.success else "failed",
                reason=behavior_result.message or behavior_result.reason,
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
                result="completed" if behavior_result.success else "failed",
                reason=behavior_result.message or behavior_result.reason,
                reward=behavior_result.reward,
                metadata=metadata,
            )

        def _execute_emergency_stop(self, goal_handle):
            """Publish zero velocity immediately without waiting for another goal."""
            goal_req = goal_handle.request
            gid = goal_req.goal_id
            behavior_id = getattr(goal_req, "behavior_id", gid)
            reason = "AGV adapter disabled; no cmd_vel publisher"
            if self._wake_orientation_adapter is not None:
                self._wake_orientation_adapter.emergency_stop()
                reason = "Nav2 Spin canceled"
            if self._mobility_adapter is not None:
                self._mobility_adapter.emergency_stop()
                reason = "Nav2 canceled and AGV zero velocity published"
            if self._agv_adapter is not None:
                self._agv_adapter.emergency_stop()
                if self._mobility_adapter is None:
                    reason = "AGV zero velocity published"

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
        return None
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
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node._mobility_adapter is not None and rclpy.ok():
            node._mobility_adapter.emergency_stop()
        if node._wake_orientation_adapter is not None and rclpy.ok():
            node._wake_orientation_adapter.emergency_stop()
        if node._agv_adapter is not None and rclpy.ok():
            node._agv_adapter.emergency_stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
