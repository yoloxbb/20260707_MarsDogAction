"""ROS2 Action Server node for the MarsDog action executor.

Provides the ``/execute_behavior`` Action Server and optional debug topics.

Architecture (v2)::

    ROS2 Action goal
        │
        ▼
    GoalParser         ← params_json → ExecutionContext
        │
        ▼
    BehaviorResolver   ← alias / inject / fallback
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
import sys
import time
from pathlib import Path

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
    from rclpy.node import Node

    _ExecuteBehavior = get_execute_behavior_action()

    class ActionExecutorNode(Node):
        """ROS2 node wrapping the v2 execution pipeline.

        Action Server: ``/execute_behavior``
        Debug Topics:  ``/debug/execute_behavior/goal|feedback|result``
        """

        def __init__(self) -> None:
            super().__init__("action_executor_node")

            # ── Load configuration ──────────────────────────────────────
            config_dir = self._resolve_config_dir()
            self.get_logger().info(f"Loading configs from: {config_dir}")
            self._config = ConfigLoader(config_dir)
            self._config.load_all()
            self.get_logger().info(
                f"Config loaded: {len(self._config.behavior_templates)} behaviors, "
                f"{len(self._config.action_catalog)} actions"
            )

            # ── Build v2 pipeline ───────────────────────────────────────
            self._goal_parser = GoalParser()
            self._behavior_resolver = BehaviorResolver(
                aliases=self._config.behavior_aliases,
            )
            self._eligibility = EligibilityChecker(
                action_catalog=self._config.action_catalog,
            )
            self._posture = PostureManager()
            self._interrupt = InterruptManager()
            self._stage_executor = StageExecutor(
                eligibility_checker=self._eligibility,
                posture_manager=self._posture,
                interrupt_manager=self._interrupt,
                action_catalog=self._config.action_catalog,
            )
            self._result_evaluator = ResultEvaluator()

            # ── Union of acceptable behavior names ──────────────────────
            self._acceptable_behaviors = (
                set(self._config.behavior_templates.keys())
                | self._behavior_resolver.get_canonical_behaviors()
                | self._behavior_resolver.get_alias_names()
            )

            # ── Debug publishers ────────────────────────────────────────
            self._debug = DebugPublishers(self, enable_legacy=False)

            # ── Action Server ───────────────────────────────────────────
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
                )
                self.get_logger().info(
                    f"Action Server /execute_behavior started "
                    f"(action from {action_source})."
                )

            self.get_logger().info("ActionExecutorNode (v2) started.")

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
            """Validate incoming goal."""
            name = goal_request.behavior_name

            if name in self._acceptable_behaviors:
                self.get_logger().info(
                    f"Accepted goal: {goal_request.goal_id} -> {name}"
                )
                return GoalResponse.ACCEPT

            canonical, _ = self._behavior_resolver._resolve_name(name)
            if canonical is not None and canonical in self._acceptable_behaviors:
                self.get_logger().info(
                    f"Accepted goal (via alias): {goal_request.goal_id} "
                    f"-> {name} (resolves to {canonical})"
                )
                return GoalResponse.ACCEPT

            self.get_logger().warning(
                f"Rejected goal: unsupported_behavior={name!r} "
                f"goal_id={goal_request.goal_id}"
            )
            return GoalResponse.REJECT

        def _on_cancel(self, goal_handle) -> CancelResponse:
            """Accept all cancel requests."""
            self.get_logger().info(
                f"Cancel requested: {goal_handle.request.goal_id}"
            )
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

            # ── Step 2: Resolve (alias / inject / fallback) ─────────────
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
            self.get_logger().info(
                f"[{gid}] Resolved: {requested_name} -> {canonical}"
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

            success_condition = template.get("success_condition")

            self.get_logger().info(
                f"[{gid}] Plan: {canonical} -> {len(stages)} stages"
            )

            # ── Step 4: Execute stages ─────────────────────────────────
            stage_results: dict[str, bool] = {}
            start_time = time.time()

            # Reset interrupt state
            self._interrupt.reset()
            self._posture.reset()

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

                # ── Resolve emotion pool if needed ───────────────────
                stage_cfg = _resolve_emotion_pool(
                    stage_cfg, canonical, ctx, self._config,
                )

                # ── Resolve sleep_depth candidates if needed ────────
                stage_cfg = _resolve_sleep_depth_candidates(
                    stage_cfg, ctx,
                )

                # ── Execute stage ────────────────────────────────────
                self.get_logger().info(
                    f"[{gid}] Stage {i+1}/{len(stages)}: {stage_id}"
                )
                self._interrupt.apply_to_context(ctx)

                result = self._stage_executor.execute_stage(stage_cfg, ctx)
                stage_results[stage_id] = result.success

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

            # ── Step 5: Evaluate result ────────────────────────────────
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

            goal_handle.succeed()

            exec_result = ExecutionResult(
                goal_id=gid,
                behavior_id=behavior_id,
                behavior_name=canonical,
                status=behavior_result.status.upper(),
                result=behavior_result.status,
                reason=behavior_result.message or behavior_result.reason,
                duration_sec=duration,
                reward=behavior_result.reward,
                emotion_delta_json="{}",
                need_delta_json="{}",
            )
            self._debug.publish_result(exec_result)

            return _make_result(
                gid, behavior_id, requested_name,
                resolved_name=canonical,
                status=behavior_result.status.upper(),
                result=behavior_result.status,
                reason=behavior_result.message or behavior_result.reason,
                reward=behavior_result.reward,
            )

        # ── Helpers ────────────────────────────────────────────────────

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
# Helpers
# ═══════════════════════════════════════════════════════════════════════════════

def _resolve_emotion_pool(
    stage_cfg: dict,
    behavior_name: str,
    ctx: ExecutionContext,
    config: ConfigLoader,
) -> dict:
    """Resolve emotion action pool reference in stage candidates.

    If the stage has ``candidates: {pool: expressJoy}``, resolve it
    from ``emotion_action_pools.yaml`` based on ``level`` and
    ``interaction_mode``.
    """
    candidates = stage_cfg.get("candidates")
    if not isinstance(candidates, dict):
        return stage_cfg  # already a list, nothing to resolve

    pool_name = candidates.get("pool", "")
    if not pool_name:
        return stage_cfg

    level = ctx.level or "MID"
    mode = ctx.interaction_mode or "solo"

    pool_candidates = config.get_emotion_pool(pool_name, level, mode)
    if not pool_candidates:
        logger.warning(
            "Emotion pool %s/%s/%s returned no candidates — "
            "trying solo fallback",
            pool_name, level, mode,
        )
        pool_candidates = config.get_emotion_pool(pool_name, level, "solo")

    if not pool_candidates:
        logger.error(
            "Emotion pool %s has no candidates for level=%s mode=%s",
            pool_name, level, mode,
        )
        return stage_cfg

    # Replace pool reference with resolved candidates
    resolved_cfg = dict(stage_cfg)
    resolved_cfg["candidates"] = pool_candidates
    logger.debug(
        "Resolved emotion pool %s (level=%s, mode=%s) → %d candidates",
        pool_name, level, mode, len(pool_candidates),
    )
    return resolved_cfg


def _resolve_sleep_depth_candidates(
    stage_cfg: dict,
    ctx: ExecutionContext,
) -> dict:
    """Resolve sleep-depth-based candidates and loop config.

    If candidates is ``{shallow: [...], deep: [...]}``, pick the list
    matching ``ctx.sleep_depth`` (default: ``"shallow"``).
    Same for ``loop`` if it's a nested dict.
    """
    raw_candidates = stage_cfg.get("candidates")
    if isinstance(raw_candidates, dict):
        depth = ctx.sleep_depth or "shallow"
        resolved = raw_candidates.get(depth)
        if resolved is not None and isinstance(resolved, list):
            resolved_cfg = dict(stage_cfg)
            resolved_cfg["candidates"] = resolved
            logger.debug(
                "Resolved sleep_depth=%s → %d candidates",
                depth, len(resolved),
            )
            stage_cfg = resolved_cfg

    # Also resolve loop config if sleep-depth nested
    loop = stage_cfg.get("loop")
    if isinstance(loop, dict):
        depth = ctx.sleep_depth or "shallow"
        resolved_loop = loop.get(depth)
        if resolved_loop is not None and isinstance(resolved_loop, dict):
            resolved_cfg = dict(stage_cfg)
            resolved_cfg["loop_policy"] = resolved_loop
            resolved_cfg["selection_policy"] = "loop_random"
            stage_cfg = resolved_cfg

    return stage_cfg


def _make_result(
    goal_id: str,
    behavior_id: str,
    behavior_name: str,
    resolved_name: str = "",
    status: str = "SUCCESS",
    result: str = "completed",
    reason: str = "",
    reward: float = 1.0,
) -> object:
    """Build an ExecuteBehavior.Result."""
    if _ExecuteBehavior is None:
        return None
    return _ExecuteBehavior.Result(
        goal_id=goal_id,
        behavior_id=behavior_id,
        behavior_name=resolved_name or behavior_name,
        status=status,
        result=result,
        reason=reason,
        reward=reward,
        emotion_delta_json="{}",
        need_delta_json="{}",
    )


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
            "  uv run marsdog-action-demo"
        )
        sys.exit(1)

    rclpy.init(args=args)
    node = ActionExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
