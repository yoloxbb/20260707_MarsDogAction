"""Concrete unit executors for all unit types."""

from __future__ import annotations

import logging
import time
from typing import Any

from ..execution_context import ExecutionContext
from ..interrupt_manager import InterruptManager, InterruptPolicy
from .base_unit_executor import BaseUnitExecutor, UnitResult, UnitState

logger = logging.getLogger(__name__)


def _remaining_budget(ctx: ExecutionContext) -> float | None:
    """Return the remaining behavior-goal budget at the call boundary."""
    return ctx.remaining_runtime_sec(time.monotonic())


class AtomicActionExecutor(BaseUnitExecutor):
    """Executes a single short-duration atomic action.

    Uses the configured controller adapter (motion/gimbal/audio etc.)
    or falls back to mock sleep execution.
    """

    def __init__(
        self,
        adapter: Any = None,
        interrupt_manager: InterruptManager | None = None,
    ) -> None:
        super().__init__(interrupt_manager)
        self._adapter = adapter

    def execute(
        self,
        unit_config: dict[str, Any],
        ctx: ExecutionContext,
    ) -> UnitResult:
        unit_id = unit_config.get("unit_id", "unknown")
        timeout = float(unit_config.get("timeout_sec", 5.0))
        duration = max(0.0, timeout * ctx.duration_scale * ctx.speed_scale)
        remaining = _remaining_budget(ctx)
        if remaining is not None:
            duration = min(duration, remaining)
        policy = self.get_interrupt_policy(unit_config)

        self._interrupt.start_unit(policy)
        start = time.time()

        if self._interrupt.cancel_requested and policy == InterruptPolicy.IMMEDIATE:
            return self._make_result(unit_id, UnitState.CANCELED, 0.0, "immediate cancel")

        if remaining is not None and remaining <= 0.0:
            return self._make_result(
                unit_id,
                UnitState.TIMEOUT,
                0.0,
                "behavior_goal_timeout",
            )

        # ── Execute via adapter or mock ──────────────────────────────
        try:
            if self._adapter is not None:
                ok = self._adapter.execute_step(unit_config, ctx, duration)
            else:
                ok = self._mock_execute(duration)
        except Exception as exc:
            logger.error("Unit %s failed: %s", unit_id, exc)
            return self._make_result(unit_id, UnitState.FAILURE, time.time() - start, str(exc))

        elapsed = time.time() - start

        if self._interrupt.cancel_requested:
            self._interrupt.unit_completed()
            return self._make_result(unit_id, UnitState.CANCELED, elapsed, "canceled")

        remaining = _remaining_budget(ctx)
        if remaining is not None and remaining <= 0.0:
            return self._make_result(
                unit_id,
                UnitState.TIMEOUT,
                elapsed,
                "behavior_goal_timeout",
            )

        if not ok:
            adapter_reason = str(
                getattr(self._adapter, "last_error", "") or ""
            ).strip()
            return self._make_result(
                unit_id,
                UnitState.FAILURE,
                elapsed,
                adapter_reason or "adapter returned failure",
            )

        return self._make_result(unit_id, UnitState.SUCCESS, elapsed, "completed")

    def _mock_execute(self, duration: float) -> bool:
        tick = 0.05
        elapsed = 0.0
        while elapsed < duration:
            if self._interrupt.cancel_requested:
                return False
            time.sleep(min(tick, duration - elapsed))
            elapsed += tick
        return True


class CompositeActionExecutor(BaseUnitExecutor):
    """Executes a sequence of sub-actions as one unit."""

    def __init__(
        self,
        sub_executors: dict[str, BaseUnitExecutor] | None = None,
        interrupt_manager: InterruptManager | None = None,
    ) -> None:
        super().__init__(interrupt_manager)
        self._sub_executors = sub_executors or {}

    def execute(
        self,
        unit_config: dict[str, Any],
        ctx: ExecutionContext,
    ) -> UnitResult:
        unit_id = unit_config.get("unit_id", "unknown")
        sub_actions = unit_config.get("sub_actions", [])
        start = time.time()

        policy = self.get_interrupt_policy(unit_config)
        self._interrupt.start_unit(policy)

        for sub in sub_actions:
            remaining = _remaining_budget(ctx)
            if remaining is not None and remaining <= 0.0:
                return self._make_result(
                    unit_id,
                    UnitState.TIMEOUT,
                    time.time() - start,
                    "behavior_goal_timeout",
                )
            if self._interrupt.cancel_requested and policy != InterruptPolicy.NON_INTERRUPTIBLE:
                self._interrupt.unit_completed()
                return self._make_result(unit_id, UnitState.CANCELED, time.time() - start, "canceled")

            sub_id = sub if isinstance(sub, str) else sub.get("unit_id", "?")
            executor = self._sub_executors.get("atomic_action")
            if executor:
                result = executor.execute({"unit_id": sub_id, "timeout_sec": sub.get("timeout_sec", 3.0) if isinstance(sub, dict) else 3.0}, ctx)
                if result.state != UnitState.SUCCESS:
                    return self._make_result(
                        unit_id,
                        result.state,
                        time.time() - start,
                        result.message or f"sub-action {sub_id} failed",
                        result.metadata,
                    )

        elapsed = time.time() - start
        return self._make_result(unit_id, UnitState.SUCCESS, elapsed, "all sub-actions completed")


class TaskExecutor(BaseUnitExecutor):
    """Executes a long-running task with RUNNING/SUCCESS/FAILURE/TIMEOUT lifecycle.

    Tasks may involve navigation, perception, target tracking, etc.
    Mock implementation simulates lifecycle stages.
    """

    def __init__(
        self,
        adapter: Any = None,
        interrupt_manager: InterruptManager | None = None,
    ) -> None:
        super().__init__(interrupt_manager)
        self._adapter = adapter

    def execute(
        self,
        unit_config: dict[str, Any],
        ctx: ExecutionContext,
    ) -> UnitResult:
        unit_id = unit_config.get("unit_id", "unknown")
        timeout = float(unit_config.get("timeout_sec", 30.0))
        remaining = ctx.remaining_runtime_sec(time.monotonic())
        if remaining is not None:
            timeout = min(timeout, remaining)
        policy = self.get_interrupt_policy(unit_config)

        self._interrupt.start_unit(policy)
        start = time.time()

        if timeout <= 0.0:
            return self._make_result(
                unit_id,
                UnitState.TIMEOUT,
                0.0,
                "behavior_goal_timeout",
            )

        if self._interrupt.cancel_requested:
            self._interrupt.unit_completed()
            return self._make_result(
                unit_id,
                UnitState.CANCELED,
                0.0,
                "canceled before task start",
            )

        # A real task adapter owns its closed-loop lifecycle.  It receives the
        # catalog timeout and must observe the shared interrupt/cancel signal.
        if self._adapter is not None:
            execute_task = getattr(self._adapter, "execute_task", None)
            if not callable(execute_task):
                return self._make_result(
                    unit_id,
                    UnitState.FAILURE,
                    0.0,
                    "task adapter does not implement execute_task",
                )
            try:
                outcome = execute_task(unit_config, ctx, timeout)
            except Exception as exc:
                logger.exception("Task %s adapter failed", unit_id)
                return self._make_result(
                    unit_id,
                    UnitState.FAILURE,
                    time.time() - start,
                    str(exc),
                )

            elapsed = time.time() - start
            metadata = getattr(outcome, "metadata", None)
            reason = str(getattr(outcome, "reason", "task completed"))
            if isinstance(outcome, UnitResult):
                remaining = _remaining_budget(ctx)
                if (
                    outcome.state == UnitState.SUCCESS
                    and remaining is not None
                    and remaining <= 0.0
                ):
                    return self._make_result(
                        unit_id,
                        UnitState.TIMEOUT,
                        elapsed,
                        "behavior_goal_timeout",
                        outcome.metadata,
                    )
                return outcome
            if isinstance(metadata, dict) and metadata.get("recovery_required"):
                return self._make_result(
                    unit_id, UnitState.FAILURE, elapsed, reason, metadata,
                )
            if (
                self._interrupt.cancel_requested
                or bool(getattr(outcome, "canceled", False))
            ):
                self._interrupt.unit_completed()
                return self._make_result(
                    unit_id,
                    UnitState.CANCELED,
                    elapsed,
                    reason,
                    metadata,
                )
            if bool(getattr(outcome, "timed_out", False)):
                return self._make_result(
                    unit_id,
                    UnitState.TIMEOUT,
                    elapsed,
                    reason,
                    metadata,
                )
            remaining = _remaining_budget(ctx)
            if remaining is not None and remaining <= 0.0:
                return self._make_result(
                    unit_id,
                    UnitState.TIMEOUT,
                    elapsed,
                    "behavior_goal_timeout",
                    metadata,
                )
            if bool(getattr(outcome, "success", outcome)):
                return self._make_result(
                    unit_id,
                    UnitState.SUCCESS,
                    elapsed,
                    reason,
                    metadata,
                )
            return self._make_result(
                unit_id,
                UnitState.FAILURE,
                elapsed,
                reason,
                metadata,
            )

        # No adapter means standalone/mock execution only.
        tick = 0.1
        elapsed = 0.0
        while elapsed < timeout:
            if self._interrupt.cancel_requested:
                if policy == InterruptPolicy.NON_INTERRUPTIBLE:
                    pass  # finish unit
                else:
                    self._interrupt.unit_completed()
                    return self._make_result(unit_id, UnitState.CANCELED, elapsed, "canceled")

            mock_duration = min(timeout, ctx.duration_scale * 2.0)
            if elapsed >= mock_duration:
                return self._make_result(
                    unit_id,
                    UnitState.SUCCESS,
                    elapsed,
                    "task completed (mock)",
                )

            time.sleep(tick)
            elapsed += tick

        return self._make_result(unit_id, UnitState.TIMEOUT, elapsed, "task timed out")


class PolicyExecutor(BaseUnitExecutor):
    """Executes a policy unit — no physical action, just state change.

    For ACT_IGNORE_* type actions: the stage succeeds without any
    motion or controller call.
    """

    def execute(
        self,
        unit_config: dict[str, Any],
        ctx: ExecutionContext,
    ) -> UnitResult:
        unit_id = unit_config.get("unit_id", "unknown")
        remaining = _remaining_budget(ctx)
        if remaining is not None and remaining <= 0.0:
            return self._make_result(
                unit_id,
                UnitState.TIMEOUT,
                0.0,
                "behavior_goal_timeout",
            )
        effect = unit_config.get("effect", {})
        if effect.get("complete_stage_without_motion"):
            logger.info("Policy %s: stage completed without motion", unit_id)
        return self._make_result(unit_id, UnitState.SUCCESS, 0.0, "policy applied")


class ModifierExecutor(BaseUnitExecutor):
    """Executes a modifier — adjusts scaling parameters without motion.

    For ACT_SLOW_MOVEMENT type actions: applies speed/accel/amplitude
    scale factors to the execution context for subsequent units.
    """

    def execute(
        self,
        unit_config: dict[str, Any],
        ctx: ExecutionContext,
    ) -> UnitResult:
        unit_id = unit_config.get("unit_id", "unknown")
        remaining = _remaining_budget(ctx)
        if remaining is not None and remaining <= 0.0:
            return self._make_result(
                unit_id,
                UnitState.TIMEOUT,
                0.0,
                "behavior_goal_timeout",
            )
        effects = unit_config.get("effects", {})
        if "speed_scale" in effects:
            ctx.speed_scale = float(effects["speed_scale"])
        if "acceleration_scale" in effects:
            ctx.acceleration_scale = float(effects["acceleration_scale"])
        if "amplitude_scale" in effects:
            ctx.amplitude_scale = float(effects["amplitude_scale"])
        logger.info("Modifier %s applied: speed=%.2f accel=%.2f amp=%.2f",
                     unit_id, ctx.speed_scale, ctx.acceleration_scale, ctx.amplitude_scale)
        return self._make_result(unit_id, UnitState.SUCCESS, 0.0, "modifier applied")


# ── Factory ───────────────────────────────────────────────────────────────────


def create_executor(
    unit_type: str,
    *,
    interrupt_manager: InterruptManager | None = None,
    adapter: Any = None,
) -> BaseUnitExecutor:
    """Create a unit executor by type name."""
    registry = {
        "atomic_action": AtomicActionExecutor,
        "composite_action": CompositeActionExecutor,
        "task": TaskExecutor,
        "policy": PolicyExecutor,
        "modifier": ModifierExecutor,
    }
    cls = registry.get(unit_type)
    if cls is None:
        logger.warning("Unknown unit_type %r — using AtomicActionExecutor", unit_type)
        cls = AtomicActionExecutor
    kwargs: dict[str, Any] = {"interrupt_manager": interrupt_manager}
    if cls in (AtomicActionExecutor, TaskExecutor):
        kwargs["adapter"] = adapter
    return cls(**kwargs)
