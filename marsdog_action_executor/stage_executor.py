"""StageExecutor — executes a single Stage within a Behavior.

Handles:
  - candidate selection (via UnitSelector)
  - eligibility filtering (via EligibilityChecker)
  - unit execution (via unit executors)
  - failure policy (skip_stage, abort, retry)
  - loop policy (loop_random with min/max loops)
"""

from __future__ import annotations

import logging
import random
import time
from typing import Any

from .eligibility_checker import EligibilityChecker
from .execution_context import ExecutionContext
from .interrupt_manager import InterruptManager, InterruptPolicy
from .posture_manager import PostureManager
from .units.unit_executors import BaseUnitExecutor, create_executor, UnitState

logger = logging.getLogger(__name__)


class StageResult:
    """Outcome of executing one Stage."""

    def __init__(
        self,
        stage_id: str,
        success: bool,
        unit_id: str = "",
        message: str = "",
    ) -> None:
        self.stage_id = stage_id
        self.success = success
        self.unit_id = unit_id
        self.message = message


class StageExecutor:
    """Executes a single stage: select candidate → filter → execute.

    Supports: fixed, sequence, random_one, weighted_random, random_n,
    loop_random, condition_first selection policies.
    """

    def __init__(
        self,
        eligibility_checker: EligibilityChecker | None = None,
        posture_manager: PostureManager | None = None,
        interrupt_manager: InterruptManager | None = None,
        action_catalog: dict[str, dict[str, Any]] | None = None,
        controller_routes: dict[str, str] | None = None,
        controller_adapters: dict[str, Any] | None = None,
    ) -> None:
        self._eligibility = eligibility_checker or EligibilityChecker()
        self._posture = posture_manager or PostureManager()
        self._interrupt = interrupt_manager or InterruptManager()
        self._catalog = action_catalog or {}
        self._controller_routes = controller_routes or {}
        self._controller_adapters = controller_adapters or {}

    def execute_stage(
        self,
        stage_config: dict[str, Any],
        ctx: ExecutionContext,
        seed: int | None = None,
    ) -> StageResult:
        """Execute a single stage.

        Args:
            stage_config: Stage definition dict.
            ctx: ExecutionContext (mutated in-place).
            seed: Optional random seed.

        Returns:
            StageResult indicating success/failure.
        """
        stage_id = stage_config.get("stage_id") or stage_config.get("stage_name") or stage_config.get("phase", "unknown")
        policy = stage_config.get("selection_policy", "random_one")
        required = stage_config.get("required", True)
        failure_policy = stage_config.get("failure_policy", "abort")
        loop_policy = stage_config.get("loop_policy")

        ctx.current_stage = stage_id
        ctx.motion_state = str(stage_config.get("motion_state", "active"))
        self._apply_stage_motion_state(ctx)
        logger.info(
            "Stage %s starting (policy=%s, required=%s, motion_state=%s)",
            stage_id,
            policy,
            required,
            ctx.motion_state,
        )

        rng = random.Random(seed)

        # ── Handle loop policy ───────────────────────────────────────
        if loop_policy and policy == "loop_random":
            return self._execute_loop(stage_config, ctx, rng)

        # ── Select candidate(s) ───────────────────────────────────────
        candidates = self._get_candidates(stage_config)
        available_candidates = [
            candidate
            for candidate in candidates
            if self._controller_candidate_available(candidate, ctx)
        ]
        # Preserve the existing precise controller error for a stage whose
        # entire candidate set is unavailable.  When at least one platform
        # implementation exists, exclude unavailable alternatives before the
        # random selection so Go2 PRAISE/SCOLD cannot choose a unit without
        # a controller on the selected chassis.
        if available_candidates:
            candidates = available_candidates

        if policy in ("fixed", "sequence"):
            chosen = candidates  # all candidates in order
        elif policy == "condition_first":
            chosen = self._select_condition_first(candidates, ctx, rng)
        else:
            chosen = self._select_best(candidates, ctx, policy, rng)

        if not chosen:
            msg = f"no_eligible_action for stage {stage_id}"
            logger.warning(msg)
            if required and failure_policy != "skip_stage":
                return StageResult(stage_id, False, "", msg)
            return StageResult(stage_id, True, "", msg)

        # ── Execute chosen unit(s) ────────────────────────────────────
        if isinstance(chosen, list):
            for unit_cfg in chosen:
                result = self._execute_unit(unit_cfg, ctx)
                if not result and required and failure_policy != "skip_stage":
                    return StageResult(
                        stage_id,
                        False,
                        unit_cfg.get("unit_id", ""),
                        str(
                            ctx.metadata.get(
                                "unit_failure_reason",
                                "unit failed",
                            )
                        ),
                    )
        else:
            result = self._execute_unit(chosen, ctx)
            if not result and required and failure_policy != "skip_stage":
                return StageResult(
                    stage_id,
                    False,
                    chosen.get("unit_id", ""),
                    str(
                        ctx.metadata.get(
                            "unit_failure_reason",
                            "unit failed",
                        )
                    ),
                )
            if not result:
                return StageResult(stage_id, True, chosen.get("unit_id", ""), "optional unit failed, stage skipped")

        ctx.completed_stages.append(stage_id)
        return StageResult(stage_id, True, chosen.get("unit_id", "") if not isinstance(chosen, list) else "sequence", "stage completed")

    # ── internal helpers ──────────────────────────────────────────────

    def _apply_stage_motion_state(self, ctx: ExecutionContext) -> None:
        """Apply a stage's chassis invariant as soon as the stage starts."""
        if ctx.motion_state != "stationary":
            return

        # Runtime installs one selected chassis backend; behavior_mobility
        # wraps that same backend. Prefer the direct route to avoid duplicate
        # stop bursts.
        adapter = self._controller_adapters.get("lite3")
        if adapter is None:
            adapter = self._controller_adapters.get("go2")
        if adapter is None:
            adapter = self._controller_adapters.get("behavior_mobility")
        hold_position = getattr(adapter, "hold_position", None)
        if callable(hold_position):
            hold_position()

    def _get_candidates(self, stage_config: dict[str, Any]) -> list[dict[str, Any]]:
        """Extract candidate list from stage config."""
        raw = stage_config.get("candidates", [])
        result: list[dict[str, Any]] = []
        for entry in raw:
            if isinstance(entry, str):
                result.append({"unit_id": entry})
            elif isinstance(entry, dict):
                result.append(dict(entry))
        return result

    def _controller_candidate_available(
        self,
        candidate: dict[str, Any],
        ctx: ExecutionContext,
    ) -> bool:
        """Return whether this runtime has a controller for the candidate."""
        unit_id = str(candidate.get("unit_id", ""))
        route = self._controller_routes.get(
            unit_id,
            self._controller_routes.get("_default", "mock"),
        )
        if route in (None, "", "mock"):
            return True
        adapter = self._controller_adapters.get(route)
        if adapter is None:
            mobility_adapter = self._controller_adapters.get(
                "behavior_mobility"
            )
            if (
                mobility_adapter is not None
                and mobility_adapter.handles(
                    ctx.resolved_behavior_name,
                    str(ctx.current_stage or ""),
                )
            ):
                adapter = mobility_adapter
        if adapter is None:
            return False
        can_execute = getattr(adapter, "can_execute", None)
        if callable(can_execute):
            return bool(can_execute(unit_id))
        unit_type = self._catalog.get(unit_id, {}).get(
            "unit_type", "atomic_action"
        )
        required_method = (
            "execute_task" if unit_type == "task" else "execute_step"
        )
        return callable(getattr(adapter, required_method, None))

    def _select_best(
        self,
        candidates: list[dict[str, Any]],
        ctx: ExecutionContext,
        policy: str,
        rng: random.Random,
    ) -> dict[str, Any] | None:
        """Filter → select one candidate."""
        eligible = self._eligibility.filter_candidates(candidates, ctx)
        if not eligible:
            return None

        if policy == "random_one":
            return rng.choice(eligible)
        if policy == "weighted_random":
            weights = [float(c.get("weight", 1.0)) for c in eligible]
            [chosen] = rng.choices(eligible, weights=weights, k=1)
            return chosen
        if policy == "first":
            return eligible[0]
        # Default
        return rng.choice(eligible)

    def _select_condition_first(
        self,
        candidates: list[dict[str, Any]],
        ctx: ExecutionContext,
        rng: random.Random,
    ) -> dict[str, Any] | None:
        """Select first candidate whose conditions all pass."""
        for candidate in candidates:
            if self._eligibility._is_eligible(candidate, ctx):
                return candidate
        return None

    def _execute_loop(
        self,
        stage_config: dict[str, Any],
        ctx: ExecutionContext,
        rng: random.Random,
    ) -> StageResult:
        """Execute a loop_random stage."""
        stage_id = stage_config.get("stage_id") or stage_config.get("stage_name") or "loop"
        loop = stage_config.get("loop_policy", {})
        min_loops = int(loop.get("min", loop.get("min_loops", 0)))
        max_loops = int(loop.get("max", loop.get("max_loops", 3)))
        interval = float(loop.get("interval_sec", 10))
        num_loops = rng.randint(min_loops, max_loops)

        last_unit = ""
        for i in range(num_loops):
            candidate = self._select_best(
                self._get_candidates(stage_config), ctx, "random_one", rng,
            )
            if candidate:
                self._execute_unit(candidate, ctx)
                last_unit = candidate.get("unit_id", "")
            if i < num_loops - 1:
                time.sleep(interval)

        ctx.completed_stages.append(stage_id)
        return StageResult(stage_id, True, last_unit, f"loop completed ({num_loops} iterations)")

    def _execute_unit(
        self,
        unit_config: dict[str, Any],
        ctx: ExecutionContext,
    ) -> bool:
        """Execute a single unit and update context."""
        unit_id = unit_config.get("unit_id", "unknown")
        ctx.current_unit = unit_id
        ctx.metadata.pop("unit_failure_state", None)
        ctx.metadata.pop("unit_failure_reason", None)

        # Get unit metadata from catalog
        meta = self._catalog.get(unit_id, {})
        merged = {**meta, **unit_config}
        unit_type = merged.get("unit_type", "atomic_action")
        route = self._controller_routes.get(
            unit_id,
            self._controller_routes.get("_default", "mock"),
        )
        adapter = self._controller_adapters.get(route)
        if adapter is None:
            mobility_adapter = self._controller_adapters.get(
                "behavior_mobility"
            )
            if (
                mobility_adapter is not None
                and mobility_adapter.handles(
                    ctx.resolved_behavior_name,
                    str(ctx.current_stage or ""),
                )
            ):
                adapter = mobility_adapter

        # Motion/safety-critical tasks must never fall through to the generic
        # TaskExecutor mock lifecycle.  A missing or stale controller route
        # otherwise sleeps briefly and reports success without issuing any
        # physical command, allowing the following expression Stage to run.
        if bool(merged.get("requires_controller", False)) and adapter is None:
            message = f"controller_required:{route or 'unconfigured'}"
            logger.error(
                "Unit %s requires a real controller; route=%s is unavailable",
                unit_id,
                route,
            )
            ctx.metadata["unit_failure_reason"] = message
            ctx.metadata["unit_failure_state"] = "controller_error"
            return False

        if adapter is None and route not in ("", "mock", None):
            message = f"controller_unavailable:{route}"
            logger.error(
                "Unit %s cannot run because controller %s is unavailable",
                unit_id,
                route,
            )
            ctx.metadata["unit_failure_reason"] = message
            ctx.metadata["unit_failure_state"] = "controller_error"
            return False

        # A routed hardware adapter implements the complete ACT_* unit, even
        # when the semantic catalog classifies it as composite_action or task.
        executor_type = (
            "task"
            if unit_type == "task"
            else "atomic_action" if adapter is not None else unit_type
        )
        executor = create_executor(
            executor_type,
            interrupt_manager=self._interrupt,
            adapter=adapter,
        )
        result = executor.execute(merged, ctx)

        if result.state == UnitState.SUCCESS:
            ctx.executed_units.append(unit_id)
            # Hardware-backed proxies must report the posture they actually
            # observed.  Applying the biological ACT_* catalog transition to
            # a Lite3 HOLD proxy would otherwise invent sitting/lying state.
            to_posture = merged.get("to_posture")
            if route == "lite3":
                lite3_result = ctx.metadata.get("lite3_action")
                if (
                    isinstance(lite3_result, dict)
                    and lite3_result.get("requested_act") == unit_id
                ):
                    physical_posture = lite3_result.get("physical_posture")
                    if isinstance(physical_posture, str) and physical_posture:
                        to_posture = physical_posture
                    elif lite3_result.get("fidelity") == "proxy":
                        to_posture = ""
            self._posture.apply_unit_to_posture(to_posture)
            return True
        elif result.state == UnitState.CANCELED:
            ctx.metadata["unit_failure_state"] = "canceled"
            ctx.metadata["unit_failure_reason"] = result.message
            logger.info("Unit %s canceled", unit_id)
            return False
        else:
            ctx.metadata["unit_failure_state"] = result.state.value
            ctx.metadata["unit_failure_reason"] = result.message
            logger.warning("Unit %s failed: %s (%s)", unit_id, result.state.value, result.message)
            return False
