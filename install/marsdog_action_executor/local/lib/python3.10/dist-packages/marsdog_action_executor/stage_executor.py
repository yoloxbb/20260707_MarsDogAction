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
        logger.info("Stage %s starting (policy=%s, required=%s)", stage_id, policy, required)

        rng = random.Random(seed)

        # ── Handle loop policy ───────────────────────────────────────
        if loop_policy and policy == "loop_random":
            return self._execute_loop(stage_config, ctx, rng)

        # ── Select candidate(s) ───────────────────────────────────────
        candidates = self._get_candidates(stage_config)

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
                    return StageResult(stage_id, False, unit_cfg.get("unit_id", ""), "unit failed")
        else:
            result = self._execute_unit(chosen, ctx)
            if not result and required and failure_policy != "skip_stage":
                return StageResult(stage_id, False, chosen.get("unit_id", ""), "unit failed")
            if not result:
                return StageResult(stage_id, True, chosen.get("unit_id", ""), "optional unit failed, stage skipped")

        ctx.completed_stages.append(stage_id)
        return StageResult(stage_id, True, chosen.get("unit_id", "") if not isinstance(chosen, list) else "sequence", "stage completed")

    # ── internal helpers ──────────────────────────────────────────────

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

        # A routed hardware adapter implements the complete ACT_* unit, even
        # when the semantic catalog classifies it as composite_action or task.
        executor_type = "atomic_action" if adapter is not None else unit_type
        executor = create_executor(
            executor_type,
            interrupt_manager=self._interrupt,
            adapter=adapter,
        )
        result = executor.execute(merged, ctx)

        if result.state == UnitState.SUCCESS:
            ctx.executed_units.append(unit_id)
            # Update posture
            to_posture = merged.get("to_posture")
            self._posture.apply_unit_to_posture(to_posture)
            return True
        elif result.state == UnitState.CANCELED:
            logger.info("Unit %s canceled", unit_id)
            return False
        else:
            logger.warning("Unit %s failed: %s (%s)", unit_id, result.state.value, result.message)
            return False
