"""ResultEvaluator — determines behavior-level success/failure from stage outcomes."""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any

from .execution_context import ExecutionContext

logger = logging.getLogger(__name__)

# ── Behavior success conditions ───────────────────────────────────────────────

BEHAVIOR_SUCCESS_CONDITIONS: dict[str, str] = {
    "eatNormally": "eating_stage_completed",
    "eatExcitedly": "eating_stage_completed",
    "defecate": "eliminating_stage_completed",
    "cleanSelf": "groom_stage_completed",
    "sleepNow": "sleep_pose_entered",
    "restInPlace": "recover_stage_completed",
    "recharge": "charging_detected",
    "testAnimalBoundary": "express_stage_completed",
    "greetAnimal": "greet_stage_completed",
    "inviteAnimalToPlay": "invite_stage_completed",
    "requestResourceFromHuman": "request_stage_completed",
    "seekHumanInteraction": "interact_stage_completed",
    "inviteHumanToPlay": "invite_stage_completed",
    "exploreRoom": "explore_stage_completed",
    "inspectObject": "inspect_stage_completed",
    "inspectKnownObject": "inspect_stage_completed",
    "expressCalm": "at_least_one_expression",
    "expressJoy": "at_least_one_expression",
    "expressExcitement": "at_least_one_expression",
    "expressAnxiety": "at_least_one_expression",
    "expressFear": "at_least_one_expression",
    "expressCuriosity": "at_least_one_expression",
    "zoomiesRun": "all_required_stages_completed",
    "walkToOwner": "all_required_stages_completed",
    "trotToOwner": "all_required_stages_completed",
    "tailUpAndWag": "all_required_stages_completed",
    "spitOut": "all_required_stages_completed",
    "spinOnce": "all_required_stages_completed",
    "sniffObject": "all_required_stages_completed",
    "sitPolitely": "all_required_stages_completed",
    "sit": "all_required_stages_completed",
    "seekSafety": "all_required_stages_completed",
    "seekInteraction": "all_required_stages_completed",
    "seekFood": "all_required_stages_completed",
    "rollOverShowBelly": "all_required_stages_completed",
    "rollOver": "all_required_stages_completed",
    "retrieveToy": "all_required_stages_completed",
    "pounceForward": "all_required_stages_completed",
    "playDead": "all_required_stages_completed",
    "lickPaws": "all_required_stages_completed",
    "jumpOnPerson": "all_required_stages_completed",
    "inspectToy": "all_required_stages_completed",
    "highFive": "all_required_stages_completed",
    "handShake": "all_required_stages_completed",
    "followUser": "all_required_stages_completed",
    "emergencyStop": "all_required_stages_completed",
    "eatImmediately": "all_required_stages_completed",
    "comeHere": "all_required_stages_completed",
    "carryAndShake": "all_required_stages_completed",
    "barkShortAlert": "all_required_stages_completed",
    "approachUser": "all_required_stages_completed",
    "alertAndApproach": "all_required_stages_completed",
}

# ── Result codes ──────────────────────────────────────────────────────────────

_VALID_STATUSES = {
    "success", "failure", "canceled", "timeout",
    "no_eligible_action", "invalid_params", "unsupported_behavior",
    "unsupported_object_category", "controller_error", "configuration_error",
}


@dataclass
class BehaviorResult:
    """Structured result for a completed behavior execution."""

    success: bool = False
    status: str = "success"
    requested_behavior_name: str = ""
    resolved_behavior_name: str = ""
    variant: str | None = None
    completed_stages: list[str] = field(default_factory=list)
    executed_units: list[str] = field(default_factory=list)
    goal_achieved: bool = False
    result_code: str = ""
    message: str = ""
    reason: str = ""
    reward: float = 1.0
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_json(self) -> str:
        import json as _json
        return _json.dumps({
            "success": self.success,
            "status": self.status,
            "requested_behavior_name": self.requested_behavior_name,
            "resolved_behavior_name": self.resolved_behavior_name,
            "variant": self.variant,
            "completed_stages": self.completed_stages,
            "executed_units": self.executed_units,
            "goal_achieved": self.goal_achieved,
            "result_code": self.result_code,
            "message": self.message,
        })


class ResultEvaluator:
    """Evaluates behavior-level success from stage completion data."""

    def evaluate(
        self,
        ctx: ExecutionContext,
        stage_results: dict[str, bool],
        success_condition: str | None = None,
    ) -> BehaviorResult:
        """Determine the final behavior result.

        Args:
            ctx: The execution context.
            stage_results: stage_id → success (True/False).
            success_condition: Override success condition key.
        """
        behavior_name = ctx.resolved_behavior_name or ctx.requested_behavior_name

        if not ctx.is_valid:
            return BehaviorResult(
                success=False,
                status=ctx.error_reason or "invalid_params",
                requested_behavior_name=ctx.requested_behavior_name,
                resolved_behavior_name=behavior_name,
                completed_stages=list(ctx.completed_stages),
                executed_units=list(ctx.executed_units),
                goal_achieved=False,
                result_code=ctx.error_reason or "INVALID_PARAMS",
                message=ctx.error_reason or "Invalid parameters",
                reward=-1.0,
            )

        if ctx.cancel_requested:
            return BehaviorResult(
                success=False,
                status="canceled",
                requested_behavior_name=ctx.requested_behavior_name,
                resolved_behavior_name=behavior_name,
                completed_stages=list(ctx.completed_stages),
                executed_units=list(ctx.executed_units),
                goal_achieved=False,
                result_code="CANCELED",
                message="Canceled by client",
                reward=-0.1,
            )

        # Determine success based on condition
        cond_key = success_condition or BEHAVIOR_SUCCESS_CONDITIONS.get(
            behavior_name, "all_required_stages_completed",
        )

        goal_achieved = self._check_condition(cond_key, stage_results, ctx)

        if goal_achieved:
            return BehaviorResult(
                success=True,
                status="success",
                requested_behavior_name=ctx.requested_behavior_name,
                resolved_behavior_name=behavior_name,
                variant=ctx.variant,
                completed_stages=list(ctx.completed_stages),
                executed_units=list(ctx.executed_units),
                goal_achieved=True,
                result_code=_condition_to_code(cond_key),
                message="behavior completed",
                reward=1.0,
            )

        # Determine specific failure reason
        if not ctx.executed_units:
            return BehaviorResult(
                success=False,
                status="no_eligible_action",
                requested_behavior_name=ctx.requested_behavior_name,
                resolved_behavior_name=behavior_name,
                completed_stages=list(ctx.completed_stages),
                executed_units=[],
                goal_achieved=False,
                result_code="NO_ELIGIBLE_ACTION",
                message="No eligible action found",
                reward=-0.5,
            )

        return BehaviorResult(
            success=False,
            status="failure",
            requested_behavior_name=ctx.requested_behavior_name,
            resolved_behavior_name=behavior_name,
            completed_stages=list(ctx.completed_stages),
            executed_units=list(ctx.executed_units),
            goal_achieved=False,
            result_code="EXECUTION_FAILED",
            message="Required stages not completed",
            reward=-0.5,
        )

    @staticmethod
    def _check_condition(
        cond: str,
        stage_results: dict[str, bool],
        ctx: ExecutionContext,
    ) -> bool:
        """Evaluate a success condition."""
        if cond == "all_required_stages_completed":
            return all(stage_results.values()) if stage_results else True
        if cond.endswith("_stage_completed"):
            stage_id = cond.replace("_stage_completed", "")
            return stage_results.get(stage_id, False)
        if cond == "sleep_pose_entered":
            return stage_results.get("sleep_pose", False)
        if cond == "charging_detected":
            return ctx.metadata.get("charging_detected", False) is True
        if cond == "at_least_one_expression":
            return len(ctx.executed_units) >= 1
        if cond == "inspect_stage_completed":
            return stage_results.get("inspect", False) or stage_results.get("explore", False)
        return False


def _condition_to_code(cond: str) -> str:
    return cond.upper().replace(" ", "_") + "_COMPLETED"
