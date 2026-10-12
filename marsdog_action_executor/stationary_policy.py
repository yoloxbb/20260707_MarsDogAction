"""Fail-closed plan validation for wake and voice-WAITING in-place expressions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping


INPLACE_WITH_HUMAN_BEHAVIORS = frozenset({
    "expressCalmInPlaceWithHuman",
    "expressJoyInPlaceWithHuman",
    "expressExcitementInPlaceWithHuman",
    "expressAnxietyInPlaceWithHuman",
    "expressFearInPlaceWithHuman",
    "expressCuriosityInPlaceWithHuman",
})

WAKE_EXPRESSION_BEHAVIOR = "respond_wake_in_place"
# These backend plans use posture gestures only; none acquires a target or walks.
WAKE_EXPRESSION_UNITS = frozenset({
    "ACT_RAISE_HEAD", "ACT_TWIST_WAIST_LEFT_RIGHT", "ACT_STRETCH",
    "ACT_ROAM_LOOK_DOWN_UP", "ACT_ROAM_SCAN_HEAD_UP", "ACT_ROAM_STRETCH_SCAN",
})

INPLACE_MOBILITY_POLICY = "in_place"
INPLACE_ALLOWED_CONTROLLER_ROUTES = frozenset(
    {"go2", "lite3", "mock"}
)


@dataclass(frozen=True)
class InPlacePlanCheck:
    valid: bool
    reason: str = ""


def validate_inplace_plan(
    behavior_name: str,
    params: Mapping[str, Any],
    stages: list[Mapping[str, Any]],
    controller_routes: Mapping[str, str],
    navigation_config: Mapping[str, Any],
) -> InPlacePlanCheck:
    """Allow expression motion, but reject every pre-expression mobility path.

    ``InPlaceWithHuman`` now means that the already selected person is not
    approached again.  Its expression stage intentionally reuses the normal
    ``WithHuman`` action units, including chassis expression motions.
    """
    wake_expression = behavior_name == WAKE_EXPRESSION_BEHAVIOR
    if not wake_expression and behavior_name not in INPLACE_WITH_HUMAN_BEHAVIORS:
        return InPlacePlanCheck(True)
    if str(params.get("mobility_policy", "")) != INPLACE_MOBILITY_POLICY:
        return InPlacePlanCheck(False, "inplace_policy_required")

    random_behaviors = set(
        navigation_config.get("random_navigation_behaviors", []) or []
    )
    behavior_routes = navigation_config.get("behavior_routes", {}) or {}
    if behavior_name in random_behaviors or behavior_name in behavior_routes:
        return InPlacePlanCheck(False, "inplace_policy_violation")

    if len(stages) != 1:
        return InPlacePlanCheck(False, "inplace_policy_violation")
    for stage in stages:
        if str(stage.get("stage_id", "")) != "expression":
            return InPlacePlanCheck(False, "inplace_policy_violation")
        if str(stage.get("motion_state", "active")) == "stationary":
            return InPlacePlanCheck(False, "inplace_policy_violation")
        if wake_expression and stage.get("selection_policy") != "random_one":
            return InPlacePlanCheck(False, "inplace_policy_violation")
        candidates = stage.get("candidates", []) or []
        if not candidates:
            return InPlacePlanCheck(False, "inplace_policy_violation")
        for candidate in candidates:
            if not isinstance(candidate, Mapping):
                return InPlacePlanCheck(False, "inplace_policy_violation")
            unit_id = str(candidate.get("unit_id", ""))
            route = str(
                controller_routes.get(
                    unit_id,
                    controller_routes.get("_default", "mock"),
                )
            )
            if wake_expression and (
                unit_id not in WAKE_EXPRESSION_UNITS
                or route not in {"go2", "lite3", "unsupported"}
            ):
                return InPlacePlanCheck(False, "inplace_policy_violation")
            # Unsupported candidates issue no expression motion and are
            # filtered/rejected by StageExecutor. They must not invalidate a
            # mixed random stage before its supported candidates can run.
            if route != "unsupported" and route not in INPLACE_ALLOWED_CONTROLLER_ROUTES:
                return InPlacePlanCheck(False, "inplace_policy_violation")

    return InPlacePlanCheck(True)


# Compatibility for local integrations that imported the old helper name.
StationaryPlanCheck = InPlacePlanCheck
validate_stationary_plan = validate_inplace_plan
