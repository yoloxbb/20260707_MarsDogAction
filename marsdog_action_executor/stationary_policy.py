"""Fail-closed plan validation for voice-WAITING in-place expressions."""

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
    if behavior_name not in INPLACE_WITH_HUMAN_BEHAVIORS:
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
            if route not in INPLACE_ALLOWED_CONTROLLER_ROUTES:
                return InPlacePlanCheck(False, "inplace_policy_violation")

    return InPlacePlanCheck(True)


# Compatibility for local integrations that imported the old helper name.
StationaryPlanCheck = InPlacePlanCheck
validate_stationary_plan = validate_inplace_plan
