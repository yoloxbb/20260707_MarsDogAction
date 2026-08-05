"""EligibilityChecker — filters action candidates by conditions, posture, and safety.

Condition names are validated at startup.  Unknown conditions raise
ConfigurationError during init, not silently skipped at runtime.
"""

from __future__ import annotations

import logging
from typing import Any, Callable

from .execution_context import ExecutionContext

logger = logging.getLogger(__name__)

# ── Known condition checkers ──────────────────────────────────────────────────

_CONDITION_CHECKERS: dict[str, Callable[[ExecutionContext], bool]] = {}


def register_condition(name: str, fn: Callable[[ExecutionContext], bool]) -> None:
    """Register a named condition checker."""
    _CONDITION_CHECKERS[name] = fn


# ── Built-in conditions ──────────────────────────────────────────────────────

def _cond_owner_visible(ctx: ExecutionContext) -> bool:
    t = ctx.target
    return bool(t and t.get("visible") and t.get("target_type") == "human" and t.get("identity") == "owner")


def _cond_target_visible(ctx: ExecutionContext) -> bool:
    t = ctx.target
    return bool(t and t.get("visible"))


def _cond_person_not_visible(ctx: ExecutionContext) -> bool:
    return not _cond_target_visible(ctx)


def _cond_person_too_far(ctx: ExecutionContext) -> bool:
    t = ctx.target
    if not t:
        return False
    dist = t.get("distance_m", t.get("distance"))
    if dist is None:
        return False
    try:
        return float(dist) > 1.5
    except (ValueError, TypeError):
        return False


def _cond_contact_distance(ctx: ExecutionContext) -> bool:
    t = ctx.target
    if not t:
        return False
    dist = t.get("distance_m", t.get("distance"))
    if dist is None:
        return False
    try:
        return float(dist) <= 0.5
    except (ValueError, TypeError):
        return False


def _cond_target_moving(ctx: ExecutionContext) -> bool:
    t = ctx.target
    return bool(t and t.get("moving", False))


def _cond_target_not_aggressive(ctx: ExecutionContext) -> bool:
    t = ctx.target
    return not bool(t and t.get("aggressive", False))


def _cond_target_distance_valid(ctx: ExecutionContext) -> bool:
    t = ctx.target
    if not t:
        return False
    dist = t.get("distance_m", t.get("distance"))
    if dist is None:
        return True  # unknown distance → assume valid
    try:
        return 0.1 <= float(dist) <= 10.0
    except (ValueError, TypeError):
        return True


def _cond_animal_target_available(ctx: ExecutionContext) -> bool:
    t = ctx.target
    return bool(t and t.get("target_type") == "animal" and t.get("visible"))


def _cond_human_target_available(ctx: ExecutionContext) -> bool:
    t = ctx.target
    return bool(t and t.get("target_type") == "human" and t.get("visible"))


def _cond_object_target_available(ctx: ExecutionContext) -> bool:
    t = ctx.target
    return bool(t and t.get("target_type") == "object" and t.get("visible"))


def _cond_rolling_food_detected(ctx: ExecutionContext) -> bool:
    return ctx.metadata.get("rolling_food_detected", False) is True


def _cond_elimination_type_pee(ctx: ExecutionContext) -> bool:
    return ctx.elimination_type == "pee"


def _cond_elimination_type_poop(ctx: ExecutionContext) -> bool:
    return ctx.elimination_type == "poop"


def _cond_suitable_rubbing_object_available(ctx: ExecutionContext) -> bool:
    return ctx.metadata.get("suitable_rubbing_object_available", False) is True


def _cond_charger_known(ctx: ExecutionContext) -> bool:
    return ctx.charger_known is True


def _cond_charger_available(ctx: ExecutionContext) -> bool:
    return ctx.charger_available is True


def _cond_charger_unavailable(ctx: ExecutionContext) -> bool:
    # True if charger is explicitly unavailable, explicitly unknown location,
    # OR if charger info is not set at all (unknown = assume unavailable).
    if ctx.charger_known is None and ctx.charger_available is None:
        return True
    return ctx.charger_available is False or ctx.charger_known is False


def _cond_contact_allowed(ctx: ExecutionContext) -> bool:
    return ctx.metadata.get("contact_allowed", True) is True


def _cond_paw_contact_allowed(ctx: ExecutionContext) -> bool:
    return ctx.metadata.get("paw_contact_allowed", True) is True


def _cond_jump_interaction_allowed(ctx: ExecutionContext) -> bool:
    return ctx.metadata.get("jump_interaction_allowed", False) is False


def _cond_gentle_mouthing_allowed(ctx: ExecutionContext) -> bool:
    return ctx.metadata.get("gentle_mouthing_allowed", False) is False


def _cond_food_resource_visible(ctx: ExecutionContext) -> bool:
    return ctx.metadata.get("food_resource_visible", False) is True


def _cond_leash_or_shoe_available(ctx: ExecutionContext) -> bool:
    return ctx.metadata.get("leash_or_shoe_available", False) is True


def _cond_food_in_hand_visible(ctx: ExecutionContext) -> bool:
    return ctx.metadata.get("food_in_hand_visible", False) is True


def _cond_bell_interaction_supported(ctx: ExecutionContext) -> bool:
    return ctx.metadata.get("bell_interaction_supported", False) is True


def _cond_toy_available(ctx: ExecutionContext) -> bool:
    return ctx.metadata.get("toy_available", False) is True


def _cond_carrying_toy(ctx: ExecutionContext) -> bool:
    return ctx.metadata.get("carrying_toy", False) is True


def _cond_object_carryable(ctx: ExecutionContext) -> bool:
    return ctx.metadata.get("object_carryable", False) is True


def _cond_object_safe_for_mouth(ctx: ExecutionContext) -> bool:
    return ctx.metadata.get("object_safe_for_mouth", False) is True


def _cond_person_target_available(ctx: ExecutionContext) -> bool:
    return _cond_human_target_available(ctx)


_COND_BUILTINS = {
    "owner_visible": _cond_owner_visible,
    "target_visible": _cond_target_visible,
    "person_not_visible": _cond_person_not_visible,
    "person_too_far": _cond_person_too_far,
    "contact_distance": _cond_contact_distance,
    "target_moving": _cond_target_moving,
    "target_not_aggressive": _cond_target_not_aggressive,
    "target_distance_valid": _cond_target_distance_valid,
    "animal_target_available": _cond_animal_target_available,
    "human_target_available": _cond_human_target_available,
    "object_target_available": _cond_object_target_available,
    "rolling_food_detected": _cond_rolling_food_detected,
    "elimination_type_pee": _cond_elimination_type_pee,
    "elimination_type_poop": _cond_elimination_type_poop,
    "suitable_rubbing_object_available": _cond_suitable_rubbing_object_available,
    "charger_known": _cond_charger_known,
    "charger_available": _cond_charger_available,
    "charger_unavailable": _cond_charger_unavailable,
    "contact_allowed": _cond_contact_allowed,
    "paw_contact_allowed": _cond_paw_contact_allowed,
    "jump_interaction_allowed": _cond_jump_interaction_allowed,
    "gentle_mouthing_allowed": _cond_gentle_mouthing_allowed,
    "food_resource_visible": _cond_food_resource_visible,
    "leash_or_shoe_available": _cond_leash_or_shoe_available,
    "food_in_hand_visible": _cond_food_in_hand_visible,
    "bell_interaction_supported": _cond_bell_interaction_supported,
    "toy_available": _cond_toy_available,
    "carrying_toy": _cond_carrying_toy,
    "object_carryable": _cond_object_carryable,
    "object_safe_for_mouth": _cond_object_safe_for_mouth,
    "person_target_available": _cond_person_target_available,
    "allow_bite_object": lambda ctx: ctx.metadata.get("allow_bite_object", False) is True,
    "allow_carry_object": lambda ctx: ctx.metadata.get("allow_carry_object", True) is True,
    "allow_hide_object": lambda ctx: ctx.metadata.get("allow_hide_object", False) is True,
    "allow_knock_over_object": lambda ctx: ctx.metadata.get("allow_knock_over_object", False) is True,
    "allow_mouth_contact": lambda ctx: ctx.metadata.get("allow_mouth_contact", False) is True,
    "allow_rummage_trash": lambda ctx: ctx.metadata.get("allow_rummage_trash", False) is True,
    "allow_scratch_door": lambda ctx: ctx.metadata.get("allow_scratch_door", False) is True,
    # ── behavior-tree action conditions ────────────────────────────────
    "person_visible": lambda ctx: _cond_human_target_available(ctx),
    "person_detected": lambda ctx: _cond_human_target_available(ctx),
    "person_nearby": lambda ctx: _cond_contact_distance(ctx),
    "person_to_find": lambda ctx: ctx.metadata.get("person_to_find", True) is True,
    "distance_to_person_gt": lambda ctx: _cond_person_too_far(ctx),
    "other_animal_detected": lambda ctx: _cond_animal_target_available(ctx),
    "other_animal_nearby": lambda ctx: _cond_contact_distance(ctx),
    "playmate_nearby": lambda ctx: _cond_contact_distance(ctx),
    "safe_to_approach": lambda ctx: _cond_target_not_aggressive(ctx),
    "object_detected": lambda ctx: _cond_object_target_available(ctx),
    "object_nearby": lambda ctx: _cond_contact_distance(ctx),
    "distance_to_object_gt": lambda ctx: _cond_person_too_far(ctx),
    "food_bowl_nearby": lambda ctx: ctx.metadata.get("food_bowl_nearby", True) is True,
    "food_in_bowl": lambda ctx: ctx.metadata.get("food_in_bowl", True) is True,
    "human_visible": lambda ctx: _cond_human_target_available(ctx),
    "elimination_ready": lambda ctx: ctx.metadata.get("elimination_ready", True) is True,
    "ground_surface_ok": lambda ctx: ctx.metadata.get("ground_surface_ok", True) is True,
    "flat_surface": lambda ctx: ctx.metadata.get("flat_surface", True) is True,
    "battery_low": lambda ctx: ctx.metadata.get("battery_low", True) is True,
    "charger_location_known": _cond_charger_known,
    "charger_location_unknown": _cond_charger_unavailable,
}

# Register all builtins
for _name, _fn in _COND_BUILTINS.items():
    register_condition(_name, _fn)


# ── EligibilityChecker ────────────────────────────────────────────────────────


class EligibilityChecker:
    """Filters candidates by conditions, posture, safety, and controller capability.

    Selection must be: read candidates → filter → eligible → select.
    Never: select → then discover it cannot execute.
    """

    def __init__(self, action_catalog: dict[str, dict[str, Any]] | None = None) -> None:
        self._catalog = action_catalog or {}

    def filter_candidates(
        self,
        candidates: list[dict[str, Any]],
        ctx: ExecutionContext,
    ) -> list[dict[str, Any]]:
        """Return only eligible candidates after all filters.

        A candidate is dict with optional:
          - unit_id: str
          - conditions: list[str]
          - from_postures: list[str]
        """
        eligible: list[dict[str, Any]] = []
        for candidate in candidates:
            if self._is_eligible(candidate, ctx):
                eligible.append(candidate)
            else:
                unit_id = candidate.get("unit_id", "?")
                logger.debug("Candidate %s filtered out", unit_id)
        return eligible

    def _is_eligible(self, candidate: dict[str, Any], ctx: ExecutionContext) -> bool:
        # ── Check explicit conditions ────────────────────────────────
        raw_conditions = candidate.get("conditions", [])
        if isinstance(raw_conditions, dict):
            # Dict format: {cond_name: expected_value}
            for cond_name, expected in raw_conditions.items():
                if cond_name not in _CONDITION_CHECKERS:
                    logger.error(
                        "Unknown condition %r — assume false for safety", cond_name,
                    )
                    return False
                actual = _CONDITION_CHECKERS[cond_name](ctx)
                if actual != bool(expected):
                    return False
        else:
            # List format: [cond_name, ...] — all must be True
            for cond_name in raw_conditions:
                if cond_name not in _CONDITION_CHECKERS:
                    logger.error(
                        "Unknown condition %r — assume false for safety",
                        cond_name,
                    )
                    return False
                if not _CONDITION_CHECKERS[cond_name](ctx):
                    return False

        # ── Check posture (if declared) ──────────────────────────────
        from_postures: list[str] = candidate.get("from_postures", [])
        if from_postures and ctx.current_posture not in from_postures:
            # Could be eligible if posture transition exists
            if ctx.current_posture == "unknown":
                return True  # unknown → allow
            return False

        # ── Check safety policy ──────────────────────────────────────
        safety = candidate.get("safety", {})
        if safety.get("require_authorization") and not ctx.metadata.get(
            f"allow_{candidate.get('unit_id', '').lower()}", False,
        ):
            return False

        return True

    @staticmethod
    def check_condition(name: str, ctx: ExecutionContext) -> bool:
        """Check a single named condition against the context."""
        checker = _CONDITION_CHECKERS.get(name)
        if checker is None:
            logger.warning("Unknown condition: %r", name)
            return False
        return checker(ctx)
