"""BehaviorResolver — resolves requested behavior_name to canonical form.

Handles:
  - alias resolution (old name → canonical name)
  - injected parameter defaults from alias definitions
  - interaction mode fallback (interactive→solo when no target)
  - level resolution from variant or safe default
"""

from __future__ import annotations

import logging
from typing import Any

from .execution_context import ExecutionContext

logger = logging.getLogger(__name__)

# ── Built-in alias table (fallback if YAML not loaded) ───────────────────────

_BUILTIN_ALIASES: dict[str, dict[str, Any]] = {
    # Core renames
    "seek_food_or_water": {"resolved": "eatNormally", "reason": "core_rename"},
    "excretion_request": {"resolved": "defecate", "reason": "core_rename"},
    "sleep_request": {"resolved": "sleepNow", "reason": "core_rename"},
    "clean_self": {"resolved": "cleanSelf", "reason": "core_rename"},
    "seek_social_interaction": {"resolved": "seekHumanInteraction", "reason": "core_rename"},
    "explore_environment": {"resolved": "exploreRoom", "reason": "core_rename"},
    "respond_owner_call": {"resolved": "seekHumanInteraction", "reason": "core_rename"},
    "respond_touch_head": {"resolved": "seekHumanInteraction", "reason": "core_rename"},
    "emergency_stop": {"resolved": "emergencyStop", "reason": "core_rename"},
    "avoid_danger": {"resolved": "seekSafety", "reason": "core_rename"},
    "idle_look_around": {"resolved": "exploreRoom", "reason": "core_rename"},
    "idle_rest": {"resolved": "restInPlace", "reason": "core_rename"},
    "express_happy": {"resolved": "expressJoy", "reason": "core_rename"},
    "express_fear": {"resolved": "expressFear", "reason": "core_rename"},
    "express_curiosity": {"resolved": "expressCuriosity", "reason": "core_rename"},
    "express_anxiety": {"resolved": "expressAnxiety", "reason": "core_rename"},
    "express_calm": {"resolved": "expressCalm", "reason": "core_rename"},
    "express_excite": {"resolved": "expressExcitement", "reason": "core_rename"},
    "express_joy": {"resolved": "expressJoy", "reason": "core_rename"},
    # Legacy emotion action → emotion behavior
    "wagTailFast": {
        "resolved": "expressJoy",
        "inject": {"level": "MID", "interaction_mode": "interactive"},
        "reason": "legacy_emotion_action_behavior",
    },
    "wagTailGently": {
        "resolved": "expressJoy",
        "inject": {"level": "LOW", "interaction_mode": "interactive"},
        "reason": "legacy_emotion_action_behavior",
    },
    "hopInPlace": {
        "resolved": "expressJoy",
        "inject": {"level": "MID", "interaction_mode": "solo"},
        "reason": "legacy_emotion_action_behavior",
    },
    "headTilt": {
        "resolved": "expressCuriosity",
        "inject": {"level": "LOW", "interaction_mode": "interactive"},
        "reason": "legacy_emotion_action_behavior",
    },
    "yawnSlowly": {
        "resolved": "expressCalm",
        "inject": {"level": "LOW", "interaction_mode": "solo"},
        "reason": "legacy_emotion_action_behavior",
    },
    "hideAway": {
        "resolved": "expressFear",
        "inject": {"level": "HIGH", "interaction_mode": "solo"},
        "reason": "legacy_emotion_action_behavior",
    },
    "playBow": {
        "resolved": "expressJoy",
        "inject": {"level": "MID", "interaction_mode": "interactive"},
        "reason": "legacy_emotion_action_behavior",
    },
    "spinInCircle": {
        "resolved": "expressJoy",
        "inject": {"level": "HIGH", "interaction_mode": "solo"},
        "reason": "legacy_emotion_action_behavior",
    },
    "restInPlace": {
        "resolved": "expressCalm",
        "inject": {"level": "LOW", "interaction_mode": "solo"},
        "reason": "legacy_emotion_action_behavior",
    },
    "freezeAlert": {
        "resolved": "expressFear",
        "inject": {"level": "LOW", "interaction_mode": "solo"},
        "reason": "legacy_emotion_action_behavior",
    },
    "fleeQuickly": {
        "resolved": "expressFear",
        "inject": {"level": "HIGH", "interaction_mode": "solo"},
        "reason": "legacy_emotion_action_behavior",
    },
    "trembleShake": {
        "resolved": "expressFear",
        "inject": {"level": "HIGH", "interaction_mode": "solo"},
        "reason": "legacy_emotion_action_behavior",
    },
    "approachSlowly": {
        "resolved": "expressCuriosity",
        "inject": {"level": "HIGH", "interaction_mode": "interactive"},
        "reason": "legacy_emotion_action_behavior",
    },
    "sniffGround": {
        "resolved": "expressCuriosity",
        "inject": {"level": "HIGH", "interaction_mode": "solo"},
        "reason": "legacy_emotion_action_behavior",
    },
    "pawAtObject": {
        "resolved": "expressCuriosity",
        "inject": {"level": "HIGH", "interaction_mode": "solo"},
        "reason": "legacy_emotion_action_behavior",
    },
    "paceBackAndForth": {
        "resolved": "expressAnxiety",
        "inject": {"level": "LOW", "interaction_mode": "solo"},
        "reason": "legacy_emotion_action_behavior",
    },
    "sleepOnSide": {
        "resolved": "expressCalm",
        "inject": {"level": "LOW", "interaction_mode": "solo"},
        "reason": "legacy_emotion_action_behavior",
    },
    "stretchLazily": {
        "resolved": "expressCalm",
        "inject": {"level": "LOW", "interaction_mode": "solo"},
        "reason": "legacy_emotion_action_behavior",
    },
    "nudgeWithNose": {
        "resolved": "expressJoy",
        "inject": {"level": "MID", "interaction_mode": "interactive"},
        "reason": "legacy_emotion_action_behavior",
    },
    "cuddlePose": {
        "resolved": "expressCalm",
        "inject": {"level": "HIGH", "interaction_mode": "interactive"},
        "reason": "legacy_emotion_action_behavior",
    },
}

_CANONICAL_BEHAVIORS = {
    "eatNormally", "eatExcitedly", "defecate", "cleanSelf", "sleepNow",
    "restInPlace", "recharge",
    "testAnimalBoundary", "greetAnimal", "inviteAnimalToPlay",
    "requestResourceFromHuman", "seekHumanInteraction", "inviteHumanToPlay",
    "exploreRoom", "inspectObject", "inspectKnownObject",
    "expressCalm", "expressJoy", "expressExcitement",
    "expressAnxiety", "expressFear", "expressCuriosity",
    # Also allow these as canonical
    "emergencyStop", "seekSafety", "sit", "comeHere", "walkToOwner",
    "followUser", "handShake", "highFive", "rollOver", "spinOnce",
    "retrieveToy", "spitOut", "playDead", "seekFood", "eatImmediately",
    "lickPaws", "seekInteraction", "approachUser", "inspectToy", "sniffObject",
    "barkShortAlert", "trotToOwner", "sitPolitely",
    "alertAndApproach", "seekInteraction",
}


class BehaviorResolver:
    """Resolves a requested behavior_name to a canonical form.

    Steps:
      1. Check for alias → resolve to canonical name, inject default params.
      2. Validate canonical name is known.
      3. Resolve interaction mode fallback (interactive→solo if no target).
      4. Resolve level from variant if missing.
    """

    def __init__(self, aliases: dict[str, dict[str, Any]] | None = None) -> None:
        self._aliases = dict(_BUILTIN_ALIASES)
        if aliases:
            self._aliases.update(aliases)

    def _resolve_name(self, name: str) -> tuple[str | None, str | None]:
        """Resolve a behavior name without a full ExecutionContext.

        Returns:
            (canonical_name, alias_source) or (None, None) if unresolvable.
        """
        alias = self._aliases.get(name)
        if alias is not None:
            return (alias["resolved"], name)
        if name in _CANONICAL_BEHAVIORS:
            return (name, None)
        return (None, None)

    def resolve(self, ctx: ExecutionContext) -> ExecutionContext:
        """Resolve an ExecutionContext in-place and return it.

        Sets ``resolved_behavior_name``, injects default params,
        resolves interaction fallback, and validates.
        """
        name = ctx.requested_behavior_name

        # ── Step 1: alias lookup ─────────────────────────────────────
        alias = self._aliases.get(name)
        if alias is not None:
            # Accept both YAML-schema keys and legacy code keys
            resolved = alias.get("resolved_behavior_name", alias.get("resolved", ""))
            reason = alias.get("alias_reason", alias.get("reason", "alias"))
            logger.info(
                "behavior alias resolved: %s -> %s (reason: %s)",
                name, resolved, reason,
            )
            ctx.resolved_behavior_name = resolved
            # Inject default params from alias — into both params dict AND typed fields
            inject = alias.get("injected_params", alias.get("inject", {}))
            for k, v in inject.items():
                if k not in ctx.params or ctx.params.get(k) is None:
                    ctx.params[k] = v
                    logger.debug("alias injected param %s=%r", k, v)
            # Re-apply injected params to typed fields
            self._apply_injected_fields(ctx, inject)
            ctx.metadata["alias_reason"] = reason
            ctx.metadata["alias_source"] = name
        else:
            ctx.resolved_behavior_name = name

        canonical = ctx.resolved_behavior_name

        # ── Step 2: validate canonical name ──────────────────────────
        if canonical not in _CANONICAL_BEHAVIORS:
            ctx.is_valid = False
            ctx.error_reason = f"unsupported_behavior: {canonical!r}"
            logger.error("Unsupported behavior: %r (requested: %r)", canonical, name)
            return ctx

        # ── Step 3: interaction fallback ─────────────────────────────
        if ctx.interaction_mode == "interactive" and not ctx.target:
            logger.info(
                "interaction fallback: no valid target, "
                "requested=interactive → resolved=solo (behavior=%r)",
                canonical,
            )
            ctx.interaction_mode = "solo"
            ctx.interactive = False
            ctx.metadata["fallback_reason"] = "no_valid_target"
            ctx.metadata["requested_interaction_mode"] = "interactive"
            ctx.metadata["resolved_interaction_mode"] = "solo"

        # ── Step 4: level from variant ───────────────────────────────
        if ctx.level is None and ctx.variant:
            ctx.level = _level_from_variant(ctx.variant)
            if ctx.level:
                logger.info("level resolved from variant %r → %s", ctx.variant, ctx.level)
            else:
                logger.warning("Could not resolve level from variant %r", ctx.variant)

        return ctx


    @staticmethod
    def _apply_injected_fields(ctx: ExecutionContext, inject: dict[str, Any]) -> None:
        """Apply injected params to typed ExecutionContext fields."""
        if "level" in inject and ctx.level is None:
            from .execution_context import _normalise_level
            ctx.level = _normalise_level(inject["level"])
        if "interaction_mode" in inject or "interactive" in inject:
            from .execution_context import _normalise_interaction
            mode, interactive = _normalise_interaction(
                inject.get("interaction_mode", ctx.interaction_mode),
                inject.get("interactive", ctx.interactive),
            )
            ctx.interaction_mode = mode
            ctx.interactive = interactive


    def get_canonical_behaviors(self) -> set[str]:
        """Return the set of all known canonical behavior names."""
        return set(_CANONICAL_BEHAVIORS)

    def get_alias_names(self) -> set[str]:
        """Return the set of all alias source names."""
        return set(self._aliases.keys())


def _level_from_variant(variant: str) -> str | None:
    """Extract level from variant string like 'joy_mid' → 'MID'."""
    upper = variant.strip().upper()
    for level in ("LOW", "MID", "HIGH"):
        if level in upper:
            return level
    return None
