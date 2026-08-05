"""ExecutionContext — unified input model for behavior execution.

Converts raw params_json from the behavior tree into a structured,
validated context object.  This is the single entry point for all
upstream parameters — no other module should parse params_json directly.

Design rules:
  - Missing optional fields → sensible defaults, no crash.
  - Unknown fields → preserved in metadata, not rejected.
  - Invalid values → logged warning, substituted with safe default.
  - level normalised to UPPERCASE.
  - sleep_depth normalised to "shallow" | "deep" | None.
  - interaction_mode derived from interactive if missing, and vice versa.
  - target non-dict → invalid_params error.
  - intensity validated as numeric.
"""

from __future__ import annotations

import logging
import math
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any

logger = logging.getLogger(__name__)

# ── Standardised values ──────────────────────────────────────────────────────

_VALID_LEVELS = {"LOW", "MID", "HIGH", "TRIGGERED", "OVERFLOW", "RECOVERED"}
_VALID_SLEEP_DEPTHS = {"shallow", "deep"}
_VALID_INTERACTION_MODES = {"solo", "interactive"}


@dataclass
class ExecutionContext:
    """All parameters needed to resolve and execute a behavior.

    Construct via :func:`ExecutionContext.from_goal` — never directly
    from raw JSON.

    Attributes:
        requested_behavior_name: As received from the behavior tree.
        resolved_behavior_name: Validated direct name (set by BehaviorResolver).
        params: Normalised dict of all upstream parameters.
        source: "need", "emotion", "audio", "external", or None.
        trigger_event: e.g. "NEED_HUNGER_TRIGGERED".
        intent: e.g. "eat_normal".
        priority_level: 0-6.
        sub_priority: Tie-breaker within same priority.
        variant: e.g. "joy_mid".
        level: LOW | MID | HIGH (normalised to uppercase).
        intensity: 0-100 numeric value.
        interaction_mode: "solo" | "interactive".
        interactive: bool derived from interaction_mode.
        target: Optional target dict (human / animal / object).
        object_category: slippers_or_socks, trash_can, etc.
        sleep_depth: "shallow" | "deep" | None.
        elimination_type: "pee" | "poop" | None.
        charger_known: bool | None.
        charger_available: bool | None.
        use_wake_angle: Whether the wake-source angle must drive chassis yaw.
        wake_angle_deg: Raw relative sound-source angle in degrees.
        wake_confidence: Upstream wake detector confidence/score.
        wake_frame_id: Coordinate frame for wake_angle_deg (base_link).
        current_stage: Updated during execution.
        current_unit: Updated during execution.
        current_posture: Tracked by PostureManager.
        speed_scale: Modifier.
        acceleration_scale: Modifier.
        amplitude_scale: Modifier.
        duration_scale: Modifier.
        cancel_requested: Set by InterruptManager.
        safe_to_interrupt: Current interrupt safety.
        completed_stages: Accumulated during execution.
        executed_units: Accumulated during execution.
        metadata: Catch-all for unknown / passthrough fields.
        is_valid: False if params_json was unparseable or critically invalid.
        error_reason: Explanation if is_valid is False.
    """

    requested_behavior_name: str
    resolved_behavior_name: str = ""

    params: dict[str, Any] = field(default_factory=dict)

    source: str | None = None
    trigger_event: str | None = None
    intent: str | None = None

    priority_level: int = 5
    sub_priority: int = 0

    variant: str | None = None
    level: str | None = None
    intensity: float | None = None

    interaction_mode: str = "solo"
    interactive: bool = False
    target: dict[str, Any] | None = None

    object_category: str | None = None
    sleep_depth: str | None = None
    elimination_type: str | None = None

    charger_known: bool | None = None
    charger_available: bool | None = None

    use_wake_angle: bool = False
    wake_angle_deg: float | None = None
    wake_confidence: float | None = None
    wake_frame_id: str | None = None

    current_stage: str | None = None
    current_unit: str | None = None
    current_posture: str = "unknown"

    speed_scale: float = 1.0
    acceleration_scale: float = 1.0
    amplitude_scale: float = 1.0
    duration_scale: float = 1.0

    cancel_requested: bool = False
    safe_to_interrupt: bool = True

    completed_stages: list[str] = field(default_factory=list)
    executed_units: list[str] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)

    is_valid: bool = True
    error_reason: str = ""

    # ── factory ──────────────────────────────────────────────────────────

    @classmethod
    def from_goal(
        cls,
        behavior_name: str,
        params_json: str | dict | None = None,
        seed: int | None = None,
    ) -> ExecutionContext:
        """Build an ExecutionContext from a raw behavior goal.

        Args:
            behavior_name: As received in the Action goal.
            params_json: Raw JSON string or dict from params_json field.
            seed: Optional random seed (from params_json.random_seed or external).

        Returns:
            An ExecutionContext.  Check ``.is_valid`` before use.
        """
        params = _safe_parse_params(params_json)

        if params is None:
            return cls(
                requested_behavior_name=behavior_name,
                is_valid=False,
                error_reason="invalid_params: params_json parse failed",
            )

        # ── Normalise ────────────────────────────────────────────────
        level = _normalise_level(params.get("level"))
        sleep_depth = _normalise_sleep_depth(params.get("sleep_depth"))
        interaction_mode, interactive = _normalise_interaction(
            params.get("interaction_mode"),
            params.get("interactive"),
        )
        intensity = _normalise_intensity(params.get("intensity"))
        target = _normalise_target(params.get("target"))

        # ── Random seed priority ─────────────────────────────────────
        effective_seed = seed
        if effective_seed is None and "random_seed" in params:
            try:
                effective_seed = int(params["random_seed"])
            except (ValueError, TypeError):
                logger.warning("Invalid random_seed in params: %r", params["random_seed"])

        return cls(
            requested_behavior_name=behavior_name,
            params=deepcopy(params),
            source=params.get("source"),
            trigger_event=params.get("trigger_event"),
            intent=params.get("intent"),
            priority_level=int(params.get("priority_level", 5)),
            sub_priority=int(params.get("sub_priority", 0)),
            variant=params.get("variant"),
            level=level,
            intensity=intensity,
            interaction_mode=interaction_mode,
            interactive=interactive,
            target=target,
            object_category=params.get("object_category"),
            sleep_depth=sleep_depth,
            elimination_type=params.get("elimination_type"),
            charger_known=_normalise_bool(params.get("charger_known")),
            charger_available=_normalise_bool(params.get("charger_available")),
            use_wake_angle=bool(
                _normalise_bool(params.get("use_wake_angle"))
            ),
            wake_angle_deg=_normalise_finite_float(
                params.get("wake_angle_deg"),
                "wake_angle_deg",
            ),
            wake_confidence=_normalise_finite_float(
                params.get("wake_confidence"),
                "wake_confidence",
            ),
            wake_frame_id=_normalise_nonempty_string(
                params.get("wake_frame_id")
            ),
            metadata={
                k: v for k, v in params.items()
                if k not in _KNOWN_PARAMS
            },
        )


# ── Internal helpers ─────────────────────────────────────────────────────────

_KNOWN_PARAMS = {
    "schema_version", "source", "trigger_event", "intent",
    "priority_level", "sub_priority", "intensity", "level", "variant",
    "interaction_mode", "interactive", "target",
    "object_category", "sleep_depth", "elimination_type",
    "charger_known", "charger_available", "random_seed",
    "use_wake_angle", "wake_angle_deg", "wake_confidence",
    "wake_frame_id",
}


def _safe_parse_params(raw: str | dict | None) -> dict[str, Any] | None:
    """Parse params_json safely.  Returns None on failure."""
    import json as _json

    if raw is None:
        return {}
    if isinstance(raw, dict):
        return raw
    if isinstance(raw, str):
        if not raw.strip():
            return {}
        try:
            return _json.loads(raw)
        except _json.JSONDecodeError as exc:
            logger.error("Failed to parse params_json: %s", exc)
            return None
    return {}


def _normalise_level(value: Any) -> str | None:
    if value is None:
        return None
    if isinstance(value, str):
        upper = value.strip().upper()
        if upper in _VALID_LEVELS:
            return upper
        logger.warning("Unknown level %r — preserved as-is", value)
        return upper
    return None


def _normalise_sleep_depth(value: Any) -> str | None:
    if value is None:
        return None
    if isinstance(value, str):
        lower = value.strip().lower()
        if lower in _VALID_SLEEP_DEPTHS:
            return lower
        if lower in ("light", "nap"):
            logger.info("sleep_depth %r → shallow", value)
            return "shallow"
        if lower in ("heavy", "rem"):
            logger.info("sleep_depth %r → deep", value)
            return "deep"
        logger.warning("Unknown sleep_depth %r", value)
        return "shallow"
    return None


def _normalise_interaction(
    mode: Any, interactive: Any,
) -> tuple[str, bool]:
    """Derive interaction_mode and interactive from each other."""
    if isinstance(mode, str) and mode.strip().lower() in _VALID_INTERACTION_MODES:
        resolved_mode = mode.strip().lower()
        resolved_bool = resolved_mode == "interactive"
        return resolved_mode, resolved_bool
    if isinstance(interactive, bool):
        if interactive:
            return "interactive", True
        return "solo", False
    if isinstance(interactive, (int, float)):
        return ("interactive", True) if interactive else ("solo", False)
    # Both missing or invalid
    return "solo", False


def _normalise_intensity(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (ValueError, TypeError):
        logger.warning("Invalid intensity %r", value)
        return None


def _normalise_finite_float(value: Any, name: str) -> float | None:
    if value is None:
        return None
    try:
        result = float(value)
    except (ValueError, TypeError):
        logger.warning("Invalid %s %r", name, value)
        return None
    if not math.isfinite(result):
        logger.warning("Invalid %s %r", name, value)
        return None
    return result


def _normalise_nonempty_string(value: Any) -> str | None:
    if value is None:
        return None
    result = str(value).strip()
    return result or None


def _normalise_target(value: Any) -> dict[str, Any] | None:
    if value is None:
        return None
    if isinstance(value, dict):
        return dict(value)
    logger.warning("target is not a dict: %r", value)
    return None


def _normalise_bool(value: Any) -> bool | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("true", "1", "yes")
    if isinstance(value, (int, float)):
        return bool(value)
    return None
