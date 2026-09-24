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
from typing import Any, Callable

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
        interaction_id: Optional upstream voice-session identifier.
        command_key: Stable key from the voice command catalog.
        command_id: Stable command identity from AudioEvent v2.
        command_catalog_version: Voice catalog version used for recognition.
        intent_source: Voice route source, such as command_lexicon or rkllm.
        dispatch_role: AudioEvent v2 dispatch role.
        specific_event_type: Exact executable voice event type.
        voice_slots: Audited voice slots forwarded by the behavior tree.
        mobility_policy: Optional chassis policy requested by the caller.
        object_category: slippers_or_socks, trash_can, etc.
        sleep_depth: "shallow" | "deep" | None.
        elimination_type: "pee" | "poop" | None.
        charger_known: bool | None.
        charger_available: bool | None.
        use_wake_angle: Whether the wake-source angle must drive chassis yaw.
        wake_angle_deg: Raw relative sound-source angle in degrees.
        wake_confidence: Upstream wake detector confidence/score.
        wake_frame_id: Raw ``microphone_array`` frame for wake_angle_deg;
            Action applies installation calibration and converts it to a
            ``base_link`` relative yaw.
        current_stage: Updated during execution.
        motion_state: Chassis policy for the current stage. ``active`` allows
            configured motion; ``stationary`` forces zero velocity.
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
    interaction_id: str | None = None
    command_key: str | None = None
    command_id: str | None = None
    command_catalog_version: str | None = None
    intent_source: str | None = None
    dispatch_role: str | None = None
    specific_event_type: str | None = None
    voice_slots: dict[str, str] = field(default_factory=dict)
    mobility_policy: str | None = None

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
    motion_state: str = "active"
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

    # Runtime-only callback installed by the ROS Action server while a Stage is
    # running.  It is deliberately not part of params/metadata serialization.
    runtime_feedback: (
        Callable[[float, str, str, bool], None] | None
    ) = field(default=None, repr=False, compare=False)
    runtime_deadline_monotonic: float | None = field(
        default=None,
        repr=False,
        compare=False,
    )

    is_valid: bool = True
    error_reason: str = ""

    def report_runtime_feedback(
        self,
        progress: float,
        action_id: str,
        message: str,
        safe_to_interrupt: bool = True,
    ) -> None:
        """Emit task progress when the ROS runtime installed a callback."""
        callback = self.runtime_feedback
        if callback is None:
            return
        callback(
            _clamp_progress(progress),
            str(action_id),
            str(message),
            bool(safe_to_interrupt),
        )

    def remaining_runtime_sec(self, now_monotonic: float) -> float | None:
        """Return the remaining ROS goal budget for a synchronous unit."""
        if self.runtime_deadline_monotonic is None:
            return None
        return max(0.0, self.runtime_deadline_monotonic - now_monotonic)

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
        params = parse_params_json_object(params_json)

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
            interaction_id=_normalise_nonempty_string(
                params.get("interaction_id")
            ),
            command_key=_normalise_nonempty_string(params.get("command_key")),
            command_id=_normalise_nonempty_string(params.get("command_id")),
            command_catalog_version=_normalise_nonempty_string(
                params.get("command_catalog_version")
            ),
            intent_source=_normalise_nonempty_string(
                params.get("intent_source")
            ),
            dispatch_role=_normalise_nonempty_string(
                params.get("dispatch_role")
            ),
            specific_event_type=_normalise_nonempty_string(
                params.get("specific_event_type")
            ),
            voice_slots=_normalise_string_mapping(params.get("voice_slots")),
            mobility_policy=_normalise_nonempty_string(
                params.get("mobility_policy")
            ),
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
    "interaction_mode", "interactive", "target", "interaction_id",
    "command_key", "command_id", "command_catalog_version",
    "intent_source", "dispatch_role", "specific_event_type", "voice_slots",
    "mobility_policy",
    "object_category", "sleep_depth", "elimination_type",
    "charger_known", "charger_available", "random_seed",
    "use_wake_angle", "wake_angle_deg", "wake_confidence",
    "wake_frame_id",
}


def _clamp_progress(value: Any) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(result):
        return 0.0
    return max(0.0, min(1.0, result))


def parse_params_json_object(
    raw: str | dict | None,
) -> dict[str, Any] | None:
    """Parse ``params_json`` as a JSON object, returning ``None`` on failure.

    This is the single parser used at both the ROS goal boundary and by
    :class:`ExecutionContext`.  JSON arrays, scalars and ``null`` are invalid
    because all downstream parameter handling requires a mapping.
    """
    import json as _json

    if raw is None:
        return {}
    if isinstance(raw, dict):
        return raw
    if isinstance(raw, str):
        if not raw.strip():
            return {}
        try:
            parsed = _json.loads(raw)
        except (_json.JSONDecodeError, RecursionError) as exc:
            logger.error("Failed to parse params_json: %s", exc)
            return None
        if not isinstance(parsed, dict):
            logger.error(
                "Failed to parse params_json: top-level value must be an object"
            )
            return None
        return parsed
    logger.error(
        "Failed to parse params_json: unsupported input type %s",
        type(raw).__name__,
    )
    return None


# Private compatibility name retained for local harnesses; both names resolve
# to the same implementation, so there remains exactly one parsing policy.
_safe_parse_params = parse_params_json_object


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


def _normalise_string_mapping(value: Any) -> dict[str, str]:
    if not isinstance(value, dict):
        return {}
    return {
        key: item.strip()
        for key, item in value.items()
        if isinstance(key, str) and isinstance(item, str) and item.strip()
    }


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
