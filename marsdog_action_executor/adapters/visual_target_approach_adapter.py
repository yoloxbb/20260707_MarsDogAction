"""Safe one-shot approach to a selected visual human, animal, or object.

The Behavior Tree already selects the semantic target.  This adapter binds
that reference to one live detector track, converts the generic visual stream
to the strict input consumed by :class:`TargetApproachAdapter`, and never
selects a different target after execution starts.
"""

from __future__ import annotations

import copy
import logging
import math
import threading
import time
import uuid
from typing import Any, Callable, Mapping

from .velocity import TwistCommand
from .target_approach_adapter import (
    TargetApproachAdapter,
    TargetApproachResult,
)


logger = logging.getLogger(__name__)


class VisualTargetApproachAdapter:
    """Bind one Tree-selected target and approach it with a configured range mode."""

    ACTION_ID = "ACT_APPROACH_VISUAL_TARGET"
    OWNER_APPROACH_ACTION_IDS = frozenset({
        "ACT_INTERACT_APPROACH_OWNER",
        "ACT_INTERACT_APPROACH_OWNER_CLOSER",
        "ACT_INTERACT_RETURN_OWNER",
    })
    ACCEPTED_ACTION_IDS = frozenset({ACTION_ID}) | OWNER_APPROACH_ACTION_IDS
    _TARGET_TYPES = frozenset({"human", "animal", "object"})
    _ANIMAL_LABELS = frozenset({"cat", "dog"})

    def __init__(
        self,
        publish_twist: Callable[[TwistCommand], None],
        *,
        behavior_policies: Mapping[str, Mapping[str, Any]],
        publish_rate_hz: float = 10.0,
        min_confidence: float = 0.5,
        visual_timeout_sec: float = 0.8,
        acquire_timeout_sec: float = 3.0,
        lost_timeout_sec: float = 1.0,
        approach_timeout_sec: float = 20.0,
        minimum_stop_distance_m: float = 0.5,
        minimum_safe_distance_m: float = 0.45,
        distance_deadband_m: float = 0.12,
        distance_hysteresis_m: float = 0.10,
        arrival_hold_sec: float = 0.4,
        linear_gain: float = 0.5,
        angular_gain: float = 0.8,
        max_linear_x: float = 0.12,
        max_angular_z: float = 0.3,
        max_linear_accel: float = 0.18,
        max_angular_accel: float = 0.3,
        heading_deadband: float = 0.06,
        max_heading_error: float = 0.25,
        allow_bbox_distance_fallback: bool = False,
        bbox_target_height: float = 0.68,
        bbox_height_deadband: float = 0.05,
        bbox_height_hysteresis: float = 0.03,
        reacquire_min_consecutive_frames: int = 3,
        stop_publish_count: int = 3,
        should_stop: Callable[[], bool] | None = None,
        require_object_stream: bool = False,
        start_object_detection: (
            Callable[[str, list[str], float, float], tuple[bool, str]]
            | None
        ) = None,
        stop_object_detection: Callable[[str], None] | None = None,
        object_detection_rate_hz: float = 4.0,
        object_detection_lease_margin_sec: float = 2.0,
        monotonic=None,
        sleep=None,
    ) -> None:
        self._policies = {
            str(name): dict(policy)
            for name, policy in behavior_policies.items()
            if isinstance(policy, Mapping)
        }
        self._bind_min_confidence = float(min_confidence)
        self._bind_max_age_ms = float(visual_timeout_sec) * 1000.0
        self._acquire_timeout_sec = max(0.05, float(acquire_timeout_sec))
        self._approach_timeout_sec = max(0.05, float(approach_timeout_sec))
        self._arrival_hold_sec = max(0.0, float(arrival_hold_sec))
        self._linear_gain = max(0.0, float(linear_gain))
        self._max_linear_x = max(0.001, float(max_linear_x))
        self._max_linear_accel = max(0.001, float(max_linear_accel))
        self._bbox_target_height = float(bbox_target_height)
        self._bbox_height_deadband = max(
            0.0,
            float(bbox_height_deadband),
        )
        self._should_stop = should_stop or (lambda: False)
        self._require_object_stream = bool(require_object_stream)
        self._start_object_detection = start_object_detection
        self._stop_object_detection = stop_object_detection
        self._object_detection_rate_hz = max(
            0.1, float(object_detection_rate_hz)
        )
        self._object_detection_lease_margin_sec = max(
            0.5, float(object_detection_lease_margin_sec)
        )
        self._monotonic = monotonic or time.monotonic
        self._sleep = sleep or time.sleep
        self._lock = threading.Lock()
        self._cancel_requested = threading.Event()
        self._latest_payload: dict[str, Any] | None = None
        self._bound_type = ""
        self._bound_target_id = ""
        self._bound_epoch = ""
        self._bound_distance_mode = "metric"
        self._object_stream_session_id = ""
        self._object_stream_epoch = ""

        options: dict[str, Any] = {
            "publish_rate_hz": publish_rate_hz,
            "min_confidence": min_confidence,
            "visual_timeout_sec": visual_timeout_sec,
            "acquire_timeout_sec": acquire_timeout_sec,
            "lost_timeout_sec": lost_timeout_sec,
            "approach_timeout_sec": approach_timeout_sec,
            "stop_distance_m": minimum_stop_distance_m,
            "minimum_safe_distance_m": minimum_safe_distance_m,
            "distance_deadband_m": distance_deadband_m,
            "distance_hysteresis_m": distance_hysteresis_m,
            "arrival_hold_sec": arrival_hold_sec,
            "linear_gain": linear_gain,
            "angular_gain": angular_gain,
            "max_linear_x": max_linear_x,
            "max_angular_z": max_angular_z,
            "max_linear_accel": max_linear_accel,
            "max_angular_accel": max_angular_accel,
            "heading_deadband": heading_deadband,
            "max_heading_error": max_heading_error,
            "allow_bbox_distance_fallback": allow_bbox_distance_fallback,
            "demo_target_height": bbox_target_height,
            "demo_height_deadband": bbox_height_deadband,
            "bbox_height_hysteresis": bbox_height_hysteresis,
            "reacquire_min_consecutive_frames": (
                reacquire_min_consecutive_frames
            ),
            "stop_publish_count": stop_publish_count,
            "action_id": self.ACTION_ID,
            "should_stop": should_stop,
        }
        if monotonic is not None:
            options["monotonic"] = monotonic
        if sleep is not None:
            options["sleep"] = sleep
        self._delegate = TargetApproachAdapter(publish_twist, **options)

    def update_visual(
        self,
        payload: Mapping[str, Any],
        now: float | None = None,
    ) -> bool:
        """Cache a valid visual snapshot and feed normalized tracks downstream."""
        with self._lock:
            if self._object_stream_session_id:
                # Animal/object motion consumes only the session-bearing v2
                # stream.  The short-lived visual_event mirror has no stream
                # owner/error boundary and must not overwrite it.
                return False
        return self._update_visual_snapshot(payload, now=now)

    def _update_visual_snapshot(
        self,
        payload: Mapping[str, Any],
        *,
        now: float | None = None,
    ) -> bool:
        if not isinstance(payload, Mapping):
            return self._delegate.update_visual({}, now=now)
        snapshot = dict(payload)
        normalized = self._normalized_payload(snapshot)
        accepted = self._delegate.update_visual(normalized, now=now)
        if accepted:
            with self._lock:
                self._latest_payload = snapshot
        return accepted

    def update_object_detection(
        self,
        payload: Mapping[str, Any],
        now: float | None = None,
    ) -> bool:
        """Consume one session-matched object-detection v2 packet."""
        if not isinstance(payload, Mapping):
            return False
        with self._lock:
            session_id = self._object_stream_session_id
            expected_epoch = self._object_stream_epoch
        if not session_id or payload.get("schema_version") != 2:
            return False
        stream = payload.get("stream")
        if not isinstance(stream, Mapping):
            return False
        if self._text(stream.get("session_id")) != session_id:
            return False

        source = str(payload.get("source", ""))
        status = str(payload.get("status", ""))
        objects = payload.get("objects")
        if source != "stream" or status != "ok" or not isinstance(objects, list):
            # A matching terminal/error packet invalidates the motion fact and
            # stops the delegate.  Foreign and single-shot service packets are
            # ignored above and can never enter this path.
            header = payload.get("header")
            if not isinstance(header, Mapping):
                return False
            return self._update_visual_snapshot(
                {
                    "schema_version": 1,
                    "header": dict(header),
                    "vision_epoch": expected_epoch,
                    "sequence": int(payload.get("sequence", 0) or 0),
                    "snapshot_id": (
                        f"{expected_epoch}:object-result:"
                        f"{int(payload.get('sequence', 0) or 0)}"
                    ),
                    "tracked_objects": [],
                },
                now=now,
            )

        candidates = [dict(item) for item in objects if isinstance(item, Mapping)]
        packet_epoch = next(
            (
                self._text(item.get("vision_epoch"))
                for item in candidates
                if self._text(item.get("vision_epoch")) is not None
            ),
            expected_epoch,
        )
        if expected_epoch and packet_epoch and packet_epoch != expected_epoch:
            self._delegate.cancel_task()
            return False
        header = payload.get("header")
        if not isinstance(header, Mapping):
            return False
        sequence = int(payload.get("sequence", 0) or 0)
        return self._update_visual_snapshot(
            {
                "schema_version": 1,
                "header": dict(header),
                "vision_epoch": packet_epoch or expected_epoch,
                "sequence": sequence,
                "snapshot_id": (
                    f"{packet_epoch or expected_epoch}:object-result:{sequence}"
                ),
                "tracked_objects": candidates,
            },
            now=now,
        )

    def execute_task(
        self,
        unit_config: Mapping[str, Any],
        ctx: Any,
        timeout_sec: float,
    ) -> TargetApproachResult:
        """Run one target approach and own any required object stream."""
        behavior_name = str(getattr(ctx, "resolved_behavior_name", ""))
        policy = self._policies.get(behavior_name)
        target = getattr(ctx, "target", None)
        target_type = (
            self._normalize_target_type(target.get("target_type"))
            if isinstance(target, Mapping)
            else None
        )
        expected_type = (
            self._normalize_target_type(policy.get("target_type"))
            if isinstance(policy, Mapping)
            else None
        )
        needs_object_stream = (
            str(unit_config.get("unit_id", "")) == self.ACTION_ID
            and target_type in {"animal", "object"}
            and target_type == expected_type
            and (
                self._require_object_stream
                or self._start_object_detection is not None
            )
        )
        session_id = ""
        if needs_object_stream:
            assert isinstance(target, Mapping)
            session_id, reason = self._begin_object_stream(
                target,
                timeout_sec,
            )
            if not session_id:
                return self._invalid(ctx, reason)
        try:
            return self._execute_task_impl(unit_config, ctx, timeout_sec)
        finally:
            if session_id:
                self._end_object_stream(session_id)

    def can_execute(self, unit_id: str) -> bool:
        return str(unit_id) in self.ACCEPTED_ACTION_IDS

    def execute_step(
        self,
        unit_config: Mapping[str, Any],
        ctx: Any,
        duration: float | None = None,
    ) -> bool:
        """Atomic-adapter facade for Go2 owner-approach route overrides."""
        outcome = self.execute_task(
            unit_config,
            ctx,
            max(0.0, float(duration or 0.0)),
        )
        if outcome.success:
            return True
        if outcome.canceled:
            return False
        raise RuntimeError(outcome.reason)

    def _execute_task_impl(
        self,
        unit_config: Mapping[str, Any],
        ctx: Any,
        timeout_sec: float,
    ) -> TargetApproachResult:
        """Bind the requested reference before allowing any non-zero Twist."""
        unit_id = str(unit_config.get("unit_id", ""))
        if unit_id not in self.ACCEPTED_ACTION_IDS:
            return self._invalid(ctx, f"unsupported_action:{unit_id}")

        behavior_name = str(getattr(ctx, "resolved_behavior_name", ""))
        policy = self._policies.get(behavior_name)
        if policy is None:
            return self._invalid(ctx, "visual_target_behavior_not_allowed")

        target = getattr(ctx, "target", None)
        if not isinstance(target, Mapping):
            return self._invalid(ctx, "visual_target_required")
        target_type = self._normalize_target_type(target.get("target_type"))
        expected_type = self._normalize_target_type(policy.get("target_type"))
        if target_type is None or target_type != expected_type:
            return self._invalid(ctx, "visual_target_type_mismatch")
        required_identity = self._normalize_label(
            policy.get("required_identity")
        )
        requested_identity = self._normalize_label(target.get("identity"))
        if (
            required_identity is not None
            and requested_identity != required_identity
        ):
            return self._invalid(ctx, "visual_target_identity_mismatch")
        target_id = self._text(target.get("target_id"))
        if target_id is None:
            return self._invalid(ctx, "visual_target_id_required")
        target_min_confidence = self._finite(
            policy.get("min_confidence", self._bind_min_confidence)
        )
        if (
            target_min_confidence is None
            or not 0.0 <= target_min_confidence <= 1.0
        ):
            return self._invalid(ctx, "invalid_visual_target_policy")

        requested_epoch = self._text(target.get("vision_epoch"))
        # A pose inference may be between frames exactly when Tree dispatches
        # the Goal. Keep the base stopped and allow the immutable semantic
        # target a short acquisition window instead of rejecting a
        # temporarily_lost active track immediately.
        self._cancel_requested.clear()
        self._delegate.cancel_task()
        acquisition_deadline = self._monotonic() + min(
            self._acquire_timeout_sec,
            max(0.05, float(timeout_sec)),
        )
        latest: dict[str, Any] | None = None
        bound: dict[str, Any] | None = None
        reason = "visual_target_not_found"
        payload_epoch: str | None = None
        acquisition_wait_logged = False
        while True:
            if self._cancel_requested.is_set() or self._should_stop():
                return self._canceled(ctx, "visual_target_acquire_canceled")
            with self._lock:
                latest = (
                    dict(self._latest_payload)
                    if self._latest_payload is not None
                    else None
                )
            if latest is not None:
                payload_epoch = self._text(latest.get("vision_epoch"))
                if payload_epoch is None:
                    reason = "visual_target_epoch_required"
                elif (
                    requested_epoch is not None
                    and requested_epoch != payload_epoch
                ):
                    return self._invalid(ctx, "visual_target_epoch_mismatch")
                else:
                    bound, reason = self._bind_target(
                        latest,
                        target,
                        target_type,
                        target_min_confidence,
                    )
                    if bound is not None:
                        break
                    if reason not in {"", "visual_target_not_found"}:
                        return self._invalid(ctx, reason)
            else:
                reason = "visual_target_snapshot_required"
            if self._monotonic() >= acquisition_deadline:
                break
            if not acquisition_wait_logged:
                logger.info(
                    "Waiting for visual target reacquisition: behavior=%s "
                    "type=%s requested_id=%s timeout=%.2fs %s",
                    behavior_name,
                    target_type,
                    target_id,
                    min(
                        self._acquire_timeout_sec,
                        max(0.05, float(timeout_sec)),
                    ),
                    self._binding_diagnostics(
                        latest or {}, target_type, target_min_confidence
                    ),
                )
                acquisition_wait_logged = True
            self._sleep(0.05)

        if bound is None:
            logger.warning(
                "Visual target bind rejected: behavior=%s type=%s "
                "requested_id=%s identity=%s count=%s reason=%s %s",
                behavior_name,
                target_type,
                target_id,
                self._text(target.get("identity")) or "",
                target.get("count"),
                reason,
                self._binding_diagnostics(
                    latest or {}, target_type, target_min_confidence
                ),
            )
            return self._invalid(ctx, reason)
        assert latest is not None
        assert payload_epoch is not None
        canonical_id = bound["target_id"]
        binding_reason = str(bound.get("binding_reason", "exact_track"))
        logger.info(
            "Visual target bound: behavior=%s type=%s requested_id=%s "
            "bound_epoch=%s bound_id=%s reason=%s",
            behavior_name,
            target_type,
            target_id,
            payload_epoch,
            canonical_id,
            binding_reason,
        )
        desired = self._positive(policy.get("desired_distance_m"))
        minimum_safe = self._positive(policy.get("minimum_safe_distance_m"))
        if desired is None or minimum_safe is None or desired < minimum_safe:
            return self._invalid(ctx, "invalid_visual_target_policy")
        target_max_age_ms = self._positive(
            policy.get("target_max_age_ms", 500.0)
        )
        target_lost_timeout = self._positive(
            policy.get("target_lost_timeout_sec", 0.8)
        )
        if target_max_age_ms is None or target_lost_timeout is None:
            return self._invalid(ctx, "invalid_visual_target_policy")
        distance_mode = str(policy.get("distance_mode", "metric")).strip()
        if distance_mode not in {"metric", "metric_then_bbox", "bbox_height"}:
            return self._invalid(ctx, "invalid_visual_target_distance_mode")
        allow_bbox = distance_mode in {"metric_then_bbox", "bbox_height"}

        # Store the immutable binding and mode before re-normalizing the bound
        # frame.  In
        # bbox_height mode an available (but potentially misleading) depth
        # sample must not override the detector-size controller requested by
        # policy.
        with self._lock:
            self._bound_type = target_type
            self._bound_target_id = canonical_id
            self._bound_epoch = payload_epoch
            self._bound_distance_mode = distance_mode
        self._delegate.update_visual(self._normalized_payload(latest))

        proxy = copy.copy(ctx)
        proxy.params = {
            **dict(getattr(ctx, "params", {}) or {}),
            "strict_target_lock": True,
            "allow_target_switch": False,
            "desired_distance_m": desired,
            "minimum_safe_distance_m": minimum_safe,
            "target_max_age_ms": target_max_age_ms,
            "target_lost_timeout_sec": target_lost_timeout,
            "target_min_confidence": target_min_confidence,
            "speaker_allow_bbox_distance_fallback": allow_bbox,
            "arrival_hold_sec": self._policy_nonnegative(
                policy,
                "arrival_hold_sec",
                self._arrival_hold_sec,
            ),
            "approach_linear_gain": self._policy_nonnegative(
                policy,
                "linear_gain",
                self._linear_gain,
            ),
            "approach_max_linear_x": self._policy_positive(
                policy,
                "max_linear_x",
                self._max_linear_x,
            ),
            "approach_max_linear_accel": self._policy_positive(
                policy,
                "max_linear_accel",
                self._max_linear_accel,
            ),
            "bbox_target_height": self._policy_positive(
                policy,
                "bbox_target_height",
                self._bbox_target_height,
            ),
            "bbox_height_deadband": self._policy_nonnegative(
                policy,
                "bbox_height_deadband",
                self._bbox_height_deadband,
            ),
        }
        proxy.interaction_id = (
            self._text(getattr(ctx, "interaction_id", None))
            or f"visual-target:{behavior_name}:{canonical_id}"
        )
        proxy.target = {
            "target_type": "human",
            "vision_epoch": payload_epoch,
            "target_id": canonical_id,
        }
        proxy._allow_behavior_policy_overrides = True
        proxy.metadata = {}

        try:
            delegate_unit_config = dict(unit_config)
            delegate_unit_config["unit_id"] = self.ACTION_ID
            outcome = self._delegate.execute_task(
                delegate_unit_config,
                proxy,
                timeout_sec,
            )
            metadata = dict(outcome.metadata)
            metadata.update(
                {
                    "target_type": target_type,
                    "requested_target_id": target_id,
                    "bound_target_id": canonical_id,
                    "binding_reason": binding_reason,
                    "behavior_name": behavior_name,
                    "configured_distance_mode": distance_mode,
                }
            )
            self._store_metadata(ctx, metadata)
            return TargetApproachResult(
                success=outcome.success,
                reason=outcome.reason,
                metadata=metadata,
                canceled=outcome.canceled,
                timed_out=outcome.timed_out,
            )
        finally:
            with self._lock:
                self._bound_type = ""
                self._bound_target_id = ""
                self._bound_epoch = ""
                self._bound_distance_mode = "metric"

    def _begin_object_stream(
        self,
        target: Mapping[str, Any],
        timeout_sec: float,
    ) -> tuple[str, str]:
        if self._start_object_detection is None:
            if self._require_object_stream:
                return "", "object_detection_stream_unavailable"
            return "", ""
        label = (
            self._normalize_label(target.get("label"))
            or self._normalize_label(target.get("species"))
        )
        if label is None:
            target_id = self._meaningful_identifier(target.get("target_id"))
            if target_id is not None and ":" not in target_id:
                label = self._normalize_label(target_id)
        if label is None:
            return "", "object_target_label_required"

        session_id = f"visual-approach-{uuid.uuid4().hex}"
        requested_epoch = self._text(target.get("vision_epoch")) or ""
        with self._lock:
            self._object_stream_session_id = session_id
            self._object_stream_epoch = requested_epoch
            self._latest_payload = None
        lease_sec = min(
            30.0,
            max(
                3.0,
                min(float(timeout_sec), self._approach_timeout_sec)
                + self._object_detection_lease_margin_sec,
            ),
        )
        try:
            started, reason = self._start_object_detection(
                session_id,
                [label],
                self._object_detection_rate_hz,
                lease_sec,
            )
        except Exception as exc:
            logger.exception("Failed to start object detection stream")
            started, reason = False, f"object_detection_start_error:{exc}"
        if started:
            logger.info(
                "Object detection stream started: session=%s labels=%s "
                "rate=%.2fHz lease=%.2fs",
                session_id,
                [label],
                self._object_detection_rate_hz,
                lease_sec,
            )
            return session_id, ""
        with self._lock:
            self._object_stream_session_id = ""
            self._object_stream_epoch = ""
            self._latest_payload = None
        return "", reason or "object_detection_stream_start_failed"

    def _end_object_stream(self, session_id: str) -> None:
        self._delegate.cancel_task()
        with self._lock:
            if self._object_stream_session_id == session_id:
                self._object_stream_session_id = ""
                self._object_stream_epoch = ""
                self._latest_payload = None
        if self._stop_object_detection is None:
            return
        try:
            self._stop_object_detection(session_id)
        except Exception:
            logger.exception(
                "Failed to stop object detection stream: session=%s",
                session_id,
            )

    def cancel_task(self) -> None:
        self._cancel_requested.set()
        self._delegate.cancel_task()

    def cancel_step(self, step: Any = None) -> None:
        del step
        self.cancel_task()

    def emergency_stop(self) -> None:
        self.cancel_task()

    def _bind_target(
        self,
        payload: Mapping[str, Any],
        target: Mapping[str, Any],
        target_type: str,
        min_confidence: float,
    ) -> tuple[dict[str, Any] | None, str]:
        candidates = [
            candidate
            for candidate in self._generic_candidates(payload)
            if candidate["target_type"] == target_type
            and candidate["stable_id"]
            and self._candidate_is_bindable(
                candidate["raw"], min_confidence
            )
        ]

        # A producer-supplied stable target/track ID always wins.  Identity
        # and labels must never make a different exact track ambiguous.
        exact_ids = {
            value
            for value in (
                self._meaningful_identifier(target.get("target_id")),
                self._meaningful_identifier(target.get("track_id")),
            )
            if value
        }
        exact = [
            candidate
            for candidate in candidates
            if exact_ids & candidate["stable_identifiers"]
        ]
        selected, reason = self._unique_candidate(exact)
        if selected is not None:
            selected["binding_reason"] = "exact_target_or_track_id"
            return selected, ""
        if reason:
            return None, reason

        # Tree's current check_person compatibility response identifies the
        # producer-selected active human by identity instead of immutable
        # target_id.  Bind that semantic reference only to active_target.
        if target_type == "human":
            active = payload.get("active_target")
            active_item = self._candidate_for_raw(
                candidates, active, min_confidence
            )
            requested_identity = self._meaningful_identifier(
                target.get("identity")
            )
            if requested_identity is None:
                requested_identity = self._meaningful_identifier(
                    target.get("target_id")
                )
            if active_item is not None and requested_identity is not None:
                active_identity = self._meaningful_identifier(
                    active.get("identity") if isinstance(active, Mapping) else None
                )
                if active_identity == requested_identity:
                    active_item["binding_reason"] = "active_target_identity"
                    return active_item, ""

            # check_person's compatibility Goal may contain only
            # target_id="unknown" + count even though Vision selected and
            # returned a stable active_target.  Binding that exact active
            # track is deterministic in both single- and multi-person scenes;
            # it is not a candidate-list fallback or target switch.
            target_count = self._positive_integer(target.get("count"))
            if active_item is not None and requested_identity is None:
                if target_count is None:
                    return None, "visual_target_count_required"
                active_item["binding_reason"] = (
                    "active_target_check_person_compat"
                )
                return active_item, ""

        requested_aliases = {
            value
            for value in (
                self._meaningful_identifier(target.get("identity")),
                self._normalize_label(target.get("label")),
                self._meaningful_identifier(target.get("target_id")),
            )
            if value
        }
        matches = [
            candidate
            for candidate in candidates
            if requested_aliases & candidate["identifiers"]
        ]
        selected, reason = self._unique_candidate(matches)
        if selected is None:
            return None, reason or "visual_target_not_found"
        if not selected["stable_id"]:
            return None, "stable_visual_target_required"
        selected["binding_reason"] = "unique_semantic_alias"
        return selected, ""

    def _binding_diagnostics(
        self,
        payload: Mapping[str, Any],
        target_type: str,
        min_confidence: float,
    ) -> str:
        seen = [
            candidate
            for candidate in self._generic_candidates(payload)
            if candidate["target_type"] == target_type
            and candidate["stable_id"]
        ]
        current = [
            candidate for candidate in seen
            if self._candidate_is_bindable(
                candidate["raw"], min_confidence
            )
        ]
        active = payload.get("active_target")
        active_id = ""
        active_track = ""
        active_state = ""
        if isinstance(active, Mapping):
            active_id = self._text(active.get("target_id")) or ""
            active_track = self._text(active.get("track_id")) or ""
            active_state = str(active.get("tracking_state", ""))
        ids = ",".join(
            f"{candidate['target_type']}:{candidate['target_id']}"
            for candidate in current
        )
        observations = ",".join(
            self._candidate_diagnostics(candidate["raw"])
            for candidate in seen
        )
        return (
            f"current=[{ids}] active_id={active_id} "
            f"active_track={active_track} active_state={active_state} "
            f"min_confidence={min_confidence:.3f} seen=[{observations}]"
        )

    def _candidate_is_bindable(
        self,
        raw: Mapping[str, Any],
        min_confidence: float,
    ) -> bool:
        if str(raw.get("tracking_state", "tracking")) != "tracking":
            return False
        confidence = self._finite(
            raw.get("confidence", raw.get("detection_confidence"))
        )
        if confidence is None or confidence < min_confidence:
            return False
        age_ms = self._finite(raw.get("last_seen_age_ms", 0.0))
        return (
            age_ms is not None
            and 0.0 <= age_ms <= self._bind_max_age_ms
        )

    def _candidate_diagnostics(self, raw: Mapping[str, Any]) -> str:
        target_id = self._text(raw.get("target_id")) or "none"
        track_id = self._text(
            raw.get("track_id", raw.get("object_track_id"))
        ) or "none"
        state = str(raw.get("tracking_state", "unknown"))
        confidence = self._finite(
            raw.get("confidence", raw.get("detection_confidence"))
        )
        distance = self._finite(raw.get("distance_m"))
        age_ms = self._finite(raw.get("last_seen_age_ms"))
        return (
            f"id={target_id}/track={track_id}/state={state}"
            f"/confidence={self._number_text(confidence)}"
            f"/distance_m={self._number_text(distance)}"
            f"/range_valid={raw.get('range_valid') is True}"
            f"/age_ms={self._number_text(age_ms)}"
        )

    @staticmethod
    def _unique_candidate(
        candidates: list[dict[str, Any]],
    ) -> tuple[dict[str, Any] | None, str]:
        unique = {
            candidate["target_id"]: candidate for candidate in candidates
        }
        if len(unique) == 1:
            return next(iter(unique.values())), ""
        if len(unique) > 1:
            return None, "visual_target_ambiguous"
        return None, ""

    def _candidate_for_raw(
        self,
        candidates: list[dict[str, Any]],
        raw: Any,
        min_confidence: float,
    ) -> dict[str, Any] | None:
        if (
            not isinstance(raw, Mapping)
            or not self._candidate_is_bindable(raw, min_confidence)
        ):
            return None
        stable_ids = {
            value
            for value in (
                self._text(raw.get("track_id")),
                self._text(raw.get("target_id")),
            )
            if value
        }
        matches = [
            candidate
            for candidate in candidates
            if stable_ids & candidate["stable_identifiers"]
        ]
        selected, _reason = self._unique_candidate(matches)
        return selected

    def _normalized_payload(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        normalized = dict(payload)
        candidates: list[dict[str, Any]] = []
        with self._lock:
            bound_type = self._bound_type
            bound_id = self._bound_target_id
            bound_epoch = self._bound_epoch
            bound_distance_mode = self._bound_distance_mode
        payload_epoch = self._text(payload.get("vision_epoch")) or ""
        for item in self._generic_candidates(payload):
            candidate = dict(item["raw"])
            canonical_id = item["target_id"]
            candidate_epoch = self._text(candidate.get("vision_epoch"))
            candidate_epoch = candidate_epoch or payload_epoch
            if bound_type:
                if item["target_type"] != bound_type:
                    continue
                if canonical_id != bound_id or candidate_epoch != bound_epoch:
                    continue
            candidate["vision_epoch"] = candidate_epoch
            candidate["target_id"] = canonical_id
            # The strict delegate deals only in a generic tracked entity.  Its
            # human tag is internal and does not alter the original target type.
            candidate["target_type"] = "human"
            candidate.setdefault("tracking_state", "tracking")
            candidate.setdefault(
                "confidence",
                candidate.get("detection_confidence", 0.0),
            )
            candidate.setdefault("last_seen_age_ms", 0.0)
            if self._finite(candidate.get("distance_m")) is not None:
                candidate.setdefault("range_valid", True)
            if bound_type and bound_distance_mode == "bbox_height":
                candidate["range_valid"] = False
                candidate["distance_m"] = None
                candidate["range_source"] = "ignored_by_bbox_height_policy"
            center = self._normalized_center(candidate, payload)
            if center is not None:
                candidate["body_center"] = [center, 0.5]
            if "bbox" not in candidate and all(
                key in candidate for key in ("x", "y", "w", "h")
            ):
                candidate["bbox"] = [
                    candidate["x"],
                    candidate["y"],
                    candidate["w"],
                    candidate["h"],
                ]
            candidates.append(candidate)
        normalized["human_candidates"] = candidates
        return normalized

    def _generic_candidates(
        self,
        payload: Mapping[str, Any],
    ) -> list[dict[str, Any]]:
        entries: list[tuple[str, Mapping[str, Any]]] = []
        human_candidates = payload.get("human_candidates")
        if isinstance(human_candidates, list):
            entries.extend(
                ("human", item)
                for item in human_candidates
                if isinstance(item, Mapping)
            )
        active_target = payload.get("active_target")
        if isinstance(active_target, Mapping):
            entries.append(("human", active_target))
        for field in ("tracked_objects", "objects"):
            raw_items = payload.get(field)
            if isinstance(raw_items, list):
                entries.extend(
                    ("", item)
                    for item in raw_items
                    if isinstance(item, Mapping)
                )

        result: list[dict[str, Any]] = []
        seen: set[tuple[str, str]] = set()
        for source_type, raw in entries:
            target_type = self._candidate_type(raw, source_type)
            if target_type is None:
                continue
            explicit_id = self._text(raw.get("target_id"))
            track_id = self._text(raw.get("track_id"))
            # Prefer the detector track id as the cross-field canonical key.
            # ``active_target`` and ``human_candidates`` may describe the same
            # entity with different target_id formatting, but they share the
            # track id.  The stream epoch supplies the restart boundary.
            canonical_id = track_id or explicit_id
            stable_id = canonical_id is not None
            if canonical_id is None:
                canonical_id = (
                    self._text(raw.get("identity"))
                    or self._normalize_label(raw.get("label"))
                )
            if canonical_id is None:
                continue
            key = (target_type, canonical_id)
            if key in seen:
                continue
            seen.add(key)
            identifiers = {
                value
                for value in (
                    explicit_id,
                    track_id,
                    self._meaningful_identifier(raw.get("identity")),
                    self._normalize_label(raw.get("label")),
                    canonical_id,
                )
                if value
            }
            result.append(
                {
                    "raw": dict(raw),
                    "target_type": target_type,
                    "target_id": canonical_id,
                    "stable_id": stable_id,
                    "stable_identifiers": {
                        value for value in (explicit_id, track_id)
                        if value
                    },
                    "identifiers": identifiers,
                }
            )
        return result

    def _candidate_type(
        self,
        raw: Mapping[str, Any],
        source_type: str,
    ) -> str | None:
        explicit = self._normalize_target_type(raw.get("target_type"))
        if explicit is not None:
            return explicit
        if source_type == "human":
            return "human"
        label = self._normalize_label(raw.get("label"))
        return "animal" if label in self._ANIMAL_LABELS else "object"

    @classmethod
    def _normalize_target_type(cls, value: Any) -> str | None:
        normalized = str(value or "").strip().lower()
        if normalized == "person":
            normalized = "human"
        return normalized if normalized in cls._TARGET_TYPES else None

    @staticmethod
    def _normalized_center(
        candidate: Mapping[str, Any],
        payload: Mapping[str, Any],
    ) -> float | None:
        for key in ("body_center", "center", "face_center"):
            center = candidate.get(key)
            if isinstance(center, (list, tuple)) and center:
                value = VisualTargetApproachAdapter._finite(center[0])
                if value is not None and 0.0 <= value <= 1.0:
                    return value
        center_x = VisualTargetApproachAdapter._finite(
            candidate.get("center_x")
        )
        if center_x is not None and 0.0 <= center_x <= 1.0:
            return center_x
        x = VisualTargetApproachAdapter._finite(candidate.get("x"))
        width = VisualTargetApproachAdapter._finite(candidate.get("w"))
        if x is None or width is None:
            return None
        center_x = x + width / 2.0
        if 0.0 <= center_x <= 1.0:
            return center_x
        frame_width = VisualTargetApproachAdapter._finite(
            payload.get("image_width", payload.get("frame_width"))
        )
        if frame_width is not None and frame_width > 0.0:
            normalized = center_x / frame_width
            if 0.0 <= normalized <= 1.0:
                return normalized
        return None

    def _invalid(self, ctx: Any, reason: str) -> TargetApproachResult:
        self._delegate.cancel_task()
        metadata = {
            "state": "failed",
            "ready": False,
            "reason": reason,
        }
        self._store_metadata(ctx, metadata)
        return TargetApproachResult(False, reason, metadata)

    @classmethod
    def _policy_positive(
        cls,
        policy: Mapping[str, Any],
        key: str,
        default: float,
    ) -> float:
        value = cls._positive(policy.get(key))
        return default if value is None else value

    @classmethod
    def _policy_nonnegative(
        cls,
        policy: Mapping[str, Any],
        key: str,
        default: float,
    ) -> float:
        value = cls._finite(policy.get(key))
        if value is None or value < 0.0:
            return default
        return value

    def _canceled(self, ctx: Any, reason: str) -> TargetApproachResult:
        self._delegate.cancel_task()
        metadata = {
            "state": "canceled",
            "ready": False,
            "reason": reason,
        }
        self._store_metadata(ctx, metadata)
        return TargetApproachResult(
            False,
            reason,
            metadata,
            canceled=True,
        )

    @staticmethod
    def _store_metadata(ctx: Any, metadata: Mapping[str, Any]) -> None:
        context_metadata = getattr(ctx, "metadata", None)
        if isinstance(context_metadata, dict):
            context_metadata["visual_target_approach"] = dict(metadata)

    @staticmethod
    def _normalize_label(value: Any) -> str | None:
        normalized = " ".join(
            str(value or "").strip().lower().replace("_", " ").split()
        )
        return normalized or None

    @staticmethod
    def _text(value: Any) -> str | None:
        if not isinstance(value, (str, int)) or isinstance(value, bool):
            return None
        normalized = str(value).strip()
        return normalized or None

    @classmethod
    def _meaningful_identifier(cls, value: Any) -> str | None:
        normalized = cls._text(value)
        if normalized is None or normalized.lower() in {
            "unknown",
            "none",
            "null",
            "-1",
            "0",
        }:
            return None
        return normalized

    @staticmethod
    def _positive_integer(value: Any) -> int | None:
        if isinstance(value, bool):
            return None
        try:
            result = int(value)
        except (TypeError, ValueError):
            return None
        return result if result > 0 else None

    @staticmethod
    def _finite(value: Any) -> float | None:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return result if math.isfinite(result) else None

    @staticmethod
    def _number_text(value: float | None) -> str:
        return "none" if value is None else f"{value:.3f}"

    @classmethod
    def _positive(cls, value: Any) -> float | None:
        result = cls._finite(value)
        return result if result is not None and result > 0.0 else None
