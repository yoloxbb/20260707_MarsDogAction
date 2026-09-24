"""Closed-loop approach to one immutable visual human target.

The core adapter is ROS-independent.  A ROS node feeds ``visual_event`` JSON
snapshots through :meth:`update_visual` and injects body-frame Twist
publisher.  A task goal must name the target by the pair
``vision_epoch + target_id``; the adapter never silently switches people.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping

from .velocity import TwistCommand


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class TargetApproachResult:
    """Terminal result returned to :class:`TaskExecutor`."""

    success: bool
    reason: str
    metadata: dict[str, Any] = field(default_factory=dict)
    canceled: bool = False
    timed_out: bool = False


@dataclass(frozen=True)
class _TargetObservation:
    vision_epoch: str
    target_id: str
    horizontal_error: float
    confidence: float
    observed_at: float
    distance_m: float | None
    bbox_height: float | None
    revision: int


@dataclass(frozen=True)
class _ApproachPolicy:
    desired_distance_m: float
    minimum_safe_distance_m: float
    target_max_age_sec: float
    target_lost_timeout_sec: float
    target_min_confidence: float
    allow_bbox_distance_fallback: bool
    arrival_hold_sec: float
    linear_gain: float
    max_linear_x: float
    max_linear_accel: float
    bbox_target_height: float
    bbox_height_deadband: float


class TargetApproachAdapter:
    """Approach a specified human and stop at a configured distance.

    Metric ``distance_m`` is authoritative.  Bounding-box height is available
    only behind an explicit demo flag.
    """

    ACTION_ID = "ACT_INTERACT_APPROACH_VOICE_CALLER"

    def __init__(
        self,
        publish_twist: Callable[[TwistCommand], None],
        *,
        publish_rate_hz: float = 10.0,
        min_confidence: float = 0.5,
        visual_timeout_sec: float = 0.8,
        acquire_timeout_sec: float = 3.0,
        lost_timeout_sec: float = 1.0,
        approach_timeout_sec: float = 20.0,
        stop_distance_m: float = 1.2,
        minimum_safe_distance_m: float = 0.8,
        distance_deadband_m: float = 0.15,
        distance_hysteresis_m: float = 0.10,
        arrival_hold_sec: float = 0.5,
        linear_gain: float = 0.5,
        angular_gain: float = 0.8,
        max_linear_x: float = 0.15,
        max_angular_z: float = 0.3,
        max_linear_accel: float = 0.2,
        max_angular_accel: float = 0.3,
        heading_deadband: float = 0.06,
        max_heading_error: float = 0.25,
        allow_bbox_distance_fallback: bool = False,
        demo_target_height: float = 0.68,
        demo_height_deadband: float = 0.05,
        bbox_height_hysteresis: float = 0.03,
        reacquire_min_consecutive_frames: int = 3,
        stop_publish_count: int = 3,
        action_id: str = ACTION_ID,
        should_stop: Callable[[], bool] | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        rate_hz = self._positive(publish_rate_hz, "publish_rate_hz")
        self._period_sec = 1.0 / rate_hz
        self._min_confidence = self._bounded(
            min_confidence, "min_confidence", 0.0, 1.0
        )
        self._visual_timeout_sec = self._positive(
            visual_timeout_sec, "visual_timeout_sec"
        )
        self._acquire_timeout_sec = self._positive(
            acquire_timeout_sec, "acquire_timeout_sec"
        )
        self._lost_timeout_sec = self._positive(
            lost_timeout_sec, "lost_timeout_sec"
        )
        self._approach_timeout_sec = self._positive(
            approach_timeout_sec, "approach_timeout_sec"
        )
        self._stop_distance_m = self._positive(
            stop_distance_m, "stop_distance_m"
        )
        self._minimum_safe_distance_m = self._positive(
            minimum_safe_distance_m, "minimum_safe_distance_m"
        )
        self._distance_deadband_m = self._nonnegative(
            distance_deadband_m, "distance_deadband_m"
        )
        self._distance_hysteresis_m = self._nonnegative(
            distance_hysteresis_m, "distance_hysteresis_m"
        )
        self._arrival_hold_sec = self._nonnegative(
            arrival_hold_sec, "arrival_hold_sec"
        )
        self._linear_gain = self._nonnegative(linear_gain, "linear_gain")
        self._angular_gain = self._nonnegative(angular_gain, "angular_gain")
        self._max_linear_x = self._positive(max_linear_x, "max_linear_x")
        self._max_angular_z = self._positive(max_angular_z, "max_angular_z")
        self._max_linear_accel = self._positive(
            max_linear_accel, "max_linear_accel"
        )
        self._max_angular_accel = self._positive(
            max_angular_accel, "max_angular_accel"
        )
        self._heading_deadband = self._bounded(
            heading_deadband, "heading_deadband", 0.0, 0.5
        )
        self._max_heading_error = self._bounded(
            max_heading_error, "max_heading_error", self._heading_deadband, 0.5
        )
        self._allow_bbox_distance_fallback = bool(
            allow_bbox_distance_fallback
        )
        self._demo_target_height = self._bounded(
            demo_target_height, "demo_target_height", 0.05, 1.0
        )
        self._demo_height_deadband = self._nonnegative(
            demo_height_deadband, "demo_height_deadband"
        )
        self._bbox_height_hysteresis = self._nonnegative(
            bbox_height_hysteresis, "bbox_height_hysteresis"
        )
        if (
            isinstance(reacquire_min_consecutive_frames, bool)
            or not isinstance(reacquire_min_consecutive_frames, int)
            or reacquire_min_consecutive_frames < 1
        ):
            raise ValueError(
                "reacquire_min_consecutive_frames must be an integer >= 1"
            )
        self._reacquire_min_consecutive_frames = (
            reacquire_min_consecutive_frames
        )
        self._stop_publish_count = max(1, int(stop_publish_count))
        self._action_id = self._strict_nonempty_string(action_id)
        if self._action_id is None:
            raise ValueError("action_id must be a non-empty string")
        self._publish_twist = publish_twist
        self._should_stop = should_stop or (lambda: False)
        self._monotonic = monotonic
        self._sleep = sleep

        self._lock = threading.Lock()
        # Every non-zero Twist passes through this lock and is tagged with the
        # current execution generation and visual revision.  Cancel therefore
        # either happens before a command (which is rejected), or after it (and
        # publishes the final zero); a MOVE can never appear after cancel returns.
        self._motion_lock = threading.RLock()
        self._execution_lock = threading.Lock()
        self._cancel_requested = threading.Event()
        self._latest_payload: dict[str, Any] | None = None
        self._latest_payload_at = 0.0
        self._latest_payload_revision = 0
        self._visual_revision = 0
        self._motion_visual_revision = 0
        self._stream_epoch = ""
        self._stream_sequence = 0
        self._retired_epochs: set[str] = set()
        self._active_key: tuple[str, str] | None = None
        self._active_policy: _ApproachPolicy | None = None
        self._observation: _TargetObservation | None = None
        self._epoch_changed = False
        self._visual_invalidation_count = 0
        self._motion_generation = 0
        self._motion_stopped = True
        self._last_linear_x = 0.0
        self._last_angular_z = 0.0
        self._last_linear_at = 0.0
        self._last_angular_at = 0.0

    def update_visual(
        self,
        payload: Mapping[str, Any],
        now: float | None = None,
    ) -> bool:
        """Consume one ``/perception/visual_event`` JSON payload."""
        if not isinstance(payload, Mapping):
            self._invalidate_active_snapshot("invalid_visual_payload")
            return False
        received_at = self._monotonic() if now is None else float(now)
        snapshot = dict(payload)
        envelope = self._validate_envelope(snapshot)
        if envelope is None:
            self._invalidate_active_snapshot("invalid_visual_envelope")
            return False
        payload_epoch, sequence = envelope
        should_stop = False
        with self._lock:
            if payload_epoch in self._retired_epochs:
                return False
            if payload_epoch == self._stream_epoch:
                if sequence <= self._stream_sequence:
                    return False
            elif self._stream_epoch:
                self._retired_epochs.add(self._stream_epoch)
                self._stream_epoch = payload_epoch
                self._stream_sequence = 0
            else:
                self._stream_epoch = payload_epoch

            self._stream_sequence = sequence
            self._visual_revision += 1
            revision = self._visual_revision
            self._latest_payload = snapshot
            self._latest_payload_at = received_at
            self._latest_payload_revision = revision
            active_key = self._active_key
            if active_key is None:
                self._set_motion_visual_revision(revision)
                return True
            should_stop = not self._apply_payload_locked(
                snapshot,
                received_at,
                active_key,
                revision,
            )
            if should_stop:
                self._visual_invalidation_count += 1
        self._set_motion_visual_revision(revision, publish_stop=should_stop)
        return True

    def execute_task(
        self,
        unit_config: Mapping[str, Any],
        ctx: Any,
        timeout_sec: float,
    ) -> TargetApproachResult:
        """Run the synchronous task lifecycle used by ``TaskExecutor``."""
        unit_id = str(unit_config.get("unit_id", ""))
        if unit_id != self._action_id:
            return self._invalid_result(ctx, f"unsupported_action:{unit_id}")

        target = getattr(ctx, "target", None)
        if not isinstance(target, Mapping):
            return self._invalid_result(ctx, "target_required")
        if target.get("target_type") != "human":
            return self._invalid_result(ctx, "human_target_required")
        vision_epoch = self._strict_nonempty_string(target.get("vision_epoch"))
        target_id = self._strict_nonempty_string(target.get("target_id"))
        if vision_epoch is None or target_id is None:
            return self._invalid_result(
                ctx,
                "target_ref_requires_vision_epoch_and_target_id",
            )

        policy, policy_error = self._resolve_goal_policy(ctx)
        if policy is None:
            return self._invalid_result(ctx, policy_error)

        requested_timeout = self._finite_or_default(
            timeout_sec,
            self._approach_timeout_sec,
        )
        effective_timeout = min(
            self._approach_timeout_sec,
            requested_timeout,
        )
        with self._execution_lock:
            with self._motion_lock:
                self._motion_generation += 1
                motion_generation = self._motion_generation
                self._cancel_requested.clear()
            start = self._monotonic()
            execution_deadline = start + effective_timeout
            last_valid_at: float | None = None
            no_range_since: float | None = None
            arrival_since: float | None = None
            reacquire_required = True
            reacquire_frame_count = 0
            reacquire_last_revision = 0
            initial_distance: float | None = None
            last_state = ""
            key = (vision_epoch, target_id)
            with self._lock:
                self._active_key = key
                self._active_policy = policy
                self._observation = None
                self._epoch_changed = False
                cached = self._latest_payload
                cached_at = self._latest_payload_at
                cached_revision = self._latest_payload_revision
                if cached is not None:
                    cached_valid = self._apply_payload_locked(
                        cached,
                        cached_at,
                        key,
                        cached_revision,
                    )
                    if not cached_valid:
                        self._visual_invalidation_count += 1
                observed_invalidation_count = self._visual_invalidation_count
            self._publish_stop(force=True)

            try:
                while True:
                    now = self._monotonic()
                    if self._cancel_requested.is_set() or self._should_stop():
                        return self._finish(
                            ctx,
                            success=False,
                            reason="target_approach_canceled",
                            state="canceled",
                            vision_epoch=vision_epoch,
                            target_id=target_id,
                            canceled=True,
                        )
                    if now - start >= effective_timeout:
                        return self._finish(
                            ctx,
                            success=False,
                            reason="target_approach_timeout",
                            state="timeout",
                            vision_epoch=vision_epoch,
                            target_id=target_id,
                            timed_out=True,
                        )

                    with self._lock:
                        observation = self._observation
                        epoch_changed = self._epoch_changed
                        invalidation_count = self._visual_invalidation_count

                    if invalidation_count != observed_invalidation_count:
                        observed_invalidation_count = invalidation_count
                        reacquire_required = True
                        reacquire_frame_count = 0
                        reacquire_last_revision = 0

                    if epoch_changed:
                        return self._finish(
                            ctx,
                            success=False,
                            reason="vision_epoch_changed",
                            state="failed",
                            vision_epoch=vision_epoch,
                            target_id=target_id,
                        )

                    fresh = (
                        observation is not None
                        and now - observation.observed_at
                        <= policy.target_max_age_sec
                    )
                    if not fresh:
                        self._publish_stop()
                        reacquire_required = True
                        reacquire_frame_count = 0
                        reacquire_last_revision = 0
                        state = (
                            "acquiring_target"
                            if last_valid_at is None
                            else "target_lost"
                        )
                        last_state = self._report_state(
                            ctx,
                            state,
                            last_state,
                            0.1 if last_valid_at is None else 0.3,
                            vision_epoch,
                            target_id,
                            None,
                            "none",
                        )
                        if (
                            last_valid_at is None
                            and now - start >= self._acquire_timeout_sec
                        ):
                            return self._finish(
                                ctx,
                                success=False,
                                reason="target_not_acquired",
                                state="failed",
                                vision_epoch=vision_epoch,
                                target_id=target_id,
                            )
                        if (
                            last_valid_at is not None
                            and now - last_valid_at
                            >= policy.target_lost_timeout_sec
                        ):
                            last_confidence = (
                                observation.confidence
                                if observation is not None
                                else None
                            )
                            last_distance = (
                                observation.distance_m
                                if observation is not None
                                else None
                            )
                            last_age = (
                                now - observation.observed_at
                                if observation is not None
                                else None
                            )
                            logger.warning(
                                "Visual target lost: epoch=%s target_id=%s "
                                "confidence=%s distance_m=%s age_sec=%s "
                                "lost_for_sec=%.3f timeout_sec=%.3f",
                                vision_epoch,
                                target_id,
                                self._format_number(last_confidence),
                                self._format_number(last_distance),
                                self._format_number(last_age),
                                now - last_valid_at,
                                policy.target_lost_timeout_sec,
                            )
                            return self._finish(
                                ctx,
                                success=False,
                                reason="target_lost",
                                state="failed",
                                vision_epoch=vision_epoch,
                                target_id=target_id,
                            )
                        self._sleep_before_deadline(execution_deadline)
                        continue

                    assert observation is not None
                    last_valid_at = now
                    distance_value, distance_source = self._distance(
                        observation,
                        policy.allow_bbox_distance_fallback,
                    )
                    if distance_value is None:
                        self._publish_stop()
                        if no_range_since is None:
                            no_range_since = now
                        last_state = self._report_state(
                            ctx,
                            "waiting_for_metric_range",
                            last_state,
                            0.2,
                            vision_epoch,
                            target_id,
                            None,
                            "none",
                        )
                        if (
                            now - no_range_since
                            >= policy.target_lost_timeout_sec
                        ):
                            return self._finish(
                                ctx,
                                success=False,
                                reason="metric_distance_unavailable",
                                state="failed",
                                vision_epoch=vision_epoch,
                                target_id=target_id,
                            )
                        self._sleep_before_deadline(execution_deadline)
                        continue
                    no_range_since = None
                    if initial_distance is None and distance_source == "metric":
                        initial_distance = distance_value

                    if reacquire_required:
                        if observation.revision != reacquire_last_revision:
                            reacquire_frame_count += 1
                            reacquire_last_revision = observation.revision
                        if (
                            reacquire_frame_count
                            < self._reacquire_min_consecutive_frames
                        ):
                            self._publish_stop()
                            last_state = self._report_state(
                                ctx,
                                "target_reacquire_confirm",
                                last_state,
                                0.25,
                                vision_epoch,
                                target_id,
                                distance_value,
                                distance_source,
                            )
                            self._sleep_before_deadline(execution_deadline)
                            continue
                        reacquire_required = False

                    horizontal_error = observation.horizontal_error
                    far = self._is_far(
                        distance_value,
                        distance_source,
                        policy,
                    )
                    arrived = (
                        self._is_within_arrival_hysteresis(
                            distance_value,
                            distance_source,
                            policy,
                        )
                        if arrival_since is not None
                        else self._is_arrived(
                            distance_value,
                            distance_source,
                            policy,
                        )
                    )

                    if arrived and abs(horizontal_error) <= self._max_heading_error:
                        self._publish_stop()
                        if arrival_since is None:
                            arrival_since = now
                        last_state = self._report_state(
                            ctx,
                            "arrival_confirm",
                            last_state,
                            0.95,
                            vision_epoch,
                            target_id,
                            distance_value,
                            distance_source,
                        )
                        if now - arrival_since >= policy.arrival_hold_sec:
                            return self._finish(
                                ctx,
                                success=True,
                                reason="target_reached",
                                state="ready",
                                vision_epoch=vision_epoch,
                                target_id=target_id,
                                distance=distance_value,
                                distance_source=distance_source,
                            )
                        self._sleep_before_deadline(execution_deadline)
                        continue

                    arrival_since = None
                    angular = self._angular_command(horizontal_error, now)
                    linear = 0.0
                    state = "aligning_target"
                    if far and abs(horizontal_error) <= self._max_heading_error:
                        linear = self._linear_command(
                            distance_value,
                            distance_source,
                            horizontal_error,
                            now,
                            policy,
                        )
                        state = "approaching_target"
                    else:
                        self._reset_linear(now)

                    self._publish_motion(
                        TwistCommand(linear_x=linear, angular_z=angular),
                        generation=motion_generation,
                        visual_revision=observation.revision,
                        execution_deadline=execution_deadline,
                        visual_deadline=(
                            observation.observed_at
                            + policy.target_max_age_sec
                        ),
                    )
                    progress = self._progress(
                        distance_value,
                        distance_source,
                        initial_distance,
                        state,
                        policy.desired_distance_m,
                    )
                    last_state = self._report_state(
                        ctx,
                        state,
                        last_state,
                        progress,
                        vision_epoch,
                        target_id,
                        distance_value,
                        distance_source,
                    )
                    self._sleep_before_deadline(execution_deadline)
            except Exception as exc:
                logger.exception("Target approach failed")
                return self._finish(
                    ctx,
                    success=False,
                    reason=f"target_approach_error:{exc}",
                    state="failed",
                    vision_epoch=vision_epoch,
                    target_id=target_id,
                )
            finally:
                self._publish_stop()
                with self._lock:
                    self._active_key = None
                    self._active_policy = None
                    self._observation = None
                    self._epoch_changed = False

    def cancel_task(self) -> None:
        """Cancel the active task and immediately publish redundant zeroes."""
        self._cancel_requested.set()
        with self._motion_lock:
            self._motion_generation += 1
            self._publish_stop_locked(force=True)

    def cancel_step(self, step: Any = None) -> None:
        del step
        self.cancel_task()

    def emergency_stop(self) -> None:
        self.cancel_task()

    def _resolve_goal_policy(
        self,
        ctx: Any,
    ) -> tuple[_ApproachPolicy | None, str]:
        interaction_id = self._strict_nonempty_string(
            getattr(ctx, "interaction_id", None)
        )
        if interaction_id is None:
            return None, "interaction_id_required"

        params = getattr(ctx, "params", None)
        if not isinstance(params, Mapping):
            return None, "approach_policy_required"
        if params.get("strict_target_lock") is not True:
            return None, "strict_target_lock_required"
        if params.get("allow_target_switch") is not False:
            return None, "target_switch_must_be_false"

        desired = self._finite(params.get("desired_distance_m"))
        minimum_safe = self._finite(params.get("minimum_safe_distance_m"))
        maximum_age_ms = self._finite(params.get("target_max_age_ms"))
        lost_timeout = self._finite(params.get("target_lost_timeout_sec"))
        requested_min_confidence = self._finite(
            params.get("target_min_confidence", self._min_confidence)
        )
        # Per-behavior motion profiles are installed only by the trusted
        # VisualTargetApproachAdapter proxy.  Raw Goal params must not expand
        # a node's arrival window or alter bbox proximity/speed limits.
        allow_profile_overrides = (
            getattr(ctx, "_allow_behavior_policy_overrides", False) is True
        )
        profile_params = params if allow_profile_overrides else {}
        arrival_hold = self._finite(
            profile_params.get("arrival_hold_sec", self._arrival_hold_sec)
        )
        linear_gain = self._finite(
            profile_params.get("approach_linear_gain", self._linear_gain)
        )
        max_linear_x = self._finite(
            profile_params.get("approach_max_linear_x", self._max_linear_x)
        )
        max_linear_accel = self._finite(
            profile_params.get(
                "approach_max_linear_accel",
                self._max_linear_accel,
            )
        )
        bbox_target_height = self._finite(
            profile_params.get("bbox_target_height", self._demo_target_height)
        )
        bbox_height_deadband = self._finite(
            profile_params.get(
                "bbox_height_deadband",
                self._demo_height_deadband,
            )
        )
        if desired is None or desired <= 0.0:
            return None, "invalid_desired_distance_m"
        if minimum_safe is None or minimum_safe <= 0.0:
            return None, "invalid_minimum_safe_distance_m"
        if maximum_age_ms is None or maximum_age_ms <= 0.0:
            return None, "invalid_target_max_age_ms"
        if lost_timeout is None or lost_timeout <= 0.0:
            return None, "invalid_target_lost_timeout_sec"
        if (
            requested_min_confidence is None
            or not 0.0 <= requested_min_confidence <= 1.0
        ):
            return None, "invalid_target_min_confidence"
        if arrival_hold is None or arrival_hold < 0.0:
            return None, "invalid_arrival_hold_sec"
        if linear_gain is None or linear_gain < 0.0:
            return None, "invalid_approach_linear_gain"
        if max_linear_x is None or max_linear_x <= 0.0:
            return None, "invalid_approach_max_linear_x"
        if max_linear_accel is None or max_linear_accel <= 0.0:
            return None, "invalid_approach_max_linear_accel"
        if (
            bbox_target_height is None
            or not 0.05 <= bbox_target_height <= 1.0
        ):
            return None, "invalid_bbox_target_height"
        if bbox_height_deadband is None or bbox_height_deadband < 0.0:
            return None, "invalid_bbox_height_deadband"

        requested_fallback = params.get(
            "speaker_allow_bbox_distance_fallback",
            False,
        )
        if not isinstance(requested_fallback, bool):
            return None, "invalid_speaker_allow_bbox_distance_fallback"

        effective_minimum_safe = max(
            self._minimum_safe_distance_m,
            minimum_safe,
        )
        effective_desired = max(
            self._stop_distance_m,
            desired,
            effective_minimum_safe,
        )
        return (
            _ApproachPolicy(
                desired_distance_m=effective_desired,
                minimum_safe_distance_m=effective_minimum_safe,
                target_max_age_sec=min(
                    self._visual_timeout_sec,
                    maximum_age_ms / 1000.0,
                ),
                target_lost_timeout_sec=min(
                    self._lost_timeout_sec,
                    lost_timeout,
                ),
                target_min_confidence=requested_min_confidence,
                allow_bbox_distance_fallback=(
                    self._allow_bbox_distance_fallback
                    and requested_fallback
                ),
                arrival_hold_sec=arrival_hold,
                linear_gain=linear_gain,
                max_linear_x=min(self._max_linear_x, max_linear_x),
                max_linear_accel=min(
                    self._max_linear_accel,
                    max_linear_accel,
                ),
                bbox_target_height=bbox_target_height,
                bbox_height_deadband=bbox_height_deadband,
            ),
            "",
        )

    def _validate_envelope(
        self,
        payload: Mapping[str, Any],
    ) -> tuple[str, int] | None:
        schema_version = payload.get("schema_version")
        if (
            isinstance(schema_version, bool)
            or not isinstance(schema_version, int)
            or schema_version != 1
        ):
            return None
        header = payload.get("header")
        if not isinstance(header, Mapping):
            return None
        stamp = self._finite(header.get("stamp"))
        frame_id = self._strict_nonempty_string(header.get("frame_id"))
        if stamp is None or stamp <= 0.0 or frame_id is None:
            return None
        vision_epoch = self._strict_nonempty_string(
            payload.get("vision_epoch")
        )
        sequence = payload.get("sequence")
        if (
            vision_epoch is None
            or isinstance(sequence, bool)
            or not isinstance(sequence, int)
            or sequence <= 0
        ):
            return None
        if self._strict_nonempty_string(payload.get("snapshot_id")) is None:
            return None
        return vision_epoch, sequence

    def _apply_payload_locked(
        self,
        payload: Mapping[str, Any],
        received_at: float,
        key: tuple[str, str],
        revision: int,
    ) -> bool:
        expected_epoch, expected_target_id = key
        payload_epoch = self._strict_nonempty_string(payload.get("vision_epoch"))
        self._observation = None
        if payload_epoch != expected_epoch:
            self._epoch_changed = True
            return False
        self._epoch_changed = False

        candidates = payload.get("human_candidates")
        if not isinstance(candidates, list):
            return False
        matched: _TargetObservation | None = None
        for raw in candidates:
            if not isinstance(raw, Mapping):
                continue
            candidate_epoch = self._strict_nonempty_string(
                raw.get("vision_epoch")
            )
            if candidate_epoch is None:
                candidate_epoch = payload_epoch
            candidate_id = self._strict_nonempty_string(raw.get("target_id"))
            if (
                candidate_epoch != expected_epoch
                or candidate_id != expected_target_id
            ):
                continue
            if raw.get("target_type") != "human":
                continue
            if str(raw.get("tracking_state", "")) != "tracking":
                continue
            confidence = self._finite(raw.get("confidence"))
            if confidence is None:
                confidence = self._finite(raw.get("detection_confidence"))
            policy = self._active_policy
            min_confidence = (
                policy.target_min_confidence
                if policy is not None
                else self._min_confidence
            )
            if confidence is None or confidence < min_confidence:
                continue
            horizontal_error = self._horizontal_error(raw)
            if horizontal_error is None:
                continue
            age_ms = self._finite(raw.get("last_seen_age_ms"))
            if age_ms is None or age_ms < 0.0:
                continue
            policy = self._active_policy
            max_age_sec = (
                policy.target_max_age_sec
                if policy is not None
                else self._visual_timeout_sec
            )
            if age_ms > max_age_sec * 1000.0:
                continue
            observed_at = received_at - age_ms / 1000.0
            range_valid = raw.get("range_valid") is True
            distance_m = (
                self._finite(raw.get("distance_m"))
                if range_valid
                else None
            )
            if distance_m is not None and distance_m <= 0.0:
                distance_m = None
            bbox_height = self._bbox_height(raw.get("bbox"))
            candidate = _TargetObservation(
                vision_epoch=candidate_epoch,
                target_id=candidate_id,
                horizontal_error=horizontal_error,
                confidence=confidence,
                observed_at=observed_at,
                distance_m=distance_m,
                bbox_height=bbox_height,
                revision=revision,
            )
            if matched is None or candidate.confidence > matched.confidence:
                matched = candidate
        if matched is not None:
            self._observation = matched
            policy = self._active_policy
            allow_fallback = (
                policy.allow_bbox_distance_fallback
                if policy is not None
                else False
            )
            return self._distance(matched, allow_fallback)[0] is not None
        return False

    def _distance(
        self,
        observation: _TargetObservation,
        allow_bbox_fallback: bool,
    ) -> tuple[float | None, str]:
        if observation.distance_m is not None:
            return observation.distance_m, "metric"
        if (
            allow_bbox_fallback
            and observation.bbox_height is not None
        ):
            return observation.bbox_height, "bbox_height"
        return None, "none"

    def _is_far(
        self,
        value: float,
        source: str,
        policy: _ApproachPolicy,
    ) -> bool:
        if source == "metric":
            return (
                value
                > policy.desired_distance_m + self._distance_deadband_m
            )
        return (
            value
            < policy.bbox_target_height - policy.bbox_height_deadband
        )

    def _is_arrived(
        self,
        value: float,
        source: str,
        policy: _ApproachPolicy,
    ) -> bool:
        if source == "metric":
            return (
                value
                <= policy.desired_distance_m + self._distance_deadband_m
            )
        return (
            value
            >= policy.bbox_target_height - policy.bbox_height_deadband
        )

    def _is_within_arrival_hysteresis(
        self,
        value: float,
        source: str,
        policy: _ApproachPolicy,
    ) -> bool:
        """Keep arrival latched until the target clearly leaves the stop zone."""
        if source == "metric":
            return value <= (
                policy.desired_distance_m
                + self._distance_deadband_m
                + self._distance_hysteresis_m
            )
        return value >= (
            policy.bbox_target_height
            - policy.bbox_height_deadband
            - self._bbox_height_hysteresis
        )

    def _linear_command(
        self,
        value: float,
        source: str,
        horizontal_error: float,
        now: float,
        policy: _ApproachPolicy,
    ) -> float:
        if source == "metric":
            error = max(0.0, value - policy.desired_distance_m)
        else:
            error = max(0.0, policy.bbox_target_height - value)
        desired = min(policy.max_linear_x, policy.linear_gain * error)
        alignment = max(
            0.0,
            1.0 - abs(horizontal_error) / self._max_heading_error,
        )
        desired *= alignment
        elapsed = self._elapsed_since(self._last_linear_at, now)
        max_delta = policy.max_linear_accel * elapsed
        delta = max(-max_delta, min(max_delta, desired - self._last_linear_x))
        self._last_linear_x = max(
            0.0,
            min(policy.max_linear_x, self._last_linear_x + delta),
        )
        self._last_linear_at = now
        return self._last_linear_x

    def _angular_command(self, error: float, now: float) -> float:
        desired = 0.0
        if abs(error) > self._heading_deadband:
            desired = -self._angular_gain * error
        desired = max(-self._max_angular_z, min(self._max_angular_z, desired))
        elapsed = self._elapsed_since(self._last_angular_at, now)
        max_delta = self._max_angular_accel * elapsed
        delta = max(
            -max_delta,
            min(max_delta, desired - self._last_angular_z),
        )
        self._last_angular_z = max(
            -self._max_angular_z,
            min(self._max_angular_z, self._last_angular_z + delta),
        )
        self._last_angular_at = now
        return self._last_angular_z

    def _elapsed_since(self, previous: float, now: float) -> float:
        if previous <= 0.0:
            return self._period_sec
        return min(0.5, max(0.0, now - previous))

    def _sleep_before_deadline(self, deadline: float) -> None:
        """Sleep at most until the behavior budget is exhausted."""
        remaining = deadline - self._monotonic()
        if remaining > 0.0:
            self._sleep(min(self._period_sec, remaining))

    def _reset_linear(self, now: float) -> None:
        self._last_linear_x = 0.0
        self._last_linear_at = now

    def _publish_stop(self, *, force: bool = False) -> None:
        with self._motion_lock:
            self._publish_stop_locked(force=force)

    def _publish_stop_locked(self, *, force: bool = False) -> None:
        self._last_linear_x = 0.0
        self._last_angular_z = 0.0
        now = self._monotonic()
        self._last_linear_at = now
        self._last_angular_at = now
        if self._motion_stopped and not force:
            return
        stop = TwistCommand()
        published = False
        for _ in range(self._stop_publish_count):
            try:
                self._publish_twist(stop)
                published = True
            except Exception as exc:
                logger.warning("Failed to publish target-approach stop: %s", exc)
                break
        self._motion_stopped = published

    def _publish_motion(
        self,
        command: TwistCommand,
        *,
        generation: int,
        visual_revision: int,
        execution_deadline: float,
        visual_deadline: float,
    ) -> bool:
        """Publish a non-zero command only while both safety tokens are live."""
        if command.is_zero:
            self._publish_stop()
            return True
        with self._motion_lock:
            now = self._monotonic()
            if (
                generation != self._motion_generation
                or visual_revision != self._motion_visual_revision
                or self._cancel_requested.is_set()
                or self._should_stop()
                or now >= execution_deadline
                or now > visual_deadline
            ):
                self._publish_stop_locked()
                return False
            self._publish_twist(command)
            self._motion_stopped = False
            return True

    def _set_motion_visual_revision(
        self,
        revision: int,
        *,
        publish_stop: bool = False,
    ) -> None:
        with self._motion_lock:
            self._motion_visual_revision = revision
            if publish_stop:
                self._publish_stop_locked()

    def _invalidate_active_snapshot(self, reason: str) -> None:
        del reason
        with self._lock:
            if self._active_key is None:
                return
            self._visual_revision += 1
            revision = self._visual_revision
            self._observation = None
            self._visual_invalidation_count += 1
        self._set_motion_visual_revision(revision, publish_stop=True)

    def _report_state(
        self,
        ctx: Any,
        state: str,
        previous: str,
        progress: float,
        vision_epoch: str,
        target_id: str,
        distance: float | None,
        distance_source: str,
    ) -> str:
        metadata = self._metadata(
            state=state,
            ready=False,
            reason="running",
            vision_epoch=vision_epoch,
            target_id=target_id,
            distance=distance,
            distance_source=distance_source,
        )
        self._store_metadata(ctx, metadata)
        if state != previous:
            reporter = getattr(ctx, "report_runtime_feedback", None)
            if callable(reporter):
                reporter(
                    progress,
                    self._action_id,
                    self._feedback_message(metadata),
                    True,
                )
        return state

    def _finish(
        self,
        ctx: Any,
        *,
        success: bool,
        reason: str,
        state: str,
        vision_epoch: str,
        target_id: str,
        distance: float | None = None,
        distance_source: str = "none",
        canceled: bool = False,
        timed_out: bool = False,
    ) -> TargetApproachResult:
        self._publish_stop(force=True)
        metadata = self._metadata(
            state=state,
            ready=success,
            reason=reason,
            vision_epoch=vision_epoch,
            target_id=target_id,
            distance=distance,
            distance_source=distance_source,
        )
        self._store_metadata(ctx, metadata)
        reporter = getattr(ctx, "report_runtime_feedback", None)
        if callable(reporter):
            reporter(
                1.0 if success else 0.0,
                self._action_id,
                self._feedback_message(metadata),
                True,
            )
        return TargetApproachResult(
            success=success,
            reason=reason,
            metadata=metadata,
            canceled=canceled,
            timed_out=timed_out,
        )

    def _invalid_result(self, ctx: Any, reason: str) -> TargetApproachResult:
        self._publish_stop(force=True)
        metadata = self._metadata(
            state="failed",
            ready=False,
            reason=reason,
            vision_epoch="",
            target_id="",
            distance=None,
            distance_source="none",
        )
        self._store_metadata(ctx, metadata)
        return TargetApproachResult(False, reason, metadata)

    def _metadata(
        self,
        *,
        state: str,
        ready: bool,
        reason: str,
        vision_epoch: str,
        target_id: str,
        distance: float | None,
        distance_source: str,
    ) -> dict[str, Any]:
        policy = self._active_policy
        return {
            "state": state,
            "ready": bool(ready),
            "reason": reason,
            "vision_epoch": vision_epoch,
            "target_id": target_id,
            "distance_m": distance if distance_source == "metric" else None,
            "distance_source": distance_source,
            "bbox_distance_fallback": distance_source == "bbox_height",
            # Compatibility for existing debug consumers; new integrations
            # should read bbox_distance_fallback and distance_source.
            "demo_distance_fallback": distance_source == "bbox_height",
            "effective_policy": (
                {
                    "desired_distance_m": policy.desired_distance_m,
                    "minimum_safe_distance_m": (
                        policy.minimum_safe_distance_m
                    ),
                    "target_max_age_ms": (
                        policy.target_max_age_sec * 1000.0
                    ),
                    "target_lost_timeout_sec": (
                        policy.target_lost_timeout_sec
                    ),
                    "target_min_confidence": (
                        policy.target_min_confidence
                    ),
                    "bbox_distance_fallback": (
                        policy.allow_bbox_distance_fallback
                    ),
                    "arrival_hold_sec": policy.arrival_hold_sec,
                    "linear_gain": policy.linear_gain,
                    "max_linear_x": policy.max_linear_x,
                    "max_linear_accel": policy.max_linear_accel,
                    "bbox_target_height": policy.bbox_target_height,
                    "bbox_height_deadband": policy.bbox_height_deadband,
                }
                if policy is not None
                else None
            ),
        }

    @staticmethod
    def _format_number(value: float | None) -> str:
        if value is None or not math.isfinite(value):
            return "none"
        return f"{value:.3f}"

    @staticmethod
    def _store_metadata(ctx: Any, metadata: Mapping[str, Any]) -> None:
        context_metadata = getattr(ctx, "metadata", None)
        if isinstance(context_metadata, dict):
            context_metadata["target_approach"] = dict(metadata)

    @staticmethod
    def _feedback_message(metadata: Mapping[str, Any]) -> str:
        distance = metadata.get("distance_m")
        distance_text = (
            f" distance_m={float(distance):.3f}"
            if isinstance(distance, (int, float))
            else ""
        )
        return (
            f"target_approach state={metadata.get('state')}"
            f" target_id={metadata.get('target_id')}"
            f" source={metadata.get('distance_source')}"
            f"{distance_text}"
        )

    def _progress(
        self,
        value: float,
        source: str,
        initial_distance: float | None,
        state: str,
        desired_distance_m: float,
    ) -> float:
        if state == "aligning_target":
            return 0.35
        if source != "metric" or initial_distance is None:
            return 0.6
        span = max(0.01, initial_distance - desired_distance_m)
        covered = max(0.0, initial_distance - value)
        return min(0.9, 0.4 + 0.5 * covered / span)

    @staticmethod
    def _horizontal_error(candidate: Mapping[str, Any]) -> float | None:
        for key in ("body_center", "center", "face_center"):
            center = candidate.get(key)
            if isinstance(center, (list, tuple)) and len(center) >= 2:
                x = TargetApproachAdapter._finite(center[0])
                y = TargetApproachAdapter._finite(center[1])
                if x is not None and y is not None and 0.0 <= x <= 1.0:
                    return x - 0.5
        bbox = candidate.get("bbox")
        if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
            x = TargetApproachAdapter._finite(bbox[0])
            width = TargetApproachAdapter._finite(bbox[2])
            if x is not None and width is not None:
                center_x = x + width / 2.0
                if 0.0 <= center_x <= 1.0:
                    return center_x - 0.5
        return None

    @staticmethod
    def _bbox_height(value: Any) -> float | None:
        if not isinstance(value, (list, tuple)) or len(value) < 4:
            return None
        height = TargetApproachAdapter._finite(value[3])
        if height is None or not 0.0 < height <= 1.0:
            return None
        return height

    @staticmethod
    def _finite(value: Any) -> float | None:
        if isinstance(value, bool):
            return None
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return result if math.isfinite(result) else None

    @staticmethod
    def _nonempty_string(value: Any) -> str | None:
        if value is None or isinstance(value, bool):
            return None
        result = str(value).strip()
        return result or None

    @staticmethod
    def _strict_nonempty_string(value: Any) -> str | None:
        if not isinstance(value, str):
            return None
        result = value.strip()
        return result or None

    @classmethod
    def _finite_or_default(cls, value: Any, default: float) -> float:
        result = cls._finite(value)
        return default if result is None or result <= 0.0 else result

    @classmethod
    def _positive(cls, value: Any, name: str) -> float:
        result = cls._finite(value)
        if result is None or result <= 0.0:
            raise ValueError(f"{name} must be finite and > 0")
        return result

    @classmethod
    def _nonnegative(cls, value: Any, name: str) -> float:
        result = cls._finite(value)
        if result is None or result < 0.0:
            raise ValueError(f"{name} must be finite and >= 0")
        return result

    @classmethod
    def _bounded(
        cls,
        value: Any,
        name: str,
        lower: float,
        upper: float,
    ) -> float:
        result = cls._finite(value)
        if result is None or not lower <= result <= upper:
            raise ValueError(
                f"{name} must be finite and within [{lower}, {upper}]"
            )
        return result
