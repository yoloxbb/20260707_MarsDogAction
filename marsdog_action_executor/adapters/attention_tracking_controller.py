"""Session-scoped visual centering controller for the AGV chassis."""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from .agv_adapter import TwistCommand


class AttentionTrackingController:
    """Centre a target and optionally follow it using its apparent size."""

    def __init__(
        self,
        *,
        gain: float = 0.8,
        deadband: float = 0.08,
        activation_deadband: float = 0.14,
        smoothing_alpha: float = 0.15,
        max_angular_z: float = 0.25,
        max_angular_accel: float = 0.25,
        visual_timeout_sec: float = 0.8,
        wake_gain: float = 0.8,
        wake_fallback_sec: float = 2.0,
        direction_sign: float = -1.0,
        follow_target_height: float = 0.68,
        follow_height_deadband: float = 0.05,
        follow_activation_deadband: float = 0.10,
        follow_linear_gain: float = 0.55,
        follow_max_linear_x: float = 0.22,
        follow_max_linear_accel: float = 0.20,
        follow_max_heading_error: float = 0.30,
    ) -> None:
        self._gain = abs(float(gain))
        self._deadband = abs(float(deadband))
        self._activation_deadband = max(
            self._deadband, abs(float(activation_deadband))
        )
        self._smoothing_alpha = min(
            1.0, max(0.01, float(smoothing_alpha))
        )
        self._max_angular_z = abs(float(max_angular_z))
        self._max_angular_accel = abs(float(max_angular_accel))
        self._visual_timeout_sec = max(0.05, float(visual_timeout_sec))
        self._wake_gain = abs(float(wake_gain))
        self._wake_fallback_sec = max(0.0, float(wake_fallback_sec))
        self._direction_sign = math.copysign(1.0, float(direction_sign))
        self._follow_target_height = min(
            1.0, max(0.05, float(follow_target_height))
        )
        self._follow_height_deadband = max(
            0.0, float(follow_height_deadband)
        )
        self._follow_activation_deadband = max(
            self._follow_height_deadband,
            float(follow_activation_deadband),
        )
        self._follow_linear_gain = abs(float(follow_linear_gain))
        self._follow_max_linear_x = abs(float(follow_max_linear_x))
        self._follow_max_linear_accel = abs(float(follow_max_linear_accel))
        self._follow_max_heading_error = max(
            self._activation_deadband,
            abs(float(follow_max_heading_error)),
        )
        self._lock = threading.Lock()
        self._enabled = False
        self._interaction_id = ""
        self._mode = "face_body_centering"
        self._started_at = 0.0
        self._wake_angle_deg = 0.0
        self._visual_error: float | None = None
        self._visual_updated_at = 0.0
        self._body_height: float | None = None
        self._target_track_id: int | None = None
        self._tracking_engaged = False
        self._following_engaged = False
        self._last_command_z = 0.0
        self._last_command_at = 0.0
        self._last_linear_x = 0.0
        self._last_linear_at = 0.0

    @property
    def enabled(self) -> bool:
        with self._lock:
            return self._enabled

    @property
    def interaction_id(self) -> str:
        with self._lock:
            return self._interaction_id

    @property
    def mode(self) -> str:
        with self._lock:
            return self._mode

    def update_control(self, payload: dict[str, Any], now: float | None = None) -> None:
        now = time.monotonic() if now is None else float(now)
        enabled = bool(payload.get("enabled", False))
        interaction_id = str(payload.get("interaction_id", "")).strip()
        requested_mode = str(payload.get("mode", "face_body_centering"))
        mode = (
            "follow_owner"
            if requested_mode in ("follow", "follow_owner")
            else "face_body_centering"
        )
        with self._lock:
            if enabled:
                same_session = (
                    self._enabled
                    and mode == self._mode
                    and (
                        not interaction_id
                        or interaction_id == self._interaction_id
                    )
                )
                if same_session:
                    # The voice/BT side may republish the session state.  It
                    # must not reset the visual EMA and velocity slew state on
                    # every duplicate control message.
                    if interaction_id:
                        self._interaction_id = interaction_id
                    if "wake_angle" in payload:
                        self._wake_angle_deg = self._finite(
                            payload.get("wake_angle", 0.0), 0.0
                        )
                    return
                self._enabled = True
                self._interaction_id = interaction_id
                self._mode = mode
                self._started_at = now
                self._wake_angle_deg = self._finite(
                    payload.get("wake_angle", 0.0), 0.0
                )
                self._visual_error = None
                self._visual_updated_at = 0.0
                self._body_height = None
                self._target_track_id = None
                self._tracking_engaged = False
                self._following_engaged = False
                self._last_command_z = 0.0
                self._last_command_at = now
                self._last_linear_x = 0.0
                self._last_linear_at = now
                return
            if (
                not interaction_id
                or not self._interaction_id
                or interaction_id == self._interaction_id
            ):
                self._enabled = False
                self._interaction_id = ""
                self._mode = "face_body_centering"
                self._visual_error = None
                self._visual_updated_at = 0.0
                self._body_height = None
                self._target_track_id = None
                self._tracking_engaged = False
                self._following_engaged = False
                self._last_command_z = 0.0
                self._last_command_at = now
                self._last_linear_x = 0.0
                self._last_linear_at = now

    def update_visual(self, payload: dict[str, Any], now: float | None = None) -> None:
        now = time.monotonic() if now is None else float(now)
        target = payload.get("active_target")
        error: float | None = None
        body_height: float | None = None
        track_id: int | None = None
        if isinstance(target, dict):
            confidence = self._finite(target.get("confidence", 0.0), 0.0)
            tracking_state = str(target.get("tracking_state", "tracking"))
            age_ms = self._finite(target.get("last_seen_age_ms", 0.0), 0.0)
            if (
                confidence > 0.0
                and tracking_state == "tracking"
                and 0.0 <= age_ms <= self._visual_timeout_sec * 1000.0
            ):
                # The pose torso centre is stable while talking/gesturing;
                # face detector jitter and intermittent face/body switching
                # were the main source of left-right oscillation.
                center = target.get("body_center")
                if not self._valid_center(center):
                    center = target.get("face_center")
                if self._valid_center(center):
                    error = float(center[0]) - 0.5
                    bbox = target.get("bbox")
                    if self._valid_bbox(bbox):
                        body_height = float(bbox[3])
                    try:
                        track_id = int(target.get("track_id", 0))
                    except (TypeError, ValueError):
                        track_id = 0
        with self._lock:
            if error is None:
                # Pose inference can miss one or two frames while the person
                # turns or gestures.  Keep the last filtered target until the
                # normal visual timeout expires instead of alternating motion
                # and zero commands for each temporary miss.
                return
            if track_id != self._target_track_id or self._visual_error is None:
                # Do not blend the previous person's position into a new target.
                self._visual_error = error
                self._body_height = body_height
                self._target_track_id = track_id
                self._tracking_engaged = False
                self._following_engaged = False
            else:
                alpha = self._smoothing_alpha
                self._visual_error = (
                    alpha * error + (1.0 - alpha) * self._visual_error
                )
                if body_height is not None:
                    self._body_height = (
                        body_height
                        if self._body_height is None
                        else alpha * body_height
                        + (1.0 - alpha) * self._body_height
                    )
            self._visual_updated_at = now

    def command(
        self,
        *,
        now: float | None = None,
        suspended: bool = False,
    ) -> TwistCommand | None:
        if suspended:
            return None
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            if not self._enabled:
                return None
            visual_error = self._visual_error
            visual_updated_at = self._visual_updated_at
            started_at = self._started_at
            wake_angle_deg = self._wake_angle_deg
            tracking_engaged = self._tracking_engaged
            following_engaged = self._following_engaged
            body_height = self._body_height
            mode = self._mode

        if (
            visual_error is not None
            and now - visual_updated_at <= self._visual_timeout_sec
        ):
            magnitude = abs(visual_error)
            if tracking_engaged and magnitude <= self._deadband:
                tracking_engaged = False
            elif not tracking_engaged and magnitude >= self._activation_deadband:
                tracking_engaged = True
            with self._lock:
                self._tracking_engaged = tracking_engaged
            desired_angular = (
                self._direction_sign * self._gain * visual_error
                if tracking_engaged else 0.0
            )
            angular = self._slew(desired_angular, now)

            linear = 0.0
            if mode == "follow_owner" and body_height is not None:
                distance_error = self._follow_target_height - body_height
                if (
                    following_engaged
                    and distance_error <= self._follow_height_deadband
                ):
                    following_engaged = False
                elif (
                    not following_engaged
                    and distance_error >= self._follow_activation_deadband
                ):
                    following_engaged = True
                with self._lock:
                    self._following_engaged = following_engaged
                desired_linear = 0.0
                if following_engaged:
                    desired_linear = min(
                        self._follow_max_linear_x,
                        self._follow_linear_gain * max(0.0, distance_error),
                    )
                    # Rotate first when the target is far from the centre;
                    # this avoids driving an arc toward the edge of frame.
                    alignment = max(
                        0.0,
                        1.0
                        - abs(visual_error) / self._follow_max_heading_error,
                    )
                    desired_linear *= alignment
                linear = self._slew_linear(desired_linear, now)
            else:
                self._reset_linear(now)
            return TwistCommand(linear_x=linear, angular_z=angular)

        if now - started_at <= self._wake_fallback_sec:
            relative_deg = (wake_angle_deg + 180.0) % 360.0 - 180.0
            if abs(relative_deg) <= 5.0:
                return TwistCommand()
            angular = self._wake_gain * math.radians(relative_deg)
            return TwistCommand(angular_z=self._slew(angular, now))

        # Lost/stale target is safety-critical: stop immediately, without a
        # deceleration tail that could continue rotating blindly.
        with self._lock:
            self._visual_error = None
            self._visual_updated_at = 0.0
            self._body_height = None
            self._target_track_id = None
            self._last_command_z = 0.0
            self._last_command_at = now
            self._last_linear_x = 0.0
            self._last_linear_at = now
            self._tracking_engaged = False
            self._following_engaged = False
        return TwistCommand()

    def _clamp(self, value: float) -> float:
        return max(-self._max_angular_z, min(self._max_angular_z, value))

    def _slew(self, desired: float, now: float) -> float:
        desired = self._clamp(desired)
        with self._lock:
            elapsed = (
                max(0.0, now - self._last_command_at)
                if self._last_command_at else 0.1
            )
            max_delta = self._max_angular_accel * min(elapsed, 0.5)
            delta = desired - self._last_command_z
            if max_delta > 0.0:
                delta = max(-max_delta, min(max_delta, delta))
            result = self._clamp(self._last_command_z + delta)
            self._last_command_z = result
            self._last_command_at = now
            return result

    def _slew_linear(self, desired: float, now: float) -> float:
        desired = max(0.0, min(self._follow_max_linear_x, desired))
        with self._lock:
            elapsed = (
                max(0.0, now - self._last_linear_at)
                if self._last_linear_at else 0.1
            )
            max_delta = self._follow_max_linear_accel * min(elapsed, 0.5)
            delta = desired - self._last_linear_x
            if max_delta > 0.0:
                delta = max(-max_delta, min(max_delta, delta))
            result = max(
                0.0,
                min(self._follow_max_linear_x, self._last_linear_x + delta),
            )
            self._last_linear_x = result
            self._last_linear_at = now
            return result

    def _reset_linear(self, now: float) -> None:
        with self._lock:
            self._last_linear_x = 0.0
            self._last_linear_at = now

    @staticmethod
    def _valid_bbox(value: Any) -> bool:
        return (
            isinstance(value, (list, tuple))
            and len(value) >= 4
            and all(isinstance(item, (int, float)) for item in value[:4])
            and 0.0 < float(value[2]) <= 1.0
            and 0.0 < float(value[3]) <= 1.0
        )

    @staticmethod
    def _valid_center(value: Any) -> bool:
        return (
            isinstance(value, (list, tuple))
            and len(value) >= 2
            and all(isinstance(item, (int, float)) for item in value[:2])
            and 0.0 <= float(value[0]) <= 1.0
            and 0.0 <= float(value[1]) <= 1.0
            and not (float(value[0]) == 0.0 and float(value[1]) == 0.0)
        )

    @staticmethod
    def _finite(value: Any, default: float) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return default
        return result if math.isfinite(result) else default
