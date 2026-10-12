"""Bind a selected visual target, locate once, and execute one Nav2 goal."""
from __future__ import annotations

import math
import threading
import time

from .person_nav_approach_adapter import PersonNavApproachAdapter
from .target_approach_adapter import TargetApproachResult
from .velocity import TwistCommand
from .visual_target_approach_adapter import VisualTargetApproachAdapter


class VisualNavApproachAdapter(VisualTargetApproachAdapter):
    """Reuse target/lease binding; only Nav2 owns approach velocity."""

    # Match Vision's locate_target_once limits. A behavior may require a
    # larger minimum, but a Goal cannot lower either minimum.
    _MIN_STAND_OFF_M = {
        "human": PersonNavApproachAdapter.MIN_STAND_OFF_M,
        "animal": 0.8,
        "object": 0.2,
    }

    def __init__(self, publish_twist, *, transport, chassis_backend=None, **options):
        # The compatibility visual-servo delegate is used for binding/cache
        # bookkeeping only. It has no physical velocity outlet in this adapter.
        super().__init__(lambda command: None, **options)
        self._nav_transport = transport
        self._publish_stop = publish_twist
        self._chassis_backend = chassis_backend
        self._execution_lock = threading.Lock()
        self._deadline = 0.0

    @property
    def recovery_required(self):
        return bool(getattr(self._nav_transport, "recovery_required", False))

    def _stop(self):
        for _ in range(3):
            self._publish_stop(TwistCommand())

    def cancel_task(self):
        super().cancel_task()
        self._nav_transport.cancel()

    def emergency_stop(self):
        self.cancel_task()
        self._stop()

    def _finish(self, ctx, success, reason, *, canceled=False, timed_out=False, **details):
        self._stop()
        metadata = {
            **details, "controller": "nav2", "ready": success,
            "state": "arrived" if success else "canceled" if canceled else "failed",
            "reason": reason, "recovery_required": self.recovery_required,
        }
        self._store_metadata(ctx, metadata)
        reporter = getattr(ctx, "report_runtime_feedback", None)
        if callable(reporter):
            reporter(1.0 if success else 0.0, self.ACTION_ID, metadata["state"], False)
        return TargetApproachResult(success, reason, metadata, canceled, timed_out)

    def _invalid(self, ctx, reason):
        return self._finish(ctx, False, reason)

    def _canceled(self, ctx, reason):
        return self._finish(ctx, False, reason, canceled=True)

    def execute_task(self, unit_config, ctx, timeout_sec):
        with self._execution_lock:
            if self.recovery_required:
                return self._invalid(ctx, "operator_recovery_required")
            try:
                budget = float(timeout_sec)
            except (ValueError, TypeError):
                return self._invalid(ctx, "invalid_approach_budget")
            if not math.isfinite(budget) or budget <= 0:
                return self._invalid(ctx, "invalid_approach_budget")
            self._deadline = time.monotonic() + budget
            self._stop()
            return super().execute_task(unit_config, ctx, budget)

    def _feedback(self, ctx, phase, progress):
        reporter = getattr(ctx, "report_runtime_feedback", None)
        if callable(reporter):
            reporter(progress, self.ACTION_ID, phase, True)

    def _execute_bound_target(self, unit_config, ctx, timeout_sec, bound, epoch, policy):
        raw = bound["raw"]
        target_type = bound["target_type"]
        target_id = str(raw.get("target_id", ""))
        kind = "human" if target_type == "human" else "object"
        prefix = epoch + ":" + kind + ":"
        if not target_id.startswith(prefix) or not target_id[len(prefix):]:
            return self._invalid(ctx, "stable_visual_target_required")
        requested_id = str(ctx.target.get("target_id", ""))
        if requested_id.startswith(epoch + ":") and requested_id != target_id:
            return self._invalid(ctx, "visual_target_identity_mismatch")
        minimum_distance = max(
            float(policy["minimum_safe_distance_m"]),
            self._MIN_STAND_OFF_M[target_type],
        )
        params = getattr(ctx, "params", {}) or {}
        if "stand_off_distance_m" in params:
            requested_distance = params["stand_off_distance_m"]
            if isinstance(requested_distance, bool):
                return self._invalid(ctx, "invalid_stand_off_distance")
            try:
                distance = float(requested_distance)
            except (TypeError, ValueError, OverflowError):
                return self._invalid(ctx, "invalid_stand_off_distance")
            if not math.isfinite(distance):
                return self._invalid(ctx, "invalid_stand_off_distance")
            if distance < minimum_distance:
                return self._invalid(ctx, "unsafe_stand_off_distance")
        else:
            distance = max(float(policy["desired_distance_m"]), minimum_distance)
        details = {
            "target_type": target_type, "vision_epoch": epoch,
            "bound_target_id": target_id,
            "requested_target_id": str(ctx.target.get("target_id", "")),
            "binding_reason": bound.get("binding_reason", "exact_track"),
            "stand_off_distance_m": distance,
        }
        if self._cancel_requested.is_set() or self._should_stop():
            return self._finish(ctx, False, "canceled_before_localization", canceled=True, **details)
        if time.monotonic() >= self._deadline:
            return self._finish(ctx, False, "approach_budget_expired", timed_out=True, **details)
        self._feedback(ctx, "locating_visual_target", 0.1)
        try:
            located, reason = self._nav_transport.locate_target(
                target_id, target_type, distance, self._deadline, self._cancel_requested,
                min_confidence=float(policy.get("min_confidence", self._bind_min_confidence)),
                required_identity=policy.get("required_identity"),
            )
        except Exception as exc:
            return self._finish(ctx, False, "vision_localization_error:%s" % type(exc).__name__, **details)
        if self._cancel_requested.is_set() or self._should_stop():
            return self._finish(ctx, False, "canceled_during_localization", canceled=True, **details)
        if located is None:
            return self._finish(ctx, False, reason, **details)
        if (located.get("ok") is not True or located.get("target_id") != target_id
                or located.get("target_type") != target_type or located.get("vision_epoch") != epoch):
            return self._finish(ctx, False, "localization_target_mismatch", **details)
        if (located.get("status") != 0
                or not PersonNavApproachAdapter._valid_person_point(located.get("target_point"))):
            return self._finish(ctx, False, "invalid_localization_success", **details)
        if time.monotonic() >= self._deadline:
            return self._finish(ctx, False, "approach_budget_expired", timed_out=True, **details)
        if located.get("navigation_required") is False:
            return self._finish(ctx, True, "already_within_stand_off", navigation_required=False, **details)
        if located.get("navigation_required") is not True:
            return self._finish(ctx, False, "navigation_required_missing", **details)
        pose = located.get("navigation_goal")
        if not self._nav_transport.valid_pose(pose):
            return self._finish(ctx, False, "invalid_navigation_goal", **details)
        prepare = getattr(self._chassis_backend, "prepare_navigation", None)
        finish = getattr(self._chassis_backend, "finish_navigation", None)
        try:
            prepared = not callable(prepare) or bool(prepare())
        except Exception:
            self._nav_transport.recovery_required = True
            return self._finish(ctx, False, "chassis_preflight_exception:operator_recovery_required", **details)
        if not prepared:
            return self._finish(ctx, False, "chassis_navigation_preflight_failed", **details)
        self._feedback(ctx, "navigating_to_visual_target", 0.5)
        settled = True
        try:
            if self._should_stop():
                self._cancel_requested.set()
            if self._cancel_requested.is_set():
                success, canceled, reason = False, True, "canceled_before_navigation"
            elif time.monotonic() >= self._deadline:
                success, canceled, reason = False, False, "navigation_timeout"
            else:
                success, canceled, reason = self._nav_transport.navigate(
                    pose, self._deadline, self._cancel_requested,
                )
        except Exception:
            self._nav_transport.recovery_required = True
            # A transport exception may follow an accepted Goal. Keep its
            # ownership latched and request cancellation before returning.
            try:
                self._nav_transport.cancel()
            except Exception:
                pass
            success, canceled, reason = False, False, "navigation_exception:operator_recovery_required"
        finally:
            # Cancel acknowledgement or an unknown Result cannot release the
            # chassis mode. The shared transport retains its actual Goal.
            if not self.recovery_required:
                try:
                    settled = not callable(finish) or finish() is not False
                except Exception:
                    settled = False
        if not settled:
            self._nav_transport.recovery_required = True
            success, canceled, reason = False, False, "chassis_navigation_settle_failed:operator_recovery_required"
        if self._cancel_requested.is_set() or self._should_stop():
            success = False
            canceled = not self.recovery_required
        return self._finish(
            ctx, success, reason, canceled=canceled, timed_out=reason == "navigation_timeout",
            navigation_required=True, **details,
        )
