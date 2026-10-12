"""Unitree Go2 SportMode backend for high-level motion requests.

The official ROS 2 bridge consumes :class:`unitree_api.msg.Request` on
``/api/sport/request``.  This module keeps that transport at the edge and
exposes the same platform-neutral chassis protocol used by visual servoing,
Nav2 coordination, UWB shutdown, and staged ``ACT_*`` execution.
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping

from .velocity import TwistCommand

logger = logging.getLogger(__name__)


SPORT_API_IDS = {
    "damp": 1001,
    "balance_stand": 1002,
    "stop_move": 1003,
    "stand_up": 1004,
    "stand_down": 1005,
    "recovery_stand": 1006,
    "euler": 1007,
    "move": 1008,
    "sit": 1009,
    "rise_sit": 1010,
    "speed_level": 1015,
    "hello": 1016,
    "stretch": 1017,
    "content": 1020,
    "dance1": 1022,
    "dance2": 1023,
    "scrape": 1029,
    "front_pounce": 1032,
    "heart": 1036,
}

_VECTOR_COMMANDS = frozenset({"move", "euler"})
_STANDING_COMMANDS = frozenset({"stand_up", "recovery_stand", "rise_sit"})
_POSTURE_COMMANDS = _STANDING_COMMANDS | {"stand_down", "sit", "damp"}
_REQUIRES_STANDING = frozenset({
    "move", "euler", "balance_stand", "sit", "hello", "stretch",
    "content", "dance1", "dance2", "scrape", "front_pounce", "heart",
    "nav2_spin",
})


@dataclass(frozen=True)
class Go2SportRequest:
    """Transport-neutral representation of one SportMode request."""

    api_id: int
    parameter: str = ""


class Ros2Go2SportPublisher:
    """Publish official ``unitree_api/msg/Request`` SportMode messages."""

    def __init__(self, node: Any, request_topic: str = "/api/sport/request") -> None:
        try:
            from unitree_api.msg import Request
        except ImportError as exc:  # pragma: no cover - board dependency
            raise RuntimeError(
                "unitree_api/msg/Request is unavailable; source the "
                "unitree_ros2 workspace before starting chassis_type:=go2"
            ) from exc
        self._node = node
        self._message_type = Request
        self._topic = str(request_topic)
        self._publisher = node.create_publisher(Request, self._topic, 10)

    @property
    def topic(self) -> str:
        return self._topic

    @property
    def matched_subscribers(self) -> int:
        return int(self._publisher.get_subscription_count())

    def __call__(self, request: Go2SportRequest) -> bool:
        message = self._message_type()
        message.header.identity.api_id = int(request.api_id)
        message.parameter = str(request.parameter)
        matched = self.matched_subscribers
        self._publisher.publish(message)
        detail = (
            f"Go2 SportMode -> {self._topic}: api_id={request.api_id}, "
            f"parameter={request.parameter or '{}'}, "
            f"matched_subscribers={matched}"
        )
        if matched == 0:
            self._node.get_logger().warning(
                detail + "; request has no matched Go2 SportMode subscriber"
            )
            return False
        self._node.get_logger().info(detail)
        return True


class Go2ChassisBackend:
    """Execute configured semantic actions through Go2 high-level SportMode.

    The backend deliberately does not implement low-level joint trajectories.
    Only explicitly configured native actions or labeled safe proxies execute.
    """

    def __init__(
        self,
        request_publisher: Callable[[Go2SportRequest], bool | None],
        action_sequences: Mapping[str, list[Mapping[str, Any]]],
        *,
        action_plans: Mapping[str, Mapping[str, Any]] | None = None,
        unsupported_actions: Mapping[str, str] | None = None,
        spin_transport: Any = None,
        publish_rate_hz: float = 10.0,
        max_linear_x: float = 0.30,
        min_linear_x: float = 0.0,
        max_linear_y: float = 0.20,
        max_angular_z: float = 1.00,
        stop_publish_count: int = 3,
        stand_up_wait_sec: float = 2.5,
        should_stop: Callable[[], bool] | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        rate = float(publish_rate_hz)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("publish_rate_hz must be finite and > 0")
        stand_wait = float(stand_up_wait_sec)
        if not math.isfinite(stand_wait) or not 0.0 < stand_wait <= 10.0:
            raise ValueError("stand_up_wait_sec must be finite and within (0, 10]")
        self._publish_request = request_publisher
        self._action_sequences = {
            str(action_id): [dict(item) for item in sequence]
            for action_id, sequence in action_sequences.items()
        }
        self._action_plans = {str(k): dict(v) for k, v in (action_plans or {}).items()}
        self._unsupported_actions = dict(unsupported_actions or {})
        self._spin_transport = spin_transport
        self._period_sec = 1.0 / rate
        self._max_linear_x = abs(float(max_linear_x))
        self._min_linear_x = abs(float(min_linear_x))
        self._max_linear_y = abs(float(max_linear_y))
        self._max_angular_z = abs(float(max_angular_z))
        if not all(math.isfinite(v) for v in (
            self._max_linear_x, self._min_linear_x, self._max_linear_y, self._max_angular_z
        )) or self._min_linear_x > self._max_linear_x:
            raise ValueError("Go2 velocity limits must be finite, with min_linear_x <= max_linear_x")
        self._stop_publish_count = max(1, int(stop_publish_count))
        self._stand_up_wait_sec = stand_wait
        # This records completed command preparation, not measured posture.
        # Unknown startup posture and canceled transitions require StandUp.
        self._standing_prepared = False
        self._balance_prepared = False
        self._stand_up_ready_at: float | None = None
        self._last_error = ""
        self._should_stop = should_stop or (lambda: False)
        self._monotonic = monotonic
        self._sleep = sleep
        self._cancel_requested = threading.Event()
        self._execution_lock = threading.Lock()
        self._spin_velocity_allowed = False
        self._behavior_active = False
        self._navigation_velocity_allowed = False

    @property
    def mapped_actions(self) -> set[str]:
        return set(self._action_sequences)

    def can_execute(self, unit_id: str) -> bool:
        return str(unit_id) in self._action_sequences

    def get_unit_overrides(self, unit_id: str) -> dict[str, Any]:
        plan = self._action_plans.get(unit_id, {})
        # Firmware waits are fixed; emotion speed/duration scales cannot
        # shorten them. The outer Behavior Goal deadline still applies.
        overrides = {"scale_duration": False}
        if "timeout_sec" in plan:
            overrides["timeout_sec"] = plan["timeout_sec"]
        return overrides

    def begin_behavior(self) -> None:
        """Reserve external velocity ingress for this complete behavior."""
        self._behavior_active = True
        self._navigation_velocity_allowed = False

    def end_behavior(self) -> None:
        self._navigation_velocity_allowed = False
        self._behavior_active = False

    def publish_external_velocity(self, command: TwistCommand) -> None:
        self.publish_velocity(command, external=True)

    def reject_unsupported(self, unit_id: str) -> bool:
        self._last_error = "go2_unsupported_action:%s:%s" % (
            unit_id, self._unsupported_actions.get(unit_id, "no native action mapping"))
        logger.error(self._last_error)
        self._publish_stop()
        return False

    @property
    def last_error(self) -> str:
        return self._last_error

    def prepare_navigation(self) -> bool:
        """Explicitly stand before Nav2/Spin or an external motion publisher.

        A recorded StandUp preparation can be reused. This is command
        preparation, not measured posture or a navigation success check.
        """
        with self._execution_lock:
            self._navigation_velocity_allowed = False
            self._cancel_requested.clear()
            self._last_error = ""
            if getattr(self._spin_transport, "recovery_required", False):
                self._invalidate_preparation()
                self._last_error = "nav2_recovery_required:operator_recovery_required"
                self._publish_stop()
                return False
            # Every new navigation Goal selects balance mode once. Reusing
            # completed StandUp avoids a second transition and fixed wait.
            self._balance_prepared = False
            ready = self._ensure_standing()
            self._navigation_velocity_allowed = ready
            return ready

    def finish_navigation(self) -> bool:
        """Restore balance best effort, preserving the navigation Result."""
        self._navigation_velocity_allowed = False
        try:
            self._publish_stop()
            if (self._standing_prepared and not self._must_stop(None, None)
                    and not getattr(self._spin_transport, "recovery_required", False)):
                self._balance_prepared = False
                self._ensure_balance_stand()
        except Exception:
            self._invalidate_preparation()
            logger.exception("Go2 cleanup after external motion failed")
        return True

    def finish_behavior(self, ctx: Any, behavior_name: str) -> bool:
        """Leave a completed expression chain ready for its next navigation.

        Explicit posture/hold commands keep their requested posture. Failed or
        canceled chains must never trigger an automatic StandUp here.
        """
        if behavior_name in {
            "sit_down", "lie_down", "play_dead", "hold_position", "wait_in_place",
            "quiet", "emergency_stop", "respond_stop_gesture",
        }:
            return True
        with self._execution_lock:
            readiness = {"prepared": False, "physical_posture_confirmed": False, "reason": ""}
            ctx.metadata["go2_readiness"] = readiness
            if self._must_stop(ctx, None):
                self._invalidate_preparation()
                self._publish_stop()
                self._last_error = "go2_behavior_finish_interrupted"
                readiness["reason"] = self._last_error
                return False
            if not self._publish_stop():
                self._invalidate_preparation()
                readiness["reason"] = self._last_error
                return False
            self._balance_prepared = False
            ok = self._ensure_standing(ctx)
            ctx.metadata["go2_readiness"] = {
                "prepared": ok, "physical_posture_confirmed": False,
                "reason": self._last_error,
            }
            return ok

    def publish_velocity(self, command: TwistCommand, *, external: bool = False) -> None:
        """Translate a body-frame velocity into Go2 ``Move``/``StopMove``."""
        if command.is_zero:
            if self._must_stop(None, None):
                self._invalidate_preparation()
            self._send_command("stop_move")
            return
        if external and self._external_velocity_blocked():
            return
        # A visual controller has already checked freshness before this call.
        # Never sleep here or queue its command behind another action: after
        # StandUp, only a newly received velocity may be forwarded.
        if not self._execution_lock.acquire(blocking=False):
            # Spin owns the sequence lock while Nav2 feeds live /cmd_vel.
            # It has already completed posture preparation; never start a
            # second preparation from inside this window.
            if (self._spin_velocity_allowed and self._standing_prepared
                    and self._balance_prepared and not self._must_stop(None, None)):
                self._forward_velocity(command)
            return
        try:
            if external and self._external_velocity_blocked():
                return
            if getattr(self._spin_transport, "recovery_required", False):
                self._invalidate_preparation()
                self._publish_stop()
                return
            if self._must_stop(None, None):
                self._invalidate_preparation()
                self._publish_stop()
                return
            if not self._standing_prepared:
                if self._stand_up_ready_at is None and not self._start_stand_up():
                    self._publish_stop()
                    return
                ready_at = self._stand_up_ready_at
                if ready_at is None or self._must_stop(None, None):
                    self._publish_stop()
                    return
                if self._monotonic() < ready_at:
                    return
                self._standing_prepared = True
                self._stand_up_ready_at = None
                self._last_error = ""
            if not self._ensure_balance_stand():
                self._publish_stop()
                return
            if self._must_stop(None, None):
                self._invalidate_preparation()
                self._publish_stop()
                return
            self._forward_velocity(command)
        except Exception:
            self._invalidate_preparation()
            self._last_error = self._last_error or "go2_velocity_publish_failed"
            logger.exception("Go2 velocity preparation/publication failed")
            self._publish_stop()
        finally:
            self._execution_lock.release()

    def _external_velocity_blocked(self) -> bool:
        return (self._behavior_active and not self._navigation_velocity_allowed
                and not self._spin_velocity_allowed)

    def _forward_velocity(self, command: TwistCommand) -> None:
        if not self._send_command(
            "move", x=self._floor_linear_x(self._clamp(command.linear_x, self._max_linear_x)),
            y=self._clamp(command.linear_y, self._max_linear_y),
            z=self._clamp(command.angular_z, self._max_angular_z),
        ):
            self._invalidate_preparation()
            self._publish_stop()

    @property
    def twist_publisher(self) -> Callable[[TwistCommand], None]:
        """Compatibility outlet used by the existing closed-loop adapters."""
        return self.publish_velocity

    @property
    def publish_twist(self) -> Callable[[TwistCommand], None]:
        return self.publish_velocity

    def execute_step(
        self,
        unit_config: Mapping[str, Any],
        ctx: Any = None,
        duration: float | None = None,
    ) -> bool:
        unit_id = str(unit_config.get("unit_id", ""))
        sequence = self._action_sequences.get(unit_id)
        self._last_error = ""
        if sequence is None:
            return self.reject_unsupported(unit_id)
        plan = self._action_plans.get(unit_id, {})
        # Close the preceding navigation window before any body unit or hold.
        self._navigation_velocity_allowed = False
        metadata = {
            "requested_act": unit_id,
            "fidelity": plan.get("fidelity", "unclassified"),
            "description": plan.get("description", ""),
            "commanded_posture": plan.get("commanded_posture", ""),
            "completion": "command_sequence_wait_only",
            "physical_completion_confirmed": False,
        }
        if ctx is not None:
            ctx.metadata["go2_action"] = metadata
            ctx.metadata.setdefault("go2_actions", []).append(metadata)
        logger.info("Go2 action %s: fidelity=%s, commanded_posture=%s, %s",
                    unit_id, metadata["fidelity"], metadata["commanded_posture"],
                    metadata["description"])
        if getattr(ctx, "motion_state", "active") == "stationary":
            metadata.update(commanded_posture="", completion="stationary_hold",
                            effective_action="stop_move", fidelity="proxy")
            return self.hold_position(duration)
        if unit_id == "ACT_SYSTEM_EMERGENCY_STOP":
            self._cancel_requested.set()
            self._invalidate_preparation()
            return self._publish_stop()

        try:
            local_budget = max(0.0, float(duration or 0.0))
        except (TypeError, ValueError):
            local_budget = 0.0
        with self._execution_lock:
            self._cancel_requested.clear()
            self._last_error = ""
            ok = True
            euler_active = False
            try:
                if any(item.get("command") != "stop_move" for item in sequence) and bool(
                    getattr(self._spin_transport, "recovery_required", False)
                ):
                    self._last_error = "nav2_recovery_required:operator_recovery_required"
                    ok = False
                    return False
                # Keep the expression's configured duration after the bounded
                # posture preflight; the outer Behavior Goal still limits both.
                first_command = next(
                    (str(item.get("command", "")).strip() for item in sequence
                     if item.get("command") != "stop_move"),
                    "",
                )
                if first_command == "euler":
                    # Move (including an external bridge) changes control
                    # mode. Select BalanceStand again before this expression.
                    self._balance_prepared = False
                if first_command in _REQUIRES_STANDING:
                    ok = self._ensure_standing(ctx)
                    if not ok:
                        return False
                deadline = (
                    self._monotonic() + local_budget if local_budget > 0.0 else None
                )
                euler_mode_selected = first_command == "euler"
                for item in sequence:
                    if self._must_stop(ctx, deadline):
                        ok = False
                        break
                    command = str(item.get("command", "")).strip()
                    if command == "euler" and not euler_mode_selected:
                        self._balance_prepared = False
                    item_duration = max(0.0, float(item.get("duration_sec", 0.0)))
                    if command != "nav2_spin" and deadline is not None and (
                        self._monotonic() + item_duration > deadline + 1e-9
                    ):
                        self._last_error = ("go2_stand_up_budget_exhausted" if command in _STANDING_COMMANDS
                                            else "go2_action_budget_exhausted")
                        ok = False
                        break
                    if (
                        command in _REQUIRES_STANDING
                        and not self._ensure_standing(ctx, deadline)
                    ):
                        ok = False
                        break
                    if (
                        command in _STANDING_COMMANDS
                        and deadline is not None
                        and self._monotonic() + item_duration > deadline
                    ):
                        self._last_error = "go2_stand_up_budget_exhausted"
                        ok = False
                        break
                    if command == "move":
                        ok = self._run_move(item, item_duration, ctx, deadline)
                    elif command == "nav2_spin":
                        if self._spin_transport is None:
                            self._last_error = "go2_spin_transport_unavailable"
                            ok = False
                        else:
                            spin_deadline = self._monotonic() + item_duration
                            if deadline is not None:
                                spin_deadline = min(spin_deadline, deadline)
                            remaining = getattr(ctx, "remaining_runtime_sec", None)
                            if callable(remaining):
                                budget = remaining(self._monotonic())
                                if budget is not None:
                                    spin_deadline = min(spin_deadline, self._monotonic() + budget)
                            class CancelView:
                                def is_set(view) -> bool:
                                    return self._must_stop(ctx, None)
                            self._spin_velocity_allowed = True
                            try:
                                ok, canceled, reason = self._spin_transport.spin(
                                    float(item["turns"]) * math.tau, spin_deadline, CancelView())
                            finally:
                                self._spin_velocity_allowed = False
                            metadata.update(
                                completion="nav2_spin_result", spin_turns=float(item["turns"]),
                                nav2_terminal_status=getattr(self._spin_transport, "last_terminal_status", None),
                                nav2_reason=reason,
                            )
                            if not ok:
                                self._last_error = reason or ("spin_canceled" if canceled else "spin_failed")
                    else:
                        ok = self._send_command(
                            command,
                            x=item.get("x"),
                            y=item.get("y"),
                            z=item.get("z"),
                            data=item.get("data"),
                        )
                        if ok and command == "euler":
                            euler_mode_selected = True
                            euler_active = any(
                                float(item.get(axis, 0.0)) != 0.0
                                for axis in ("x", "y", "z")
                            )
                        if ok:
                            ok = self._wait(item_duration, ctx, deadline)
                        if ok and command in _STANDING_COMMANDS:
                            self._standing_prepared = True
                            self._stand_up_ready_at = None
                    if command not in {"euler", "stop_move"}:
                        euler_mode_selected = False
                    if not ok:
                        break
            except Exception:
                logger.exception("Go2 SportMode sequence %s failed", unit_id)
                self._last_error = self._last_error or "go2_sequence_error"
                ok = False
            finally:
                # Cancel/failure may occur before the sequence's neutral pose.
                # StopMove alone does not reset a held SportMode Euler pose.
                if euler_active:
                    ok = self._send_command(
                        "euler", x=0.0, y=0.0, z=0.0
                    ) and ok
                if not ok:
                    self._invalidate_preparation()
                try:
                    stopped = self._publish_stop()
                except Exception:
                    logger.exception("Go2 sequence final stop failed")
                    stopped = False
                    self._last_error = self._last_error or "go2_stop_publish_failed"
                # Best-effort stop does not reinterpret a real Spin Result.
                if metadata["completion"] != "nav2_spin_result":
                    ok = stopped and ok
                    if (ok and plan.get("commanded_posture") == "standing"
                            and self._standing_prepared):
                        self._balance_prepared = False
                        # A unit tail selects balance only. StandUp belongs
                        # to an explicit next motion or whole-chain finish.
                        ok = not self._must_stop(ctx, None) and self._ensure_balance_stand()
                else:
                    # Cleanup failures cannot rewrite an authoritative Spin Result.
                    if ok and not self._must_stop(ctx, None):
                        self._balance_prepared = False
                        self._ensure_balance_stand()
                    elif self._must_stop(ctx, None):
                        self._invalidate_preparation()
                    self._last_error = "" if ok else str(metadata["nav2_reason"])
                if not ok:
                    self._invalidate_preparation()
            return ok

    def hold_position(self, duration_sec: float | None = None) -> bool:
        try:
            duration = max(0.0, float(duration_sec or 0.0))
        except (TypeError, ValueError):
            duration = 0.0
        deadline = self._monotonic() + duration if duration > 0.0 else None
        ok = self._send_command("stop_move")
        while ok and deadline is not None and self._monotonic() < deadline:
            if self._must_stop(None, deadline):
                self._invalidate_preparation()
                return False
            self._sleep(min(self._period_sec, deadline - self._monotonic()))
            ok = self._send_command("stop_move")
        return self._publish_stop() and ok

    def cancel_step(self, step: Any = None) -> None:
        del step
        self._cancel_requested.set()
        self._spin_velocity_allowed = False
        self._navigation_velocity_allowed = False
        self._invalidate_preparation()
        if self._spin_transport is not None:
            self._spin_transport.cancel()
        self._publish_stop()

    def emergency_stop(self) -> None:
        self.cancel_step()

    def _invalidate_preparation(self) -> None:
        self._standing_prepared = False
        self._balance_prepared = False
        self._stand_up_ready_at = None

    def _start_stand_up(self) -> bool:
        logger.info("Go2 motion preparation: StandUp, wait %.2fs", self._stand_up_wait_sec)
        if not self._send_command("stop_move") or not self._send_command("stand_up"):
            self._invalidate_preparation()
            self._last_error = "go2_stand_up_publish_failed"
            return False
        self._stand_up_ready_at = self._monotonic() + self._stand_up_wait_sec
        return True

    def _ensure_standing(self, ctx: Any = None, deadline: float | None = None) -> bool:
        if self._must_stop(ctx, deadline):
            self._standing_prepared = False
            self._balance_prepared = False
            self._stand_up_ready_at = None
            self._last_error = "go2_stand_up_interrupted"
            self._publish_stop()
            return False
        if self._standing_prepared and self._balance_prepared:
            return True
        end = (
            self._monotonic() if self._standing_prepared else (
                self._stand_up_ready_at
                if self._stand_up_ready_at is not None
                else self._monotonic() + self._stand_up_wait_sec
            )
        )
        if deadline is not None and end > deadline:
            self._invalidate_preparation()
            self._last_error = "go2_stand_up_budget_exhausted"
            self._publish_stop()
            return False
        try:
            if not self._standing_prepared:
                if self._stand_up_ready_at is None and not self._start_stand_up():
                    return False
                wait_sec = max(0.0, self._stand_up_ready_at - self._monotonic())
                if not self._wait(wait_sec, ctx, deadline):
                    self._last_error = "go2_stand_up_interrupted"
                    return False
                self._standing_prepared = True
                self._stand_up_ready_at = None
            if not self._ensure_balance_stand():
                return False
            if self._must_stop(ctx, deadline):
                self._standing_prepared = False
                self._balance_prepared = False
                self._last_error = "go2_stand_up_interrupted"
                return False
            self._last_error = ""
            return True
        except Exception:
            self._standing_prepared = False
            self._balance_prepared = False
            self._last_error = "go2_stand_up_publish_failed"
            logger.exception("Go2 standing preparation failed")
            return False
        finally:
            if not self._standing_prepared:
                self._stand_up_ready_at = None
                self._publish_stop()

    def _ensure_balance_stand(self) -> bool:
        """Prepare the walking posture once, after the StandUp wait.

        StandUp completion only records a position stand. The independent
        BalanceStand request must precede Move or handing velocity to Nav2.
        Publication success is not physical posture confirmation.
        """
        if self._balance_prepared:
            return True
        try:
            sent = self._send_command("balance_stand")
        except Exception:
            sent = False
            logger.exception("Go2 BalanceStand publication failed")
        if not sent:
            self._invalidate_preparation()
            self._last_error = "go2_balance_stand_publish_failed"
            return False
        self._balance_prepared = True
        return True

    def _run_move(
        self,
        item: Mapping[str, Any],
        duration: float,
        ctx: Any,
        deadline: float | None,
    ) -> bool:
        end = self._monotonic() + duration
        if deadline is not None:
            end = min(end, deadline)
        if end <= self._monotonic():
            return False
        while self._monotonic() < end:
            if self._must_stop(ctx, deadline):
                return False
            if not self._send_command(
                "move",
                x=self._floor_linear_x(
                    self._clamp(item.get("x", 0.0), self._max_linear_x)
                ),
                y=self._clamp(item.get("y", 0.0), self._max_linear_y),
                z=self._clamp(item.get("z", 0.0), self._max_angular_z),
            ):
                return False
            self._sleep(min(self._period_sec, end - self._monotonic()))
        # Reaching the local deadline exactly means the bounded command has
        # completed; only cancellation may turn that completed segment into a
        # failure.  The caller still enforces the shared whole-Goal deadline.
        return not self._must_stop(ctx, None)

    def _wait(self, duration: float, ctx: Any, deadline: float | None) -> bool:
        end = self._monotonic() + duration
        if deadline is not None:
            end = min(end, deadline)
        while self._monotonic() < end:
            if self._must_stop(ctx, deadline):
                return False
            self._sleep(min(0.05, end - self._monotonic()))
        return not self._must_stop(ctx, None)

    def _must_stop(self, ctx: Any, deadline: float | None) -> bool:
        if self._cancel_requested.is_set() or self._should_stop():
            return True
        if bool(getattr(ctx, "cancel_requested", False)):
            return True
        remaining = getattr(ctx, "remaining_runtime_sec", None)
        if callable(remaining):
            budget = remaining(self._monotonic())
            if budget is not None and budget <= 0.0:
                return True
        return deadline is not None and self._monotonic() >= deadline

    def _send_command(
        self,
        command: str,
        *,
        x: Any = None,
        y: Any = None,
        z: Any = None,
        data: Any = None,
    ) -> bool:
        api_id = SPORT_API_IDS.get(command)
        if api_id is None:
            logger.error("Unsupported Go2 SportMode command: %s", command)
            self._last_error = "go2_unsupported_command:%s" % command
            return False
        parameter = ""
        if command in _VECTOR_COMMANDS:
            parameter = json.dumps(
                {"x": float(x or 0.0), "y": float(y or 0.0), "z": float(z or 0.0)},
                separators=(",", ":"),
            )
        elif command == "speed_level":
            if type(data) is not int or data not in {-1, 0, 1}:
                self._last_error = "go2_invalid_speed_level"
                return False
            parameter = json.dumps({"data": data}, separators=(",", ":"))
        if command not in {"euler", "balance_stand", "stop_move", "move", "speed_level"}:
            self._balance_prepared = False
        if command in _POSTURE_COMMANDS:
            self._standing_prepared = False
            self._balance_prepared = False
            self._stand_up_ready_at = None
        try:
            result = self._publish_request(Go2SportRequest(api_id, parameter))
        except Exception:
            logger.exception("Go2 request publication failed: %s", command)
            result = False
        if result is False:
            # Latch transport/preparation failure. A continuous old velocity
            # stream cannot retry StandUp and resume motion on its own; only
            # a new explicit action/preparation clears this gate.
            self._cancel_requested.set()
            self._invalidate_preparation()
            self._last_error = self._last_error or "go2_publish_failed:%s" % command
        return result is not False

    def _publish_stop(self) -> bool:
        ok = True
        for _ in range(self._stop_publish_count):
            ok = self._send_command("stop_move") and ok
        return ok

    def _floor_linear_x(self, value: float) -> float:
        """Apply the configured device-specific minimum translation speed.

        This is a deployment setting, not a universal SportMode protocol
        threshold. Zero stays zero and the maximum limit remains enforced.
        """
        if self._min_linear_x <= 0.0:
            return value
        if 0.0 < abs(value) < self._min_linear_x:
            return math.copysign(self._min_linear_x, value)
        return value

    @staticmethod
    def _clamp(value: Any, limit: float) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(number):
            return 0.0
        return max(-limit, min(limit, number))
