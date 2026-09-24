"""Lite3 chassis backend using ``/cmd_vel`` and ``/simple_cmd``.

The transport-facing classes import Lite3 ROS messages lazily.  The core
backend remains ROS-independent so command gating and status-based completion
can be tested without the board workspace.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping

from .velocity import Ros2TwistPublisher, TwistCommand

logger = logging.getLogger(__name__)


CMD_SWITCH_JOYSTICK_MODE = 0x21000C02
CMD_SWITCH_VISION_MODE = 0x21000C03
CMD_SWITCH_IN_PLACE_MODE = 0x21010D05
CMD_SWITCH_MOVING_MODE = 0x21010D06
CMD_POSE_ROLL = 0x21010131
CMD_SOFT_EMERGENCY_STOP = 0x21020C0E
CMD_VOICE = 0x21010C0A
CMD_STAND_LIE_TOGGLE = 0x21010202
CMD_TWIST_JUMP = 0x2101020D
CMD_ROLL_OVER = 0x21010205
CMD_BACKFLIP = 0x21010502
CMD_JUMP_FORWARD = 0x2101050B
CMD_MOONWALK = 0x2101030C

# These commands are deliberately unavailable as raw ``simple`` behavior
# steps.  ``CMD_STAND_LIE_TOGGLE`` is used only by the state-aware
# ``target_posture`` controller, which fixes its direction from fresh feedback
# and never retries blindly.
#
# ``CMD_MOONWALK`` is blocked for a different reason: it is not a one-shot
# action but an unbounded *gait*.  Sending it puts the dog into 太空步 and it
# keeps stepping until something explicitly commands it out; ``gait_state``
# stays 12 and never returns to 0 on its own.  That is incompatible with the
# bounded "reach a terminal state within N seconds" contract every plan step
# relies on.  Measured on the real dog (2026-09-18): the command was acked as
# ``(basic=6, gait=12, motion=1)``, the backend timed out at 4.6 s as designed,
# and the dog kept moonwalking for another 22 s until it entered 失控保护
# (``basic_state=8``), passed through an undocumented ``basic_state=99`` and
# finally lay down on its own -- leaving ``gait_state=12`` stuck, which makes
# ``_check_simple_preconditions`` reject *every* subsequent action.
BLOCKED_BEHAVIOR_COMMANDS = {
    CMD_VOICE,
    CMD_POSE_ROLL,
    CMD_SOFT_EMERGENCY_STOP,
    CMD_STAND_LIE_TOGGLE,
    CMD_TWIST_JUMP,
    CMD_ROLL_OVER,
    CMD_BACKFLIP,
    CMD_JUMP_FORWARD,
    CMD_MOONWALK,
}

# Lite3 documents all joystick axes over [-32767, 32767].  The per-axis
# values below are dead-zone boundaries, not maximum magnitudes: commands at
# or below them are intentionally treated as zero by the motion controller.
POSE_AXIS_MAX_ABS = 32767
POSE_AXIS_DEAD_ZONES = {
    0x21010130: 6553,   # pitch
    0x21010135: 9553,   # yaw
    0x21010102: 20000,  # body height
}

STATE_LIE = 1
STATE_STAND = 6
STATE_LOST_CONTROL = 8


@dataclass(frozen=True)
class Lite3SimpleCommand:
    """Transport-neutral representation of ``MotionSimpleCMD``."""

    cmd_code: int
    value: int = 0
    command_type: int = 0


@dataclass(frozen=True)
class Lite3RobotStatus:
    """Fields from ``transfer_interfaces/msg/RobotStatus`` used by Action."""

    basic_state: int
    gait_state: int
    motion_state: int
    battery_level: float
    ultrasound_forward: float = 0.0
    ultrasound_backward: float = 0.0


class Ros2Lite3SimpleCommandPublisher:
    """Publish ``MotionSimpleCMD`` and report missing downstream consumers."""

    def __init__(
        self,
        node: Any,
        topic: str = "/simple_cmd",
        *,
        discovery_timeout_sec: float = 0.3,
    ) -> None:
        try:
            from transfer_interfaces.msg import MotionSimpleCMD
        except ImportError as exc:  # pragma: no cover - board dependency
            raise RuntimeError(
                "transfer_interfaces/msg/MotionSimpleCMD is unavailable; "
                "source the Lite3_ROS workspace before starting "
                "chassis_type:=lite3"
            ) from exc
        self._node = node
        self._message_type = MotionSimpleCMD
        self._topic = str(topic)
        self._publisher = node.create_publisher(MotionSimpleCMD, self._topic, 10)
        # DDS discovery is asynchronous.  ``get_subscription_count()`` reports
        # zero for a healthy motion_sender during the first moments after
        # either end restarts, so the match table is re-read for a bounded
        # interval instead of declaring a transport failure immediately.
        self._discovery_timeout_sec = max(0.0, float(discovery_timeout_sec))

    @property
    def topic(self) -> str:
        return self._topic

    @property
    def matched_subscribers(self) -> int:
        return int(self._publisher.get_subscription_count())

    def _await_match(self) -> bool:
        """Re-read the match table for a bounded interval after discovery."""
        deadline = time.monotonic() + self._discovery_timeout_sec
        while time.monotonic() < deadline:
            time.sleep(0.02)
            if self.matched_subscribers > 0:
                return True
        return self.matched_subscribers > 0

    def __call__(self, command: Lite3SimpleCommand) -> bool:
        message = self._message_type()
        message.cmd_code = int(command.cmd_code)
        message.size = int(command.value)
        message.type = int(command.command_type)
        if self.matched_subscribers == 0:
            # Still unmatched after the bounded wait: publish anyway (the DDS
            # layer may already route it) but report the miss so the caller
            # still fails closed on a genuinely absent motion_sender.
            self._await_match()
        matched = self.matched_subscribers
        self._publisher.publish(message)
        detail = (
            f"Lite3 SimpleCMD -> {self._topic}: "
            f"cmd_code=0x{command.cmd_code:08X}, value={command.value}, "
            f"matched_subscribers={matched}"
        )
        if matched == 0:
            self._node.get_logger().warning(
                detail + "; command has no matched motion_sender subscriber"
            )
            return False
        self._node.get_logger().info(detail)
        return True


class Ros2Lite3StatusSubscriber:
    """Translate ``RobotStatus`` messages into the core status snapshot."""

    def __init__(
        self,
        node: Any,
        callback: Callable[[Lite3RobotStatus], None],
        topic: str = "/robot_status",
    ) -> None:
        try:
            from transfer_interfaces.msg import RobotStatus
        except ImportError as exc:  # pragma: no cover - board dependency
            raise RuntimeError(
                "transfer_interfaces/msg/RobotStatus is unavailable; source "
                "the Lite3_ROS workspace before starting chassis_type:=lite3"
            ) from exc
        self._callback = callback
        self.subscription = node.create_subscription(
            RobotStatus,
            str(topic),
            self._on_status,
            10,
        )

    def _on_status(self, message: Any) -> None:
        self._callback(
            Lite3RobotStatus(
                basic_state=int(message.basic_state),
                gait_state=int(message.gait_state),
                motion_state=int(message.motion_state),
                battery_level=float(message.battery_level),
                ultrasound_forward=float(message.ultrasound_forward),
                ultrasound_backward=float(message.ultrasound_backward),
            )
        )


class Lite3ChassisBackend:
    """Execute approved ``ACT_*`` plans against the Lite3 motion host.

    Plans marked as proxies or not hardware-verified are rejected unless their
    corresponding runtime gates are explicitly enabled.  The APP voice command
    channel and software emergency-stop command are rejected unconditionally.
    """

    def __init__(
        self,
        command_publisher: Callable[[Lite3SimpleCommand], bool | None],
        twist_publisher: Callable[[TwistCommand], None],
        action_plans: Mapping[str, Mapping[str, Any]],
        *,
        allow_proxies: bool = False,
        allow_unverified: bool = False,
        accepted_unverified_actions: tuple[str, ...] | list[str] = (),
        status_timeout_sec: float = 1.0,
        min_battery_percent: float = 25.0,
        minimum_backward_clearance_m: float = 0.60,
        mode_settle_sec: float = 0.10,
        navigation_settle_timeout_sec: float = 2.0,
        navigation_stable_samples: int = 5,
        posture_recovery_timeout_sec: float = 6.0,
        publish_rate_hz: float = 20.0,
        max_linear_x: float = 0.20,
        max_linear_y: float = 0.15,
        max_angular_z: float = 0.80,
        stop_publish_count: int = 5,
        command_repeat: int = 1,
        command_repeat_interval_sec: float = 0.05,
        control_ownership_checker: Callable[[], str] | None = None,
        should_stop: Callable[[], bool] | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        rate = float(publish_rate_hz)
        if not math.isfinite(rate) or rate < 20.0:
            raise ValueError("publish_rate_hz must be finite and >= 20")
        if not math.isfinite(float(status_timeout_sec)) or status_timeout_sec <= 0:
            raise ValueError("status_timeout_sec must be finite and > 0")
        if (
            not math.isfinite(float(navigation_settle_timeout_sec))
            or navigation_settle_timeout_sec <= 0
        ):
            raise ValueError(
                "navigation_settle_timeout_sec must be finite and > 0"
            )
        if isinstance(navigation_stable_samples, bool) or int(
            navigation_stable_samples
        ) < 1:
            raise ValueError("navigation_stable_samples must be >= 1")
        if (
            not math.isfinite(float(posture_recovery_timeout_sec))
            or posture_recovery_timeout_sec <= 0
        ):
            raise ValueError("posture_recovery_timeout_sec must be finite and > 0")
        self._publish_command = command_publisher
        self._publish_twist = twist_publisher
        self._action_plans = {
            str(unit_id): {
                **dict(plan),
                "sequence": [dict(item) for item in plan.get("sequence", [])],
            }
            for unit_id, plan in action_plans.items()
        }
        self._allow_proxies = bool(allow_proxies)
        self._allow_unverified = bool(allow_unverified)
        self._accepted_unverified_actions = {
            str(unit_id) for unit_id in accepted_unverified_actions
        }
        self._status_timeout_sec = float(status_timeout_sec)
        self._min_battery_percent = float(min_battery_percent)
        self._minimum_backward_clearance_m = max(
            0.0, float(minimum_backward_clearance_m)
        )
        self._mode_settle_sec = max(0.0, float(mode_settle_sec))
        self._navigation_settle_timeout_sec = float(
            navigation_settle_timeout_sec
        )
        self._navigation_stable_samples = int(navigation_stable_samples)
        self._posture_recovery_timeout_sec = float(posture_recovery_timeout_sec)
        self._period_sec = 1.0 / rate
        self._max_linear_x = abs(float(max_linear_x))
        self._max_linear_y = abs(float(max_linear_y))
        self._max_angular_z = abs(float(max_angular_z))
        self._stop_publish_count = max(1, int(stop_publish_count))
        self._command_repeat = max(1, int(command_repeat))
        self._command_repeat_interval_sec = max(
            0.0, float(command_repeat_interval_sec)
        )
        self._control_ownership_checker = control_ownership_checker
        self._should_stop = should_stop or (lambda: False)
        self._monotonic = monotonic
        self._sleep = sleep
        self._cancel_requested = threading.Event()
        self._execution_lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._mode_lock = threading.Lock()
        self._status: Lite3RobotStatus | None = None
        self._status_received_at = 0.0
        self._status_sequence = 0
        self._vision_mode = False
        self._last_error = ""

    @property
    def last_error(self) -> str:
        with self._status_lock:
            return self._last_error

    @property
    def mapped_actions(self) -> set[str]:
        return set(self._action_plans)

    @property
    def enabled_actions(self) -> set[str]:
        return {
            unit_id
            for unit_id in self._action_plans
            if self.can_execute(unit_id)
        }

    @property
    def twist_publisher(self) -> Callable[[TwistCommand], None]:
        return self.publish_velocity

    @property
    def publish_twist(self) -> Callable[[TwistCommand], None]:
        return self.publish_velocity

    def update_status(self, status: Lite3RobotStatus) -> None:
        with self._status_lock:
            self._status = status
            self._status_received_at = self._monotonic()
            self._status_sequence += 1

    def can_execute(self, unit_id: str) -> bool:
        plan = self._action_plans.get(str(unit_id))
        if plan is None:
            return False
        if plan.get("fidelity") == "proxy" and not self._allow_proxies:
            return False
        if not bool(plan.get("verified", False)):
            if not self._allow_unverified:
                return False
            plan_id = str(plan.get("plan_id", unit_id))
            if (
                str(unit_id) not in self._accepted_unverified_actions
                and plan_id not in self._accepted_unverified_actions
            ):
                return False
        return True

    def publish_velocity(self, command: TwistCommand) -> None:
        command = self._clamp_twist(command)
        if command.is_zero:
            self._publish_twist(command)
            self._release_joystick_mode()
            return
        if not command.is_zero:
            ready = (
                self._runtime_motion_ready(command)
                if self._vision_mode
                else self._motion_ready(command)
            )
            if not ready:
                logger.error(
                    "Lite3 rejected non-zero velocity: fresh standing status "
                    "with sufficient battery and obstacle clearance is required"
                )
                self._publish_stop()
                self._release_joystick_mode()
                return
            if not self._ensure_vision_mode(force=not self._vision_mode):
                self._publish_stop()
                return
        self._publish_twist(command)

    def prepare_navigation(self, *, allow_uwb_chain: bool = False) -> bool:
        """Enter Vision Mode before an external Nav2/UWB publisher moves.

        ``allow_uwb_chain`` is for the UWB follow adapter only: it hands
        /cmd_vel to go2_uwb_behavior, so that chain must be allowed to be up
        for the preflight to pass.  Everyone else leaves it False and gets the
        loud ``lite3_control_conflict`` failure.
        """
        self._clear_error()
        # Navigation drives the standing chassis too, and this is a step entry
        # exactly like _run_twist.  Recovery has to happen *here* rather than
        # inside _motion_ready: that check is shared with publish_velocity's
        # per-tick streaming path, where a posture change mid-action must still
        # abort instead of standing the dog up.  Skipping it left the whole
        # recovery unreachable on this route -- expressCalmAlone is a random
        # *navigation* behavior, so a dog lying after ACT_SPLOOT died at the
        # preflight before any step could stand it back up, every time.
        if not self._ensure_entry_posture({"require": "stand"}, None, None):
            self._publish_stop()
            return False
        if not self._motion_ready(allow_uwb_chain=allow_uwb_chain):
            if not self.last_error:
                self._set_error("lite3_navigation_preflight_rejected")
            self._publish_stop()
            return False
        return self._ensure_vision_mode(force=True)

    def finish_navigation(self) -> bool:
        """Stop, release remote control, and wait for a fresh stable chassis."""
        self._publish_stop()
        if not self._release_joystick_mode():
            self._set_error("lite3_joystick_mode_release_failed")
            return False
        return self._wait_for_navigation_idle()

    def execute_step(
        self,
        unit_config: Mapping[str, Any],
        ctx: Any = None,
        duration: float | None = None,
    ) -> bool:
        unit_id = str(unit_config.get("unit_id", ""))
        self._clear_error()
        plan = self._action_plans.get(unit_id)
        if plan is None:
            self._set_error(f"lite3_action_unmapped:unit_id={unit_id}")
            self._publish_stop()
            return False

        if not self.can_execute(unit_id):
            self._set_error(
                "lite3_action_gated:"
                f"unit_id={unit_id},fidelity={plan.get('fidelity')},"
                f"verified={bool(plan.get('verified', False))}"
            )
            self._publish_stop()
            return False

        if getattr(ctx, "motion_state", "active") == "stationary":
            ok = self.hold_position(duration, require_status=True)
            self._record_execution(ctx, unit_id, plan, ok)
            return ok

        try:
            local_budget = max(0.0, float(duration or 0.0))
        except (TypeError, ValueError):
            local_budget = 0.0
        deadline = self._monotonic() + local_budget if local_budget > 0.0 else None

        with self._execution_lock:
            self._cancel_requested.clear()
            ok = True
            executed_item = False
            initial_status = self._fresh_status()
            initial_basic_state = (
                initial_status.basic_state
                if initial_status is not None
                else None
            )
            try:
                for item in plan.get("sequence", []):
                    if self._must_stop(ctx, deadline):
                        ok = False
                        break
                    initial_states = item.get("when_initial_basic_states")
                    if (
                        isinstance(initial_states, list)
                        and initial_basic_state not in initial_states
                    ):
                        continue
                    executed_item = True
                    kind = str(item.get("type", ""))
                    if kind == "simple":
                        ok = self._run_simple(item, ctx, deadline)
                    elif kind == "target_posture":
                        ok = self._run_target_posture(item, ctx, deadline)
                    elif kind == "pose":
                        ok = self._run_pose(item, ctx, deadline)
                    elif kind == "twist":
                        ok = self._run_twist(item, ctx, deadline)
                    elif kind == "hold":
                        ok = self._run_hold(item, ctx, deadline)
                    else:
                        logger.error("Unsupported Lite3 plan step type: %s", kind)
                        ok = False
                    if not ok:
                        break
                if ok and not executed_item:
                    self._set_error(
                        "lite3_plan_no_matching_posture_branch:"
                        f"unit_id={unit_id},basic_state={initial_basic_state}"
                    )
                    ok = False
            except Exception:
                logger.exception("Lite3 action plan %s failed", unit_id)
                self._set_error(f"lite3_action_exception:unit_id={unit_id}")
                ok = False
            finally:
                self._publish_stop()
            self._record_execution(ctx, unit_id, plan, ok)
            return ok

    def execute_task(
        self,
        unit_config: Mapping[str, Any],
        ctx: Any = None,
        timeout: float | None = None,
    ) -> bool:
        """Execute task-shaped ACT_* proxies through the same safe plan path."""
        return self.execute_step(unit_config, ctx, timeout)

    def hold_position(
        self,
        duration_sec: float | None = None,
        *,
        require_status: bool = False,
    ) -> bool:
        try:
            duration = max(0.0, float(duration_sec or 0.0))
        except (TypeError, ValueError):
            duration = 0.0
        deadline = self._monotonic() + duration if duration > 0.0 else None
        self._cancel_requested.clear()
        if require_status and not self._check_simple_preconditions(
            {"require": "none"}, self._fresh_status()
        ):
            self._publish_stop()
            return False
        self._publish_stop()
        while deadline is not None and self._monotonic() < deadline:
            if self._must_stop(None, deadline):
                return False
            if require_status and not self._stationary_runtime_ready():
                return False
            self._sleep_until(deadline, self._period_sec)
            self._publish_twist(TwistCommand())
        self._release_joystick_mode()
        return not self._must_stop(None, None)

    def cancel_step(self, step: Any = None) -> None:
        del step
        self._cancel_requested.set()
        self._publish_stop()
        self._release_joystick_mode()

    def emergency_stop(self) -> None:
        """Stop motion without sending Lite3's motor-disabling soft E-stop."""
        self.cancel_step()

    def _run_simple(
        self,
        item: Mapping[str, Any],
        ctx: Any,
        deadline: float | None,
    ) -> bool:
        command = Lite3SimpleCommand(
            cmd_code=int(item["cmd_code"]),
            value=int(item.get("value", 0)),
        )
        status = self._fresh_status()
        satisfied_states = {
            int(value) for value in item.get("satisfied_basic_states", [])
        }
        if (
            status is not None
            and status.basic_state in satisfied_states
            and status.gait_state == 0
            and status.motion_state == 0
        ):
            return True
        if command.cmd_code in BLOCKED_BEHAVIOR_COMMANDS:
            logger.error("Blocked unsafe Lite3 command 0x%08X", command.cmd_code)
            # Name the refusal: without this the caller reports the useless
            # "adapter returned failure" and the operator cannot tell a policy
            # block from a transport fault.
            self._set_error(
                "lite3_command_blocked:"
                f"cmd_code=0x{command.cmd_code:08X}"
            )
            return False
        # A control-mode switch is itself a command, so a call that is already
        # known to be rejected must not reach the motion host -- and the blocked
        # check above stays ahead of the recovery so a refused command can never
        # move the chassis either.  The status is re-checked on a fresh sample
        # after the switch, and that second check remains the deciding one.
        if not self._ensure_entry_posture(item, ctx, deadline):
            return False
        if not self._force_joystick_mode():
            return False
        status = self._fresh_status()
        if not self._check_simple_preconditions(item, status):
            return False

        for index in range(self._command_repeat):
            if self._publish_command(command) is False:
                self._set_error(
                    "lite3_simple_command_publish_failed:"
                    f"cmd_code=0x{command.cmd_code:08X}"
                )
                return False
            if index + 1 < self._command_repeat:
                if self._must_stop(ctx, deadline):
                    return False
                self._sleep(self._command_repeat_interval_sec)

        timeout = float(item.get("completion_timeout_sec", 0.0))
        if timeout <= 0.0:
            return True
        completion_deadline = self._monotonic() + timeout
        if deadline is not None:
            completion_deadline = min(completion_deadline, deadline)
        return self._wait_for_terminal(item, ctx, completion_deadline)

    def _run_target_posture(
        self,
        item: Mapping[str, Any],
        ctx: Any,
        deadline: float | None,
    ) -> bool:
        """Reach a stable standing or lying state with one guarded toggle.

        Lite3 exposes one directionless command for both transitions.  Read
        fresh feedback before sending it, send it at most once, then require
        new transition/target samples.  A timeout is a failure; it never
        causes an automatic second toggle that could undo a successful move.
        """
        target = str(item.get("target", ""))
        if target not in {"stand", "lie"}:
            self._set_error(
                f"lite3_posture_target_invalid:target={target or '<empty>'}"
            )
            return False
        target_state = STATE_STAND if target == "stand" else STATE_LIE
        source_state = STATE_LIE if target == "stand" else STATE_STAND

        status = self._fresh_status()
        reason = self._simple_precondition_reason({"require": "none"}, status)
        if reason:
            self._set_error(reason)
            return False
        assert status is not None
        if status.basic_state == target_state:
            return True
        if status.basic_state != source_state:
            self._set_error(
                "lite3_posture_transition_source_invalid:"
                f"target={target},basic_state={status.basic_state}"
            )
            return False

        if not self._force_joystick_mode():
            # ``_force_joystick_mode`` already recorded the precise transport
            # reason; overwriting it here would hide which command failed.
            return False

        # Mode switching itself takes time. Re-read the direction immediately
        # before the one permitted toggle so delayed external motion cannot
        # reverse the requested transition.
        status, initial_sequence = self._fresh_status_sample()
        reason = self._simple_precondition_reason({"require": "none"}, status)
        if reason:
            self._set_error(reason)
            return False
        assert status is not None
        if status.basic_state == target_state:
            return True
        if status.basic_state != source_state:
            self._set_error(
                "lite3_posture_transition_source_changed:"
                f"target={target},basic_state={status.basic_state}"
            )
            return False

        if self._publish_command(
            Lite3SimpleCommand(CMD_STAND_LIE_TOGGLE)
        ) is False:
            self._set_error("lite3_posture_toggle_publish_failed")
            return False

        timeout = float(item.get("completion_timeout_sec", 4.5))
        completion_deadline = self._monotonic() + timeout
        if deadline is not None:
            completion_deadline = min(completion_deadline, deadline)
        return self._wait_for_target_posture(
            target=target,
            target_state=target_state,
            initial_sequence=initial_sequence,
            stable_samples=int(
                item.get("stable_samples", self._navigation_stable_samples)
            ),
            ctx=ctx,
            deadline=completion_deadline,
        )

    def _wait_for_target_posture(
        self,
        *,
        target: str,
        target_state: int,
        initial_sequence: int,
        stable_samples: int,
        ctx: Any,
        deadline: float,
    ) -> bool:
        last_sequence = initial_sequence
        consecutive_target_samples = 0
        observed_transition = False
        last_status = "waiting_for_new_status"

        while self._monotonic() < deadline:
            if (
                self._cancel_requested.is_set()
                or self._should_stop()
                or bool(getattr(ctx, "cancel_requested", False))
            ):
                self._set_error(
                    f"lite3_posture_transition_interrupted:target={target}"
                )
                return False
            status, sequence = self._fresh_status_sample()
            if sequence != last_sequence:
                last_sequence = sequence
                if status is None:
                    consecutive_target_samples = 0
                    last_status = "status_missing_or_stale"
                elif status.basic_state == STATE_LOST_CONTROL:
                    self._set_error(
                        "lite3_lost_control:basic_state=8"
                    )
                    return False
                elif status.battery_level < self._min_battery_percent:
                    self._set_error(
                        "lite3_low_battery:"
                        f"battery={status.battery_level:.1f},"
                        f"minimum={self._min_battery_percent:.1f}"
                    )
                    return False
                elif (
                    status.basic_state == target_state
                    and status.gait_state == 0
                    and status.motion_state == 0
                ):
                    observed_transition = True
                    consecutive_target_samples += 1
                    last_status = self._describe_status(status)
                    if consecutive_target_samples >= stable_samples:
                        self._clear_error()
                        return True
                else:
                    consecutive_target_samples = 0
                    observed_transition = observed_transition or (
                        status.basic_state not in {STATE_LIE, STATE_STAND}
                    )
                    last_status = self._describe_status(status)
            self._sleep_until(deadline, 0.02)

        self._set_error(
            "lite3_posture_transition_timeout:"
            f"target={target},observed_transition={observed_transition},"
            f"last_status={last_status}"
        )
        return False

    def _run_pose(
        self,
        item: Mapping[str, Any],
        ctx: Any,
        deadline: float | None,
    ) -> bool:
        cmd_code = int(item["cmd_code"])
        value = int(item.get("value", 0))
        dead_zone = POSE_AXIS_DEAD_ZONES.get(cmd_code)
        if (
            dead_zone is None
            or value == 0
            or abs(value) <= dead_zone
            or abs(value) > POSE_AXIS_MAX_ABS
        ):
            logger.error(
                "Invalid Lite3 pose axis command: code=0x%08X value=%s "
                "dead_zone=%s max_abs=%s",
                cmd_code,
                value,
                dead_zone,
                POSE_AXIS_MAX_ABS,
            )
            self._set_error(
                "lite3_pose_axis_invalid:"
                f"cmd_code=0x{cmd_code:08X},value={value}"
            )
            return False
        if not self._ensure_entry_posture(item, ctx, deadline):
            return False
        if not self._force_joystick_mode():
            return False
        if not self._check_simple_preconditions(item, self._fresh_status()):
            return False
        if self._publish_command(
            Lite3SimpleCommand(CMD_SWITCH_IN_PLACE_MODE)
        ) is False:
            self._set_error("lite3_in_place_mode_switch_failed")
            return False
        if self._mode_settle_sec > 0.0:
            self._sleep(self._mode_settle_sec)

        duration = max(0.0, float(item.get("duration_sec", 0.0)))
        end = self._monotonic() + duration
        if deadline is not None:
            end = min(end, deadline)
        command = Lite3SimpleCommand(cmd_code, value)
        reset = Lite3SimpleCommand(cmd_code, 0)
        ok = end > self._monotonic()
        reset_ok = True
        moving_mode_ok = True
        try:
            if ok:
                ok = self._publish_command(reset) is not False
            if ok:
                self._sleep(self._period_sec)
            while ok and self._monotonic() < end:
                if self._must_stop(ctx, deadline):
                    ok = False
                    break
                if not self._pose_runtime_ready():
                    ok = False
                    break
                ok = self._publish_command(command) is not False
                if ok:
                    self._sleep_until(end, self._period_sec)
        finally:
            reset_ok = self._publish_command(reset) is not False
            moving_mode_ok = self._publish_command(
                Lite3SimpleCommand(CMD_SWITCH_MOVING_MODE)
            ) is not False
        if not reset_ok:
            self._set_error("lite3_pose_reset_failed")
            return False
        if not moving_mode_ok:
            self._set_error("lite3_pose_moving_mode_restore_failed")
            return False
        if not ok or self._must_stop(ctx, None):
            # A refused axis publish, a cancel and a timeout all land here.
            # Keep whichever reason is already set, otherwise name the axis.
            self._set_error_if_clear(
                f"lite3_pose_axis_publish_failed:cmd_code=0x{cmd_code:08X}"
            )
            return False
        return self._wait_for_stable_idle(
            error_prefix="lite3_pose_settle",
            ctx=ctx,
            action_deadline=deadline,
        )

    def _run_twist(
        self,
        item: Mapping[str, Any],
        ctx: Any,
        deadline: float | None,
    ) -> bool:
        command = self._clamp_twist(
            TwistCommand(
                linear_x=float(item.get("linear_x", 0.0)),
                linear_y=float(item.get("linear_y", 0.0)),
                angular_z=float(item.get("angular_z", 0.0)),
            )
        )
        # Twist plans all drive the standing chassis, so a dog left lying by an
        # earlier step stands up here rather than failing the whole plan.
        if not self._ensure_entry_posture({"require": "stand"}, ctx, deadline):
            self._publish_stop()
            return False
        if not self._motion_ready(
            command,
            minimum_backward_clearance_m=float(
                item.get(
                    "minimum_backward_clearance_m",
                    self._minimum_backward_clearance_m,
                )
            ),
            minimum_forward_clearance_m=float(
                item.get("minimum_forward_clearance_m", 0.0)
            ),
        ):
            self._publish_stop()
            return False
        if not self._ensure_vision_mode(force=True):
            self._publish_stop()
            return False
        duration = max(0.0, float(item.get("duration_sec", 0.0)))
        end = self._monotonic() + duration
        if deadline is not None:
            end = min(end, deadline)
        ok = end > self._monotonic() and not command.is_zero
        settled = True
        try:
            while ok and self._monotonic() < end:
                if self._must_stop(ctx, deadline):
                    ok = False
                    break
                if not self._runtime_motion_ready(
                    command,
                    minimum_backward_clearance_m=float(
                        item.get(
                            "minimum_backward_clearance_m",
                            self._minimum_backward_clearance_m,
                        )
                    ),
                    minimum_forward_clearance_m=float(
                        item.get("minimum_forward_clearance_m", 0.0)
                    ),
                ):
                    ok = False
                    break
                self._publish_twist(command)
                self._sleep_until(end, self._period_sec)
        finally:
            settled = self.finish_navigation()
        return ok and settled and not self._must_stop(ctx, None)

    def _run_hold(
        self,
        item: Mapping[str, Any],
        ctx: Any,
        deadline: float | None,
    ) -> bool:
        require_status = bool(item.get("require_status", True))
        if require_status and not self._ensure_entry_posture(item, ctx, deadline):
            self._publish_stop()
            return False
        duration = max(0.0, float(item.get("duration_sec", 0.0)))
        end = self._monotonic() + duration
        if deadline is not None:
            end = min(end, deadline)
        self._publish_stop()
        while self._monotonic() < end:
            if self._must_stop(ctx, deadline):
                return False
            if require_status and not self._stationary_runtime_ready(item):
                return False
            self._sleep_until(end, self._period_sec)
            self._publish_twist(TwistCommand())
        self._release_joystick_mode()
        return not self._must_stop(ctx, None)

    def _check_simple_preconditions(
        self,
        item: Mapping[str, Any],
        status: Lite3RobotStatus | None,
    ) -> bool:
        reason = self._simple_precondition_reason(item, status)
        if reason:
            self._set_error(reason)
            return False
        return True

    def _ensure_entry_posture(
        self,
        item: Mapping[str, Any],
        ctx: Any,
        deadline: float | None,
    ) -> bool:
        """Check an entry-time ``require`` and, if the dog is merely lying, fix it.

        ``require: stand`` used to be a pure precondition, which made a lying
        dog unrecoverable: 平静行为 mixes postures inside one randomly selected
        stage (``ACT_SPLOOT`` lies the dog down, the other four candidates need
        a standing dog) and no step anywhere stood it back up.  One unlucky
        pick therefore left ``basic_state=1`` permanently, and every later
        standing action failed with ``lite3_posture_mismatch`` -- 4 of the 5
        calm candidates, and every other behavior that needs the dog up.

        Only a *lying* dog is recovered, and only when the step asked for
        ``stand``.  Recovery is bounded by the step's own deadline, and the
        streaming runtime checks stay strict: a posture change in the middle of
        an action still aborts it rather than standing up mid-motion.
        """
        status = self._fresh_status()
        reason = self._simple_precondition_reason(item, status)
        if not reason:
            return True
        if (
            status is None
            or status.basic_state != STATE_LIE
            or str(item.get("require", "none")) != "stand"
            or not reason.startswith("lite3_posture_mismatch:")
        ):
            self._set_error(reason)
            return False

        logger.info(
            "Lite3 is lying while this step requires standing; standing up first"
        )
        # ``_run_target_posture`` names the precise transport reason when the
        # stand-up itself fails, so only add a fallback when it recorded none.
        if not self._run_target_posture(
            {
                "target": "stand",
                "completion_timeout_sec": self._posture_recovery_timeout_sec,
                "stable_samples": self._navigation_stable_samples,
            },
            ctx,
            deadline,
        ):
            self._set_error_if_clear("lite3_posture_recovery_failed")
            return False

        reason = self._simple_precondition_reason(item, self._fresh_status())
        if reason:
            self._set_error(reason)
            return False
        return True

    def _simple_precondition_reason(
        self,
        item: Mapping[str, Any],
        status: Lite3RobotStatus | None,
    ) -> str:
        if status is None:
            return "lite3_status_missing_or_stale"
        if status.basic_state == STATE_LOST_CONTROL:
            return "lite3_lost_control:basic_state=8"
        if status.battery_level < self._min_battery_percent:
            return (
                "lite3_low_battery:"
                f"battery={status.battery_level:.1f},"
                f"minimum={self._min_battery_percent:.1f}"
            )
        if status.motion_state != 0:
            return f"lite3_motion_busy:motion_state={status.motion_state}"
        if status.gait_state != 0:
            return f"lite3_gait_busy:gait_state={status.gait_state}"
        if status.basic_state not in {STATE_LIE, STATE_STAND}:
            return (
                "lite3_basic_state_transitional:"
                f"basic_state={status.basic_state}"
            )
        require = str(item.get("require", "none"))
        wanted = STATE_STAND if require == "stand" else STATE_LIE
        if require in {"stand", "lie"} and status.basic_state != wanted:
            return (
                "lite3_posture_mismatch:"
                f"require={require},basic_state={status.basic_state}"
            )
        clearance = float(item.get("minimum_forward_clearance_m", 0.0))
        if clearance > 0.0 and status.ultrasound_forward < clearance:
            return (
                "lite3_forward_clearance:"
                f"actual={status.ultrasound_forward:.2f},"
                f"minimum={clearance:.2f}"
            )
        return ""

    def _wait_for_terminal(
        self,
        item: Mapping[str, Any],
        ctx: Any,
        deadline: float,
    ) -> bool:
        active_basic = {int(value) for value in item.get("active_basic_states", [])}
        active_gait = {int(value) for value in item.get("active_gait_states", [])}
        active_motion = {
            int(value) for value in item.get("active_motion_states", [])
        }
        terminal_basic = {
            int(value) for value in item.get("terminal_basic_states", [])
        }
        terminal_gait = {
            int(value) for value in item.get("terminal_gait_states", [])
        }
        terminal_motion = {
            int(value) for value in item.get("terminal_motion_states", [0])
        }
        requires_active = bool(active_basic or active_gait or active_motion)
        observed_active = not requires_active
        label = f"cmd_code=0x{int(item.get('cmd_code', 0)):08X}"
        last_status = "waiting_for_status"

        while self._monotonic() < deadline:
            if (
                self._cancel_requested.is_set()
                or self._should_stop()
                or bool(getattr(ctx, "cancel_requested", False))
            ):
                self._set_error(
                    f"lite3_action_terminal_interrupted:{label}"
                )
                return False
            if self._monotonic() >= deadline:
                break
            status = self._fresh_status()
            if status is None:
                self._set_error(
                    f"lite3_action_terminal_status_missing:{label}"
                )
                return False
            last_status = self._describe_status(status)
            if status.basic_state == STATE_LOST_CONTROL:
                self._set_error(
                    f"lite3_lost_control:basic_state=8,{label}"
                )
                return False
            if status.battery_level < self._min_battery_percent:
                self._set_error(
                    "lite3_low_battery:"
                    f"battery={status.battery_level:.1f},"
                    f"minimum={self._min_battery_percent:.1f},{label}"
                )
                return False
            if (
                status.basic_state in active_basic
                or status.gait_state in active_gait
                or status.motion_state in active_motion
            ):
                observed_active = True
            if observed_active and self._matches_terminal(
                status,
                terminal_basic,
                terminal_gait,
                terminal_motion,
            ):
                return True
            self._sleep_until(deadline, 0.02)
        if not observed_active:
            # The declared active-state sample was never seen at all.  That is
            # the signature of an over-narrow active-state set rather than a
            # slow action, so it is named separately from a plain timeout.
            self._set_error(
                "lite3_action_terminal_active_state_unobserved:"
                f"{label},last_status={last_status}"
            )
            return False
        self._set_error(
            "lite3_action_terminal_timeout:"
            f"{label},observed_active={observed_active},"
            f"last_status={last_status}"
        )
        return False

    def _sleep_until(self, deadline: float, maximum_interval: float) -> None:
        """Sleep for a bounded positive interval before ``deadline``.

        Real monotonic time can cross the deadline between a loop condition
        and the sleep calculation.  Never pass that resulting negative value
        to ``time.sleep``.
        """
        remaining = deadline - self._monotonic()
        if remaining <= 0.0:
            return
        self._sleep(min(maximum_interval, remaining))

    @staticmethod
    def _matches_terminal(
        status: Lite3RobotStatus,
        basic: set[int],
        gait: set[int],
        motion: set[int],
    ) -> bool:
        return (
            (not basic or status.basic_state in basic)
            and (not gait or status.gait_state in gait)
            and (not motion or status.motion_state in motion)
        )

    @staticmethod
    def _describe_status(status: Lite3RobotStatus) -> str:
        """Compact status rendering shared by every wait-loop failure reason."""
        return (
            f"basic_state={status.basic_state},"
            f"gait_state={status.gait_state},"
            f"motion_state={status.motion_state}"
        )

    def _motion_ready(
        self,
        command: TwistCommand | None = None,
        *,
        minimum_backward_clearance_m: float | None = None,
        minimum_forward_clearance_m: float | None = None,
        allow_uwb_chain: bool = False,
    ) -> bool:
        if not self._control_ownership_ready(allow_uwb_chain=allow_uwb_chain):
            return False
        status = self._fresh_status()
        reason = self._simple_precondition_reason({"require": "stand"}, status)
        if reason:
            self._set_error(reason)
            return False
        return self._clearance_ready(
            status,
            command,
            minimum_backward_clearance_m=minimum_backward_clearance_m,
            minimum_forward_clearance_m=minimum_forward_clearance_m,
        )

    def _control_ownership_ready(self, *, allow_uwb_chain: bool = False) -> bool:
        if self._control_ownership_checker is None:
            return True
        try:
            conflict = str(self._control_ownership_checker() or "").strip()
        except Exception as exc:
            self._set_error(
                "lite3_control_ownership_check_failed:"
                f"{type(exc).__name__}"
            )
            return False
        if conflict and allow_uwb_chain:
            conflict = _without_uwb_chain(conflict)
        if not conflict:
            return True
        self._set_error(f"lite3_control_conflict:{conflict}")
        return False

    def _runtime_motion_ready(
        self,
        command: TwistCommand,
        *,
        minimum_backward_clearance_m: float | None = None,
        minimum_forward_clearance_m: float | None = None,
    ) -> bool:
        status = self._fresh_status()
        if status is None:
            self._set_error("lite3_status_missing_or_stale")
            return False
        if status.basic_state == STATE_LOST_CONTROL:
            self._set_error("lite3_lost_control:basic_state=8")
            return False
        if status.battery_level < self._min_battery_percent:
            self._set_error(
                "lite3_low_battery:"
                f"battery={status.battery_level:.1f},"
                f"minimum={self._min_battery_percent:.1f}"
            )
            return False
        if status.basic_state != STATE_STAND:
            self._set_error(
                "lite3_motion_posture_invalid:"
                f"basic_state={status.basic_state}"
            )
            return False
        if status.gait_state != 0:
            self._set_error(f"lite3_gait_busy:gait_state={status.gait_state}")
            return False
        if status.motion_state not in {0, 1}:
            self._set_error(
                f"lite3_motion_busy:motion_state={status.motion_state}"
            )
            return False
        return self._clearance_ready(
            status,
            command,
            minimum_backward_clearance_m=minimum_backward_clearance_m,
            minimum_forward_clearance_m=minimum_forward_clearance_m,
        )

    def _stationary_runtime_ready(
        self,
        item: Mapping[str, Any] | None = None,
    ) -> bool:
        status = self._fresh_status()
        if status is None:
            return False
        return self._check_simple_preconditions(item or {}, status)

    def _pose_runtime_ready(self) -> bool:
        status = self._fresh_status()
        if status is None:
            self._set_error("lite3_status_missing_or_stale")
            return False
        if status.battery_level < self._min_battery_percent:
            self._set_error(
                "lite3_low_battery:"
                f"battery={status.battery_level:.1f},"
                f"minimum={self._min_battery_percent:.1f}"
            )
            return False
        if status.basic_state not in {STATE_STAND, 9}:
            self._set_error(
                "lite3_pose_state_invalid:"
                f"basic_state={status.basic_state}"
            )
            return False
        if status.gait_state != 0:
            self._set_error(f"lite3_gait_busy:gait_state={status.gait_state}")
            return False
        if status.motion_state != 0:
            self._set_error(
                f"lite3_motion_busy:motion_state={status.motion_state}"
            )
            return False
        return True

    def _clearance_ready(
        self,
        status: Lite3RobotStatus | None,
        command: TwistCommand | None,
        *,
        minimum_backward_clearance_m: float | None,
        minimum_forward_clearance_m: float | None = None,
    ) -> bool:
        if status is None or command is None:
            return status is not None
        # Forward arcs are only guarded when a plan asks for it: reverse has a
        # configured default because every reverse plan needs it, while forward
        # motion was previously only ever issued by Nav2, outside this backend.
        if command.linear_x > 0.0:
            forward_threshold = max(
                0.0, float(minimum_forward_clearance_m or 0.0)
            )
            if forward_threshold <= 0.0:
                return True
            if status.ultrasound_forward < forward_threshold:
                self._set_error(
                    "lite3_forward_clearance:"
                    f"actual={status.ultrasound_forward:.2f},"
                    f"minimum={forward_threshold:.2f}"
                )
                return False
            return True
        if command.linear_x == 0.0:
            return True
        threshold = (
            self._minimum_backward_clearance_m
            if minimum_backward_clearance_m is None
            else max(0.0, float(minimum_backward_clearance_m))
        )
        if threshold <= 0.0:
            return True
        if status.ultrasound_backward < threshold:
            self._set_error(
                "lite3_backward_clearance:"
                f"actual={status.ultrasound_backward:.2f},"
                f"minimum={threshold:.2f}"
            )
            return False
        return True

    def _fresh_status(self) -> Lite3RobotStatus | None:
        with self._status_lock:
            status = self._status
            received_at = self._status_received_at
        if status is None:
            return None
        if self._monotonic() - received_at > self._status_timeout_sec:
            return None
        return status

    def _fresh_status_sample(
        self,
    ) -> tuple[Lite3RobotStatus | None, int]:
        with self._status_lock:
            status = self._status
            received_at = self._status_received_at
            sequence = self._status_sequence
        if status is None:
            return None, sequence
        if self._monotonic() - received_at > self._status_timeout_sec:
            return None, sequence
        return status, sequence

    def _wait_for_navigation_idle(self) -> bool:
        # This is stop cleanup. A canceled goal must still confirm fresh,
        # stable chassis feedback before motion ownership is released.
        return self._wait_for_stable_idle(
            error_prefix="lite3_navigation_settle",
            ignore_cancel=True,
        )

    def _wait_for_stable_idle(
        self,
        *,
        error_prefix: str,
        ctx: Any = None,
        action_deadline: float | None = None,
        ignore_cancel: bool = False,
    ) -> bool:
        """Wait for fresh standing-idle samples after chassis motion.

        Pose-axis reset and mode restoration are asynchronous on Lite3.  A
        plan is not complete until the robot has actually returned to stable
        ``basic=6, gait=0, motion=0`` feedback.
        """
        deadline = self._monotonic() + self._navigation_settle_timeout_sec
        if action_deadline is not None:
            deadline = min(deadline, action_deadline)
        initial_status, last_sequence = self._fresh_status_sample()
        stable_samples = 0
        last_reason = self._simple_precondition_reason(
            {"require": "stand"}, initial_status
        )
        if not last_reason:
            last_reason = "lite3_waiting_for_new_status"

        while self._monotonic() < deadline:
            if not ignore_cancel and (
                self._cancel_requested.is_set()
                or self._should_stop()
                or bool(getattr(ctx, "cancel_requested", False))
            ):
                self._set_error(f"{error_prefix}_interrupted")
                return False
            status, sequence = self._fresh_status_sample()
            if sequence != last_sequence:
                last_sequence = sequence
                reason = self._simple_precondition_reason(
                    {"require": "stand"}, status
                )
                if reason:
                    stable_samples = 0
                    last_reason = reason
                else:
                    stable_samples += 1
                    if stable_samples >= self._navigation_stable_samples:
                        self._clear_error()
                        return True
            remaining = deadline - self._monotonic()
            if remaining > 0.0:
                self._sleep(min(self._period_sec, remaining))

        self._set_error(
            f"{error_prefix}_timeout:"
            f"last_status={last_reason}"
        )
        return False

    def _ensure_vision_mode(self, *, force: bool = False) -> bool:
        with self._mode_lock:
            if self._vision_mode and not force:
                return True
            ok = self._publish_command(
                Lite3SimpleCommand(CMD_SWITCH_VISION_MODE)
            ) is not False
            if ok:
                self._vision_mode = True
                if self._mode_settle_sec > 0.0:
                    self._sleep(self._mode_settle_sec)
            else:
                self._set_error(
                    "lite3_vision_mode_switch_failed:"
                    f"cmd_code=0x{CMD_SWITCH_VISION_MODE:08X}"
                )
            return ok

    def _force_joystick_mode(self) -> bool:
        with self._mode_lock:
            ok = self._publish_command(
                Lite3SimpleCommand(CMD_SWITCH_JOYSTICK_MODE)
            ) is not False
            if ok:
                self._vision_mode = False
                if self._mode_settle_sec > 0.0:
                    self._sleep(self._mode_settle_sec)
            else:
                self._set_error(
                    "lite3_joystick_mode_switch_failed:"
                    f"cmd_code=0x{CMD_SWITCH_JOYSTICK_MODE:08X}"
                )
            return ok

    def _release_joystick_mode(self) -> bool:
        with self._mode_lock:
            if not self._vision_mode:
                return True
            ok = self._publish_command(
                Lite3SimpleCommand(CMD_SWITCH_JOYSTICK_MODE)
            ) is not False
            if ok:
                self._vision_mode = False
                if self._mode_settle_sec > 0.0:
                    self._sleep(self._mode_settle_sec)
            else:
                self._set_error(
                    "lite3_joystick_mode_release_failed:"
                    f"cmd_code=0x{CMD_SWITCH_JOYSTICK_MODE:08X}"
                )
            return ok

    def _set_error(self, reason: str) -> None:
        reason = str(reason).strip() or "lite3_unknown_failure"
        with self._status_lock:
            self._last_error = reason
        logger.error("Lite3 failure: %s", reason)

    def _set_error_if_clear(self, reason: str) -> None:
        """Record a fallback reason only when no precise one is already set."""
        if not self.last_error:
            self._set_error(reason)

    def _clear_error(self) -> None:
        with self._status_lock:
            self._last_error = ""

    def _publish_stop(self) -> None:
        for _ in range(self._stop_publish_count):
            try:
                self._publish_twist(TwistCommand())
            except Exception:
                logger.exception("Failed to publish Lite3 zero Twist")

    def _record_execution(
        self,
        ctx: Any,
        unit_id: str,
        plan: Mapping[str, Any],
        success: bool,
    ) -> None:
        metadata = getattr(ctx, "metadata", None)
        if not isinstance(metadata, dict):
            return
        status = self._fresh_status()
        physical_posture = None
        if status is not None:
            if status.basic_state == STATE_STAND:
                physical_posture = "standing"
            elif status.basic_state == STATE_LIE:
                physical_posture = "lying"
        entry: dict[str, Any] = {
            "requested_act": str(unit_id),
            "executed_plan": str(plan.get("plan_id", unit_id)),
            "fidelity": str(plan.get("fidelity", "proxy")),
            "semantic_effect": str(
                plan.get(
                    "semantic_effect",
                    "physical"
                    if plan.get("fidelity") == "exact"
                    else "simulated",
                )
            ),
            "hardware_verified": bool(plan.get("verified", False)),
            "success": bool(success),
        }
        if not success and self.last_error:
            entry["failure_reason"] = self.last_error
        if physical_posture is not None:
            entry["physical_posture"] = physical_posture
        if status is not None:
            entry["terminal_status"] = {
                "basic_state": status.basic_state,
                "gait_state": status.gait_state,
                "motion_state": status.motion_state,
                "battery_level": status.battery_level,
            }
        metadata["lite3_action"] = entry
        history = metadata.setdefault("lite3_actions", [])
        if isinstance(history, list):
            history.append(dict(entry))

    def _must_stop(self, ctx: Any, deadline: float | None) -> bool:
        if self._cancel_requested.is_set() or self._should_stop():
            return True
        if bool(getattr(ctx, "cancel_requested", False)):
            return True
        return deadline is not None and self._monotonic() >= deadline

    def _clamp_twist(self, command: TwistCommand) -> TwistCommand:
        return TwistCommand(
            linear_x=self._clamp(command.linear_x, self._max_linear_x),
            linear_y=self._clamp(command.linear_y, self._max_linear_y),
            angular_z=self._clamp(command.angular_z, self._max_angular_z),
        )

    @staticmethod
    def _clamp(value: Any, limit: float) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(number):
            return 0.0
        return max(-limit, min(limit, number))


def make_ros2_lite3_backend(
    node: Any,
    *,
    simple_cmd_topic: str,
    cmd_vel_topic: str,
    action_plans: Mapping[str, Mapping[str, Any]],
    uwb_driving: Callable[[], bool] | None = None,
    **kwargs: Any,
) -> tuple[
    Lite3ChassisBackend,
    Ros2Lite3SimpleCommandPublisher,
    Ros2TwistPublisher,
]:
    """Construct the ROS publishers and the transport-neutral backend."""
    command_publisher = Ros2Lite3SimpleCommandPublisher(node, simple_cmd_topic)
    twist_publisher = Ros2TwistPublisher(node, cmd_vel_topic, label="Lite3")
    if "control_ownership_checker" not in kwargs:
        kwargs["control_ownership_checker"] = lambda: _ros2_control_conflicts(
            node, uwb_driving=uwb_driving
        )
    backend = Lite3ChassisBackend(
        command_publisher=command_publisher,
        twist_publisher=twist_publisher,
        action_plans=action_plans,
        **kwargs,
    )
    return backend, command_publisher, twist_publisher


UWB_CHAIN_NODE_NAME = "/uwb_behavior_controller_node"


def _ros2_control_conflicts(
    node: Any,
    *,
    uwb_driving: Callable[[], bool] | None = None,
) -> str:
    """Describe nodes that would compete with Lite3 Action ownership.

    Covers the Lite3 helper nodes and the go2_uwb_behavior follow chain.  That
    chain is a conflict for every twist action because it drives /cmd_vel at
    20 Hz while a follow or roam goal is active, and would chop an open-loop
    twist like ACT_CIRCLE_AROUND into pieces.

    It is reported here unconditionally, even though the UWB follow adapter
    legitimately runs alongside it: this is the honest inventory, and the
    exemption belongs to the one caller that can justify it.  See
    ``_without_uwb_chain`` and ``_control_ownership_ready``.

    ``uwb_driving`` narrows that entry from "the node is up" to "the node is
    driving right now".  With ``publish_idle_velocity: false`` the chain stays
    silent on /cmd_vel between goals, so an idle node no longer blocks anyone
    else: the wake turn and Nav2 get the topic back the moment a follow ends.
    Callers omit it to keep the unconditional inventory; a callback that
    raises counts as driving (fail closed).
    """
    counts = {
        "/lite3_action": 0,
        "/lite3_twist_bridge": 0,
        UWB_CHAIN_NODE_NAME: 0,
    }
    if uwb_driving is not None:
        try:
            if not uwb_driving():
                del counts[UWB_CHAIN_NODE_NAME]
        except Exception:
            pass
    for name, namespace in node.get_node_names_and_namespaces():
        prefix = str(namespace).rstrip("/")
        full_name = f"{prefix}/{name}" if prefix else f"/{name}"
        if full_name in counts:
            counts[full_name] += 1
    active = [
        f"{name}={count}"
        for name, count in sorted(counts.items())
        if count > 0
    ]
    return "active_nodes=" + ",".join(active) if active else ""


def _without_uwb_chain(report: str) -> str:
    """Drop the UWB chain from a conflict report, keeping its format.

    Following *is* handing /cmd_vel to that chain, so its presence is the
    expected state for the follow adapter and a conflict for everyone else.
    Only the named entry is removed -- any other competing node still fails.
    """
    prefix = "active_nodes="
    if not report.startswith(prefix):
        return report
    entries = [
        entry
        for entry in report[len(prefix):].split(",")
        if entry and not entry.startswith(f"{UWB_CHAIN_NODE_NAME}=")
    ]
    return prefix + ",".join(entries) if entries else ""
