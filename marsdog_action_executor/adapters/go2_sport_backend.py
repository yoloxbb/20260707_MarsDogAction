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
    Actions absent from ``action_sequences`` therefore fail closed instead of
    being substituted with an unrelated canned motion.
    """

    def __init__(
        self,
        request_publisher: Callable[[Go2SportRequest], bool | None],
        action_sequences: Mapping[str, list[Mapping[str, Any]]],
        *,
        publish_rate_hz: float = 10.0,
        max_linear_x: float = 0.30,
        min_linear_x: float = 0.0,
        max_linear_y: float = 0.20,
        max_angular_z: float = 1.00,
        stop_publish_count: int = 3,
        should_stop: Callable[[], bool] | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        rate = float(publish_rate_hz)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("publish_rate_hz must be finite and > 0")
        self._publish_request = request_publisher
        self._action_sequences = {
            str(action_id): [dict(item) for item in sequence]
            for action_id, sequence in action_sequences.items()
        }
        self._period_sec = 1.0 / rate
        self._max_linear_x = abs(float(max_linear_x))
        self._min_linear_x = abs(float(min_linear_x))
        self._max_linear_y = abs(float(max_linear_y))
        self._max_angular_z = abs(float(max_angular_z))
        self._stop_publish_count = max(1, int(stop_publish_count))
        self._should_stop = should_stop or (lambda: False)
        self._monotonic = monotonic
        self._sleep = sleep
        self._cancel_requested = threading.Event()
        self._execution_lock = threading.Lock()

    @property
    def mapped_actions(self) -> set[str]:
        return set(self._action_sequences)

    def can_execute(self, unit_id: str) -> bool:
        return str(unit_id) in self._action_sequences

    def publish_velocity(self, command: TwistCommand) -> None:
        """Translate a body-frame velocity into Go2 ``Move``/``StopMove``."""
        if command.is_zero:
            self._send_command("stop_move")
            return
        self._send_command(
            "move",
            x=self._floor_linear_x(
                self._clamp(command.linear_x, self._max_linear_x)
            ),
            y=self._clamp(command.linear_y, self._max_linear_y),
            z=self._clamp(command.angular_z, self._max_angular_z),
        )

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
        if getattr(ctx, "motion_state", "active") == "stationary":
            return self.hold_position(duration)

        unit_id = str(unit_config.get("unit_id", ""))
        sequence = self._action_sequences.get(unit_id)
        if sequence is None:
            logger.error("No Go2 SportMode sequence is mapped for %s", unit_id)
            self._publish_stop()
            return False
        if unit_id == "ACT_SYSTEM_EMERGENCY_STOP":
            self.emergency_stop()
            return True

        try:
            local_budget = max(0.0, float(duration or 0.0))
        except (TypeError, ValueError):
            local_budget = 0.0
        deadline = (
            self._monotonic() + local_budget if local_budget > 0.0 else None
        )

        with self._execution_lock:
            self._cancel_requested.clear()
            ok = True
            try:
                for item in sequence:
                    if self._must_stop(ctx, deadline):
                        ok = False
                        break
                    command = str(item.get("command", "")).strip()
                    item_duration = max(0.0, float(item.get("duration_sec", 0.0)))
                    if command == "move":
                        ok = self._run_move(item, item_duration, ctx, deadline)
                    else:
                        ok = self._send_command(
                            command,
                            x=item.get("x"),
                            y=item.get("y"),
                            z=item.get("z"),
                        )
                        if ok:
                            ok = self._wait(item_duration, ctx, deadline)
                    if not ok:
                        break
            except Exception:
                logger.exception("Go2 SportMode sequence %s failed", unit_id)
                ok = False
            finally:
                self._publish_stop()
            return ok

    def hold_position(self, duration_sec: float | None = None) -> bool:
        try:
            duration = max(0.0, float(duration_sec or 0.0))
        except (TypeError, ValueError):
            duration = 0.0
        deadline = self._monotonic() + duration if duration > 0.0 else None
        self._cancel_requested.clear()
        ok = self._send_command("stop_move")
        while ok and deadline is not None and self._monotonic() < deadline:
            if self._must_stop(None, deadline):
                return False
            self._sleep(min(self._period_sec, deadline - self._monotonic()))
            ok = self._send_command("stop_move")
        self._publish_stop()
        return ok

    def cancel_step(self, step: Any = None) -> None:
        del step
        self._cancel_requested.set()
        self._publish_stop()

    def emergency_stop(self) -> None:
        self.cancel_step()

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
        return deadline is not None and self._monotonic() >= deadline

    def _send_command(
        self,
        command: str,
        *,
        x: Any = None,
        y: Any = None,
        z: Any = None,
    ) -> bool:
        api_id = SPORT_API_IDS.get(command)
        if api_id is None:
            logger.error("Unsupported Go2 SportMode command: %s", command)
            return False
        parameter = ""
        if command in _VECTOR_COMMANDS:
            parameter = json.dumps(
                {"x": float(x or 0.0), "y": float(y or 0.0), "z": float(z or 0.0)},
                separators=(",", ":"),
            )
        result = self._publish_request(Go2SportRequest(api_id, parameter))
        return result is not False

    def _publish_stop(self) -> None:
        for _ in range(self._stop_publish_count):
            self._send_command("stop_move")

    def _floor_linear_x(self, value: float) -> float:
        """Floor a non-zero forward speed to the Go2 Move dead-zone.

        The Go2 SportMode ``Move`` request silently ignores linear velocity
        below ``min_linear_x``, so a controller that intends to translate must
        never emit a command the firmware would drop. Zero stays zero so the
        ``is_zero`` -> ``StopMove`` boundary is preserved.
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
