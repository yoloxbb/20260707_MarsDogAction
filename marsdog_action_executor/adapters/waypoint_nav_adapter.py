"""Strict client for the shared ``waypoint_nav`` task service.

The service response only acknowledges that a task was queued.  Completion is
decided exclusively from the matching task's terminal status message (or a
durable ``query`` response).  This distinction is important for Action's
execution lock: accepting a cancel request does not mean that Nav2 has stopped.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import threading
import time
import uuid
from typing import Any, Callable, Mapping


_STATUS_CODES = {
    "RUNNING": {
        "QUEUED",
        "DISPATCHED",
        "NAV2_ACCEPTED",
        "RECOVERY_REQUIRED",
    },
    "SUCCEEDED": {"NAV2_SUCCEEDED"},
    "FAILED": {
        "GOAL_REJECTED",
        "NAV2_EXCEPTION",
        "INTERNAL_ERROR",
        "TIMEOUT",
        "NAV2_FAILED",
        "NAV2_UNKNOWN",
    },
    "INTERRUPTED": {
        "CLIENT_CANCELLED",
        "PREEMPTED",
        "NAV2_CANCELED",
    },
}
_TERMINAL_STATES = frozenset({"SUCCEEDED", "FAILED", "INTERRUPTED"})
_RANDOM_PLACE_ALIASES = frozenset({"11", "K"})
_RANDOM_MATCHED_ID = "11"
_RANDOM_PLACE_NAME = "随机点位"


class WaypointNavProtocolError(ValueError):
    """A matching response/status violated the agreed wire contract."""


@dataclass(frozen=True)
class WaypointNavStatus:
    task_id: str
    task_type: str
    state: str
    code: str
    matched_id: str
    place: str
    message: str
    safe_to_interrupt: bool = False
    sequence: int | None = None

    @property
    def terminal(self) -> bool:
        return self.state in _TERMINAL_STATES


@dataclass(frozen=True)
class WaypointNavOutcome:
    success: bool
    task_id: str
    state: str
    code: str
    message: str
    terminal_confirmed: bool


@dataclass
class _TaskRecord:
    task_id: str
    requested_place: str
    status_callback: Callable[[WaypointNavStatus], None] | None
    matched_id: str = ""
    matched_place: str = ""
    last_sequence: int | None = None
    terminal: WaypointNavStatus | None = None
    protocol_error: str = ""
    cancel_sent: bool = False
    cancel_started_at: float | None = None
    next_query_at: float = 0.0
    force_query: bool = False
    last_unattributed_query_at: float | None = None
    last_status: WaypointNavStatus | None = None
    accepted: bool = False


def _required_string(payload: Mapping[str, Any], key: str) -> str:
    value = payload.get(key)
    if not isinstance(value, str) or not value.strip():
        raise WaypointNavProtocolError(f"{key} must be a non-empty string")
    return value.strip()


def parse_waypoint_status(
    raw: str | Mapping[str, Any],
    *,
    expected_task_id: str,
    expected_matched_id: str = "",
    expected_place: str = "",
    last_sequence: int | None = None,
) -> WaypointNavStatus | None:
    """Parse one status, returning ``None`` for another task.

    Foreign tasks are filtered before validating their remaining fields.  A
    payload which claims the current task is validated strictly and therefore
    cannot silently advance the behavior tree with an unknown state/code.
    """
    if isinstance(raw, str):
        try:
            payload = json.loads(raw)
        except (json.JSONDecodeError, TypeError) as exc:
            raise WaypointNavProtocolError(f"invalid status JSON: {exc}") from exc
    elif isinstance(raw, Mapping):
        payload = dict(raw)
    else:
        raise WaypointNavProtocolError("status must be a JSON object")
    if not isinstance(payload, dict):
        raise WaypointNavProtocolError("status must be a JSON object")

    task_id = payload.get("task_id")
    if not isinstance(task_id, str) or not task_id.strip():
        raise WaypointNavProtocolError("task_id must be a non-empty string")
    task_id = task_id.strip()
    if task_id != expected_task_id:
        return None

    task_type = _required_string(payload, "task_type")
    if task_type != "goto_place":
        raise WaypointNavProtocolError(
            f"matching status task_type conflict: {task_type!r}"
        )
    state = _required_string(payload, "state")
    code = _required_string(payload, "code")
    if state not in _STATUS_CODES:
        raise WaypointNavProtocolError(f"unknown state: {state!r}")
    if code not in _STATUS_CODES[state]:
        raise WaypointNavProtocolError(
            f"invalid state/code combination: {state}/{code}"
        )

    matched_id = _required_string(payload, "matched_id")
    place = _required_string(payload, "place")
    message = _required_string(payload, "message")
    if expected_matched_id and matched_id != expected_matched_id:
        raise WaypointNavProtocolError(
            "matched_id conflicts with accepted response: "
            f"{matched_id!r} != {expected_matched_id!r}"
        )
    if expected_place and place != expected_place:
        raise WaypointNavProtocolError(
            "place conflicts with accepted response: "
            f"{place!r} != {expected_place!r}"
        )

    safe = payload.get("safe_to_interrupt", False)
    if not isinstance(safe, bool):
        raise WaypointNavProtocolError("safe_to_interrupt must be boolean")
    # QUEUED is known not to have reached Nav2, and NAV2_ACCEPTED may be
    # cancelable when the server explicitly says so.  DISPATCHED is the
    # persisted send-intent window, where Nav2 receipt is unknown;
    # RECOVERY_REQUIRED is likewise unresolved after restart.  Neither phase
    # may advertise a safe point.  Terminal records are never interruptible.
    if safe and (
        state in _TERMINAL_STATES
        or code in {"DISPATCHED", "RECOVERY_REQUIRED"}
    ):
        raise WaypointNavProtocolError(
            f"safe_to_interrupt=true is invalid for {state}/{code}"
        )

    sequence = payload.get("sequence")
    if sequence is not None:
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
            raise WaypointNavProtocolError("sequence must be a non-negative integer")
        if last_sequence is not None and sequence < last_sequence:
            raise WaypointNavProtocolError(
                f"decreasing sequence: {sequence} < {last_sequence}"
            )
    elif last_sequence is not None:
        raise WaypointNavProtocolError(
            "sequence disappeared after the task began using it"
        )

    return WaypointNavStatus(
        task_id=task_id,
        task_type=task_type,
        state=state,
        code=code,
        matched_id=matched_id,
        place=place,
        message=message,
        safe_to_interrupt=safe,
        sequence=sequence,
    )


class Ros2WaypointNavClient:
    """Synchronous facade over ``VoiceTask`` plus the JSON status topic."""

    def __init__(
        self,
        node: Any,
        *,
        service_name: str = "/waypoint_nav/task",
        status_topic: str = "/waypoint_nav/status",
        protocol_version: str = "1.0",
        client_id: str = "marsdog_action_executor",
        service_timeout_sec: float = 5.0,
        query_timeout_sec: float = 2.0,
        cancel_confirmation_timeout_sec: float = 5.0,
        terminal_retention_sec: float = 86400.0,
        should_stop: Callable[[], bool] | None = None,
    ) -> None:
        from marsdog_voice_interaction.srv import VoiceTask
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self._node = node
        self._service_type = VoiceTask
        self._service_name = service_name
        self._status_topic = status_topic
        self._protocol_version = str(protocol_version)
        self._client_id = str(client_id)
        self._service_timeout_sec = self._positive(
            service_timeout_sec, "service_timeout_sec"
        )
        self._query_timeout_sec = self._positive(
            query_timeout_sec, "query_timeout_sec"
        )
        self._cancel_confirmation_timeout_sec = self._positive(
            cancel_confirmation_timeout_sec,
            "cancel_confirmation_timeout_sec",
        )
        self._terminal_retention_sec = self._positive(
            terminal_retention_sec, "terminal_retention_sec"
        )
        self._should_stop = should_stop or (lambda: False)

        callback_group = ReentrantCallbackGroup()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        # Subscription is deliberately created before the service client.  A
        # fast server may publish DISPATCHED before its service reply arrives.
        self._subscription = node.create_subscription(
            String,
            status_topic,
            self._on_status,
            qos,
            callback_group=callback_group,
        )
        self._client = node.create_client(
            VoiceTask,
            service_name,
            callback_group=callback_group,
        )

        self._condition = threading.Condition()
        self._active: _TaskRecord | None = None
        self._records: dict[str, _TaskRecord] = {}
        self._seen_task_ids: set[str] = set()
        self._cancel_requested = False
        self._quarantined_task_id = ""
        self._last_outcome = WaypointNavOutcome(
            False, "", "IDLE", "NOT_STARTED", "", True
        )

    @staticmethod
    def _positive(value: float, name: str) -> float:
        number = float(value)
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f"{name} must be finite and > 0")
        return number

    @property
    def last_outcome(self) -> WaypointNavOutcome:
        with self._condition:
            return self._last_outcome

    @property
    def navigation_interlocked(self) -> bool:
        with self._condition:
            return bool(self._active or self._quarantined_task_id)

    def __call__(
        self,
        task_id: str,
        place: str,
        timeout_sec: float,
        status_callback: Callable[[WaypointNavStatus], None] | None = None,
    ) -> bool:
        task_id = str(task_id).strip()
        place = str(place).strip()
        try:
            total_timeout = self._positive(timeout_sec, "timeout_sec")
        except (TypeError, ValueError) as exc:
            self._set_outcome(False, task_id, "FAILED", "INVALID_TIMEOUT", str(exc), True)
            return False
        if not task_id or not place:
            self._set_outcome(
                False, task_id, "FAILED", "INVALID_REQUEST",
                "task_id and place must be non-empty", True,
            )
            return False

        with self._condition:
            if task_id in self._seen_task_ids:
                self._set_outcome_locked(
                    False, task_id, "FAILED", "DUPLICATE_TASK_ID",
                    "duplicate task_id cannot start a second navigation", True,
                )
                return False
            if self._active is not None or self._quarantined_task_id:
                blocking = (
                    self._active.task_id
                    if self._active is not None
                    else self._quarantined_task_id
                )
                self._set_outcome_locked(
                    False, task_id, "FAILED", "NAVIGATION_INTERLOCKED",
                    f"previous task has no confirmed terminal: {blocking}", False,
                )
                return False
            if self._should_stop() or self._cancel_requested:
                self._cancel_requested = False
                self._set_outcome_locked(
                    False, task_id, "INTERRUPTED", "CANCELED_BEFORE_DISPATCH",
                    "cancel requested before waypoint dispatch", True,
                )
                return False

            now = time.monotonic()
            record = _TaskRecord(
                task_id,
                place,
                status_callback,
                next_query_at=now + self._cancel_confirmation_timeout_sec,
            )
            self._active = record
            self._records[task_id] = record
            self._seen_task_ids.add(task_id)
            self._cancel_requested = False

        deadline = time.monotonic() + total_timeout
        response = self._call_service(
            task_id,
            "goto_place",
            {
                "protocol_version": self._protocol_version,
                "client_id": self._client_id,
                "place": place,
                "timeout_sec": total_timeout,
                "preempt": False,
                "terminal_retention_sec": self._terminal_retention_sec,
            },
            min(self._service_timeout_sec, total_timeout),
        )
        if response is None:
            with self._condition:
                record.protocol_error = "goto_place service timeout/unavailable"
                self._cancel_requested = True
                self._condition.notify_all()
        elif not bool(getattr(response, "success", False)):
            # A duplicate may be the same durable task after this Action node
            # restarted.  Query once before treating a synchronous rejection
            # as final; the server remains responsible for not dispatching a
            # second navigation for the duplicate ID.
            message = str(getattr(response, "error_message", "")) or (
                "goto_place was rejected"
            )
            try:
                query_result = self._query_task(record)
            except Exception as exc:
                self._node.get_logger().error(
                    f"waypoint_nav duplicate recovery query failed: {exc}"
                )
                query_result = "invalid"
            if query_result != "found":
                with self._condition:
                    self._active = None
                    self._cancel_requested = False
                    record.status_callback = None
                    self._set_outcome_locked(
                        False,
                        task_id,
                        "FAILED",
                        "REQUEST_REJECTED",
                        message,
                        True,
                    )
                return False
        else:
            try:
                self._validate_goto_response(response, record)
            except WaypointNavProtocolError as exc:
                with self._condition:
                    record.protocol_error = str(exc)
                    self._cancel_requested = True
                    self._condition.notify_all()

        self._wait_for_terminal(record, deadline)

        with self._condition:
            terminal = record.terminal
            protocol_error = record.protocol_error
            if terminal is None:
                self._quarantined_task_id = task_id
            self._active = None
            self._cancel_requested = False
            record.status_callback = None

            if terminal is None:
                code = (
                    "PROTOCOL_ERROR" if protocol_error
                    else "TERMINAL_UNCONFIRMED"
                )
                message = protocol_error or (
                    "navigation ended without a confirmed terminal status"
                )
                self._set_outcome_locked(
                    False, task_id, "FAILED", code, message, False
                )
                return False
            if protocol_error:
                self._set_outcome_locked(
                    False, task_id, "FAILED", "PROTOCOL_ERROR",
                    protocol_error, True,
                )
                return False
            success = (
                terminal.state == "SUCCEEDED"
                and terminal.code == "NAV2_SUCCEEDED"
            )
            self._set_outcome_locked(
                success,
                task_id,
                terminal.state,
                terminal.code,
                terminal.message,
                True,
            )
            return success

    def cancel_navigation(self) -> None:
        """Signal the active call; its thread sends targeted cancel and waits."""
        with self._condition:
            if self._active is not None:
                self._cancel_requested = True
                self._condition.notify_all()

    def _wait_for_terminal(self, record: _TaskRecord, deadline: float) -> None:
        while True:
            now = time.monotonic()
            with self._condition:
                if record.terminal is not None:
                    return
                cancel_requested = (
                    self._cancel_requested
                    or self._should_stop()
                    or bool(record.protocol_error)
                    or now >= deadline
                )
                if cancel_requested and not record.cancel_sent:
                    record.cancel_sent = True
                    record.cancel_started_at = now
                    record.next_query_at = (
                        now + self._cancel_confirmation_timeout_sec
                    )
                should_send_cancel = (
                    record.cancel_sent and record.cancel_started_at == now
                )
                should_query = record.force_query or now >= record.next_query_at
                if should_query:
                    record.force_query = False
                    record.next_query_at = (
                        now + self._cancel_confirmation_timeout_sec
                    )

            if should_send_cancel:
                try:
                    self._send_targeted_cancel(record.task_id)
                except Exception as exc:
                    with self._condition:
                        record.protocol_error = (
                            f"unexpected cancel transport error: {exc}"
                        )
                    self._node.get_logger().error(record.protocol_error)

            if should_query:
                try:
                    self._query_task(record)
                except Exception as exc:
                    with self._condition:
                        record.protocol_error = (
                            f"unexpected query transport error: {exc}"
                        )
                    self._node.get_logger().error(record.protocol_error)
                continue

            # Deliberately no "cancel accepted" escape here: the caller owns
            # the Action execution lock until the original task has a real
            # terminal.  A replacement Goal has its own 8 s lock-wait budget.
            with self._condition:
                self._condition.wait(timeout=0.05)

    def _send_targeted_cancel(self, target_task_id: str) -> None:
        cancel_id = f"{target_task_id}:cancel:{uuid.uuid4().hex[:8]}"
        response = self._call_service(
            cancel_id,
            "cancel",
            {
                "protocol_version": self._protocol_version,
                "client_id": self._client_id,
                "target_task_id": target_task_id,
            },
            self._service_timeout_sec,
        )
        if response is None:
            self._node.get_logger().error(
                f"waypoint_nav cancel service timeout: target={target_task_id}"
            )
            return
        try:
            self._validate_ack_response(response, cancel_id, "cancel", target_task_id)
        except WaypointNavProtocolError as exc:
            self._node.get_logger().error(
                f"waypoint_nav invalid cancel response: {exc}"
            )

    def _query_task(self, record: _TaskRecord) -> str:
        query_id = f"{record.task_id}:query:{uuid.uuid4().hex[:8]}"
        response = self._call_service(
            query_id,
            "query",
            {
                "protocol_version": self._protocol_version,
                "client_id": self._client_id,
                "target_task_id": record.task_id,
            },
            self._query_timeout_sec,
        )
        if response is None or not bool(getattr(response, "success", False)):
            return "unavailable"
        try:
            if str(response.task_id) != query_id or str(response.task_type) != "query":
                raise WaypointNavProtocolError("query response identity conflict")
            payload = json.loads(str(response.result_json))
            if not isinstance(payload, dict):
                raise WaypointNavProtocolError("query result_json must be an object")
            if payload.get("target_task_id") != record.task_id:
                raise WaypointNavProtocolError(
                    "query target_task_id conflict"
                )
            status_payload = payload.get("status", payload)
            if payload.get("found", True) is False:
                with self._condition:
                    if record.accepted or record.last_status is not None:
                        record.protocol_error = (
                            "query lost a previously accepted task within "
                            "the retention window"
                        )
                        if self._active is record:
                            self._cancel_requested = True
                        self._condition.notify_all()
                        return "invalid"
                return "missing"
            status = parse_waypoint_status(
                status_payload,
                expected_task_id=record.task_id,
                expected_matched_id=record.matched_id,
                expected_place=record.matched_place,
                last_sequence=record.last_sequence,
            )
            if status is not None:
                return (
                    "found"
                    if self._ingest_status(record, status, allow_gap=True)
                    else "invalid"
                )
            return "missing"
        except (json.JSONDecodeError, WaypointNavProtocolError) as exc:
            with self._condition:
                record.protocol_error = f"invalid query response: {exc}"
                self._condition.notify_all()
            return "invalid"

    def _call_service(
        self,
        task_id: str,
        task_type: str,
        params: Mapping[str, Any],
        timeout_sec: float,
    ) -> Any | None:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        try:
            available = self._client.wait_for_service(
                timeout_sec=max(0.0, deadline - time.monotonic())
            )
        except Exception as exc:
            self._node.get_logger().error(
                f"waypoint_nav service wait failed: {exc}"
            )
            return None
        if not available:
            self._node.get_logger().error(
                f"waypoint_nav service unavailable: {self._service_name}"
            )
            return None
        request = self._service_type.Request()
        request.task_id = task_id
        request.task_type = task_type
        request.params_json = json.dumps(
            dict(params), ensure_ascii=False, separators=(",", ":")
        )
        try:
            future = self._client.call_async(request)
        except Exception as exc:
            self._node.get_logger().error(
                f"waypoint_nav {task_type} dispatch failed: {exc}"
            )
            return None
        while time.monotonic() < deadline:
            if future.done():
                try:
                    return future.result()
                except Exception as exc:
                    self._node.get_logger().error(
                        f"waypoint_nav {task_type} call failed: {exc}"
                    )
                    return None
            time.sleep(0.02)
        return None

    def _validate_goto_response(
        self,
        response: Any,
        record: _TaskRecord,
    ) -> None:
        self._validate_ack_response(response, record.task_id, "goto_place")
        try:
            payload = json.loads(str(response.result_json))
        except json.JSONDecodeError as exc:
            raise WaypointNavProtocolError(f"invalid result_json: {exc}") from exc
        if not isinstance(payload, dict) or payload.get("accepted") is not True:
            raise WaypointNavProtocolError("goto response must contain accepted=true")
        matched_id = _required_string(payload, "matched_id")
        matched_place = _required_string(payload, "place")
        _required_string(payload, "message")
        requested_place = record.requested_place.strip()
        if requested_place in _RANDOM_PLACE_ALIASES:
            target_matches = (
                matched_id == _RANDOM_MATCHED_ID
                and matched_place == _RANDOM_PLACE_NAME
            )
        else:
            # waypoint_nav accepts either an exact YAML ID or an exact YAML
            # name, then returns both canonical fields.  One of them must
            # therefore equal the request.
            target_matches = requested_place in {matched_id, matched_place}
        if not target_matches:
            raise WaypointNavProtocolError(
                "resolved place conflicts with requested YAML ID/name: "
                f"request={requested_place!r}, matched_id={matched_id!r}, "
                f"place={matched_place!r}"
            )
        with self._condition:
            if record.matched_id and record.matched_id != matched_id:
                raise WaypointNavProtocolError(
                    "matched_id conflicts with an early status: "
                    f"{matched_id!r} != {record.matched_id!r}"
                )
            if record.matched_place and record.matched_place != matched_place:
                raise WaypointNavProtocolError(
                    "place conflicts with an early status: "
                    f"{matched_place!r} != {record.matched_place!r}"
                )
            record.matched_id = matched_id
            record.matched_place = matched_place
            record.accepted = True

    @staticmethod
    def _validate_ack_response(
        response: Any,
        request_task_id: str,
        task_type: str,
        target_task_id: str = "",
    ) -> None:
        if str(getattr(response, "task_id", "")) != request_task_id:
            raise WaypointNavProtocolError("service response task_id conflict")
        if str(getattr(response, "task_type", "")) != task_type:
            raise WaypointNavProtocolError("service response task_type conflict")
        if not bool(getattr(response, "success", False)):
            error = str(getattr(response, "error_message", ""))
            raise WaypointNavProtocolError(error or f"{task_type} was rejected")
        if target_task_id:
            try:
                payload = json.loads(str(response.result_json))
            except json.JSONDecodeError as exc:
                raise WaypointNavProtocolError(f"invalid result_json: {exc}") from exc
            if not isinstance(payload, dict) or payload.get("accepted") is not True:
                raise WaypointNavProtocolError(
                    f"{task_type} response must contain accepted=true"
                )
            echoed_target = payload.get("target_task_id")
            if echoed_target != target_task_id:
                raise WaypointNavProtocolError("target_task_id conflict")

    def _on_status(self, message: Any) -> None:
        raw = str(getattr(message, "data", ""))
        try:
            payload = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            # Without a task_id the packet cannot safely be attributed.  Ask
            # the durable query endpoint about our active task instead of
            # guessing a terminal from malformed shared-topic traffic.
            self._request_active_query_for_unattributed_status()
            self._node.get_logger().warning(
                "Invalid waypoint_nav status JSON; querying active task"
            )
            return
        if not isinstance(payload, dict):
            self._request_active_query_for_unattributed_status()
            self._node.get_logger().warning(
                "Non-object waypoint_nav status; querying active task"
            )
            return
        task_id = payload.get("task_id")
        if not isinstance(task_id, str) or not task_id.strip():
            self._request_active_query_for_unattributed_status()
            self._node.get_logger().warning(
                "waypoint_nav status without task_id; querying active task"
            )
            return

        with self._condition:
            record = self._records.get(task_id)
        if record is None:
            return
        try:
            status = parse_waypoint_status(
                payload,
                expected_task_id=record.task_id,
                expected_matched_id=record.matched_id,
                expected_place=record.matched_place,
                last_sequence=record.last_sequence,
            )
        except WaypointNavProtocolError as exc:
            with self._condition:
                record.protocol_error = str(exc)
                if self._active is record:
                    self._cancel_requested = True
                self._condition.notify_all()
            self._node.get_logger().error(
                f"waypoint_nav protocol error for {record.task_id}: {exc}"
            )
            return
        if status is not None:
            self._ingest_status(record, status, allow_gap=False)

    def _request_active_query_for_unattributed_status(self) -> None:
        """Wake the waiter for a rate-limited query of the active task.

        A malformed shared-topic packet carries no trustworthy task ID, so it
        must not directly fail the active task.  The first packet triggers an
        immediate durable query; later packets are limited to the configured
        five-second confirmation cadence to avoid a malformed-packet storm
        turning into a service storm.
        """
        now = time.monotonic()
        with self._condition:
            record = self._active
            if record is None:
                return
            last_query_at = record.last_unattributed_query_at
            if (
                last_query_at is not None
                and now - last_query_at
                < self._cancel_confirmation_timeout_sec
            ):
                return
            record.last_unattributed_query_at = now
            record.force_query = True
            self._condition.notify_all()

    def _ingest_status(
        self,
        record: _TaskRecord,
        status: WaypointNavStatus,
        *,
        allow_gap: bool,
    ) -> bool:
        with self._condition:
            previous = record.last_status
            if (
                previous is not None
                and previous.sequence is not None
                and status.sequence is not None
            ):
                if status.sequence == previous.sequence:
                    if status != previous:
                        record.protocol_error = (
                            "conflicting statuses share sequence "
                            f"{status.sequence}"
                        )
                        if self._active is record:
                            self._cancel_requested = True
                        self._condition.notify_all()
                        return False
                    return True
                if not allow_gap and status.sequence > previous.sequence + 1:
                    record.force_query = True
                    self._condition.notify_all()
                    return False
        self._accept_status(record, status)
        return True

    def _accept_status(self, record: _TaskRecord, status: WaypointNavStatus) -> None:
        callback = record.status_callback
        with self._condition:
            if not record.matched_id:
                record.matched_id = status.matched_id
            if not record.matched_place:
                record.matched_place = status.place
            record.last_sequence = status.sequence
            record.last_status = status
            record.next_query_at = (
                time.monotonic() + self._cancel_confirmation_timeout_sec
            )
            if status.terminal:
                record.terminal = status
                if self._quarantined_task_id == record.task_id:
                    self._quarantined_task_id = ""
            self._condition.notify_all()
        if callback is not None:
            try:
                callback(status)
            except Exception as exc:
                self._node.get_logger().warning(
                    f"waypoint_nav status callback failed: {exc}"
                )

    def _set_outcome(
        self,
        success: bool,
        task_id: str,
        state: str,
        code: str,
        message: str,
        terminal_confirmed: bool,
    ) -> None:
        with self._condition:
            self._set_outcome_locked(
                success, task_id, state, code, message, terminal_confirmed
            )

    def _set_outcome_locked(
        self,
        success: bool,
        task_id: str,
        state: str,
        code: str,
        message: str,
        terminal_confirmed: bool,
    ) -> None:
        self._last_outcome = WaypointNavOutcome(
            success=success,
            task_id=task_id,
            state=state,
            code=code,
            message=message,
            terminal_confirmed=terminal_confirmed,
        )
