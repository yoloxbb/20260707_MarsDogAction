"""Contract tests for the Action-side waypoint_nav status parser."""

from __future__ import annotations

import json
from types import SimpleNamespace
import threading
import time

import pytest

from marsdog_action_executor.adapters.waypoint_nav_adapter import (
    Ros2WaypointNavClient,
    WaypointNavProtocolError,
    parse_waypoint_status,
)
from marsdog_action_executor.ros_node import _navigation_cancel_won


def _status(**overrides):
    payload = {
        "task_id": "action:g-1:waypoint",
        "task_type": "goto_place",
        "state": "RUNNING",
        "code": "NAV2_ACCEPTED",
        "matched_id": "C",
        "place": "food",
        "message": "Nav2 accepted",
        "safe_to_interrupt": True,
        "sequence": 2,
    }
    payload.update(overrides)
    return payload


def test_foreign_task_status_is_filtered() -> None:
    assert parse_waypoint_status(
        _status(task_id="another-task"),
        expected_task_id="action:g-1:waypoint",
        expected_matched_id="C",
        expected_place="food",
    ) is None


def test_matching_status_is_strict_and_preserves_safe_flag() -> None:
    status = parse_waypoint_status(
        _status(),
        expected_task_id="action:g-1:waypoint",
        expected_matched_id="C",
        expected_place="food",
        last_sequence=1,
    )

    assert status is not None
    assert status.state == "RUNNING"
    assert status.code == "NAV2_ACCEPTED"
    assert status.safe_to_interrupt is True
    assert not status.terminal


@pytest.mark.parametrize(
    "code,safe",
    [
        ("QUEUED", True),
        ("QUEUED", False),
        ("RECOVERY_REQUIRED", False),
    ],
)
def test_v1_running_phases_are_accepted(code, safe) -> None:
    status = parse_waypoint_status(
        _status(code=code, safe_to_interrupt=safe),
        expected_task_id="action:g-1:waypoint",
        expected_matched_id="C",
        expected_place="food",
    )

    assert status is not None
    assert status.state == "RUNNING"
    assert status.code == code
    assert status.safe_to_interrupt is safe
    assert not status.terminal


@pytest.mark.parametrize("code", ["DISPATCHED", "RECOVERY_REQUIRED"])
def test_unresolved_running_phases_cannot_claim_safe_interrupt(code) -> None:
    with pytest.raises(WaypointNavProtocolError, match="safe_to_interrupt"):
        parse_waypoint_status(
            _status(code=code, safe_to_interrupt=True),
            expected_task_id="action:g-1:waypoint",
            expected_matched_id="C",
            expected_place="food",
        )


@pytest.mark.parametrize(
    "change,match",
    [
        ({"state": "PAUSED"}, "unknown state"),
        ({"code": "SOMETHING_NEW"}, "state/code"),
        ({"task_type": "cancel"}, "task_type conflict"),
        ({"matched_id": "D"}, "matched_id conflicts"),
        ({"place": "toilet"}, "place conflicts"),
        ({"safe_to_interrupt": "true"}, "must be boolean"),
        ({"sequence": 0}, "decreasing sequence"),
        ({"sequence": None}, "sequence disappeared"),
    ],
)
def test_matching_status_rejects_unknown_or_conflicting_fields(
    change, match
) -> None:
    with pytest.raises(WaypointNavProtocolError, match=match):
        parse_waypoint_status(
            _status(**change),
            expected_task_id="action:g-1:waypoint",
            expected_matched_id="C",
            expected_place="food",
            last_sequence=1,
        )


def test_only_exact_success_pair_is_terminal_success() -> None:
    status = parse_waypoint_status(
        _status(
            state="SUCCEEDED",
            code="NAV2_SUCCEEDED",
            safe_to_interrupt=False,
            sequence=3,
        ),
        expected_task_id="action:g-1:waypoint",
        expected_matched_id="C",
        expected_place="food",
        last_sequence=2,
    )

    assert status is not None
    assert status.terminal
    assert status.state == "SUCCEEDED"
    assert status.code == "NAV2_SUCCEEDED"


def test_terminal_status_cannot_claim_safe_interrupt() -> None:
    with pytest.raises(WaypointNavProtocolError, match="safe_to_interrupt"):
        parse_waypoint_status(
            _status(
                state="SUCCEEDED",
                code="NAV2_SUCCEEDED",
                safe_to_interrupt=True,
                sequence=3,
            ),
            expected_task_id="action:g-1:waypoint",
            expected_matched_id="C",
            expected_place="food",
            last_sequence=2,
        )


def test_current_status_without_optional_safe_or_sequence_is_compatible() -> None:
    payload = _status()
    payload.pop("safe_to_interrupt")
    payload.pop("sequence")

    status = parse_waypoint_status(
        payload,
        expected_task_id="action:g-1:waypoint",
        expected_matched_id="C",
        expected_place="food",
    )

    assert status is not None
    assert status.safe_to_interrupt is False
    assert status.sequence is None


class _Logger:
    def error(self, _message) -> None:
        pass

    def warning(self, _message) -> None:
        pass


def _client_without_ros(call_service):
    client = Ros2WaypointNavClient.__new__(Ros2WaypointNavClient)
    client._node = SimpleNamespace(get_logger=lambda: _Logger())
    client._service_name = "/waypoint_nav/task"
    client._protocol_version = "1.0"
    client._client_id = "marsdog_action_executor"
    client._service_timeout_sec = 0.05
    client._query_timeout_sec = 0.02
    client._cancel_confirmation_timeout_sec = 0.02
    client._terminal_retention_sec = 86400.0
    client._should_stop = lambda: False
    client._condition = threading.Condition()
    client._active = None
    client._records = {}
    client._seen_task_ids = set()
    client._cancel_requested = False
    client._quarantined_task_id = ""
    client._last_outcome = SimpleNamespace(task_id="")
    client._call_service = call_service
    return client


def _response(task_id, task_type, *, success=True, result=None, error=""):
    return SimpleNamespace(
        success=success,
        task_id=task_id,
        task_type=task_type,
        result_json=json.dumps(result or {}),
        error_message=error,
    )


def _terminal(
    task_id,
    *,
    state="SUCCEEDED",
    code="NAV2_SUCCEEDED",
    matched_id="C",
    place="food",
):
    return {
        "task_id": task_id,
        "task_type": "goto_place",
        "state": state,
        "code": code,
        "matched_id": matched_id,
        "place": place,
        "message": "terminal",
        "safe_to_interrupt": False,
        "sequence": 3,
    }


def test_status_subscription_race_before_goto_response_is_safe() -> None:
    task_id = "action:g-race:waypoint"
    holder = {}

    def call_service(request_id, task_type, _params, _timeout):
        assert task_type == "goto_place"
        holder["client"]._on_status(
            SimpleNamespace(data=json.dumps(_terminal(task_id)))
        )
        return _response(
            request_id,
            task_type,
            result={
                "accepted": True,
                "matched_id": "C",
                "place": "food",
                "message": "accepted",
            },
        )

    client = _client_without_ros(call_service)
    holder["client"] = client

    assert client(task_id, "C", 0.5)
    assert client.last_outcome.terminal_confirmed
    assert client.last_outcome.code == "NAV2_SUCCEEDED"


def test_exact_yaml_name_is_accepted_and_canonical_id_is_tracked() -> None:
    task_id = "action:g-name:waypoint"
    holder = {}
    sent_params = {}

    def call_service(request_id, task_type, params, _timeout):
        sent_params.update(params)
        holder["client"]._on_status(
            SimpleNamespace(
                data=json.dumps(
                    _terminal(task_id, matched_id="A", place="客厅")
                )
            )
        )
        return _response(
            request_id,
            task_type,
            result={
                "accepted": True,
                "matched_id": "A",
                "place": "客厅",
                "message": "accepted",
            },
        )

    client = _client_without_ros(call_service)
    holder["client"] = client

    assert client(task_id, "客厅", 0.5)
    assert sent_params["place"] == "客厅"
    assert client.last_outcome.code == "NAV2_SUCCEEDED"


@pytest.mark.parametrize("requested_place", ["K", "11"])
def test_reserved_random_target_accepts_server_normalization(
    requested_place,
) -> None:
    task_id = f"action:g-random-{requested_place}:waypoint"
    holder = {}
    sent_params = {}

    def call_service(request_id, task_type, params, _timeout):
        sent_params.update(params)
        holder["client"]._on_status(
            SimpleNamespace(
                data=json.dumps(
                    _terminal(
                        task_id,
                        matched_id="11",
                        place="随机点位",
                    )
                )
            )
        )
        return _response(
            request_id,
            task_type,
            result={
                "accepted": True,
                "matched_id": "11",
                "place": "随机点位",
                "message": "accepted",
            },
        )

    client = _client_without_ros(call_service)
    holder["client"] = client

    assert client(task_id, requested_place, 0.5)
    assert sent_params["place"] == requested_place
    assert client.last_outcome.code == "NAV2_SUCCEEDED"


def test_goto_response_rejects_a_different_yaml_target() -> None:
    task_id = "action:g-wrong-place:waypoint"
    holder = {}

    def call_service(request_id, task_type, _params, _timeout):
        holder["client"]._on_status(
            SimpleNamespace(
                data=json.dumps(
                    _terminal(task_id, matched_id="B", place="厨房")
                )
            )
        )
        return _response(
            request_id,
            task_type,
            result={
                "accepted": True,
                "matched_id": "B",
                "place": "厨房",
                "message": "accepted",
            },
        )

    client = _client_without_ros(call_service)
    holder["client"] = client

    assert not client(task_id, "客厅", 0.5)
    assert client.last_outcome.code == "PROTOCOL_ERROR"


def test_cancel_is_targeted_and_waits_for_original_terminal() -> None:
    task_id = "action:g-cancel:waypoint"
    goto_called = threading.Event()
    calls = []
    holder = {}

    def call_service(request_id, task_type, params, _timeout):
        calls.append((request_id, task_type, dict(params)))
        if task_type == "goto_place":
            goto_called.set()
            return _response(
                request_id,
                task_type,
                result={
                    "accepted": True,
                    "matched_id": "C",
                    "place": "food",
                    "message": "accepted",
                },
            )
        if task_type == "cancel":
            threading.Timer(
                0.01,
                lambda: holder["client"]._on_status(
                    SimpleNamespace(
                        data=json.dumps(
                            _terminal(
                                task_id,
                                state="INTERRUPTED",
                                code="CLIENT_CANCELLED",
                            )
                        )
                    )
                ),
            ).start()
            return _response(
                request_id,
                task_type,
                result={
                    "accepted": True,
                    "target_task_id": task_id,
                    "message": "cancel accepted",
                },
            )
        return _response(request_id, task_type, success=False)

    client = _client_without_ros(call_service)
    holder["client"] = client
    result = []
    worker = threading.Thread(
        target=lambda: result.append(client(task_id, "C", 0.5))
    )
    worker.start()
    assert goto_called.wait(0.2)
    time.sleep(0.01)
    client.cancel_navigation()
    worker.join(0.5)

    assert not worker.is_alive()
    assert result == [False]
    cancel_params = [params for _, kind, params in calls if kind == "cancel"]
    assert cancel_params == [{
        "protocol_version": "1.0",
        "client_id": "marsdog_action_executor",
        "target_task_id": task_id,
    }]
    assert client.last_outcome.state == "INTERRUPTED"
    assert client.last_outcome.terminal_confirmed


def test_duplicate_rejection_recovers_durable_terminal_via_query() -> None:
    task_id = "action:g-restart:waypoint"
    calls = []

    def call_service(request_id, task_type, _params, _timeout):
        calls.append(task_type)
        if task_type == "goto_place":
            return _response(
                request_id,
                task_type,
                success=False,
                result={"accepted": False, "code": "DUPLICATE_TASK_ID"},
                error="duplicate task_id",
            )
        assert task_type == "query"
        return _response(
            request_id,
            task_type,
            result={
                "found": True,
                "target_task_id": task_id,
                "status": _terminal(task_id),
            },
        )

    client = _client_without_ros(call_service)

    assert client(task_id, "C", 0.5)
    assert calls == ["goto_place", "query"]
    assert client.last_outcome.code == "NAV2_SUCCEEDED"


def test_runtime_query_recovers_a_missed_terminal() -> None:
    task_id = "action:g-query:waypoint"
    calls = []

    def call_service(request_id, task_type, _params, _timeout):
        calls.append(task_type)
        if task_type == "goto_place":
            return _response(
                request_id,
                task_type,
                result={
                    "accepted": True,
                    "matched_id": "C",
                    "place": "food",
                    "message": "accepted",
                },
            )
        return _response(
            request_id,
            task_type,
            result={
                "found": True,
                "target_task_id": task_id,
                "status": _terminal(task_id),
            },
        )

    client = _client_without_ros(call_service)

    assert client(task_id, "C", 0.5)
    assert calls == ["goto_place", "query"]


@pytest.mark.parametrize("bad_status", ["{", "[]", "{}"])
def test_unattributed_bad_status_immediately_queries_active_task(
    bad_status,
) -> None:
    task_id = "action:g-bad-status:waypoint"
    goto_called = threading.Event()
    query_called = threading.Event()
    holder = {}

    def call_service(request_id, task_type, _params, _timeout):
        if task_type == "goto_place":
            goto_called.set()
            return _response(
                request_id,
                task_type,
                result={
                    "accepted": True,
                    "matched_id": "C",
                    "place": "food",
                    "message": "accepted",
                },
            )
        assert task_type == "query"
        query_called.set()
        return _response(
            request_id,
            task_type,
            result={
                "found": True,
                "target_task_id": task_id,
                "status": _terminal(task_id),
            },
        )

    client = _client_without_ros(call_service)
    client._cancel_confirmation_timeout_sec = 0.5
    holder["client"] = client
    result = []
    worker = threading.Thread(
        target=lambda: result.append(client(task_id, "C", 1.0))
    )
    worker.start()
    assert goto_called.wait(0.2)

    client._on_status(SimpleNamespace(data=bad_status))

    assert query_called.wait(0.2)
    worker.join(0.5)
    assert result == [True]


def test_failed_query_after_bad_status_keeps_navigation_interlocked() -> None:
    task_id = "action:g-bad-status-query-failed:waypoint"
    goto_called = threading.Event()
    query_called = threading.Event()
    holder = {}

    def call_service(request_id, task_type, _params, _timeout):
        if task_type == "goto_place":
            goto_called.set()
            return _response(
                request_id,
                task_type,
                result={
                    "accepted": True,
                    "matched_id": "C",
                    "place": "food",
                    "message": "accepted",
                },
            )
        if task_type == "query":
            query_called.set()
            return None
        return _response(request_id, task_type, success=False)

    client = _client_without_ros(call_service)
    client._cancel_confirmation_timeout_sec = 0.5
    holder["client"] = client
    result = []
    worker = threading.Thread(
        target=lambda: result.append(client(task_id, "C", 1.0))
    )
    worker.start()
    assert goto_called.wait(0.2)

    client._on_status(SimpleNamespace(data="{"))

    assert query_called.wait(0.2)
    assert worker.is_alive()
    assert client.navigation_interlocked

    client._on_status(
        SimpleNamespace(
            data=json.dumps(
                _terminal(
                    task_id,
                    state="INTERRUPTED",
                    code="CLIENT_CANCELLED",
                )
            )
        )
    )
    worker.join(0.5)
    assert result == [False]
    assert client.last_outcome.terminal_confirmed


def test_query_cannot_lose_an_accepted_task_within_retention() -> None:
    task_id = "action:g-query-missing:waypoint"
    calls = []
    holder = {}

    def call_service(request_id, task_type, _params, _timeout):
        calls.append(task_type)
        if task_type == "goto_place":
            return _response(
                request_id,
                task_type,
                result={
                    "accepted": True,
                    "matched_id": "C",
                    "place": "food",
                    "message": "accepted",
                },
            )
        if task_type == "query":
            return _response(
                request_id,
                task_type,
                result={
                    "found": False,
                    "target_task_id": task_id,
                },
            )
        threading.Timer(
            0.01,
            lambda: holder["client"]._on_status(
                SimpleNamespace(
                    data=json.dumps(
                        _terminal(
                            task_id,
                            state="INTERRUPTED",
                            code="CLIENT_CANCELLED",
                        )
                    )
                )
            ),
        ).start()
        return _response(
            request_id,
            task_type,
            result={
                "accepted": True,
                "target_task_id": task_id,
                "message": "cancel accepted",
            },
        )

    client = _client_without_ros(call_service)
    holder["client"] = client

    assert not client(task_id, "C", 0.5)
    assert calls == ["goto_place", "query", "cancel"]
    assert client.last_outcome.code == "PROTOCOL_ERROR"
    assert client.last_outcome.terminal_confirmed


def test_same_task_id_cannot_start_navigation_twice() -> None:
    task_id = "action:g-duplicate:waypoint"
    calls = []
    holder = {}

    def call_service(request_id, task_type, _params, _timeout):
        calls.append(task_type)
        holder["client"]._on_status(
            SimpleNamespace(data=json.dumps(_terminal(task_id)))
        )
        return _response(
            request_id,
            task_type,
            result={
                "accepted": True,
                "matched_id": "C",
                "place": "food",
                "message": "accepted",
            },
        )

    client = _client_without_ros(call_service)
    holder["client"] = client

    assert client(task_id, "C", 0.5)
    assert not client(task_id, "C", 0.5)
    assert calls == ["goto_place"]
    assert client.last_outcome.code == "DUPLICATE_TASK_ID"


@pytest.mark.parametrize(
    "state,terminal_confirmed,expected",
    [
        ("INTERRUPTED", True, True),
        ("SUCCEEDED", True, False),
        ("FAILED", True, False),
        ("INTERRUPTED", False, False),
    ],
)
def test_cancel_race_uses_real_waypoint_terminal(
    state,
    terminal_confirmed,
    expected,
) -> None:
    outcome = SimpleNamespace(
        state=state,
        terminal_confirmed=terminal_confirmed,
    )

    assert _navigation_cancel_won(True, outcome) is expected


def test_cancel_before_waypoint_dispatch_is_still_canceled() -> None:
    assert _navigation_cancel_won(True, None)
    assert not _navigation_cancel_won(False, None)
