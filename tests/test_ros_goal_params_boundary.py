from __future__ import annotations

import threading
from types import SimpleNamespace

import pytest

from marsdog_action_executor.execution_context import (
    ExecutionContext,
    parse_params_json_object,
)
from marsdog_action_executor.ros2_compat import HAS_ROS2
from marsdog_action_executor.ros_node import _debug_goal_from_request

if HAS_ROS2:
    from marsdog_action_executor.ros_node import ActionExecutorNode, GoalResponse


INVALID_PARAMS_JSON = [
    "{source:visual_direct,trigger_event:EVT_VISION_PERSON_FALL}",
    "[]",
    "true",
    "false",
    "null",
    "17",
]


def _request(params_json: str):
    return SimpleNamespace(
        goal_id="goal-visual-1",
        behavior_id="behavior-visual-1",
        behavior_name="respond_person_fall",
        priority_level=1,
        params_json=params_json,
        timeout_sec=8.0,
    )


@pytest.mark.parametrize("raw", INVALID_PARAMS_JSON)
def test_single_params_parser_rejects_malformed_and_non_object_json(raw: str) -> None:
    assert parse_params_json_object(raw) is None

    ctx = ExecutionContext.from_goal("respond_person_fall", raw)
    assert not ctx.is_valid
    assert ctx.error_reason == "invalid_params: params_json parse failed"


def test_single_params_parser_preserves_valid_json_object() -> None:
    raw = '{"source":"visual_direct","details":{"confidence":0.93}}'

    assert parse_params_json_object(raw) == {
        "source": "visual_direct",
        "details": {"confidence": 0.93},
    }
    ctx = ExecutionContext.from_goal("respond_person_fall", raw)
    assert ctx.is_valid
    assert ctx.params["source"] == "visual_direct"


def test_voice_command_identity_is_typed_but_cannot_select_action() -> None:
    params = {
        "source": "audio_direct",
        "trigger_event": "EVT_VOICE_COMMAND_SIT",
        "intent": "command_sit",
        "command_key": "SIT",
        "command_id": "CMD_SIT",
        "command_catalog_version": "2026-08-29-expanded-v1",
        "intent_source": "command_lexicon",
        "dispatch_role": "specific_command",
        "specific_event_type": "EVT_VOICE_COMMAND_SIT",
        "voice_slots": {
            "command_key": "SIT",
            "command_catalog_version": "2026-08-29-expanded-v1",
        },
        "trace_note": "preserved as metadata",
    }

    ctx = ExecutionContext.from_goal("sit_down", params)

    assert ctx.is_valid
    assert ctx.requested_behavior_name == "sit_down"
    assert ctx.command_key == "SIT"
    assert ctx.command_id == "CMD_SIT"
    assert ctx.command_catalog_version == "2026-08-29-expanded-v1"
    assert ctx.intent_source == "command_lexicon"
    assert ctx.dispatch_role == "specific_command"
    assert ctx.specific_event_type == "EVT_VOICE_COMMAND_SIT"
    assert ctx.voice_slots == {
        "command_key": "SIT",
        "command_catalog_version": "2026-08-29-expanded-v1",
    }
    assert ctx.metadata == {"trace_note": "preserved as metadata"}


@pytest.mark.parametrize("raw", INVALID_PARAMS_JSON)
def test_debug_goal_boundary_records_invalid_params_safely(raw: str) -> None:
    goal, params_valid = _debug_goal_from_request(
        _request(raw),
        timestamp=123.0,
    )

    assert not params_valid
    assert goal.params == {}
    assert goal.goal_id == "goal-visual-1"
    assert goal.behavior_name == "respond_person_fall"
    assert goal.timestamp == 123.0


def test_debug_goal_boundary_preserves_valid_object() -> None:
    goal, params_valid = _debug_goal_from_request(
        _request('{"source":"visual_direct","confidence":0.9}'),
        timestamp=123.0,
    )

    assert params_valid
    assert goal.params == {"source": "visual_direct", "confidence": 0.9}


class _Logger:
    def __init__(self) -> None:
        self.messages: list[str] = []

    def info(self, message: str) -> None:
        self.messages.append(message)

    def warning(self, message: str) -> None:
        self.messages.append(message)

    def error(self, message: str) -> None:
        self.messages.append(message)


class _DebugPublisher:
    def __init__(self) -> None:
        self.goals = []

    def publish_goal(self, goal) -> None:
        self.goals.append(goal)


class _FakeNode:
    def __init__(self) -> None:
        self.logger = _Logger()
        self._acceptable_behaviors = {"respond_person_fall"}
        self._behavior_execution_lock = threading.Lock()
        self._goal_reservation_lock = threading.Lock()
        self._reserved_goal_id = None
        self._uwb_follow_adapter = None
        self._debug = _DebugPublisher()

    def get_logger(self):
        return self.logger


class _GoalHandle:
    def __init__(self, request) -> None:
        self.request = request
        self.execute_calls = 0

    def execute(self) -> None:
        self.execute_calls += 1


@pytest.mark.skipif(not HAS_ROS2, reason="rclpy is not installed")
@pytest.mark.parametrize("raw", INVALID_PARAMS_JSON)
def test_goal_callback_rejects_invalid_params_before_accept(raw: str) -> None:
    node = _FakeNode()

    response = ActionExecutorNode._on_goal(node, _request(raw))

    assert response == GoalResponse.REJECT
    assert any("invalid_params" in message for message in node.logger.messages)


@pytest.mark.skipif(not HAS_ROS2, reason="rclpy is not installed")
def test_goal_callback_accepts_valid_json_object() -> None:
    node = _FakeNode()

    response = ActionExecutorNode._on_goal(
        node,
        _request('{"source":"visual_direct"}'),
    )

    assert response == GoalResponse.ACCEPT


@pytest.mark.skipif(not HAS_ROS2, reason="rclpy is not installed")
@pytest.mark.parametrize("raw", INVALID_PARAMS_JSON)
def test_accepted_callback_never_throws_for_invalid_params(raw: str) -> None:
    node = _FakeNode()
    goal_handle = _GoalHandle(_request(raw))

    ActionExecutorNode._on_accepted(node, goal_handle)

    assert goal_handle.execute_calls == 1
    assert len(node._debug.goals) == 1
    assert node._debug.goals[0].params == {}
    assert any(
        "recorded params as an empty object" in message
        for message in node.logger.messages
    )
