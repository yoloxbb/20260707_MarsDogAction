"""A cancel acknowledgement does not hand motion to a replacement Goal."""

import threading
import time
import json
from types import SimpleNamespace

import pytest

from marsdog_action_executor.ros2_compat import HAS_ROS2

if HAS_ROS2:
    from marsdog_action_executor.ros_node import ActionExecutorNode, GoalResponse


class _Logger:
    def info(self, message):
        pass

    def warning(self, message):
        pass

    def error(self, message):
        pass


class _Node:
    def __init__(self):
        self._acceptable_behaviors = {
            "follow_owner", "play_alone", "respond_person_fall",
            "respond_stop_gesture", "sit_down", "emergency_stop",
        }
        self._goal_reservation_lock = threading.Lock()
        self._reserved_goal_id = None
        self._behavior_execution_lock = threading.Lock()
        self._uwb_follow_adapter = None
        self._uwb_roam_adapter = None
        self._logger = _Logger()
        self._lease_lock = threading.Lock()
        self._lease_seen = {}

    def get_logger(self):
        return self._logger

    def get_parameter(self, name):
        return SimpleNamespace(value=(3.0 if name.endswith("grace_sec") else 2.0))


def _goal(name, gid, priority=1):
    return SimpleNamespace(
        behavior_name=name, goal_id=gid, priority_level=priority,
        params_json="{}",
    )


@pytest.mark.skipif(not HAS_ROS2, reason="rclpy unavailable")
def test_cancel_pending_rejects_same_event_and_safety_replacement():
    node = _Node()
    assert ActionExecutorNode._on_goal(node, _goal("follow_owner", "old")) == GoalResponse.ACCEPT
    node._behavior_execution_lock.acquire()
    try:
        assert ActionExecutorNode._on_goal(node, _goal("follow_owner", "duplicate")) == GoalResponse.REJECT
        assert ActionExecutorNode._on_goal(node, _goal("respond_person_fall", "safety")) == GoalResponse.REJECT
        assert ActionExecutorNode._on_goal(node, _goal("sit_down", "replacement")) == GoalResponse.REJECT
    finally:
        node._behavior_execution_lock.release()
    # CancelGoal acceptance does not clear the reservation. Only the real
    # Result path clears it after the old execution releases the mutex.
    assert ActionExecutorNode._on_goal(node, _goal("respond_person_fall", "safety")) == GoalResponse.REJECT
    node._reserved_goal_id = None
    assert ActionExecutorNode._on_goal(node, _goal("respond_person_fall", "safety")) == GoalResponse.ACCEPT


@pytest.mark.skipif(not HAS_ROS2, reason="rclpy unavailable")
def test_uwb_terminal_unknown_rejects_replacement():
    node = _Node()
    node._uwb_roam_adapter = SimpleNamespace(recovery_required=True)
    assert ActionExecutorNode._on_goal(node, _goal("play_alone", "new")) == GoalResponse.REJECT


@pytest.mark.skipif(not HAS_ROS2, reason="rclpy unavailable")
def test_lease_is_keyed_by_goal_and_behavior_id():
    node = _Node()
    node._reserved_goal_id = "goal-1"
    started = time.monotonic() - 4.0
    payload = {
        "schema_version": 1, "goal_id": "goal-1",
        "behavior_id": "different-behavior", "interaction_id": "old-session",
    }
    ActionExecutorNode._on_goal_lease(node, SimpleNamespace(data=json.dumps(payload)))
    assert ActionExecutorNode._lease_expired(node, "goal-1", "behavior-1", started)
    payload["behavior_id"] = "behavior-1"
    ActionExecutorNode._on_goal_lease(node, SimpleNamespace(data=json.dumps(payload)))
    assert not ActionExecutorNode._lease_expired(node, "goal-1", "behavior-1", started)


@pytest.mark.skipif(not HAS_ROS2, reason="rclpy unavailable")
def test_voice_idle_does_not_stop_goal_owned_uwb():
    node = _Node()
    node._attention_controller = None
    node._twist_publisher = None

    class Follow:
        def update_control(self, payload):
            raise AssertionError("voice idle must not control Goal UWB")

    node._uwb_follow_adapter = Follow()
    ActionExecutorNode._on_attention_control(
        node, SimpleNamespace(data=json.dumps({
            "enabled": False, "mode": "follow_owner", "interaction_id": "old-session",
        }))
    )
