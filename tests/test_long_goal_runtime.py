"""Long Goal execution keeps one identity through follow and roam cycles."""

import threading
import time
from types import SimpleNamespace as NS

import pytest

from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.ros2_compat import HAS_ROS2

if HAS_ROS2:
    from marsdog_action_executor.ros_node import ActionExecutorNode


class _Goal:
    def __init__(self, name):
        self.request = NS(goal_id="goal-1", behavior_id="behavior-1", behavior_name=name)
        self.is_cancel_requested = False
        self.terminal = ""

    def canceled(self):
        self.terminal = "CANCELED"

    def abort(self):
        self.terminal = "FAILED"


class _Interrupt:
    cancel_requested = False

    def reset(self):
        self.cancel_requested = False

    def request_cancel(self):
        self.cancel_requested = True


class _Adapter:
    def __init__(self, goal):
        self.goal = goal
        self.starts = 0
        self.polls = 0
        self.stops = 0
        self.active = False
        self.last_error = ""

    def start(self, *, wait_for_grace):
        self.starts += 1
        self.active = True
        return True

    def enable_target_monitor(self):
        pass

    def poll_health(self):
        self.polls += 1
        if self.polls == 2:
            self.goal.is_cancel_requested = True
        return True

    def stop_confirmed(self, reason):
        self.stops += 1
        self.active = False
        return True


class _Node:
    def __init__(self):
        self._interrupt = _Interrupt()
        self._posture = NS(reset=lambda: None)
        self._debug = NS(publish_result=lambda result: None)
        self._stationary_expression_adapter = NS(cancel_step=lambda: None)
        self._chassis_backend = NS(
            emergency_stop=lambda: None, finish_navigation=lambda: True,
        )
        self._uwb_follow_adapter = None
        self._uwb_roam_adapter = None
        self._long_goal_id = None
        self.feedback = []

    def _lease_expired(self, gid, bid, started):
        return False

    def _publish_feedback(self, *args):
        self.feedback.append(args)

    def get_logger(self):
        return NS(error=lambda message: None)


@pytest.mark.skipif(not HAS_ROS2, reason="rclpy unavailable")
def test_follow_starts_once_and_returns_canceled_after_controller_stop():
    goal = _Goal("follow_owner")
    node = _Node()
    node._uwb_follow_adapter = _Adapter(goal)
    ctx = ExecutionContext.from_goal("follow_owner", {})
    ctx.resolved_behavior_name = "follow_owner"

    result = ActionExecutorNode._execute_long_behavior(node, goal, ctx, [], time.time())

    assert node._uwb_follow_adapter.starts == 1
    assert node._uwb_follow_adapter.stops == 1
    assert goal.terminal == result.status == "CANCELED"
    assert node._long_goal_id is None


@pytest.mark.skipif(not HAS_ROS2, reason="rclpy unavailable")
def test_play_completes_two_full_cycles_in_one_goal():
    goal = _Goal("play_alone")
    node = _Node()
    called = []
    stops = []
    node._chassis_backend = NS(
        emergency_stop=lambda: stops.append("stop"),
        finish_navigation=lambda: True,
    )
    node._uwb_roam_adapter = NS(active=False, cancel_step=lambda: None)

    def execute(stage, ctx):
        called.append(stage["stage_id"])
        if len(called) == 4:
            goal.is_cancel_requested = True
        return NS(success=True, unit_id=stage["stage_id"], message="ok")

    node._stage_executor = NS(execute_stage=execute)
    ctx = ExecutionContext.from_goal("play_alone", {})
    ctx.resolved_behavior_name = "play_alone"
    stages = [{"stage_id": "uwb_roam"}, {"stage_id": "play"}]

    result = ActionExecutorNode._execute_long_behavior(
        node, goal, ctx, stages, time.time(),
    )

    assert called == ["uwb_roam", "play", "uwb_roam", "play"]
    assert result.status == goal.terminal == "CANCELED"
    assert result.metadata_json.find('"cycles_completed": 2') >= 0
    assert stops == ["stop"]


@pytest.mark.skipif(not HAS_ROS2, reason="rclpy unavailable")
def test_play_failure_does_not_begin_next_cycle():
    goal = _Goal("play_alone")
    node = _Node()
    called = []
    node._uwb_roam_adapter = NS(active=False, cancel_step=lambda: None)

    def execute(stage, ctx):
        called.append(stage["stage_id"])
        ctx.metadata["unit_failure_reason"] = "uwb_roam_failed:INPUT_TIMEOUT"
        return NS(success=False, unit_id="ACT_UWB_RANDOM_ROAM", message="failed")

    node._stage_executor = NS(execute_stage=execute)
    ctx = ExecutionContext.from_goal("play_alone", {})
    ctx.resolved_behavior_name = "play_alone"

    result = ActionExecutorNode._execute_long_behavior(
        node, goal, ctx,
        [{"stage_id": "uwb_roam"}, {"stage_id": "play"}], time.time(),
    )

    assert called == ["uwb_roam"]
    assert result.status == goal.terminal == "FAILED"
    assert "INPUT_TIMEOUT" in result.reason
