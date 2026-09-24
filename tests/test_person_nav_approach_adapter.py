import threading
import time
import sys
import json
from types import ModuleType
from types import SimpleNamespace

from marsdog_action_executor.adapters.person_nav_approach_adapter import (
    PersonNavApproachAdapter,
    Ros2PersonApproachTransport,
)
from marsdog_action_executor.adapters.target_approach_adapter import TargetApproachResult
from marsdog_action_executor.ros_node import _dispatch_visual_event
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.interrupt_manager import InterruptManager
from marsdog_action_executor.result_evaluator import ResultEvaluator
from marsdog_action_executor.units.base_unit_executor import UnitState
from marsdog_action_executor.units.unit_executors import TaskExecutor


TARGET_ID = "epoch-1:human:7"
POSE = {
    "header": {"frame_id": "map"},
    "pose": {
        "position": {"x": 1.0, "y": 2.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    },
}
PERSON_POINT = {
    "header": {"frame_id": "map"},
    "point": {"x": 1.0, "y": 2.0, "z": 0.0},
}


def context(role="owner", speaker_id="owner"):
    return SimpleNamespace(
        target={"target_type": "human", "vision_epoch": "epoch-1", "target_id": TARGET_ID},
        params={
            "interaction_id": "interaction-1", "wake_id": "wake-1",
            "speaker_role": role, "speaker_id": speaker_id,
            "speaker_status": "matched",
            "strict_target_lock": True, "allow_target_switch": False,
            "stand_off_distance_m": 1.5,
        },
        metadata={},
        report_runtime_feedback=lambda *args: None,
    )


class Transport:
    def __init__(self, result):
        self.result = result
        self.locates = []
        self.navigations = []
        self.cancels = 0

    def locate(self, target_id, stand_off, deadline, cancel):
        self.locates.append((target_id, stand_off))
        return self.result, "ok"

    def valid_pose(self, pose):
        return Ros2PersonApproachTransport.valid_pose(pose)

    def navigate(self, pose, deadline, cancel):
        self.navigations.append(pose)
        return True, False, "navigation_succeeded"

    def cancel(self):
        self.cancels += 1


def run(adapter, ctx):
    return adapter.execute_task({"unit_id": adapter.ACTION_ID}, ctx, 160.0)


def test_owner_locates_once_and_navigates_once():
    transport = Transport({
        "ok": True, "target_id": TARGET_ID, "navigation_required": True,
        "navigation_goal": POSE, "status": 0, "person_point": PERSON_POINT,
    })
    stops = []
    result = run(PersonNavApproachAdapter(transport, stops.append), context())
    assert result.success
    assert transport.locates == [(TARGET_ID, 1.5)]
    assert transport.navigations == [POSE]
    assert len(stops) == 3


def _visual_target(*, identity="unknown", tracking_state="tracking", age_ms=20.0):
    return {
        "vision_epoch": "epoch-1",
        "active_target": {
            "vision_epoch": "epoch-1", "target_id": TARGET_ID,
            "target_type": "human", "identity": identity,
            "identity_state": "unverified", "tracking_state": tracking_state,
            "last_seen_age_ms": age_ms,
        },
    }


def test_explicit_owner_commands_bind_unknown_visual_track_once():
    transport = Transport({
        "ok": True, "target_id": TARGET_ID, "navigation_required": False,
        "status": 0, "person_point": PERSON_POINT,
    })
    adapter = PersonNavApproachAdapter(transport, lambda _: None)
    commands = {
        "ACT_INTERACT_APPROACH_OWNER": "come_to_owner",
        "ACT_INTERACT_APPROACH_OWNER_CLOSER": "approach_owner",
        "ACT_INTERACT_RETURN_OWNER": "return_to_owner",
    }
    for unit_id, behavior in commands.items():
        ctx = context()
        ctx.resolved_behavior_name = behavior
        ctx.target = None
        ctx.params.pop("wake_id")
        adapter.update_visual(_visual_target())
        assert adapter.execute_task({"unit_id": unit_id}, ctx, 160.0).success
        assert transport.locates[-1] == (TARGET_ID, 1.5)
        assert ctx.target["identity"] == "unknown"
        assert ctx.metadata["target_resolution"] == "action_visual_event"

        ctx.resolved_behavior_name = "approach_voice_caller"
        assert adapter.execute_task({"unit_id": unit_id}, ctx, 160.0).reason == (
            "owner_target_not_authorized"
        )
    assert len(transport.locates) == len(commands)


def test_explicit_owner_command_fails_without_fresh_human_track():
    transport = Transport({
        "ok": True, "target_id": TARGET_ID, "navigation_required": False,
        "status": 0, "person_point": PERSON_POINT,
    })
    adapter = PersonNavApproachAdapter(
        transport, lambda _: None, target_acquire_timeout_sec=0.04,
    )
    ctx = context()
    ctx.target = None
    ctx.resolved_behavior_name = "come_to_owner"
    unit = {"unit_id": "ACT_INTERACT_APPROACH_OWNER"}
    adapter.update_visual(_visual_target(tracking_state="temporarily_lost"))
    assert adapter.execute_task(unit, ctx, 1.0).reason == "visual_target_unavailable"
    adapter.update_visual(_visual_target(age_ms=600.0))
    assert adapter.execute_task(unit, ctx, 1.0).reason == "visual_target_unavailable"
    adapter.update_visual(_visual_target())
    adapter.update_visual({})
    assert adapter.execute_task(unit, ctx, 1.0).reason == "visual_target_unavailable"
    assert transport.locates == []


def test_visual_dispatch_updates_and_revokes_person_nav_target():
    adapter = PersonNavApproachAdapter(Transport(None), lambda _: None)
    assert _dispatch_visual_event(
        json.dumps(_visual_target()), person_nav_approach_adapter=adapter,
    )
    assert adapter._current_human_target()["target_id"] == TARGET_ID
    assert not _dispatch_visual_event(
        "not json", person_nav_approach_adapter=adapter,
    )
    assert adapter._current_human_target() is None


def test_cancel_while_waiting_for_visual_target_ends_without_localization():
    transport = Transport(None)
    adapter = PersonNavApproachAdapter(
        transport, lambda _: None, target_acquire_timeout_sec=1.0,
    )
    ctx = context()
    ctx.target = None
    ctx.resolved_behavior_name = "come_to_owner"
    results = []
    worker = threading.Thread(target=lambda: results.append(
        adapter.execute_task({"unit_id": "ACT_INTERACT_APPROACH_OWNER"}, ctx, 2.0)
    ))
    worker.start()
    time.sleep(0.05)
    adapter.cancel_task()
    worker.join(1.0)
    assert not worker.is_alive()
    assert results[0].canceled
    assert transport.locates == []


def test_lite3_navigation_preflight_and_settle_gate_result():
    transport = Transport({
        "ok": True, "target_id": TARGET_ID, "navigation_required": True,
        "navigation_goal": POSE, "status": 0, "person_point": PERSON_POINT,
    })

    class Chassis:
        def __init__(self):
            self.prepared = 0
            self.finished = 0
            self.settle_ok = True

        def prepare_navigation(self):
            self.prepared += 1
            return True

        def finish_navigation(self):
            self.finished += 1
            return self.settle_ok

    chassis = Chassis()
    adapter = PersonNavApproachAdapter(
        transport, lambda _: None, chassis_backend=chassis,
    )
    assert run(adapter, context()).success
    assert (chassis.prepared, chassis.finished) == (1, 1)

    chassis.settle_ok = False
    result = run(adapter, context())
    assert not result.success
    assert result.metadata["recovery_required"] is True
    assert result.reason.startswith("chassis_navigation_settle_failed")
    assert (chassis.prepared, chassis.finished) == (2, 2)


def test_within_stand_off_does_not_navigate():
    transport = Transport({
        "ok": True, "target_id": TARGET_ID, "navigation_required": False,
        "status": 0, "person_point": PERSON_POINT,
    })
    result = run(PersonNavApproachAdapter(transport, lambda _: None), context())
    assert result.success
    assert not transport.navigations


def test_stranger_and_target_mismatch_never_navigate():
    transport = Transport({
        "ok": True, "target_id": "epoch-1:human:8",
        "navigation_required": True, "navigation_goal": POSE,
        "status": 0, "person_point": PERSON_POINT,
    })
    adapter = PersonNavApproachAdapter(transport, lambda _: None)
    assert run(adapter, context("stranger", "unknown")).reason == "wake_identity_not_authorized"
    assert not transport.locates
    assert run(adapter, context()).reason == "localization_target_mismatch"
    assert not transport.navigations


def test_unmatched_voice_and_short_stand_off_are_rejected():
    transport = Transport({
        "ok": True, "target_id": TARGET_ID, "navigation_required": False,
        "status": 0, "person_point": PERSON_POINT,
    })
    adapter = PersonNavApproachAdapter(transport, lambda _: None)
    unmatched = context()
    unmatched.params["speaker_status"] = "ambiguous"
    assert run(adapter, unmatched).reason == "wake_identity_not_authorized"
    too_close = context()
    too_close.params["stand_off_distance_m"] = 0.5
    assert run(adapter, too_close).reason == "unsafe_stand_off_distance"
    assert not transport.locates


def test_cancel_waits_for_inner_navigation_terminal():
    class BlockingTransport(Transport):
        def __init__(self):
            super().__init__({
                "ok": True, "target_id": TARGET_ID,
                "navigation_required": True, "navigation_goal": POSE,
                "status": 0, "person_point": PERSON_POINT,
            })
            self.started = threading.Event()
            self.terminal = threading.Event()

        def navigate(self, pose, deadline, cancel):
            self.started.set()
            assert self.terminal.wait(2.0)
            return False, True, "navigation_canceled"

    transport = BlockingTransport()
    adapter = PersonNavApproachAdapter(transport, lambda _: None)
    results = []
    worker = threading.Thread(target=lambda: results.append(run(adapter, context())))
    worker.start()
    assert transport.started.wait(1.0)
    adapter.cancel_task()
    assert worker.is_alive()
    assert not results
    transport.terminal.set()
    worker.join(2.0)
    assert not worker.is_alive()
    assert results[0].canceled
    assert transport.cancels == 1


def test_malformed_success_fails_without_navigation():
    transport = Transport({
        "ok": True, "target_id": TARGET_ID,
        "navigation_required": True, "navigation_goal": POSE,
        "status": 8,
    })
    result = run(PersonNavApproachAdapter(transport, lambda _: None), context())
    assert not result.success
    assert result.reason == "invalid_localization_success"
    assert not transport.navigations
    transport.result = {
        "ok": True, "target_id": TARGET_ID,
        "navigation_required": True, "navigation_goal": POSE,
        "status": 0, "person_point": {"point": {}},
    }
    assert run(PersonNavApproachAdapter(transport, lambda _: None), context()).reason == (
        "invalid_localization_success"
    )
    assert not transport.navigations


def test_unknown_inner_terminal_is_failure_even_after_cancel_requested():
    interrupt = InterruptManager()

    class UnknownTerminal:
        def execute_task(self, unit_config, ctx, timeout):
            interrupt.request_cancel()
            return TargetApproachResult(
                False, "nav2_terminal_unknown:operator_recovery_required",
                {"recovery_required": True}, canceled=True,
            )

    outcome = TaskExecutor(
        adapter=UnknownTerminal(), interrupt_manager=interrupt,
    ).execute(
        {"unit_id": "ACT_INTERACT_APPROACH_VOICE_CALLER",
         "timeout_sec": 160.0, "interrupt_policy": "immediate"},
        ExecutionContext(requested_behavior_name="approach_voice_caller"),
    )
    assert outcome.state == UnitState.FAILURE
    assert "recovery_required" in outcome.message

    ctx = ExecutionContext(requested_behavior_name="approach_voice_caller")
    ctx.cancel_requested = True
    ctx.metadata["target_approach"] = {
        "recovery_required": True, "reason": outcome.message,
    }
    behavior = ResultEvaluator().evaluate(
        ctx, {"target_approach": False}, "all_required_stages_completed",
    )
    assert behavior.status == "controller_error"
    assert behavior.reason == outcome.message


class _Future:
    def __init__(self, value=None, *, done=False):
        self.value = value
        self.completed = done
        self.callbacks = []

    def done(self):
        return self.completed

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        self.callbacks.append(callback)
        if self.completed:
            callback(self)

    def finish(self, value):
        self.value = value
        self.completed = True
        for callback in self.callbacks:
            callback(self)


def _ros_transport(monkeypatch, send_future):
    rclpy = ModuleType("rclpy")
    rclpy.ok = lambda: True
    action_msgs = ModuleType("action_msgs")
    action_msgs.msg = ModuleType("action_msgs.msg")
    action_msgs.msg.GoalStatus = SimpleNamespace(
        STATUS_SUCCEEDED=4, STATUS_CANCELED=5,
    )
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "action_msgs", action_msgs)
    monkeypatch.setitem(sys.modules, "action_msgs.msg", action_msgs.msg)

    class Nav:
        def wait_for_server(self, timeout_sec):
            return True

        def send_goal_async(self, goal):
            return send_future

    class Goal:
        def __init__(self):
            self.pose = SimpleNamespace(
                header=SimpleNamespace(frame_id="", stamp=None),
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            )

    transport = object.__new__(Ros2PersonApproachTransport)
    transport._nav_type = SimpleNamespace(Goal=Goal)
    transport._nav = Nav()
    transport._node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: object()),
        ),
        get_logger=lambda: SimpleNamespace(error=lambda *args: None),
    )
    transport._server_timeout = 0.05
    transport._lock = threading.Lock()
    transport._goal = None
    transport._cancel_sent = False
    transport.recovery_required = False
    return transport


def test_nav2_cancel_ack_does_not_release_before_result(monkeypatch):
    result_future = _Future()

    class Handle:
        accepted = True

        def __init__(self):
            self.cancels = 0

        def get_result_async(self):
            return result_future

        def cancel_goal_async(self):
            self.cancels += 1
            return _Future(done=True)

    handle = Handle()
    transport = _ros_transport(monkeypatch, _Future(handle, done=True))
    cancel = threading.Event()
    results = []
    worker = threading.Thread(target=lambda: results.append(
        transport.navigate(POSE, time.monotonic() + 1.0, cancel)
    ))
    worker.start()
    deadline = time.monotonic() + 1.0
    while transport._goal is None and time.monotonic() < deadline:
        time.sleep(0.005)
    assert transport._goal is handle
    cancel.set()
    while handle.cancels == 0 and time.monotonic() < deadline:
        time.sleep(0.005)
    assert handle.cancels == 1
    assert worker.is_alive()
    assert not results
    result_future.finish(SimpleNamespace(status=5))
    worker.join(1.0)
    assert results == [(False, True, "navigation_canceled")]
    assert not transport.recovery_required


def test_nav2_acceptance_timeout_latches_recovery_and_cancels_late_goal(monkeypatch):
    send_future = _Future()
    transport = _ros_transport(monkeypatch, send_future)
    outcome = transport.navigate(POSE, time.monotonic() + 1.0, threading.Event())
    assert outcome[0] is False
    assert "recovery_required" in outcome[2]
    assert transport.recovery_required

    class LateHandle:
        accepted = True

        def __init__(self):
            self.cancels = 0

        def cancel_goal_async(self):
            self.cancels += 1

        def get_result_async(self):
            return _Future()

    handle = LateHandle()
    send_future.finish(handle)
    assert handle.cancels == 1
