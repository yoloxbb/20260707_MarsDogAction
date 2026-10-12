"""Native semantics, counted turns and honest unsupported-action results."""
import json
import math
import sys
import threading
import time
from copy import copy, deepcopy
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS

import pytest

from marsdog_action_executor.adapters.go2_sport_backend import Go2ChassisBackend
from marsdog_action_executor.adapters.nav2_goal_transport import Ros2Nav2SpinTransport
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.stage_executor import StageExecutor


@pytest.fixture(scope="module")
def loader():
    result = ConfigLoader(Path(__file__).resolve().parents[1] / "config")
    result.load_all()
    return result


class Clock:
    now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def backend(loader, publisher=None, spin=None):
    requests, clock = [], Clock()
    obj = Go2ChassisBackend(
        publisher or (lambda req: requests.append(req) or True),
        loader.get_go2_action_sequences(), action_plans=loader.get_go2_action_plans(),
        unsupported_actions=loader.go2_sport_config["unsupported_actions"],
        spin_transport=spin, monotonic=clock.monotonic, sleep=clock.sleep,
    )
    return obj, requests, clock


@pytest.mark.parametrize("success", [True, False])
def test_sleep_three_turns_uses_spin_result_before_lie_down(loader, success):
    calls = []
    spin = NS(spin=lambda yaw, deadline, cancel: calls.append((yaw, deadline)) or
              (success, False, "spin_succeeded" if success else "spin_status_6"),
              last_terminal_status=4 if success else 6)
    obj, requests, clock = backend(loader, spin=spin)
    executor = StageExecutor(action_catalog=loader.action_catalog,
                             controller_routes=loader.get_controller_routes("go2"),
                             controller_adapters={"go2": obj})
    ctx = ExecutionContext.from_goal("sleepNow", {})
    stages = loader.get_behavior_template("sleepNow")["stages"]
    result = executor.execute_stage(stages[0], ctx)
    assert result.success is success
    assert calls == [(pytest.approx(6 * math.pi), pytest.approx(62.5))]
    assert not any(req.api_id in {1005, 1008} for req in requests)
    assert ctx.metadata["go2_action"]["nav2_terminal_status"] == (4 if success else 6)
    if success:
        assert executor.execute_stage(stages[2], ctx, seed=1).success
        assert any(req.api_id == 1005 for req in requests)
    else:
        assert result.message == "spin_status_6"
        assert ctx.completed_stages == []
    assert loader.action_catalog["ACT_CIRCLE_AROUND"]["timeout_sec"] == 20  # shared Lite3 catalog


def test_spin_never_falls_back_to_timed_move(loader):
    obj, requests, clock = backend(loader)
    assert not obj.execute_step({"unit_id": "ACT_CIRCLE_AROUND"}, None, 65)
    assert obj.last_error == "go2_spin_transport_unavailable"
    assert all(req.api_id != 1008 for req in requests)


def test_fixed_object_action_executes_a_labeled_safe_proxy(loader):
    obj, requests, clock = backend(loader)
    executor = StageExecutor(action_catalog=loader.action_catalog,
                             controller_routes=loader.get_controller_routes("go2"),
                             controller_adapters={"go2": obj})
    ctx = ExecutionContext.from_goal("drop_object", {})
    stage = {"stage_id": "fixed", "selection_policy": "fixed", "required": True,
             "candidates": [{"unit_id": "ACT_OBJECT_DROP"}]}
    assert executor.execute_stage(stage, ctx).success
    assert ctx.executed_units == ["ACT_OBJECT_DROP"]
    assert ctx.metadata["go2_action"]["fidelity"] == "proxy"
    assert "不实际抓取" in ctx.metadata["go2_action"]["description"]
    assert ctx.metadata["go2_action"]["physical_completion_confirmed"] is False
    assert {req.api_id for req in requests} == {1002, 1003, 1004, 1007}
    assert requests[-1].api_id == 1002


@pytest.mark.parametrize("behavior", ["expressCalmAlone", "sleepNow", "unhappy", "give_paw"])
def test_completed_chain_restores_balance_after_prone_expression(loader, behavior):
    obj, requests, clock = backend(loader)
    ctx = ExecutionContext.from_goal(behavior, {})
    assert obj.execute_step({"unit_id": "ACT_SPLOOT"}, ctx, 11)
    requests.clear()
    started = clock.now
    assert obj.finish_behavior(ctx, behavior)
    assert [req.api_id for req in requests] == [1003] * 4 + [1004, 1002]
    assert clock.now - started == pytest.approx(2.5)
    assert ctx.metadata["go2_readiness"] == {
        "prepared": True, "physical_posture_confirmed": False, "reason": ""}
    requests.clear()
    assert obj.prepare_navigation()
    assert [req.api_id for req in requests] == [1002]  # StandUp preparation is reused.


@pytest.mark.parametrize("behavior", ["expressCalmAlone", "expressCalmWithHuman", "expressCalmInPlaceWithHuman"])
@pytest.mark.parametrize("seed", range(16))
def test_go2_calm_executes_distinct_safe_random_combination(loader, behavior, seed):
    obj, requests, clock = backend(loader)
    template = loader.get_behavior_template(behavior, chassis_type="go2")
    shared = loader.get_behavior_template(behavior)
    assert shared["stages"][-1]["selection_policy"] == "random_one"
    assert loader.get_behavior_template(behavior, chassis_type="lite3") == shared
    assert template["stages"][:-1] == shared["stages"][:-1]  # Keep human approach.
    stage = template["stages"][-1]
    assert stage["selection_policy"] == "random_n"
    ctx = ExecutionContext.from_goal(behavior, {})
    ctx.speed_scale, ctx.duration_scale = .3, .5  # Cannot truncate native waits.
    executor = StageExecutor(action_catalog=loader.action_catalog,
                             controller_routes=loader.get_controller_routes("go2"),
                             controller_adapters={"go2": obj})
    obj.begin_behavior()
    assert executor.execute_stage(stage, ctx, seed=seed).success
    assert 2 <= len(ctx.executed_units) <= 3
    assert len(set(ctx.executed_units)) == len(ctx.executed_units)
    assert set(ctx.executed_units) <= {item["unit_id"] for item in stage["candidates"]}
    assert [item["requested_act"] for item in ctx.metadata["go2_actions"]] == ctx.executed_units
    assert {r.api_id for r in requests} <= {1002, 1003, 1004, 1005, 1007, 1009, 1017}
    for request in requests:
        if request.api_id == 1007:
            vector = json.loads(request.parameter)
            assert abs(vector["x"]) <= .10
            assert abs(vector["y"]) <= .15
            assert abs(vector["z"]) <= .20
    assert obj.finish_behavior(ctx, behavior)
    assert requests[-1].api_id == 1002
    assert obj._standing_prepared and obj._balance_prepared
    obj.end_behavior()


@pytest.mark.parametrize("behavior", ["expressCalmAlone", "expressCalmWithHuman", "expressCalmInPlaceWithHuman"])
def test_go2_calm_pool_contains_only_gentle_stationary_commands(loader, behavior):
    stage = loader.get_behavior_template(behavior, chassis_type="go2")["stages"][-1]
    for candidate in stage["candidates"]:
        sequence = loader.get_go2_action_sequences()[candidate["unit_id"]]
        assert {item["command"] for item in sequence} <= {
            "euler", "stretch", "sit", "stand_down", "stand_up", "stop_move"}


@pytest.mark.parametrize("next_action", ["ACT_NUZZLE_BODY", "ACT_BASIC_SIT", "ACT_STRETCH"])
def test_calm_rest_keeps_full_hold_before_random_next_action(loader, next_action):
    obj, requests, clock = backend(loader)
    ctx = ExecutionContext.from_goal("expressCalmAlone", {})
    assert obj.execute_step({"unit_id": "ACT_SPLOOT"}, ctx, 11)
    assert clock.now == pytest.approx(8.5)  # 2.5s transition + 6s prone rest.
    assert all(r.api_id in {1005, 1003} for r in requests)
    events = []
    obj._publish_request = lambda req: events.append((clock.now, req.api_id)) or True
    assert obj.execute_step({"unit_id": next_action}, ctx, 10)
    up_at = next(at for at, api in events if api == 1004)
    motion_at = next(at for at, api in events if api in {1007, 1009, 1017})
    assert motion_at - up_at >= 2.5


def test_unit_tail_cannot_insert_stand_up_when_only_stop_was_requested():
    requests = []
    obj = Go2ChassisBackend(lambda req: requests.append(req) or True,
                           {"pause": [{"command": "stop_move", "duration_sec": 0}]},
                           action_plans={"pause": {"commanded_posture": "standing"}})
    assert obj.execute_step({"unit_id": "pause"})
    assert requests and all(req.api_id == 1003 for req in requests)


@pytest.mark.parametrize("invalid", [
    {"missing_behavior": {"expression": {"selection_policy": "sequence", "candidates": [{"unit_id": "ACT_SPLOOT"}]}}},
    {"expressCalmAlone": {"missing_stage": {"selection_policy": "sequence", "candidates": [{"unit_id": "ACT_SPLOOT"}]}}},
    {"expressCalmAlone": {"expression": {"selection_policy": "sequence", "candidates": [{"unit_id": "ACT_UNKNOWN"}]}}},
])
def test_invalid_go2_expression_override_is_rejected_at_startup(loader, invalid):
    other = copy(loader)
    other.go2_sport_config = deepcopy(loader.go2_sport_config)
    other.go2_sport_config["behavior_stage_overrides"] = invalid
    other._errors = []
    other._validate_go2_sport_config()
    assert any("Go2 stage override" in error for error in other._errors)


def test_cancel_during_calm_rest_cannot_run_final_stand(loader):
    obj, requests, clock = backend(loader)
    ctx = ExecutionContext.from_goal("expressCalmAlone", {})

    def sleep(seconds):
        clock.now += seconds
        if requests and any(r.api_id == 1005 for r in requests):
            ctx.cancel_requested = True

    obj._sleep = sleep
    obj.begin_behavior()
    assert not obj.execute_step({"unit_id": "ACT_SPLOOT"}, ctx, 11)
    assert not obj.finish_behavior(ctx, "expressCalmAlone")
    assert all(r.api_id in {1005, 1003} for r in requests)
    assert not obj._standing_prepared and not obj._balance_prepared


def test_cancel_random_calm_rest_cannot_execute_remaining_sit(loader):
    obj, requests, clock = backend(loader)
    ctx = ExecutionContext.from_goal("expressCalmAlone", {})

    def sleep(seconds):
        clock.now += seconds
        if any(r.api_id == 1005 for r in requests):
            ctx.cancel_requested = True

    obj._sleep = sleep
    executor = StageExecutor(action_catalog=loader.action_catalog,
                             controller_routes=loader.get_controller_routes("go2"),
                             controller_adapters={"go2": obj})
    stage = loader.get_behavior_template("expressCalmAlone", chassis_type="go2")["stages"][-1]
    # Seed 6 selects [ACT_SPLOOT, ACT_BASIC_SIT]. Cancel the first unit.
    result = executor.execute_stage(stage, ctx, seed=6)
    assert not result.success
    assert ctx.metadata["go2_actions"][0]["requested_act"] == "ACT_SPLOOT"
    assert len(ctx.metadata["go2_actions"]) == 1
    assert ctx.completed_stages == []
    assert not obj.finish_behavior(ctx, "expressCalmAlone")
    assert {r.api_id for r in requests} <= {1005, 1003}


@pytest.mark.parametrize("selection_count", [None, 2, {}, {"min": 0, "max": 2},
    {"min": 3, "max": 2}, {"min": 2, "max": 99}, {"min": True, "max": 3},
    {"min": 2.0, "max": 3}, {"min": 2, "max": "3"}])
def test_invalid_random_combination_count_is_rejected(loader, selection_count):
    other = copy(loader)
    other.go2_sport_config = deepcopy(loader.go2_sport_config)
    stage = other.go2_sport_config["behavior_stage_overrides"]["expressCalmAlone"]["expression"]
    stage["selection_count"] = selection_count
    other._errors = []
    other._validate_go2_sport_config()
    assert any("selection_count" in error for error in other._errors)


def test_duplicate_random_combination_candidates_are_rejected(loader):
    other = copy(loader)
    other.go2_sport_config = deepcopy(loader.go2_sport_config)
    stage = other.go2_sport_config["behavior_stage_overrides"]["expressCalmAlone"]["expression"]
    stage["candidates"].append(dict(stage["candidates"][0]))
    other._errors = []
    other._validate_go2_sport_config()
    assert any("unique unit_id" in error for error in other._errors)


def test_native_stretch_budget_includes_command_overhead(loader):
    obj, requests, clock = backend(loader)
    # Advance the clock on reads as well, to model publication/scheduling
    # overhead that a frozen fake clock otherwise hides.
    def monotonic():
        clock.now += .0001
        return clock.now
    obj._monotonic = monotonic
    executor = StageExecutor(action_catalog=loader.action_catalog,
                             controller_routes=loader.get_controller_routes("go2"),
                             controller_adapters={"go2": obj})
    ctx = ExecutionContext.from_goal("test", {})
    ctx.speed_scale = .3
    assert executor.execute_stage({"stage_id": "stretch", "selection_policy": "fixed",
                                   "candidates": [{"unit_id": "ACT_STRETCH"}]}, ctx).success
    assert any(r.api_id == 1017 for r in requests)


@pytest.mark.parametrize("behavior", ["lie_down", "sit_down", "play_dead", "hold_position", "wait_in_place"])
def test_explicit_posture_behavior_does_not_automatically_stand(loader, behavior):
    obj, requests, clock = backend(loader)
    ctx = ExecutionContext.from_goal(behavior, {})
    assert obj.execute_step({"unit_id": "ACT_BASIC_LIE_DOWN"}, ctx, 5)
    requests.clear()
    assert obj.finish_behavior(ctx, behavior)
    assert requests == []
    assert not obj._standing_prepared and not obj._balance_prepared


def test_canceled_chain_cannot_stand_during_finalization(loader):
    obj, requests, clock = backend(loader)
    obj.cancel_step()
    requests.clear()
    assert not obj.finish_behavior(ExecutionContext.from_goal("unhappy", {}), "unhappy")
    assert all(req.api_id == 1003 for req in requests)
    assert not obj._standing_prepared and not obj._balance_prepared


def test_balance_tail_failure_fails_body_action_and_invalidates_both_flags(loader):
    requests = []

    def publish(req):
        requests.append(req)
        return not (req.api_id == 1002 and sum(r.api_id == 1002 for r in requests) > 1)

    obj, _, clock = backend(loader, publisher=publish)
    assert not obj.execute_step({"unit_id": "ACT_STRETCH"}, None, 10)
    assert obj.last_error == "go2_balance_stand_publish_failed"
    assert not obj._standing_prepared and not obj._balance_prepared
    assert not any(req.api_id == 1008 for req in requests)


def test_unmapped_id_still_fails_without_unconfigured_substitution(loader):
    obj, requests, clock = backend(loader)
    assert not obj.execute_step({"unit_id": "ACT_UNKNOWN"}, None, 10)
    assert obj.last_error.startswith("go2_unsupported_action:ACT_UNKNOWN:")
    assert all(req.api_id == 1003 for req in requests)


def test_getup_sit_records_commanded_sitting_instead_of_catalog_standing(loader):
    obj, requests, clock = backend(loader)
    executor = StageExecutor(action_catalog=loader.action_catalog,
                             controller_routes=loader.get_controller_routes("go2"),
                             controller_adapters={"go2": obj})
    ctx = ExecutionContext.from_goal("test", {})
    assert executor.execute_stage({"stage_id": "wake", "selection_policy": "fixed",
                                   "candidates": [{"unit_id": "ACT_GETUP_SIT"}]}, ctx).success
    assert [r.api_id for r in requests if r.api_id in {1004, 1006, 1009}] == [1004, 1009]
    assert ctx.metadata["go2_action"]["commanded_posture"] == "sitting"
    assert executor._posture.current == "sitting"
    assert ctx.metadata["go2_action"]["physical_completion_confirmed"] is False


def test_native_stretch_cannot_report_success_after_truncated_wait(loader):
    obj, requests, clock = backend(loader)
    assert not obj.execute_step({"unit_id": "ACT_STRETCH"}, None, 1.0)
    assert obj.last_error == "go2_action_budget_exhausted"
    assert all(req.api_id != 1017 for req in requests)


@pytest.mark.parametrize("action,api_id", [
    ("ACT_STRETCH", 1017), ("ACT_STRETCH_BODY", 1017),
    ("ACT_COMFY_STRETCH", 1017), ("ACT_BONE_DANCE", 1022),
])
def test_expression_calls_its_native_api_instead_of_hello(loader, action, api_id):
    obj, requests, clock = backend(loader)
    assert obj.execute_step({"unit_id": action}, None, 10)
    assert sum(req.api_id == api_id for req in requests) == 1
    assert not any(req.api_id in {1016, 1006, 1029, 1032} for req in requests)


def test_unknown_spin_terminal_blocks_new_posture_or_motion_but_allows_stop(loader):
    spin = NS(recovery_required=True)
    obj, requests, clock = backend(loader, spin=spin)
    for action in ("ACT_BASIC_STAND", "ACT_BASIC_LIE_DOWN", "ACT_STRETCH"):
        assert not obj.execute_step({"unit_id": action}, None, 10)
        assert "recovery_required" in obj.last_error
    assert not obj.prepare_navigation()
    assert obj.execute_step({"unit_id": "ACT_BASIC_WAIT"}, None, 5)
    assert requests and all(req.api_id == 1003 for req in requests)


def test_euler_reselects_balance_mode_after_walking_without_standing_again(loader):
    obj, requests, clock = backend(loader)
    assert obj.execute_step({"unit_id": "ACT_BASIC_BACK_UP"}, None, 5)
    requests.clear()
    assert obj.execute_step({"unit_id": "ACT_TWIST_WAIST_LEFT_RIGHT"}, None, 5)
    assert requests[0].api_id == 1002
    assert requests[1].api_id == 1007
    assert all(req.api_id not in {1004, 1008} for req in requests)


@pytest.mark.parametrize("level", [-1, 0, 1])
def test_speed_level_uses_official_data_field(level):
    requests = []
    clock = Clock()
    obj = Go2ChassisBackend(lambda req: requests.append(req) or True,
                            {"speed": [{"command": "speed_level", "data": level, "duration_sec": 0}]},
                            monotonic=clock.monotonic, sleep=clock.sleep)
    assert obj.execute_step({"unit_id": "speed"})
    assert requests[0].api_id == 1015
    assert json.loads(requests[0].parameter) == {"data": level}


@pytest.mark.parametrize("spin_result", [None, (True, False, "spin_succeeded")])
def test_native_stop_failure_propagates_but_does_not_override_spin_result(loader, spin_result):
    spinning = False
    def publish(req):
        return not spinning or req.api_id != 1003
    def spin(*args):
        nonlocal spinning
        spinning = True
        return spin_result
    obj, requests, clock = backend(loader, publisher=publish,
                                   spin=NS(spin=spin, last_terminal_status=4))
    if spin_result is not None:
        assert obj.execute_step({"unit_id": "ACT_TRICK_SPIN"}, None, 25)
        assert obj.last_error == ""
    else:
        spinning = True
        assert not obj.execute_step({"unit_id": "ACT_BASIC_WAIT"}, None, 5)
        assert "publish_failed" in obj.last_error


class Future:
    def __init__(self, value=None, done=False):
        self.value, self.completed, self.callbacks = value, done, []
    def done(self): return self.completed
    def result(self): return self.value
    def add_done_callback(self, callback):
        self.callbacks.append(callback)
        if self.completed: callback(self)
    def finish(self, value):
        self.value, self.completed = value, True
        for callback in self.callbacks: callback(self)


def transport(monkeypatch, sent):
    ros = ModuleType("rclpy")
    ros.ok = lambda: True
    status = ModuleType("action_msgs.msg")
    status.GoalStatus = NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5)
    monkeypatch.setitem(sys.modules, "rclpy", ros)
    monkeypatch.setitem(sys.modules, "action_msgs.msg", status)
    goals = []
    obj = object.__new__(Ros2Nav2SpinTransport)
    obj._nav_type = NS(Goal=lambda: NS(target_yaw=0., time_allowance=NS(sec=0, nanosec=0)))
    obj._nav = NS(wait_for_server=lambda **kw: True,
                  send_goal_async=lambda goal: goals.append(goal) or sent)
    obj._node = NS(get_logger=lambda: NS(info=lambda *a: None, error=lambda *a: None))
    obj._server_timeout, obj._lock = .03, threading.Lock()
    obj._goal = obj._pending_send = obj._result_future = None
    obj.recovery_required = obj._cancel_sent = False
    obj.last_terminal_status = None
    return obj, goals


@pytest.mark.parametrize("code,expected", [(4, (True, False)), (5, (False, True)), (6, (False, False))])
def test_spin_transmits_unwrapped_six_pi_and_uses_real_terminal(monkeypatch, code, expected):
    handle = NS(accepted=True, get_result_async=lambda: Future(NS(status=code), True))
    obj, goals = transport(monkeypatch, Future(handle, True))
    result = obj.spin(6 * math.pi, time.monotonic() + 60, threading.Event())
    assert result[:2] == expected
    assert goals[0].target_yaw == pytest.approx(6 * math.pi)
    assert goals[0].time_allowance.sec >= 59
    assert obj.last_terminal_status == code and obj._goal is None


def test_spin_cancel_ack_is_not_terminal_and_late_success_is_preserved(monkeypatch):
    future, canceled = Future(), threading.Event()
    handle = NS(accepted=True, get_result_async=lambda: future,
                cancel_goal_async=lambda: canceled.set())
    obj, goals = transport(monkeypatch, Future(handle, True))
    obj._server_timeout = 1.0
    cancel, results = threading.Event(), []
    worker = threading.Thread(target=lambda: results.append(obj.spin(
        6 * math.pi, time.monotonic() + 1, cancel)))
    worker.start()
    limit = time.monotonic() + 1
    while obj._goal is None and time.monotonic() < limit: time.sleep(.001)
    cancel.set()
    assert canceled.wait(.5)
    assert worker.is_alive() and not results
    future.finish(NS(status=4))
    worker.join(1)
    assert results == [(True, False, "spin_succeeded")]


def test_spin_unknown_acceptance_cancels_late_goal_and_blocks_new_goal(monkeypatch):
    sent, canceled = Future(), []
    obj, goals = transport(monkeypatch, sent)
    result = obj.spin(6 * math.pi, time.monotonic() + .01, threading.Event())
    assert "acceptance_unknown" in result[2] and obj.recovery_required
    late_result = Future()
    handle = NS(accepted=True, get_result_async=lambda: late_result,
                cancel_goal_async=lambda: canceled.append(True))
    sent.finish(handle)
    assert canceled == [True] and obj._goal is handle
    assert "recovery_required" in obj.spin(6 * math.pi, time.monotonic() + 1, threading.Event())[2]
    assert len(goals) == 1
    late_result.finish(NS(status=5))
    assert obj._goal is None and obj.recovery_required
