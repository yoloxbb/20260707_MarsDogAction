from concurrent.futures import Future
from types import SimpleNamespace as NS
import threading
import time

import pytest

from marsdog_action_executor.adapters.uwb_roam_adapter import UwbRoamAdapter
from marsdog_action_executor.execution_context import ExecutionContext


def done(value):
    f = Future()
    f.set_result(value)
    return f


class Backend:
    last_error = ""

    def __init__(self):
        self.calls = []
        self.settle = True

    def prepare_navigation(self, **kwargs):
        self.calls.append(("prepare", kwargs))
        return True

    def finish_navigation(self):
        self.calls.append(("finish", {}))
        return self.settle


class Handle:
    accepted = True

    def __init__(self, result):
        self.result = result
        self.cancels = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancels += 1
        return done(NS(goals_canceling=[1]))


class Client:
    def __init__(self, handle):
        self.send = done(handle)
        self.goal = None

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        self.feedback_callback = feedback_callback
        self.goal = goal
        return self.send


def wrapped(status=4, code=0):
    return NS(status=status, result=NS(code=code, message="test", owner_distance=1., min_clearance=.5))


def _make_roam_setup(result):
    backend = Backend()
    handle = Handle(result)
    client = Client(handle)
    adapter = UwbRoamAdapter(backend=backend, client=client, goal_factory=NS)
    return adapter, backend, handle, client


def test_success_and_exact_wire_contract():
    adapter, backend, _, client = _make_roam_setup(done(wrapped()))
    ctx = ExecutionContext("play_alone")
    assert adapter.execute_step({}, ctx, 40)
    assert vars(client.goal) == dict(random_seed=0, timeout_sec=30., min_radius=.5, max_radius=2.)
    assert backend.calls == [("prepare", {"allow_uwb_chain": True}), ("finish", {})]
    assert ctx.metadata["uwb_roam_result"]["code"] == 0
    assert not adapter.active


def test_stop_latch_is_released_before_sending_roam_goal():
    adapter, backend, _, client = _make_roam_setup(done(wrapped()))
    events = []
    adapter._arm_chain = lambda: events.append("idle") or True
    original_send = client.send_goal_async

    def send_after_idle(goal, feedback_callback=None):
        events.append("send")
        return original_send(goal, feedback_callback=feedback_callback)

    client.send_goal_async = send_after_idle

    assert adapter.execute_step({}, duration=40)
    assert events == ["idle", "send"]
    assert backend.calls == [("prepare", {"allow_uwb_chain": True}), ("finish", {})]


def test_unconfirmed_idle_blocks_roam_goal_and_releases_chassis():
    adapter, backend, _, client = _make_roam_setup(done(wrapped()))
    adapter._arm_chain = lambda: False

    assert not adapter.execute_step({}, duration=40)
    assert adapter.last_error == "uwb_roam_idle_unconfirmed"
    assert client.goal is None
    assert backend.calls == [("prepare", {"allow_uwb_chain": True}), ("finish", {})]


@pytest.mark.parametrize("status,code", [(4, x) for x in range(1, 10)] + [(5, 0), (6, 0)])
def test_non_success_blocks_play(status, code):
    adapter, _, _, _ = _make_roam_setup(done(wrapped(status, code)))
    assert not adapter.execute_step({}, duration=40)
    assert "uwb_roam_failed" in adapter.last_error


@pytest.mark.parametrize("late_acceptance", [False, True])
def test_cancel_ack_does_not_release_motion(late_acceptance):
    result = Future()
    adapter, backend, handle, client = _make_roam_setup(result)
    if late_acceptance:
        client.send = Future()
    outcome = []
    worker = threading.Thread(target=lambda: outcome.append(adapter.execute_step({}, duration=40)))
    worker.start()
    deadline = time.monotonic() + 2
    while client.goal is None and time.monotonic() < deadline:
        time.sleep(.005)
    adapter.cancel_step()
    if late_acceptance:
        client.send.set_result(handle)
    deadline = time.monotonic() + 2
    while handle.cancels == 0 and time.monotonic() < deadline:
        time.sleep(.005)
    try:
        assert handle.cancels == 1
        assert worker.is_alive()
        assert adapter.active
        assert len(backend.calls) == 1
    finally:
        result.set_result(wrapped(5, 1))
        worker.join(2)
    assert outcome == [False]
    assert not adapter.active
    assert backend.calls[-1][0] == "finish"


def test_settle_failure_blocks_next_stage():
    adapter, backend, _, _ = _make_roam_setup(done(wrapped()))
    backend.settle = False
    with pytest.raises(RuntimeError, match="settle_failed"):
        adapter.execute_step({}, duration=40)


def test_rejected_goal_and_short_budget():
    adapter, _, handle, client = _make_roam_setup(done(wrapped()))
    assert not adapter.execute_step({}, duration=4)
    assert client.goal is None
    handle.accepted = False
    assert not adapter.execute_step({}, duration=40)
    assert adapter.last_error == "uwb_roam_rejected"


def test_missing_stop_service_refuses_roam_before_motion():
    adapter, backend, _, client = _make_roam_setup(done(wrapped()))
    adapter._brake_ready = lambda: False

    assert not adapter.execute_step({}, duration=40)
    assert adapter.last_error == "uwb_roam_brake_unavailable"
    assert backend.calls == []
    assert client.goal is None


def test_transport_error_retains_ownership_and_requires_recovery():
    result = Future()
    result.set_exception(RuntimeError("transport lost"))
    adapter, backend, _, _ = _make_roam_setup(result)
    with pytest.raises(RuntimeError, match="transport lost"):
        adapter.execute_step({}, duration=40)
    assert adapter.active and adapter.recovery_required
    assert len(backend.calls) == 1
    assert not adapter.execute_step({}, duration=40)


@pytest.mark.parametrize("status,code,expected_plays", [(4, 0, 1), (4, 9, 0), (6, 0, 0)])
def test_real_stage_mapping_gates_random_play(status, code, expected_plays):
    from pathlib import Path
    from marsdog_action_executor.config_loader import ConfigLoader
    from marsdog_action_executor.stage_executor import StageExecutor
    loader = ConfigLoader(Path(__file__).resolve().parents[1] / "config")
    loader.load_all()
    adapter, _, _, _ = _make_roam_setup(done(wrapped(status, code)))
    plays = []

    class Pose:
        def execute_step(self, config, ctx, duration):
            plays.append(config)
            return True

    routes = dict(loader.controller_routes["routes"])
    candidates = {"ACT_GUARD_DOOR", "ACT_SNIFF_BOWL_AND_WAIT_FOR_FOOD", "ACT_STRETCH"}
    for unit_id in candidates:
        routes[unit_id] = "pose"
    executor = StageExecutor(action_catalog=loader.action_catalog, controller_routes=routes,
                             controller_adapters={"uwb_roam": adapter, "pose": Pose()})
    ctx = ExecutionContext("play_alone", resolved_behavior_name="play_alone")
    for stage in loader.behavior_tree_templates["play_alone"]["stages"]:
        if not executor.execute_stage(stage, ctx, seed=3).success:
            break
    assert len(plays) == expected_plays
    if plays:
        assert plays[0]["unit_id"] in candidates
        assert plays[0]["interrupt_policy"] == "immediate"


def test_shutdown_cancels_without_waiting_for_stopped_ros_executor():
    adapter, backend, handle, client = _make_roam_setup(Future())
    errors = []
    brakes = []
    adapter._brake_chain = lambda: brakes.append("STOP") or True

    def run():
        try:
            adapter.execute_step({}, duration=40)
        except RuntimeError as error:
            errors.append(str(error))

    worker = threading.Thread(target=run)
    worker.start()
    deadline = time.monotonic() + 2
    while adapter._handle is None and time.monotonic() < deadline:
        time.sleep(.005)
    adapter.close()
    worker.join(2)
    assert not worker.is_alive()
    assert handle.cancels >= 1
    assert errors == ["uwb_roam_shutdown_with_pending_result"]
    assert adapter.recovery_required
    assert brakes == ["STOP"]
    assert len(backend.calls) == 1
