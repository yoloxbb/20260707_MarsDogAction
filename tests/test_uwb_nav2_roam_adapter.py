from concurrent.futures import Future
from types import SimpleNamespace as NS
import threading
import time

import pytest

from marsdog_action_executor.adapters.uwb_nav2_roam_adapter import (
    Ros2UwbNavigationGoalProvider, UwbNav2RoamAdapter,
)
from marsdog_action_executor.execution_context import ExecutionContext
POSE = {
    "header": {"frame_id": "map"},
    "pose": {"position": {"x": 1., "y": 2., "z": 0.},
             "orientation": {"x": 0., "y": 0., "z": 0., "w": 1.}},
}


TARGET = {"request_id": "point-1", "navigation_goal": POSE, "center_pose": POSE}


class Backend:
    last_error = ""

    def __init__(self):
        self.calls = []

    def prepare_navigation(self, **kwargs):
        self.calls.append("prepare")
        return True

    def finish_navigation(self):
        self.calls.append("finish")
        return True


class Provider:
    def generate(self, deadline, cancel):
        return TARGET, "ok"


class Navigator:
    recovery_required = False
    last_terminal_status = 4

    def __init__(self):
        self.poses, self.cancels = [], 0

    def navigate(self, pose, deadline, cancel, **kwargs):
        self.poses.append(pose)
        self.feedback = kwargs["feedback_callback"]
        return True, False, "navigation_succeeded"

    def cancel(self):
        self.cancels += 1


def setup(provider=None, navigator=None):
    backend, nav = Backend(), navigator or Navigator()
    adapter = UwbNav2RoamAdapter(backend=backend, provider=provider or Provider(), navigator=nav)
    return adapter, backend, nav


def test_generated_target_requires_one_successful_nav2_result_and_settling():
    adapter, backend, nav = setup()
    ctx = ExecutionContext("play_alone")
    assert adapter.execute_step({}, ctx, 40.)
    assert nav.poses == [POSE]
    assert backend.calls == ["prepare", "finish"]
    assert ctx.metadata["uwb_roam_target"]["request_id"] == "point-1"
    assert ctx.metadata["uwb_roam_result"]["execution_mode"] == "nav2"
    assert not adapter.active


def test_target_generation_failure_never_sends_nav2_or_unlocks_play():
    provider = NS(generate=lambda *args: (None, "uwb_target_failed:code=4"))
    adapter, backend, nav = setup(provider)
    ctx = ExecutionContext("play_alone")
    assert not adapter.execute_step({}, ctx, 40.)
    assert nav.poses == []
    assert "uwb_roam_result" not in ctx.metadata
    assert backend.calls == ["prepare", "finish"]


def test_cancel_ack_keeps_motion_until_real_navigation_result():
    arrived = threading.Event()
    result = Future()
    nav = Navigator()

    def navigate(pose, deadline, cancel, **kwargs):
        arrived.set()
        result.result(timeout=2.)
        nav.last_terminal_status = 5
        return False, True, "navigation_canceled"

    nav.navigate = navigate
    adapter, backend, _ = setup(navigator=nav)
    outcome = []
    worker = threading.Thread(target=lambda: outcome.append(adapter.execute_step({}, duration=40.)))
    worker.start()
    assert arrived.wait(1.)
    adapter.cancel_step()
    try:
        assert nav.cancels == 1
        assert adapter.active
        assert backend.calls == ["prepare"]
        assert outcome == []
    finally:
        result.set_result(True)
        worker.join(1.)
    assert outcome == [False]
    assert backend.calls == ["prepare", "finish"]


def test_unknown_navigation_terminal_retains_ownership_and_blocks_replacement():
    nav = Navigator()

    def navigate(*args, **kwargs):
        nav.recovery_required = True
        return False, False, "nav2_terminal_unknown:operator_recovery_required"

    nav.navigate = navigate
    adapter, backend, _ = setup(navigator=nav)
    assert not adapter.execute_step({}, duration=40.)
    assert adapter.active and adapter.recovery_required
    assert backend.calls == ["prepare"]
    assert not adapter.execute_step({}, duration=40.)
    assert backend.calls == ["prepare"]


def test_geofence_failure_cancels_nav2_and_blocks_play():
    nav = Navigator()

    def navigate(pose, deadline, cancel, **kwargs):
        current = NS(header=NS(frame_id="map"), pose=NS(position=NS(x=20., y=0.)))
        kwargs["feedback_callback"](NS(feedback=NS(current_pose=current)))
        assert cancel.is_set()
        nav.last_terminal_status = 5
        return False, True, "navigation_canceled"

    nav.navigate = navigate
    adapter, backend, _ = setup(navigator=nav)
    assert not adapter.execute_step({}, duration=40.)
    assert adapter.last_error == "uwb_nav2_geofence_stop"
    assert backend.calls == ["prepare", "finish"]


@pytest.mark.parametrize("raises", [False, True])
def test_settle_failure_latches_recovery_and_releases_python_lock(raises):
    adapter, backend, _ = setup()

    def finish():
        if raises:
            raise OSError("chassis transport unavailable")
        return False

    backend.finish_navigation = finish
    with pytest.raises(RuntimeError):
        adapter.execute_step({}, duration=40.)
    assert adapter.recovery_required and adapter.active
    assert not adapter.execute_step({}, duration=40.)
    assert "operator_recovery_required" in adapter.last_error


def pose_message(*, frame="map", stamp=100, x=1., y=0.):
    return NS(header=NS(frame_id=frame, stamp=NS(sec=stamp, nanosec=0)),
              pose=NS(position=NS(x=x, y=y, z=0.), orientation=NS(x=0., y=0., z=0., w=1.)))


def provider_with_response(mutator=lambda response: None):
    provider = object.__new__(Ros2UwbNavigationGoalProvider)
    provider._type = NS(Request=NS)
    provider._node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000)))

    def respond(request):
        response = NS(request_id=request.request_id, success=True, code=0, message="generated",
                      navigation_goal=pose_message(), center_pose=pose_message(x=0.))
        mutator(response)
        future = Future()
        future.set_result(response)
        return future

    provider._client = NS(wait_for_service=lambda **kwargs: True, call_async=respond)
    return provider


def test_provider_checks_correlation_and_map_pose():
    target, reason = provider_with_response().generate(time.monotonic() + 1., threading.Event())
    assert reason == "ok" and target["navigation_goal"]["header"]["frame_id"] == "map"


@pytest.mark.parametrize("change,reason", [
    (lambda r: setattr(r, "request_id", "old-request"), "uwb_target_response_mismatch"),
    (lambda r: setattr(r.navigation_goal.header, "frame_id", "base_footprint"), "uwb_target_invalid_map_pose"),
    (lambda r: setattr(r.navigation_goal.header.stamp, "sec", 95), "uwb_target_stale"),
    (lambda r: setattr(r.navigation_goal.pose.position, "x", 10.), "uwb_target_outside_requested_annulus"),
])
def test_provider_rejects_unusable_target(change, reason):
    target, error = provider_with_response(change).generate(time.monotonic() + 1., threading.Event())
    assert target is None and error == reason
