"""Tests for the go2_uwb_behavior follow client adapter."""

from __future__ import annotations

import threading
import time
import pytest

from marsdog_action_executor.adapters.uwb_follow_action_adapter import (
    MAX_FOLLOW_TIMEOUT_SEC,
    MIN_FOLLOW_TIMEOUT_SEC,
    SET_BEHAVIOR_IDLE,
    SET_BEHAVIOR_STOP,
    UwbFollowActionAdapter,
)


class FakeResult:
    def __init__(self, code: int, message: str = "") -> None:
        self.code = code
        self.message = message


class FakeWrapped:
    def __init__(self, code: int, message: str = "") -> None:
        self.result = FakeResult(code, message)


class FakeGoalHandle:
    def __init__(self) -> None:
        self.cancel_count = 0

    def cancel_goal_async(self) -> None:
        self.cancel_count += 1


class FakeFollowClient:
    """Records goals and replays a scripted terminal result."""

    def __init__(
        self,
        *,
        server_ready: bool = True,
        accept: bool = True,
        outcome: tuple[FakeWrapped | None, str] = (FakeWrapped(0), ""),
    ) -> None:
        self.server_ready = server_ready
        self.accept = accept
        self.outcome = outcome
        self.timeouts: list[float] = []
        self.cancel_count = 0
        self.handle = FakeGoalHandle()
        self.taken = False
        self.prepared_kwargs: list[dict] = []

    def wait_for_server(self, timeout_sec: float) -> bool:
        return self.server_ready

    def start_follow(self, timeout_sec, *, deadline):
        self.timeouts.append(float(timeout_sec))
        if not self.accept:
            return None, "uwb_follow_goal_rejected"
        return self.handle, ""

    def wait_for_result(self, deadline):
        wrapped, reason = self.outcome
        if wrapped is None and reason in ("", "done"):
            # A budget that ran out with no scripted terminal result.
            return None, "timeout"
        return wrapped, reason

    def poll_result(self):
        return self.outcome

    def cancel(self) -> None:
        self.cancel_count += 1
        self.handle.cancel_goal_async()

    def take_goal_handle(self):
        # Mirrors the real client, which clears the handle: a second stop on
        # an already-stopped session must have nothing left to cancel.
        if self.taken:
            return None
        self.taken = True
        return self.handle


class FakeBehaviorClient:
    def __init__(self, *, server_ready: bool = True, accept: bool = True) -> None:
        self.server_ready = server_ready
        self.accept = accept
        self.modes: list[int] = []

    def wait_for_server(self, timeout_sec: float) -> bool:
        return self.server_ready

    def set_mode(self, mode: int) -> tuple[bool, str]:
        self.modes.append(int(mode))
        return self.accept, "ok" if self.accept else "rejected"


def _adapter(**kwargs):
    events: list[str] = []
    follow = kwargs.pop("follow_client", None) or FakeFollowClient()
    behavior = kwargs.pop("behavior_client", None) or FakeBehaviorClient()
    prepare_ok = kwargs.pop("prepare_ok", True)

    def prepare_motion(**call_kwargs):
        events.append("prepare")
        follow.prepared_kwargs.append(dict(call_kwargs))
        return prepare_ok

    adapter = UwbFollowActionAdapter(
        enabled=kwargs.pop("enabled", True),
        node=object(),
        prepare_motion=prepare_motion,
        finish_motion=lambda: events.append("finish") or True,
        follow_client_factory=lambda *a, **k: follow,
        behavior_client_factory=lambda *a, **k: behavior,
        **kwargs,
    )
    return adapter, follow, behavior, events


class FakeContext:
    interaction_id = ""


def test_long_follow_cancel_waits_for_real_inner_result() -> None:
    follow = FakeFollowClient(outcome=(None, ""))
    adapter, _, behavior, events = _adapter(follow_client=follow)
    assert adapter.start(timeout_sec=0.0)
    adapter._long_goal_active = True
    stopped = []
    worker = threading.Thread(
        target=lambda: stopped.append(adapter.stop_confirmed("canceled")),
    )
    worker.start()
    deadline = time.monotonic() + 1.0
    while follow.cancel_count == 0 and time.monotonic() < deadline:
        time.sleep(0.005)
    try:
        assert worker.is_alive()
        assert adapter.active
        assert "finish" not in events
        assert behavior.modes[-1] == SET_BEHAVIOR_STOP
    finally:
        follow.outcome = (FakeWrapped(1), "")
        worker.join(1.0)
    assert stopped == [True]
    assert adapter.active is False
    assert "finish" in events


def test_rejected_idle_mode_never_sends_follow_goal() -> None:
    behavior = FakeBehaviorClient(accept=False)
    adapter, follow, _, events = _adapter(behavior_client=behavior)

    assert adapter.start(timeout_sec=0.0) is False
    assert follow.timeouts == []
    assert events == []
    assert "idle_rejected" in adapter.last_error


def test_stop_service_failure_still_waits_for_inner_result() -> None:
    follow = FakeFollowClient(outcome=(None, ""))
    adapter, _, behavior, _ = _adapter(follow_client=follow)
    assert adapter.start(timeout_sec=0.0)
    adapter._long_goal_active = True
    behavior.accept = False
    stopped = []
    worker = threading.Thread(
        target=lambda: stopped.append(adapter.stop_confirmed("canceled")),
    )
    worker.start()
    deadline = time.monotonic() + 1.0
    while follow.cancel_count == 0 and time.monotonic() < deadline:
        time.sleep(0.005)
    # Cancel is sent before STOP. Wait for the STOP response to be processed
    # before asserting the recovery latch from its rejection.
    while not adapter.recovery_required and time.monotonic() < deadline:
        time.sleep(0.005)
    try:
        assert worker.is_alive()
        assert adapter.recovery_required
    finally:
        follow.outcome = (FakeWrapped(1), "")
        worker.join(1.0)
    assert stopped == [False]


def test_stage_budget_is_clamped_into_the_protocol_range() -> None:
    for duration, expected in (
        (1.0, MIN_FOLLOW_TIMEOUT_SEC),
        (5.0, 5.0),
        (12.5, 12.5),
        (99999.0, MAX_FOLLOW_TIMEOUT_SEC),
    ):
        adapter, follow, _, _ = _adapter()
        assert adapter.execute_step({}, FakeContext(), duration) is True
        assert follow.timeouts == [expected], duration


def test_missing_stage_budget_refuses_instead_of_following_forever() -> None:
    for duration in (0.0, None, -3.0, float("nan")):
        adapter, follow, behavior, events = _adapter()
        assert adapter.execute_step({}, FakeContext(), duration) is False
        # No goal, and no motion handoff either: refusing must be inert.
        assert follow.timeouts == []
        assert events == []
        assert "uwb_follow_no_time_budget" in adapter.last_error
        assert behavior.modes == []


@pytest.mark.parametrize(
    "code,name",
    [
        (2, "PREEMPTED_BY_MODE"),
        (3, "NOT_READY"),
        (4, "TIMEOUT"),
        (5, "INPUT_TIMEOUT"),
        (6, "STOP_UNCONFIRMED"),
    ],
)
def test_terminal_failure_codes_name_themselves(code: int, name: str) -> None:
    follow = FakeFollowClient(outcome=(FakeWrapped(code, "boom"), ""))
    adapter, _, _, _ = _adapter(follow_client=follow)

    assert adapter.execute_step({}, FakeContext(), 10.0) is False
    assert name in adapter.last_error
    assert "boom" in adapter.last_error


def test_success_result_completes_the_stage() -> None:
    follow = FakeFollowClient(outcome=(FakeWrapped(0), ""))
    adapter, _, _, events = _adapter(follow_client=follow)

    assert adapter.execute_step({}, FakeContext(), 10.0) is True
    assert "prepare" in events
    assert "finish" in events


def test_follow_preflight_yields_the_chassis_to_the_uwb_chain() -> None:
    adapter, follow, _, _ = _adapter()

    assert adapter.execute_step({}, FakeContext(), 10.0) is True
    # Without this the Lite3 ownership check would reject the very chain that
    # following depends on.
    assert follow.prepared_kwargs == [{"allow_uwb_chain": True}]


def test_failed_preflight_sends_no_goal() -> None:
    follow = FakeFollowClient()
    adapter, _, _, events = _adapter(follow_client=follow, prepare_ok=False)

    with pytest.raises(RuntimeError):
        adapter.execute_step({}, FakeContext(), 10.0)
    assert follow.timeouts == []
    # The chassis never changed hands, so there is nothing to give back.
    assert events == ["prepare"]
    assert adapter.last_error == "uwb_motion_preflight_failed"


def test_unreachable_mode_service_refuses_the_handoff() -> None:
    follow = FakeFollowClient()
    behavior = FakeBehaviorClient(server_ready=False)
    adapter, _, _, events = _adapter(
        follow_client=follow, behavior_client=behavior
    )

    with pytest.raises(RuntimeError):
        adapter.execute_step({}, FakeContext(), 10.0)
    # The mode service is the only brake, so /cmd_vel must not be handed over.
    assert follow.timeouts == []
    assert events == []
    assert "uwb_set_behavior_unavailable" in adapter.last_error


def test_budget_expiry_completes_the_bounded_session() -> None:
    follow = FakeFollowClient(outcome=(None, ""))
    adapter, _, _, _ = _adapter(follow_client=follow)

    # The Stage asked for a bounded session; running it out is success.
    assert adapter.execute_step({}, FakeContext(), 10.0) is True
    assert follow.cancel_count == 1


def test_external_cancel_reports_failure_not_a_finished_session() -> None:
    follow = FakeFollowClient(outcome=(None, "canceled"))
    adapter, _, _, _ = _adapter(follow_client=follow)

    assert adapter.execute_step({}, FakeContext(), 10.0) is False


def test_cancel_step_stops_the_goal_and_never_resends() -> None:
    follow = FakeFollowClient()
    adapter, _, behavior, events = _adapter(follow_client=follow)
    assert adapter.execute_step({}, FakeContext(), 10.0) is True
    follow.timeouts.clear()

    adapter.cancel_step()

    assert follow.handle.cancel_count == 1
    assert follow.taken is True
    assert follow.timeouts == []
    assert behavior.modes[-1] == SET_BEHAVIOR_STOP
    # A canceled session must not restart on the next control tick.
    assert adapter.active is False


def test_emergency_stop_brakes_an_engaged_session() -> None:
    follow = FakeFollowClient()
    behavior = FakeBehaviorClient()
    adapter, _, _, _ = _adapter(
        follow_client=follow, behavior_client=behavior
    )
    adapter.update_control({"enabled": True, "mode": "follow"})
    behavior.modes.clear()

    adapter.emergency_stop()

    assert behavior.modes == [SET_BEHAVIOR_STOP]
    assert follow.handle.cancel_count == 1
    assert adapter.active is False


def test_unengaged_stop_never_blocks_on_a_missing_service() -> None:
    follow = FakeFollowClient()
    behavior = FakeBehaviorClient(server_ready=False)
    adapter, _, _, events = _adapter(
        follow_client=follow, behavior_client=behavior
    )

    # Node shutdown calls emergency_stop() unconditionally.  With no session
    # engaged there is no brake to send, and reaching for one would stall the
    # shutdown on a service that is not there.
    adapter.emergency_stop()
    adapter.stop("shutdown")

    assert behavior.modes == []
    assert events == []
    assert adapter.active is False


def test_stop_brakes_the_chain_before_releasing_the_chassis() -> None:
    events: list[str] = []
    behavior = FakeBehaviorClient()
    original_set_mode = behavior.set_mode

    def recording_set_mode(mode: int):
        events.append("stop" if mode == SET_BEHAVIOR_STOP else "idle")
        return original_set_mode(mode)

    behavior.set_mode = recording_set_mode
    follow = FakeFollowClient()
    adapter = UwbFollowActionAdapter(
        enabled=True,
        node=object(),
        prepare_motion=lambda **_: True,
        finish_motion=lambda: events.append("finish") or True,
        follow_client_factory=lambda *a, **k: follow,
        behavior_client_factory=lambda *a, **k: behavior,
    )
    # A background session keeps the chassis after start() returns, so the
    # stop below is the one that actually hands it back.
    assert adapter.update_control({"enabled": True, "mode": "follow"}) is True
    events.clear()

    adapter.stop("test")

    # The chain publishes /cmd_vel at 20 Hz, so it has to be quiet before the
    # chassis is handed back.
    assert events == ["stop", "finish"]


def test_idle_stop_does_not_run_the_settle_handshake() -> None:
    events: list[str] = []
    adapter = UwbFollowActionAdapter(
        enabled=True,
        node=object(),
        prepare_motion=lambda **_: True,
        finish_motion=lambda: events.append("finish") or True,
        follow_client_factory=lambda *a, **k: FakeFollowClient(),
        behavior_client_factory=lambda *a, **k: FakeBehaviorClient(),
    )

    # Nothing was ever handed over, so there is no mode to release and no
    # reason to wait for a settled chassis.
    adapter.stop("idle")
    adapter.cancel_step()
    adapter.emergency_stop()

    assert events == []


def test_start_clears_a_stop_latch_with_idle() -> None:
    follow = FakeFollowClient()
    behavior = FakeBehaviorClient()
    adapter, _, _, _ = _adapter(
        follow_client=follow, behavior_client=behavior
    )

    assert adapter.execute_step({}, FakeContext(), 10.0) is True

    # IDLE first (release any STOP latch), then STOP on the way out.
    assert behavior.modes[0] == SET_BEHAVIOR_IDLE
    assert behavior.modes[-1] == SET_BEHAVIOR_STOP


def test_roam_arm_releases_stop_latch_without_starting_follow() -> None:
    behavior = FakeBehaviorClient()
    adapter, follow, _, _ = _adapter(behavior_client=behavior)

    assert adapter.arm_chain()
    assert behavior.modes == [SET_BEHAVIOR_IDLE]
    assert follow.timeouts == []


def test_roam_arm_refuses_unconfirmed_idle() -> None:
    behavior = FakeBehaviorClient(accept=False)
    adapter, _, _, _ = _adapter(behavior_client=behavior)

    assert not adapter.arm_chain()
    assert adapter.last_error == "uwb_set_behavior_idle_failed:rejected"


def test_resume_does_not_restart_by_default() -> None:
    follow = FakeFollowClient()
    adapter, _, _, events = _adapter(follow_client=follow)
    assert adapter.update_control({"enabled": True, "mode": "follow"}) is True
    events.clear()
    follow.timeouts.clear()

    adapter.suspend()
    assert events == ["finish"]
    assert follow.handle.cancel_count == 1
    assert adapter.active is False

    # Being preempted ends the follow.  The dog must not start walking again
    # on its own just because the interrupting behavior finished.
    assert adapter.resume() is True
    assert follow.timeouts == []
    assert adapter.active is False

    # ...but the re-arm path stays open, and only a fresh session message
    # (the owner asking again) takes it.
    assert adapter.update_control({"enabled": True, "mode": "follow"}) is True
    assert follow.timeouts == [0.0]
    assert adapter.active is True


def test_resume_restarts_when_auto_resume_is_enabled() -> None:
    follow = FakeFollowClient()
    adapter, _, _, _ = _adapter(follow_client=follow, auto_resume=True)
    assert adapter.update_control({"enabled": True, "mode": "follow"}) is True
    follow.timeouts.clear()

    adapter.suspend()
    assert adapter.resume() is True

    assert follow.timeouts == [0.0]


def test_session_desired_survives_suspend_but_not_cancel() -> None:
    follow = FakeFollowClient()
    adapter, _, _, _ = _adapter(follow_client=follow)
    assert adapter.session_desired is False

    adapter.update_control({"enabled": True, "mode": "follow"})
    assert adapter.session_desired is True

    # This is the whole point of the property: a suspended session still owns
    # the chassis, so the goal gate keeps rejecting lower-priority arrivals
    # even though ``active`` has gone False.
    adapter.suspend()
    assert adapter.session_desired is True
    assert adapter.active is False

    adapter.cancel_step()
    assert adapter.session_desired is False


def test_session_desired_clears_on_emergency_stop_and_stop() -> None:
    follow = FakeFollowClient()
    adapter, _, _, _ = _adapter(follow_client=follow)

    adapter.update_control({"enabled": True, "mode": "follow"})
    adapter.emergency_stop()
    assert adapter.session_desired is False

    adapter.update_control({"enabled": True, "mode": "follow"})
    assert adapter.session_desired is True

    adapter.stop("test")
    assert adapter.session_desired is False


def test_resume_after_cancel_does_not_restart_the_session() -> None:
    follow = FakeFollowClient()
    adapter, _, _, _ = _adapter(follow_client=follow)
    adapter.update_control({"enabled": True, "mode": "follow"})

    adapter.cancel_step()

    assert adapter.resume() is True
    assert follow.timeouts == [0.0]
    assert adapter.active is False


def test_background_session_follows_until_canceled() -> None:
    follow = FakeFollowClient()
    adapter, _, _, _ = _adapter(follow_client=follow)

    assert adapter.update_control({"enabled": True, "mode": "follow"}) is True

    # 0 is the protocol's "follow until canceled", which is what a session
    # with no Stage budget actually means.
    assert follow.timeouts == [0.0]
    assert adapter.active is True


def test_attention_disable_stops_the_session() -> None:
    follow = FakeFollowClient()
    adapter, _, behavior, _ = _adapter(follow_client=follow)
    adapter.update_control({"enabled": True, "mode": "follow"})

    assert adapter.update_control({"enabled": False}) is True

    assert adapter.active is False
    assert behavior.modes[-1] == SET_BEHAVIOR_STOP


def test_poll_health_reaps_a_session_that_failed_on_its_own() -> None:
    follow = FakeFollowClient(outcome=(FakeWrapped(5, "uwb lost"), ""))
    adapter, _, _, events = _adapter(follow_client=follow)
    adapter.update_control({"enabled": True, "mode": "follow"})

    assert adapter.poll_health() is False

    assert "INPUT_TIMEOUT" in adapter.last_error
    assert adapter.active is False
    assert "finish" in events


def test_poll_health_is_quiet_while_the_session_runs() -> None:
    follow = FakeFollowClient(outcome=(None, ""))
    adapter, _, _, _ = _adapter(follow_client=follow)
    adapter.update_control({"enabled": True, "mode": "follow"})

    assert adapter.poll_health() is True
    assert adapter.active is True


def test_missing_message_package_is_reported_not_raised() -> None:
    def exploding_factory(*args, **kwargs):
        raise ModuleNotFoundError("No module named 'go2_uwb_behavior'")

    # Building the node must not die just because the other workspace is not
    # sourced; this package deliberately has no dependency on it.
    adapter = UwbFollowActionAdapter(
        enabled=True,
        node=object(),
        follow_client_factory=exploding_factory,
    )

    assert "uwb_clients_unavailable" in adapter.last_error
    assert "ModuleNotFoundError" in adapter.last_error
    # And the Stage fails with that reason instead of a bare KeyError.
    with pytest.raises(RuntimeError, match="uwb_clients_unavailable"):
        adapter.execute_step({}, FakeContext(), 10.0)


def test_disabled_adapter_refuses_before_touching_the_chassis() -> None:
    follow = FakeFollowClient()
    adapter, _, behavior, events = _adapter(
        follow_client=follow, enabled=False
    )

    with pytest.raises(RuntimeError, match="uwb_follow_disabled"):
        adapter.execute_step({}, FakeContext(), 10.0)
    assert follow.timeouts == []
    assert events == []
    assert behavior.modes == []
