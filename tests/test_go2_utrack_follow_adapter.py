from __future__ import annotations

import json
import sys
from types import ModuleType, SimpleNamespace

from marsdog_action_executor.adapters.velocity import TwistCommand
from marsdog_action_executor.adapters.go2_utrack_follow_adapter import (
    Go2UtrackFollowAdapter,
    Ros2Go2UtrackClient,
    UTRACK_SWITCH_SET_API_ID,
    UtrackSwitchResult,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration


class FakeLogger:
    def __init__(self) -> None:
        self.info_messages: list[str] = []
        self.warning_messages: list[str] = []

    def info(self, message: str) -> None:
        self.info_messages.append(message)

    def warning(self, message: str) -> None:
        self.warning_messages.append(message)


class FakeRequest:
    def __init__(self) -> None:
        self.header = SimpleNamespace(
            identity=SimpleNamespace(id=0, api_id=0),
            policy=SimpleNamespace(noreply=True),
        )
        self.parameter = ""
        self.binary = []


class FakeResponse:
    pass


class FakeUwbState:
    pass


class FakePublisher:
    def __init__(self, node, matched: int) -> None:
        self._node = node
        self._matched = matched
        self.messages: list[FakeRequest] = []

    def get_subscription_count(self) -> int:
        return self._matched

    def publish(self, message: FakeRequest) -> None:
        self.messages.append(message)
        callback = self._node.callbacks["/api/uwbswitch/response"]
        callback(SimpleNamespace(
            header=SimpleNamespace(
                identity=SimpleNamespace(
                    id=message.header.identity.id,
                    api_id=message.header.identity.api_id,
                ),
                status=SimpleNamespace(code=0),
            ),
            data='{"enable":1}',
        ))


class FakeNode:
    def __init__(self, matched: int = 1) -> None:
        self.callbacks = {}
        self.logger = FakeLogger()
        self.publisher = FakePublisher(self, matched)

    def create_publisher(self, message_type, topic, depth):
        assert message_type is FakeRequest
        assert topic == "/api/uwbswitch/request"
        assert depth == 10
        return self.publisher

    def create_subscription(
        self, message_type, topic, callback, depth, *, callback_group
    ):
        del message_type, depth, callback_group
        self.callbacks[topic] = callback
        return object()

    def get_logger(self) -> FakeLogger:
        return self.logger


def _install_fake_unitree_messages(monkeypatch) -> None:
    rclpy_module = ModuleType("rclpy")
    callback_groups_module = ModuleType("rclpy.callback_groups")

    class FakeMutuallyExclusiveCallbackGroup:
        pass

    callback_groups_module.MutuallyExclusiveCallbackGroup = (
        FakeMutuallyExclusiveCallbackGroup
    )
    rclpy_module.callback_groups = callback_groups_module
    api_module = ModuleType("unitree_api")
    api_msg_module = ModuleType("unitree_api.msg")
    api_msg_module.Request = FakeRequest
    api_msg_module.Response = FakeResponse
    api_module.msg = api_msg_module
    go_module = ModuleType("unitree_go")
    go_msg_module = ModuleType("unitree_go.msg")
    go_msg_module.UwbState = FakeUwbState
    go_module.msg = go_msg_module
    monkeypatch.setitem(sys.modules, "rclpy", rclpy_module)
    monkeypatch.setitem(
        sys.modules, "rclpy.callback_groups", callback_groups_module
    )
    monkeypatch.setitem(sys.modules, "unitree_api", api_module)
    monkeypatch.setitem(sys.modules, "unitree_api.msg", api_msg_module)
    monkeypatch.setitem(sys.modules, "unitree_go", go_module)
    monkeypatch.setitem(sys.modules, "unitree_go.msg", go_msg_module)


def test_ros_client_publishes_official_correlated_switch_request(monkeypatch) -> None:
    _install_fake_unitree_messages(monkeypatch)
    node = FakeNode()
    client = Ros2Go2UtrackClient(node)

    result = client.switch_set(True)

    assert result == UtrackSwitchResult(True, code=0)
    assert len(node.publisher.messages) == 1
    request = node.publisher.messages[0]
    assert request.header.identity.id > 0
    assert request.header.identity.api_id == UTRACK_SWITCH_SET_API_ID
    assert request.header.policy.noreply is False
    assert json.loads(request.parameter) == {"enable": 1}


def test_ros_client_fails_closed_without_robot_subscriber(monkeypatch) -> None:
    _install_fake_unitree_messages(monkeypatch)
    client = Ros2Go2UtrackClient(FakeNode(matched=0))

    assert client.switch_set(True) == UtrackSwitchResult(
        False, "uwb_switch_no_subscriber"
    )


def test_session_enable_is_deduplicated_and_matching_disable_switches_off() -> None:
    calls: list[tuple[bool, float | None]] = []
    stops: list[TwistCommand] = []

    def switch(enable: bool, timeout: float | None) -> UtrackSwitchResult:
        calls.append((enable, timeout))
        return UtrackSwitchResult(True, code=0)

    adapter = Go2UtrackFollowAdapter(
        enabled=True,
        switch_set=switch,
        publish_twist=stops.append,
        stop_publish_count=3,
    )

    control = {
        "enabled": True,
        "mode": "follow_owner",
        "interaction_id": "voice-1",
    }
    assert adapter.update_control(control)
    assert adapter.update_control(control)
    assert adapter.active
    assert calls == [(True, 2.0)]

    assert adapter.update_control({
        "enabled": False,
        "mode": "follow_owner",
        "interaction_id": "voice-1",
    })
    assert not adapter.active
    assert calls == [(True, 2.0), (False, 2.0)]
    assert stops == [TwistCommand()] * 3


def test_foreground_behavior_suspends_and_resumes_robot_side_follow() -> None:
    calls: list[bool] = []

    def switch(enable: bool, timeout: float | None) -> UtrackSwitchResult:
        del timeout
        calls.append(enable)
        return UtrackSwitchResult(True, code=0)

    adapter = Go2UtrackFollowAdapter(enabled=True, switch_set=switch)
    adapter.update_control({
        "enabled": True,
        "mode": "follow_owner",
        "interaction_id": "voice-1",
    })

    adapter.suspend()
    assert not adapter.active
    assert adapter.resume()
    assert adapter.active
    assert calls == [True, False, True]


def test_direct_action_is_bounded_and_disables_follow() -> None:
    clock = FakeClock()
    calls: list[bool] = []

    def switch(enable: bool, timeout: float | None) -> UtrackSwitchResult:
        del timeout
        calls.append(enable)
        return UtrackSwitchResult(True, code=0)

    adapter = Go2UtrackFollowAdapter(
        enabled=True,
        switch_set=switch,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    assert adapter.execute_step({}, duration=1.0)
    assert clock.now == 1.0
    assert calls == [True, False]
    assert not adapter.active


def test_enable_failure_is_exposed_to_stage_executor() -> None:
    calls: list[bool] = []

    def switch(enable: bool, timeout: float | None) -> UtrackSwitchResult:
        del timeout
        calls.append(enable)
        if enable:
            return UtrackSwitchResult(False, "uwb_switch_response_timeout")
        return UtrackSwitchResult(True, code=0)

    adapter = Go2UtrackFollowAdapter(
        enabled=True,
        switch_set=switch,
    )

    try:
        adapter.execute_step({})
    except RuntimeError as exc:
        assert str(exc) == "uwb_switch_response_timeout"
    else:  # pragma: no cover - assertion aid
        raise AssertionError("expected the UTrack enable failure to propagate")
    assert calls == [True, False]


def test_failed_disable_is_retried_by_emergency_stop() -> None:
    calls: list[bool] = []
    disable_attempts = 0

    def switch(enable: bool, timeout: float | None) -> UtrackSwitchResult:
        nonlocal disable_attempts
        del timeout
        calls.append(enable)
        if not enable:
            disable_attempts += 1
            if disable_attempts == 1:
                return UtrackSwitchResult(False, "uwb_switch_response_timeout")
        return UtrackSwitchResult(True, code=0)

    adapter = Go2UtrackFollowAdapter(enabled=True, switch_set=switch)
    assert adapter.update_control({
        "enabled": True,
        "mode": "follow_owner",
        "interaction_id": "voice-1",
    })
    assert not adapter.update_control({
        "enabled": False,
        "mode": "follow_owner",
        "interaction_id": "voice-1",
    })

    adapter.emergency_stop()

    assert calls == [True, False, False]
    assert not adapter.active
