"""Tests for the isolated Unitree Go2 SportMode backend."""

from __future__ import annotations

import json
from types import SimpleNamespace
from pathlib import Path

import pytest

from marsdog_action_executor.adapters.velocity import TwistCommand
from marsdog_action_executor.adapters.go2_sport_backend import (
    Go2ChassisBackend,
    Go2SportRequest,
)
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.interrupt_manager import InterruptManager
from marsdog_action_executor.stage_executor import StageExecutor


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


class FakeTime:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration


def _loader() -> ConfigLoader:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    return loader


def _backend(
    requests: list[Go2SportRequest],
    fake_time: FakeTime,
    *,
    should_stop=None,
) -> Go2ChassisBackend:
    loader = _loader()
    config = loader.go2_sport_config
    limits = config["limits"]
    return Go2ChassisBackend(
        request_publisher=lambda request: requests.append(request) or True,
        action_sequences=loader.get_go2_action_sequences(),
        action_plans=loader.get_go2_action_plans(),
        unsupported_actions=config["unsupported_actions"],
        spin_transport=SimpleNamespace(spin=lambda *args: (True, False, "spin_succeeded"),
                                       cancel=lambda: None, last_terminal_status=4),
        publish_rate_hz=config["publish_rate_hz"],
        max_linear_x=limits["max_linear_x"],
        min_linear_x=limits["min_linear_x"],
        max_linear_y=limits["max_linear_y"],
        max_angular_z=limits["max_angular_z"],
        stop_publish_count=config["stop_publish_count"],
        stand_up_wait_sec=config["stand_up_wait_sec"],
        should_stop=should_stop,
        monotonic=fake_time.monotonic,
        sleep=fake_time.sleep,
    )


def test_effective_go2_routes_exclude_removed_agv_backend() -> None:
    loader = _loader()
    go2_routes = loader.get_controller_routes("go2")

    assert go2_routes["ACT_BASIC_SIT"] == "go2"
    assert go2_routes["ACT_INTERACT_APPROACH_OWNER"] == (
        "person_nav_approach"
    )
    assert go2_routes["ACT_INTERACT_APPROACH_OWNER_CLOSER"] == (
        "person_nav_approach"
    )
    assert go2_routes["ACT_INTERACT_RETURN_OWNER"] == (
        "person_nav_approach"
    )
    for unit_id in (
        "ACT_INTERACT_APPROACH_OWNER",
        "ACT_INTERACT_APPROACH_OWNER_CLOSER",
        "ACT_INTERACT_RETURN_OWNER",
    ):
        unit = loader.get_action_config(unit_id)
        assert unit["unit_type"] == "task"
        assert unit["timeout_sec"] == 160.0
        assert unit["requires_controller"] is True
    assert go2_routes["ACT_INTERACT_FOLLOW_OWNER"] == "uwb_follow"
    assert go2_routes["ACT_NAV_WALK_RANDOM"] == "behavior_mobility"
    assert go2_routes["ACT_NAV_GO_OUT_TO_PLAY"] == "behavior_mobility"
    assert go2_routes["ACT_NAV_GO_HOME"] == "behavior_mobility"

    # Motions with no exact high-level equivalent use explicit Go2 mappings.
    assert go2_routes["ACT_TRICK_ROLL_OVER"] == "go2"
    assert go2_routes["ACT_OBJECT_DROP"] == "go2"
    assert loader.get_controller_routes() == go2_routes
    assert "agv" not in go2_routes.values()
    with pytest.raises(ValueError, match="Unsupported chassis_type"):
        loader.get_controller_routes("agv")


def test_basic_postures_use_official_sport_api_ids_and_stop() -> None:
    expected = {
        "ACT_BASIC_SIT": 1009,
        "ACT_BASIC_LIE_DOWN": 1005,
        "ACT_BASIC_STAND": 1004,
        "ACT_INTERACT_GIVE_PAW": 1016,
        "ACT_INTERACT_HIGH_FIVE": 1016,
        "ACT_TRICK_PLAY_DEAD": 1005,
    }
    for action_id, api_id in expected.items():
        requests: list[Go2SportRequest] = []
        backend = _backend(requests, FakeTime())

        assert backend.execute_step(
            {"unit_id": action_id},
            ExecutionContext.from_goal("test", {}),
            11.0,
        )
        if api_id in (1009, 1016):
            assert requests[:3] == [Go2SportRequest(1003), Go2SportRequest(1004), Go2SportRequest(1002)]
            assert requests[3] == Go2SportRequest(api_id)
        else:
            assert requests[0] == Go2SportRequest(api_id)
        if action_id in {"ACT_BASIC_STAND", "ACT_INTERACT_GIVE_PAW", "ACT_INTERACT_HIGH_FIVE"}:
            assert requests[-4:] == [Go2SportRequest(1003)] * 3 + [Go2SportRequest(1002)]
        else:
            assert requests[-3:] == [Go2SportRequest(1003)] * 3


def test_command_finishing_exactly_at_its_action_budget_succeeds() -> None:
    requests: list[Go2SportRequest] = []
    backend = _backend(requests, FakeTime())

    assert backend.execute_step(
        {"unit_id": "ACT_OWNER_GOING_OUT"},
        ExecutionContext.from_goal("farewell_leave", {}),
        2.0,
    )
    assert requests[:4] == [Go2SportRequest(1003), Go2SportRequest(1004), Go2SportRequest(1002), Go2SportRequest(1016)]
    assert requests[-4:] == [Go2SportRequest(1003)] * 3 + [Go2SportRequest(1002)]


def test_back_up_refreshes_move_with_negative_x_then_stops() -> None:
    requests: list[Go2SportRequest] = []
    backend = _backend(requests, FakeTime())

    assert backend.execute_step(
        {"unit_id": "ACT_BASIC_BACK_UP"},
        ExecutionContext.from_goal("back_up", {}),
        4.0,
    )

    moves = [request for request in requests if request.api_id == 1008]
    assert len(moves) >= 10
    assert all(json.loads(request.parameter) == {
        "x": -0.25,
        "y": 0.0,
        "z": 0.0,
    } for request in moves)
    assert requests[-4:] == [Go2SportRequest(1003)] * 3 + [Go2SportRequest(1002)]


def test_sub_threshold_nonzero_linear_x_is_floored_to_min_linear_x() -> None:
    requests: list[Go2SportRequest] = []
    backend = _backend(requests, FakeTime())
    assert backend.prepare_navigation()
    requests.clear()

    backend.publish_velocity(TwistCommand(linear_x=0.05))
    assert requests == [Go2SportRequest(1008, '{"x":0.25,"y":0.0,"z":0.0}')]

    requests.clear()
    backend.publish_velocity(TwistCommand(linear_x=-0.05))
    assert requests == [Go2SportRequest(1008, '{"x":-0.25,"y":0.0,"z":0.0}')]


def test_velocity_outlet_translates_and_clamps_twist() -> None:
    requests: list[Go2SportRequest] = []
    backend = _backend(requests, FakeTime())
    assert backend.prepare_navigation()
    requests.clear()

    backend.publish_velocity(
        TwistCommand(linear_x=9.0, linear_y=-9.0, angular_z=9.0)
    )
    assert requests == [
        Go2SportRequest(
            1008,
            '{"x":0.3,"y":-0.2,"z":1.2}',
        )
    ]

    backend.publish_velocity(TwistCommand())
    assert requests[-1] == Go2SportRequest(1003, "")


def test_special_controller_action_cannot_bypass_its_route_through_go2() -> None:
    requests: list[Go2SportRequest] = []
    backend = _backend(requests, FakeTime())

    assert not backend.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"},
        ExecutionContext.from_goal("respond_owner_call", {}),
        5.0,
    )
    assert requests == [Go2SportRequest(1003, "")] * 3


def test_every_behavior_action_has_an_executable_go2_route() -> None:
    loader = _loader()
    sequences = loader.get_go2_action_sequences()
    routes = loader.get_controller_routes("go2")
    external_routes = {
        "behavior_mobility",
        "mock",  # quiet is an audio-only policy
        "target_approach",
        "person_nav_approach",
        "uwb_follow",
        "visual_nav_approach",
        "wake_orientation",
    }

    referenced = {
        candidate["unit_id"]
        for behavior in loader.behavior_tree_templates.values()
        for stage in behavior["stages"]
        for candidate in stage.get("candidates", [])
    }
    assert referenced == set(loader.action_catalog)
    # RandomRoam is currently registered only for Lite3. Go2 must not gain
    # a mock success path for the required navigation stage.
    assert routes["ACT_UWB_RANDOM_ROAM"] == "uwb_roam"
    assert "ACT_UWB_RANDOM_ROAM" not in sequences
    referenced.remove("ACT_UWB_RANDOM_ROAM")
    unsupported = loader.go2_sport_config["unsupported_actions"]
    assert len(sequences) == 179 and not unsupported
    assert all(
        (routes[unit_id] == "go2" and unit_id in sequences)
        or (routes[unit_id] == "unsupported" and unit_id in unsupported)
        or routes[unit_id] in external_routes
        for unit_id in referenced
    )
    assert set(sequences).isdisjoint(unsupported)



def test_reported_problem_actions_use_go2_sequences() -> None:
    loader = _loader()
    sequences = loader.get_go2_action_sequences()

    assert sequences["ACT_STRETCH"] == [
        {"command": "stretch", "duration_sec": 5.0}
    ]
    assert sequences["ACT_SPLoot_LIE_DOWN"] == [
        {"command": "stand_down", "duration_sec": 2.5},
        {"command": "stop_move", "duration_sec": 6.0},
    ]
    assert [item["command"] for item in sequences["ACT_YAWN"]] == [
        "euler",
        "euler",
    ]


def test_all_go2_sequences_execute_through_the_backend() -> None:
    loader = _loader()
    sequences = loader.get_go2_action_sequences()
    requests: list[Go2SportRequest] = []
    fake_time = FakeTime()
    backend = _backend(requests, fake_time)

    for unit_id in sorted(sequences):
        requests.clear()
        timeout = float(
            loader.get_go2_action_plans()[unit_id].get("timeout_sec", loader.action_catalog[unit_id].get("timeout_sec", 0.0)) or 10.0
        )

        assert backend.execute_step(
            {"unit_id": unit_id},
            ExecutionContext.from_goal("go2_contract_test", {}),
            timeout,
        ), unit_id
        assert requests, unit_id
        if loader.get_go2_action_plans()[unit_id]["commanded_posture"] == "standing":
            assert requests[-4:] == [Go2SportRequest(1003)] * 3 + [Go2SportRequest(1002)], unit_id
        else:
            assert requests[-3:] == [Go2SportRequest(1003)] * 3, unit_id


def test_praise_and_scold_inplace_candidates_all_use_go2_actions() -> None:
    loader = _loader()
    requests: list[Go2SportRequest] = []
    backend = _backend(requests, FakeTime())
    executor = StageExecutor(
        interrupt_manager=InterruptManager(),
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes("go2"),
        controller_adapters={"go2": backend},
    )

    behavior_names = {
        "expressJoyInPlaceWithHuman",
        "expressExcitementInPlaceWithHuman",
        "expressAnxietyInPlaceWithHuman",
        "expressFearInPlaceWithHuman",
        "expressCuriosityInPlaceWithHuman",
        "expressJoyAlone",
        "expressExcitementAlone",
        "expressAnxietyAlone",
        "expressCuriosityAlone",
        "expressFearAlone",
    }
    sequences = loader.get_go2_action_sequences()
    for behavior_name in behavior_names:
        ctx = ExecutionContext.from_goal(
            behavior_name,
            {"mobility_policy": "in_place", "interaction_id": "voice-1"},
        )
        ctx.resolved_behavior_name = behavior_name
        stage = loader.get_behavior_template(behavior_name)["stages"][0]
        result = executor.execute_stage(
            stage,
            ctx,
            seed=42,
        )
        available = {candidate["unit_id"] for candidate in stage["candidates"]} & set(sequences)
        if not available:
            assert not result.success and "go2_unsupported_action:" in result.message
            continue
        assert result.success, (behavior_name, result.message)
        assert result.unit_id in {
            candidate["unit_id"] for candidate in stage["candidates"]
        }
        assert result.unit_id in sequences


def test_owner_behavior_policies_require_confirmed_owner() -> None:
    loader = _loader()
    policies = loader.visual_target_approach_config[
        "go2_owner_approach_policies"
    ]

    assert set(policies) == {
        "come_to_owner",
        "approach_owner",
        "return_to_owner",
    }
    assert all(policy["target_type"] == "human" for policy in policies.values())
    assert all(
        policy["required_identity"] == "owner"
        for policy in policies.values()
    )


@pytest.mark.parametrize("rest_action", ["ACT_SPLOOT", "ACT_SLEEP_BY_FEET", "ACT_BASIC_SIT"])
@pytest.mark.parametrize("next_action", ["ACT_GUARD_DOOR", "ACT_STRETCH", "ACT_INTERACT_GIVE_PAW"])
def test_rest_then_expression_stands_before_motion(rest_action, next_action) -> None:
    requests = []
    clock = FakeTime()
    backend = _backend(requests, clock)
    ctx = ExecutionContext.from_goal("test", {})
    assert backend.execute_step({"unit_id": rest_action}, ctx, 11.0)
    assert requests[-3:] == [Go2SportRequest(1003)] * 3
    requests.clear()
    start = clock.now
    assert backend.execute_step({"unit_id": next_action}, ctx, 5.0)
    assert requests[:3] == [Go2SportRequest(1003), Go2SportRequest(1004), Go2SportRequest(1002)]
    assert sum(request.api_id == 1004 for request in requests) == 1
    duration = sum(item["duration_sec"] for item in _loader().get_go2_action_sequences()[next_action])
    assert clock.now - start == pytest.approx(2.5 + duration)


def test_velocity_stream_stands_once_then_reprepares_after_rest() -> None:
    requests = []
    clock = FakeTime()
    backend = _backend(requests, clock)
    for _ in range(3):
        backend.publish_velocity(TwistCommand(linear_x=0.3))
    assert clock.now == 0.0  # No blocking wait inside the visual publisher.
    assert [r.api_id for r in requests] == [1003, 1004]
    clock.sleep(2.5)
    assert [r.api_id for r in requests] == [1003, 1004]  # Old velocities are discarded.
    backend.publish_velocity(TwistCommand(angular_z=0.3))
    backend.publish_velocity(TwistCommand(angular_z=0.3))
    assert [r.api_id for r in requests] == [1003, 1004, 1002, 1008, 1008]
    assert json.loads(requests[-1].parameter)["x"] == 0.0
    assert backend.execute_step({"unit_id": "ACT_SPLOOT"}, None, 11.0)
    requests.clear()
    backend.publish_velocity(TwistCommand(angular_z=0.3))
    assert [r.api_id for r in requests] == [1003, 1004]
    clock.sleep(2.5)
    backend.publish_velocity(TwistCommand(angular_z=0.3))
    assert [r.api_id for r in requests] == [1003, 1004, 1002, 1008]


def test_velocity_cannot_wait_behind_another_action() -> None:
    requests = []
    backend = _backend(requests, FakeTime())
    with backend._execution_lock:
        backend.publish_velocity(TwistCommand(linear_x=0.3))
    assert requests == []


def test_cancel_while_velocity_stands_never_replays_the_velocity() -> None:
    requests = []
    clock = FakeTime()
    backend = _backend(requests, clock)
    backend.publish_velocity(TwistCommand(linear_x=0.3))
    backend.cancel_step()
    clock.sleep(3.0)
    backend.publish_velocity(TwistCommand(linear_x=0.3))
    assert all(r.api_id != 1008 for r in requests)


def test_stop_hold_and_emergency_never_stand_up() -> None:
    requests = []
    backend = _backend(requests, FakeTime())
    assert backend.execute_step({"unit_id": "ACT_SPLOOT"}, None, 11.0)
    requests.clear()
    backend.publish_velocity(TwistCommand())
    assert backend.hold_position(0.2)
    backend.emergency_stop()
    backend.publish_velocity(TwistCommand(linear_x=0.3))
    assert requests and all(r.api_id == 1003 for r in requests)


def test_cancel_during_stand_up_prevents_expression_and_allows_next_goal() -> None:
    requests = []
    clock = FakeTime()
    canceled = False
    backend = None

    def sleep(duration):
        nonlocal canceled
        clock.sleep(duration)
        if not canceled:
            canceled = True
            backend.cancel_step()

    backend = Go2ChassisBackend(
        lambda request: requests.append(request) or True,
        {"wave": [{"command": "hello", "duration_sec": 1.0}]},
        monotonic=clock.monotonic, sleep=sleep,
    )
    assert not backend.execute_step({"unit_id": "wave"}, None, 5.0)
    assert backend.last_error == "go2_stand_up_interrupted"
    assert 1016 not in [r.api_id for r in requests]
    requests.clear()
    assert backend.execute_step({"unit_id": "wave"}, None, 5.0)
    assert [r.api_id for r in requests[:4]] == [1003, 1004, 1002, 1016]


@pytest.mark.parametrize("failed_api", [1003, 1004, 1002])
def test_preparation_publish_failure_prevents_motion(failed_api) -> None:
    requests = []
    clock = FakeTime()
    backend = Go2ChassisBackend(
        lambda request: requests.append(request) or request.api_id != failed_api,
        {"wave": [{"command": "hello", "duration_sec": 1.0}]},
        monotonic=clock.monotonic, sleep=clock.sleep,
    )
    assert not backend.execute_step({"unit_id": "wave"}, None, 5.0)
    expected_error = (
        "go2_balance_stand_publish_failed" if failed_api == 1002
        else "go2_stand_up_publish_failed"
    )
    # The richer backend can report the subsequent best-effort stop failure.
    if failed_api == 1003:
        assert backend.last_error in {expected_error, "go2_publish_failed:stop_move"}
    else:
        assert backend.last_error == expected_error
    backend.publish_velocity(TwistCommand(linear_x=0.3))
    assert all(r.api_id not in (1008, 1016) for r in requests)


def test_stand_up_respects_outer_behavior_deadline() -> None:
    requests = []
    clock = FakeTime()
    backend = _backend(requests, clock)
    ctx = ExecutionContext.from_goal("test", {})
    ctx.runtime_deadline_monotonic = 0.3
    assert not backend.execute_step({"unit_id": "ACT_GUARD_DOOR"}, ctx, 5.0)
    assert backend.last_error == "go2_stand_up_interrupted"
    assert clock.now <= 0.35
    assert all(r.api_id != 1008 for r in requests)


def test_explicit_stand_command_is_not_sent_twice_or_shortened() -> None:
    requests = []
    backend = _backend(requests, FakeTime())
    assert backend.execute_step({"unit_id": "ACT_BASIC_STAND"}, None, 5.0)
    assert sum(r.api_id == 1004 for r in requests) == 1
    requests.clear()
    assert not backend.execute_step({"unit_id": "ACT_BASIC_STAND"}, None, 1.0)
    assert backend.last_error == "go2_stand_up_budget_exhausted"
    assert all(r.api_id == 1003 for r in requests)


def test_lie_stand_then_velocity_restores_walking_without_another_stand_up() -> None:
    requests = []
    walking_allowed = False
    moves_accepted = []

    def firmware(request):
        nonlocal walking_allowed
        requests.append(request)
        if request.api_id in (1004, 1005):
            walking_allowed = False
        elif request.api_id == 1002:
            walking_allowed = True
        elif request.api_id == 1008:
            moves_accepted.append(walking_allowed)
        return True

    clock = FakeTime()
    backend = Go2ChassisBackend(
        firmware,
        {"lie": [{"command": "stand_down", "duration_sec": 2.5}],
         "stand": [{"command": "stand_up", "duration_sec": 2.5}]},
        monotonic=clock.monotonic, sleep=clock.sleep,
    )
    for _ in range(2):
        assert backend.execute_step({"unit_id": "lie"}, None, 5.0)
        assert backend.execute_step({"unit_id": "stand"}, None, 5.0)
        requests.clear()
        backend.publish_velocity(TwistCommand(linear_x=0.3))
        backend.publish_velocity(TwistCommand(linear_x=0.3))
        assert sum(r.api_id == 1004 for r in requests) == 0
        assert sum(r.api_id == 1002 for r in requests) == 1
    assert moves_accepted == [True] * 4


@pytest.mark.parametrize("velocity_outlet", [False, True])
def test_cancel_when_balance_stand_is_published_prevents_motion(velocity_outlet) -> None:
    requests = []
    clock = FakeTime()

    def publish(request):
        requests.append(request)
        if request.api_id == 1002:
            backend.cancel_step()
        return True

    backend = Go2ChassisBackend(
        publish, {}, monotonic=clock.monotonic, sleep=clock.sleep,
    )
    if velocity_outlet:
        backend.publish_velocity(TwistCommand(linear_x=0.3))
        clock.sleep(2.5)
        backend.publish_velocity(TwistCommand(linear_x=0.3))
    else:
        assert not backend.prepare_navigation()
    assert 1002 in [r.api_id for r in requests]
    assert all(r.api_id != 1008 for r in requests)
    assert requests[-3:] == [Go2SportRequest(1003)] * 3


@pytest.mark.parametrize("nav_result", [True, False])
@pytest.mark.parametrize("fixed", [True, False])
def test_navigation_prepares_standing_and_preserves_terminal_result(nav_result, fixed) -> None:
    from marsdog_action_executor.adapters.navigation_adapter import BehaviorMobilityAdapter

    requests = []
    clock = FakeTime()
    backend = _backend(requests, clock)
    assert backend.execute_step({"unit_id": "ACT_SPLOOT"}, None, 11.0)
    requests.clear()
    started = clock.now
    calls = []

    def navigate(*args):
        calls.append(args)
        assert [r.api_id for r in requests] == [1003, 1004, 1002]
        assert clock.now - started == pytest.approx(2.5)
        return nav_result

    config = _loader().navigation_config
    adapter = BehaviorMobilityAdapter(
        navigate_waypoint=navigate,
        navigate_fixed_place=navigate if fixed else None,
        motion_adapter=backend, fixed_places=config["waypoint_nav"]["places"],
        waypoints=config["waypoints"], behavior_routes=config["behavior_routes"],
        stage_actions=config["stage_actions"],
    )
    assert adapter.navigate_for_behavior("sleepNow", task_id="test-nav", timeout_sec=20.0) is nav_result
    assert len(calls) == 1
    assert requests[-4:] == [Go2SportRequest(1003)] * 3 + [Go2SportRequest(1002)]


def test_finish_does_not_override_navigation_or_trigger_stand_up() -> None:
    backend = Go2ChassisBackend(lambda request: False, {})
    assert backend.finish_navigation() is True


@pytest.mark.parametrize("spin_result", [True, False])
def test_wake_spin_stands_first_and_keeps_spin_result(spin_result) -> None:
    from marsdog_action_executor.adapters.wake_orientation_adapter import WakeOrientationAdapter

    requests = []
    clock = FakeTime()
    backend = _backend(requests, clock)
    assert backend.execute_step({"unit_id": "ACT_SPLOOT"}, None, 11.0)
    requests.clear()

    def spin(*args):
        assert [r.api_id for r in requests] == [1003, 1004, 1002]
        return spin_result

    adapter = WakeOrientationAdapter(
        spin, prepare_motion=backend.prepare_navigation,
        finish_motion=backend.finish_navigation,
        linear_array_back_search_enabled=False,
    )
    ctx = ExecutionContext.from_goal("respond_owner_call", {
        "use_wake_angle": True, "wake_angle_deg": 120.0,
        "wake_confidence": 1205.0, "wake_frame_id": "microphone_array",
    })
    assert adapter.execute_step({"unit_id": "ACT_INTERACT_RESPOND_CALL"}, ctx) is spin_result
    assert requests[-4:] == [Go2SportRequest(1003)] * 3 + [Go2SportRequest(1002)]


@pytest.mark.parametrize("wait", [0.0, -1.0, 11.0, float("nan"), float("inf")])
def test_invalid_stand_up_wait_is_rejected(wait) -> None:
    with pytest.raises(ValueError, match="stand_up_wait_sec"):
        Go2ChassisBackend(lambda request: True, {}, stand_up_wait_sec=wait)
    loader = _loader()
    loader.go2_sport_config["stand_up_wait_sec"] = wait
    loader._validate_go2_sport_config()
    assert any("stand_up_wait_sec" in error for error in loader._errors)
