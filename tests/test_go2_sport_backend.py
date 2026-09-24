"""Tests for the isolated Unitree Go2 SportMode backend."""

from __future__ import annotations

import json
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
        publish_rate_hz=config["publish_rate_hz"],
        max_linear_x=limits["max_linear_x"],
        min_linear_x=limits["min_linear_x"],
        max_linear_y=limits["max_linear_y"],
        max_angular_z=limits["max_angular_z"],
        stop_publish_count=config["stop_publish_count"],
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
            8.0,
        )
        assert requests[0] == Go2SportRequest(api_id, "")
        assert requests[-3:] == [Go2SportRequest(1003, "")] * 3


def test_command_finishing_exactly_at_its_action_budget_succeeds() -> None:
    requests: list[Go2SportRequest] = []
    backend = _backend(requests, FakeTime())

    assert backend.execute_step(
        {"unit_id": "ACT_OWNER_GOING_OUT"},
        ExecutionContext.from_goal("farewell_leave", {}),
        2.0,
    )
    assert requests[0] == Go2SportRequest(1016, "")
    assert requests[-3:] == [Go2SportRequest(1003, "")] * 3


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
    assert requests[-3:] == [Go2SportRequest(1003, "")] * 3


def test_sub_threshold_nonzero_linear_x_is_floored_to_min_linear_x() -> None:
    requests: list[Go2SportRequest] = []
    backend = _backend(requests, FakeTime())

    backend.publish_velocity(TwistCommand(linear_x=0.05))
    assert requests == [Go2SportRequest(1008, '{"x":0.25,"y":0.0,"z":0.0}')]

    requests.clear()
    backend.publish_velocity(TwistCommand(linear_x=-0.05))
    assert requests == [Go2SportRequest(1008, '{"x":-0.25,"y":0.0,"z":0.0}')]


def test_velocity_outlet_translates_and_clamps_twist() -> None:
    requests: list[Go2SportRequest] = []
    backend = _backend(requests, FakeTime())

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
        "visual_target_approach",
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
    # 171 = 193 + ACT_TWIST_WAIST_LEFT_RIGHT (the eating ritual's new 扭腰)
    # - the 23 eating-only units the ritual rewrite orphaned.
    assert len(sequences) == 171
    assert all(
        (routes[unit_id] == "go2" and unit_id in sequences)
        or routes[unit_id] in external_routes
        for unit_id in referenced
    )
    assert not {
        unit_id
        for unit_id in referenced
        if routes[unit_id]
        in {"agv", "unsupported", "perception_navigation_manipulation"}
    }


def test_reported_problem_actions_use_go2_sequences() -> None:
    loader = _loader()
    sequences = loader.get_go2_action_sequences()

    assert sequences["ACT_STRETCH"] == [
        {"command": "hello", "duration_sec": 2.5}
    ]
    assert sequences["ACT_SPLoot_LIE_DOWN"] == [
        {"command": "stand_down", "duration_sec": 2.5}
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
            loader.action_catalog[unit_id].get("timeout_sec", 0.0) or 10.0
        )

        assert backend.execute_step(
            {"unit_id": unit_id},
            ExecutionContext.from_goal("go2_contract_test", {}),
            timeout,
        ), unit_id
        assert requests, unit_id
        assert requests[-3:] == [Go2SportRequest(1003, "")] * 3, unit_id


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
        assert result.success
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
