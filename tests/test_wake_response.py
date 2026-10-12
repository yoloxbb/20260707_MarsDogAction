"""Wake responses select one complete posture expression without navigation."""

from copy import deepcopy
from pathlib import Path
import random

import pytest

from marsdog_action_executor.adapters.go2_sport_backend import Go2ChassisBackend
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.stage_executor import StageExecutor
from marsdog_action_executor.stationary_policy import (
    WAKE_EXPRESSION_BEHAVIOR, WAKE_EXPRESSION_UNITS, validate_inplace_plan,
)


@pytest.fixture(scope="module")
def loader():
    config = ConfigLoader(Path(__file__).resolve().parents[1] / "config")
    config.load_all()
    return config


def check(loader, stages=None, params=None, routes=None, navigation=None):
    return validate_inplace_plan(
        WAKE_EXPRESSION_BEHAVIOR,
        {"mobility_policy": "in_place"} if params is None else params,
        loader.get_behavior_template(WAKE_EXPRESSION_BEHAVIOR)["stages"]
        if stages is None else stages,
        loader.get_controller_routes("go2") if routes is None else routes,
        loader.navigation_config if navigation is None else navigation,
    )


def test_response_requires_inplace_policy_and_no_visual_target(loader):
    assert check(loader).valid
    assert not check(loader, params={}).valid
    stage = loader.get_behavior_template(WAKE_EXPRESSION_BEHAVIOR)["stages"][0]
    assert {c["unit_id"] for c in stage["candidates"]} == WAKE_EXPRESSION_UNITS
    assert all(not c.get("conditions") for c in stage["candidates"])


@pytest.mark.parametrize("unit,route", [
    ("ACT_APPROACH_VISUAL_TARGET", "person_nav_approach"),
    ("ACT_FETCH_TOY", "go2"),
    ("ACT_BASIC_BACK_UP", "go2"),
    ("ACT_RAISE_HEAD", "behavior_mobility"),
    ("ACT_RAISE_HEAD", "mock"),
])
def test_response_rejects_moving_units_and_non_chassis_routes(loader, unit, route):
    stages = [{"stage_id": "expression", "selection_policy": "random_one",
               "candidates": [{"unit_id": unit}]}]
    assert not check(loader, stages=stages, routes={unit: route}).valid


@pytest.mark.parametrize("navigation", [
    {"random_navigation_behaviors": [WAKE_EXPRESSION_BEHAVIOR]},
    {"behavior_routes": {WAKE_EXPRESSION_BEHAVIOR: "A"}},
])
def test_response_cannot_gain_a_pre_expression_navigation_route(loader, navigation):
    assert not check(loader, navigation=navigation).valid


class Clock:
    now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


@pytest.mark.parametrize("unit", sorted(WAKE_EXPRESSION_UNITS))
def test_each_random_choice_executes_real_go2_posture_plan(loader, unit):
    requests, clock = [], Clock()
    backend = Go2ChassisBackend(
        request_publisher=lambda request: requests.append(request) or True,
        action_sequences=loader.get_go2_action_sequences(),
        action_plans=loader.get_go2_action_plans(),
        monotonic=clock.monotonic, sleep=clock.sleep,
    )
    stage = loader.get_behavior_template(WAKE_EXPRESSION_BEHAVIOR)["stages"][0]
    # Every configured single action and group must be reachable in the real pool.
    seed = next(n for n in range(1000)
                if random.Random(n).choice(stage["candidates"])["unit_id"] == unit)
    ctx = ExecutionContext.from_goal(WAKE_EXPRESSION_BEHAVIOR, {
        "mobility_policy": "in_place", "interaction_id": "session-1", "wake_id": "wake-1",
    })
    ctx.resolved_behavior_name = WAKE_EXPRESSION_BEHAVIOR
    ctx.current_posture = "standing"
    executor = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes("go2"),
        controller_adapters={"go2": backend},
    )
    result = executor.execute_stage(stage, ctx, seed=seed)
    assert result.success, result.message
    assert ctx.executed_units == [unit]
    assert ctx.completed_stages == ["expression"]
    assert requests
    # StopMove, StandUp, BalanceStand, Euler, Stretch; no Move/Spin/navigation.
    assert {request.api_id for request in requests} <= {1003, 1004, 1002, 1007, 1017}
    assert ctx.metadata["go2_action"]["commanded_posture"] == "standing"


def test_random_choice_varies_and_missing_hardware_cannot_succeed(loader):
    stage = loader.get_behavior_template(WAKE_EXPRESSION_BEHAVIOR)["stages"][0]
    choices = {random.Random(seed).choice(stage["candidates"])["unit_id"]
               for seed in range(100)}
    assert choices == WAKE_EXPRESSION_UNITS
    ctx = ExecutionContext.from_goal(WAKE_EXPRESSION_BEHAVIOR, {})
    ctx.resolved_behavior_name = WAKE_EXPRESSION_BEHAVIOR
    result = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes("go2"),
    ).execute_stage(deepcopy(stage), ctx, seed=1)
    assert not result.success
    assert not ctx.executed_units
