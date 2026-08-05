"""Tests for semantic waypoint navigation and existing-stage AGV motion."""

from __future__ import annotations

from pathlib import Path

from marsdog_action_executor.adapters.navigation_adapter import (
    BehaviorMobilityAdapter,
)
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.stage_executor import StageExecutor


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


class RecordingNavigator:
    def __init__(self, result: bool = True) -> None:
        self.result = result
        self.calls: list[tuple[str, dict, float]] = []
        self.cancel_count = 0

    def __call__(
        self,
        waypoint_name: str,
        waypoint: dict,
        timeout_sec: float,
    ) -> bool:
        self.calls.append((waypoint_name, dict(waypoint), timeout_sec))
        return self.result

    def cancel_navigation(self) -> None:
        self.cancel_count += 1


class RecordingMotionAdapter:
    def __init__(self) -> None:
        self.groups: list[str] = []
        self.cancel_count = 0

    def execute_group(self, group_name: str, ctx) -> bool:
        del ctx
        self.groups.append(group_name)
        return True

    def cancel_step(self) -> None:
        self.cancel_count += 1


def _mobility(
    navigator: RecordingNavigator,
    motion: RecordingMotionAdapter,
) -> BehaviorMobilityAdapter:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    config = loader.navigation_config
    return BehaviorMobilityAdapter(
        navigate_waypoint=navigator,
        motion_adapter=motion,
        waypoints=config["waypoints"],
        behavior_routes=config["behavior_routes"],
        action_motion_groups=config["action_motion_groups"],
        result_timeout_sec=config["result_timeout_sec"],
    )


def test_semantic_behaviors_route_to_expected_waypoints() -> None:
    navigator = RecordingNavigator()
    adapter = _mobility(navigator, RecordingMotionAdapter())

    expected = {
        "sleepOnSide": "A",
        "sleepNow": "A",
        "restInPlace": "B",
        "recharge": "B",
        "eatNormally": "C",
        "eatExcitedly": "C",
        "seekFood": "C",
        "seekFoodUrgently": "C",
        "inspectDogFood": "C",
        "barkShortAlert": "D",
        "lickPaws": "E",
    }
    assert {
        behavior: adapter.waypoint_for_behavior(behavior)
        for behavior in expected
    } == expected

    assert adapter.navigate_for_behavior("sleepNow", timeout_sec=120.0)
    waypoint_name, waypoint, timeout = navigator.calls[-1]
    assert waypoint_name == "A"
    assert waypoint["label"] == "sleep"
    assert timeout == 120.0


def test_unrouted_behavior_neither_navigates_nor_claims_stage() -> None:
    navigator = RecordingNavigator()
    adapter = _mobility(navigator, RecordingMotionAdapter())

    assert adapter.navigate_for_behavior("stand_up")
    assert not navigator.calls
    assert not adapter.handles("stand_up", "action")


def test_navigation_failure_is_returned_to_behavior_executor() -> None:
    navigator = RecordingNavigator(result=False)
    adapter = _mobility(navigator, RecordingMotionAdapter())

    assert not adapter.navigate_for_behavior(
        "eatNormally",
        timeout_sec=60.0,
    )
    assert navigator.calls[-1][0] == "C"


def test_existing_stage_uses_mobility_without_changing_exact_action() -> None:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    navigator = RecordingNavigator()
    motion = RecordingMotionAdapter()
    mobility = _mobility(navigator, motion)
    executor = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes(),
        controller_adapters={"behavior_mobility": mobility},
    )
    ctx = ExecutionContext.from_goal("sleepOnSide", {})
    ctx.resolved_behavior_name = "sleepOnSide"

    result = executor.execute_stage(
        {
            "stage_id": "prepare",
            "selection_policy": "first",
            "required": True,
            "candidates": [{"unit_id": "ACT_CIRCLE_AROUND"}],
        },
        ctx,
    )

    assert result.success
    assert result.unit_id == "ACT_CIRCLE_AROUND"
    assert ctx.executed_units == ["ACT_CIRCLE_AROUND"]
    assert motion.groups == ["approach_stop"]


def test_cancel_stops_nav2_and_twist_motion() -> None:
    navigator = RecordingNavigator()
    motion = RecordingMotionAdapter()
    adapter = _mobility(navigator, motion)

    adapter.cancel_step()

    assert navigator.cancel_count == 1
    assert motion.cancel_count == 1
