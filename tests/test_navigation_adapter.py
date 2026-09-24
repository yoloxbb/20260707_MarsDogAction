"""Tests for semantic waypoint navigation and platform stage actions."""

from __future__ import annotations

from pathlib import Path

import yaml

from marsdog_action_executor.adapters.navigation_adapter import (
    BehaviorMobilityAdapter,
)
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.stage_executor import StageExecutor


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"

EXPECTED_WAYPOINTS = {
    "A": ("sleep", -3.715, -1.065, 0.136, 0.991),
    "B": ("charging", -1.855, -0.665, 0.790, 0.613),
    "C": ("food", -2.961, 1.478, 0.259, 0.966),
    "D": ("toilet", -1.905, 3.360, -0.708, 0.706),
    "E": ("cleaning", -4.039, 2.627, -0.421, 0.907),
}

# Exact names from ~/.ros/waypoints.yaml.  The keys are Action's internal
# semantic route slots, not the occupancy editor's A-J waypoint IDs.
EXPECTED_PLACES = {
    "A": "卧室",
    "B": "充电桩",
    "C": "厨房",
    "D": "卫生间",
    "E": "客厅",
}


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


class RecordingFixedNavigator:
    def __init__(self, result: bool = True) -> None:
        self.result = result
        self.calls: list[tuple[str, str, float, object]] = []
        self.cancel_count = 0

    def __call__(
        self,
        task_id: str,
        place_id: str,
        timeout_sec: float,
        status_callback,
    ) -> bool:
        self.calls.append((task_id, place_id, timeout_sec, status_callback))
        return self.result

    def cancel_navigation(self) -> None:
        self.cancel_count += 1


class RecordingMotionAdapter:
    def __init__(self) -> None:
        self.actions: list[str] = []
        self.cancel_count = 0

    def execute_step(self, unit_config, ctx, duration=None) -> bool:
        del ctx, duration
        self.actions.append(unit_config["unit_id"])
        return True

    def cancel_step(self) -> None:
        self.cancel_count += 1


def _mobility(
    navigator: RecordingNavigator,
    motion: RecordingMotionAdapter,
    *,
    fixed_navigator: RecordingFixedNavigator | None = None,
    in_place_probability=None,
    fixed_pool=None,
) -> BehaviorMobilityAdapter:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    config = loader.navigation_config
    return BehaviorMobilityAdapter(
        navigate_waypoint=navigator,
        navigate_fixed_place=fixed_navigator,
        fixed_places=config["waypoint_nav"]["places"],
        motion_adapter=motion,
        waypoints=config["waypoints"],
        behavior_routes=config["behavior_routes"],
        stage_actions=config["stage_actions"],
        result_timeout_sec=config["result_timeout_sec"],
        random_navigation_behaviors=config.get(
            "random_navigation_behaviors", []
        ),
        random_navigation_place=config.get("random_navigation_place"),
        random_navigation_result_timeout_sec=config.get(
            "random_navigation_result_timeout_sec"
        ),
        random_navigation_in_place_probability=(
            in_place_probability
            if in_place_probability is not None
            else config.get("random_navigation_in_place_probability", {})
        ),
        random_navigation_fixed_pool=fixed_pool,
    )


def test_semantic_behaviors_route_to_expected_waypoints() -> None:
    navigator = RecordingNavigator()
    adapter = _mobility(navigator, RecordingMotionAdapter())

    expected = {
        "go_home": "A",
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


def test_waypoint_poses_match_latest_measured_values() -> None:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()

    actual = {
        name: (
            waypoint["label"],
            waypoint["x"],
            waypoint["y"],
            waypoint["orientation_z"],
            waypoint["orientation_w"],
        )
        for name, waypoint in loader.navigation_config["waypoints"].items()
    }

    assert actual == EXPECTED_WAYPOINTS


def test_manual_pose_helper_waypoints_match_navigation_config() -> None:
    with open(CONFIG_DIR.parent / "waypoints.yaml", encoding="utf-8") as stream:
        helper_waypoints = yaml.safe_load(stream)["waypoints"]

    assert helper_waypoints == {
        name: [x, y, orientation_z, orientation_w]
        for name, (
            _label,
            x,
            y,
            orientation_z,
            orientation_w,
        ) in EXPECTED_WAYPOINTS.items()
    }


def test_random_navigation_uses_waypoint_nav_reserved_k_target() -> None:
    direct_nav2 = RecordingNavigator()
    fixed_nav = RecordingFixedNavigator()
    adapter = _mobility(
        direct_nav2,
        RecordingMotionAdapter(),
        fixed_navigator=fixed_nav,
    )
    for index in range(10):
        assert adapter.navigate_for_behavior(
            "walk_to_random_point",
            timeout_sec=60.0,
            task_id=f"action:random-{index}:waypoint",
        )

    selected = [place for _, place, _, _ in fixed_nav.calls]
    assert selected == ["K"] * 10
    assert all(timeout == 45.0 for _, _, timeout, _ in fixed_nav.calls)
    assert direct_nav2.calls == []


def test_core_walk_commands_use_distinct_random_navigation_behaviors() -> None:
    navigator = RecordingNavigator()
    fixed_nav = RecordingFixedNavigator()
    motion = RecordingMotionAdapter()
    adapter = _mobility(navigator, motion, fixed_navigator=fixed_nav)

    for behavior_name, unit_id in (
        ("walk_to_random_point", "ACT_NAV_WALK_RANDOM"),
        ("go_out_to_play", "ACT_NAV_GO_OUT_TO_PLAY"),
    ):
        assert adapter.waypoint_for_behavior(behavior_name) == "__random__"
        assert adapter.navigate_for_behavior(
            behavior_name,
            timeout_sec=30.0,
            task_id=f"action:{behavior_name}:waypoint",
        )
        ctx = ExecutionContext.from_goal(behavior_name, {})
        ctx.resolved_behavior_name = behavior_name
        ctx.current_stage = "navigation"
        assert adapter.execute_step({"unit_id": unit_id}, ctx, 1.0)

    assert navigator.calls == []
    assert len(fixed_nav.calls) == 2
    assert all(call[1] == "K" for call in fixed_nav.calls)


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


def test_fixed_and_random_points_use_waypoint_nav() -> None:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    config = loader.navigation_config
    direct_nav2 = RecordingNavigator()
    fixed_nav = RecordingFixedNavigator()
    adapter = BehaviorMobilityAdapter(
        navigate_waypoint=direct_nav2,
        navigate_fixed_place=fixed_nav,
        fixed_places=config["waypoint_nav"]["places"],
        motion_adapter=RecordingMotionAdapter(),
        waypoints=config["waypoints"],
        behavior_routes=config["behavior_routes"],
        stage_actions=config["stage_actions"],
        result_timeout_sec=config["result_timeout_sec"],
        random_navigation_behaviors=config["random_navigation_behaviors"],
        random_navigation_place=config["random_navigation_place"],
        random_navigation_result_timeout_sec=(
            config["random_navigation_result_timeout_sec"]
        ),
    )

    callback = object()
    assert adapter.navigate_for_behavior(
        "eatNormally",
        timeout_sec=60.0,
        task_id="action:g-1:waypoint",
        status_callback=callback,
    )
    assert fixed_nav.calls == [
        ("action:g-1:waypoint", EXPECTED_PLACES["C"], 60.0, callback)
    ]
    assert direct_nav2.calls == []

    random_callback = object()
    assert adapter.navigate_for_behavior(
        "walk_to_random_point",
        timeout_sec=30.0,
        task_id="action:g-2:waypoint",
        status_callback=random_callback,
    )
    assert direct_nav2.calls == []
    assert len(fixed_nav.calls) == 2
    assert fixed_nav.calls[1][0] == "action:g-2:waypoint"
    assert fixed_nav.calls[1][1] == "K"
    assert fixed_nav.calls[1][2] == 30.0
    assert fixed_nav.calls[1][3] is random_callback


def test_fixed_point_requires_task_id_before_service_dispatch() -> None:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    config = loader.navigation_config
    fixed_nav = RecordingFixedNavigator()
    adapter = BehaviorMobilityAdapter(
        navigate_waypoint=RecordingNavigator(),
        navigate_fixed_place=fixed_nav,
        fixed_places=config["waypoint_nav"]["places"],
        motion_adapter=RecordingMotionAdapter(),
        waypoints=config["waypoints"],
        behavior_routes=config["behavior_routes"],
        stage_actions=config["stage_actions"],
        random_navigation_behaviors=config["random_navigation_behaviors"],
        random_navigation_place=config["random_navigation_place"],
        random_navigation_result_timeout_sec=(
            config["random_navigation_result_timeout_sec"]
        ),
    )

    assert not adapter.navigate_for_behavior("sleepNow", timeout_sec=30.0)
    assert not adapter.navigate_for_behavior(
        "walk_to_random_point",
        timeout_sec=30.0,
    )
    assert fixed_nav.calls == []


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
    assert motion.actions == ["ACT_CIRCLE_AROUND"]


def test_cancel_stops_nav2_and_twist_motion() -> None:
    navigator = RecordingNavigator()
    fixed_nav = RecordingFixedNavigator()
    motion = RecordingMotionAdapter()
    adapter = _mobility(navigator, motion, fixed_navigator=fixed_nav)

    adapter.cancel_step()

    assert navigator.cancel_count == 1
    assert fixed_nav.cancel_count == 1
    assert motion.cancel_count == 1


def test_random_navigation_stays_in_place_at_probability_one() -> None:
    navigator = RecordingNavigator()
    adapter = _mobility(
        navigator,
        RecordingMotionAdapter(),
        in_place_probability={"expressCalmAlone": 1.0},
    )

    assert adapter.waypoint_for_behavior("expressCalmAlone") == "__random__"
    assert adapter.navigate_for_behavior("expressCalmAlone", timeout_sec=45.0)
    assert navigator.calls == []


def test_random_navigation_always_navigates_at_probability_zero() -> None:
    navigator = RecordingNavigator()
    fixed_nav = RecordingFixedNavigator()
    adapter = _mobility(
        navigator,
        RecordingMotionAdapter(),
        fixed_navigator=fixed_nav,
        in_place_probability={"expressCalmAlone": 0.0},
    )

    assert adapter.navigate_for_behavior(
        "expressCalmAlone",
        timeout_sec=45.0,
        task_id="action:calm-random:waypoint",
    )
    assert navigator.calls == []
    assert fixed_nav.calls[-1][1] == "K"


def test_packaged_random_navigation_fixed_pool_covers_all_fixed_points() -> None:
    """The temporary override ships enabled for A-E; K stays reserved."""
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    config = loader.navigation_config

    assert config["random_navigation_fixed_pool"] == ["A", "B", "C", "D", "E"]
    assert config["random_navigation_place"] == "K"


def test_random_navigation_fixed_pool_draws_only_fixed_points() -> None:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    config = loader.navigation_config
    pool = config["random_navigation_fixed_pool"]
    fixed_nav = RecordingFixedNavigator()
    adapter = BehaviorMobilityAdapter(
        navigate_waypoint=RecordingNavigator(),
        navigate_fixed_place=fixed_nav,
        fixed_places=config["waypoint_nav"]["places"],
        motion_adapter=RecordingMotionAdapter(),
        waypoints=config["waypoints"],
        behavior_routes=config["behavior_routes"],
        stage_actions=config["stage_actions"],
        result_timeout_sec=config["result_timeout_sec"],
        random_navigation_behaviors=config["random_navigation_behaviors"],
        random_navigation_place=config["random_navigation_place"],
        random_navigation_result_timeout_sec=(
            config["random_navigation_result_timeout_sec"]
        ),
        random_navigation_fixed_pool=pool,
    )

    for index in range(20):
        assert adapter.navigate_for_behavior(
            "walk_to_random_point",
            timeout_sec=30.0,
            task_id=f"action:pool-{index}:waypoint",
        )

    selected = [call[1] for call in fixed_nav.calls]
    assert set(selected) <= {EXPECTED_PLACES[name] for name in pool}
    assert "K" not in selected
    # Independent draws: 20 picks must not collapse onto a single point.
    assert len(set(selected)) > 1
    # Random navigation keeps its own per-task budget.
    assert all(call[2] == 30.0 for call in fixed_nav.calls)


def test_empty_random_navigation_fixed_pool_keeps_reserved_k_target() -> None:
    navigator = RecordingNavigator()
    fixed_nav = RecordingFixedNavigator()
    adapter = _mobility(
        navigator,
        RecordingMotionAdapter(),
        fixed_navigator=fixed_nav,
        fixed_pool=[],
    )

    assert adapter.navigate_for_behavior(
        "walk_to_random_point",
        timeout_sec=30.0,
        task_id="action:empty-pool:waypoint",
    )
    assert navigator.calls == []
    assert [call[1] for call in fixed_nav.calls] == ["K"]


def _load_navigation_errors(tmp_path, **changes):
    """Copy the packaged config tree, patch navigation_waypoints.yaml, load."""
    import shutil
    import tempfile

    target = Path(tempfile.mkdtemp(dir=tmp_path)) / "config"
    shutil.copytree(CONFIG_DIR, target)
    path = target / "navigation_waypoints.yaml"
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    config.update(changes)
    path.write_text(
        yaml.safe_dump(config, allow_unicode=True), encoding="utf-8"
    )
    try:
        loader = ConfigLoader(target)
        loader.load_all()
    except Exception as exc:  # ConfigurationError carries the joined list
        return [
            line.strip("- ").strip()
            for line in str(exc).splitlines()
            if "random_navigation_fixed_pool" in line
        ]
    return [
        error
        for error in loader._errors
        if "random_navigation_fixed_pool" in error
    ]


def test_random_navigation_fixed_pool_rejects_unknown_waypoints(tmp_path) -> None:
    errors = _load_navigation_errors(
        tmp_path, random_navigation_fixed_pool=["A", "Z"]
    )
    assert any("unknown waypoint" in error for error in errors)


def test_random_navigation_fixed_pool_rejects_non_list(tmp_path) -> None:
    errors = _load_navigation_errors(tmp_path, random_navigation_fixed_pool="A")
    assert any("must be a list" in error for error in errors)


def test_empty_random_navigation_fixed_pool_passes_validation(tmp_path) -> None:
    assert (
        _load_navigation_errors(tmp_path, random_navigation_fixed_pool=[]) == []
    )
