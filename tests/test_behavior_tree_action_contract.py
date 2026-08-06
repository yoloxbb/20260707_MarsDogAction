"""Contract tests for the upstream behavior-tree action mapping."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from marsdog_action_executor.behavior_resolver import BehaviorResolver
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"

EXPECTED_BEHAVIORS = {
    "eatNormally",
    "seekFood",
    "eatExcitedly",
    "seekFoodUrgently",
    "barkShortAlert",
    "lickPaws",
    "sleepOnSide",
    "sleepNow",
    "restInPlace",
    "recharge",
    "testAnimalBoundary",
    "greetAnimal",
    "inviteAnimalToPlay",
    "seekHumanInteraction",
    "seekInteraction",
    "inviteHumanToPlay",
    "exploreRoom",
    "inspectObject",
    "inspectFamiliarPlayItem",
    "inspectTrashCan",
    "inspectDeliveryBox",
    "inspectTissuePaper",
    "inspectDoor",
    "inspectDogFood",
    "expressCalmWithHuman",
    "expressCalmAlone",
    "expressJoyWithHuman",
    "expressJoyAlone",
    "expressCuriosityWithHuman",
    "expressCuriosityAlone",
    "expressExcitementWithHuman",
    "expressExcitementAlone",
    "expressAnxietyWithHuman",
    "expressAnxietyAlone",
    "expressFearWithHuman",
    "expressFearAlone",
    "respond_owner_call",
    "sit_down",
    "lie_down",
    "stand_up",
    "wait_in_place",
    "come_to_owner",
    "follow_owner",
    "give_paw",
    "high_five",
    "roll_over",
    "spin_around",
    "return_to_owner",
    "drop_object",
    "play_dead",
    "bring_object",
    "fetch_object",
    "emergency_stop",
}

# Snapshot of behavior -> ordered stages -> ordered candidate ACT_* IDs.
EXPECTED_MAPPING_SHA256 = (
    "adf70be3c96b6b53de54625ea20e9d7712e2322224b87949e0b7762a142150ae"
)


def _load() -> ConfigLoader:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    return loader


def _mapping(loader: ConfigLoader) -> dict[str, list[list[str]]]:
    return {
        name: [
            [candidate["unit_id"] for candidate in stage["candidates"]]
            for stage in config["stages"]
        ]
        for name, config in loader.behavior_tree_templates.items()
    }


def test_behavior_tree_mapping_matches_reviewed_contract() -> None:
    loader = _load()
    mapping = _mapping(loader)

    assert set(mapping) == EXPECTED_BEHAVIORS
    assert sum(
        len(stage)
        for stages in mapping.values()
        for stage in stages
    ) == 211
    assert len({
        unit_id
        for stages in mapping.values()
        for stage in stages
        for unit_id in stage
    }) == 188

    payload = json.dumps(
        mapping,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    assert hashlib.sha256(payload).hexdigest() == EXPECTED_MAPPING_SHA256


def test_every_tree_action_is_registered_and_every_stage_selects_one() -> None:
    loader = _load()
    referenced_actions = {
        candidate["unit_id"]
        for config in loader.behavior_tree_templates.values()
        for stage in config["stages"]
        for candidate in stage["candidates"]
    }

    for behavior, config in loader.behavior_tree_templates.items():
        orders = []
        for stage in config["stages"]:
            orders.append(stage["order"])
            assert stage["selection_policy"] == "random_one", behavior
            assert stage["required"] is True
            assert stage["candidates"]
            for candidate in stage["candidates"]:
                assert candidate["unit_id"] in loader.action_catalog
        assert orders == list(range(1, len(orders) + 1))

    assert set(loader.action_catalog) == referenced_actions
    assert len(loader.action_catalog) == 188
    assert {
        config["unit_id"]
        for config in loader.action_catalog.values()
    } == referenced_actions
    assert {
        config["interrupt_policy"]
        for config in loader.action_catalog.values()
    } <= {"immediate", "safe_point", "non_interruptible"}

    routes = loader.controller_routes["routes"]
    assert set(routes) - {"_default"} <= referenced_actions

    expected_agv_actions = set(loader.agv_motion_config["action_motion_groups"])
    agv_actions = set(loader.agv_motion_config["action_motion_groups"])
    agv_groups = loader.agv_motion_config["motion_groups"]
    routed_agv_actions = {
        action
        for action, route in routes.items()
        if route == "agv"
    }
    assert agv_actions == routed_agv_actions
    assert agv_actions == expected_agv_actions
    assert len(agv_actions) >= 51  # at minimum all previously mapped actions
    assert len(agv_groups) == len(loader.agv_motion_config["motion_groups"])

    stationary_sleep_actions = {
        "ACT_FLIP_BODY",
        "ACT_WHINE_SOFTLY",
        "ACT_TWITCH_OR_KICK_LEGS",
    }
    for behavior_name in ("sleepOnSide", "sleepNow"):
        sleeping_stage = next(
            stage
            for stage in loader.behavior_tree_templates[behavior_name]["stages"]
            if stage["stage_id"] == "sleeping"
        )
        assert sleeping_stage["motion_state"] == "stationary"
        assert {
            candidate["unit_id"]
            for candidate in sleeping_stage["candidates"]
        } == stationary_sleep_actions

    assert {
        action: loader.agv_motion_config["action_motion_groups"][action]
        for action in stationary_sleep_actions
    } == {action: "stop" for action in stationary_sleep_actions}
    assert {
        action: loader.navigation_config["action_motion_groups"][action]
        for action in stationary_sleep_actions
    } == {action: "stop" for action in stationary_sleep_actions}

    limits = loader.agv_motion_config["limits"]
    for group_config in agv_groups.values():
        if isinstance(group_config, dict):
            # pick_random: validate candidates
            for segment in group_config.get("candidates", []):
                assert (
                    abs(segment.get("linear_x", 0.0)) <= limits["max_linear_x"]
                )
                assert (
                    abs(segment.get("linear_y", 0.0)) <= limits["max_linear_y"]
                )
                assert (
                    abs(segment.get("angular_z", 0.0)) <= limits["max_angular_z"]
                )
        else:
            for segment in group_config:
                assert (
                    abs(segment.get("linear_x", 0.0)) <= limits["max_linear_x"]
                )
                assert (
                    abs(segment.get("linear_y", 0.0)) <= limits["max_linear_y"]
                )
                assert (
                    abs(segment.get("angular_z", 0.0)) <= limits["max_angular_z"]
                )

    for action, group_name in loader.agv_motion_config[
        "action_motion_groups"
    ].items():
        timeout = loader.action_catalog[action]["timeout_sec"]
        group_config = agv_groups[group_name]
        if isinstance(group_config, dict):
            # pick_random: worst-case is duration_max_sec
            group_duration = float(
                group_config.get("duration_max_sec", 0.0)
            )
        else:
            group_duration = sum(
                segment["duration_sec"]
                for segment in group_config
            )
        if timeout > 0.0:
            assert group_duration <= timeout

    navigation = loader.navigation_config
    assert set(navigation["waypoints"]) == {"A", "B", "C", "D", "E"}
    assert {
        name: route["waypoint"]
        for name, route in navigation["behavior_routes"].items()
    } == {
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


def test_only_tree_templates_and_actions_are_exposed() -> None:
    loader = _load()

    assert (
        loader.get_behavior_template("barkShortAlert")["stages"][0]
        ["candidates"][0]["unit_id"]
        == "ACT_SNIFF_AND_CIRCLE_AT_TOILET_SPOT"
    )
    assert (
        loader.get_behavior_template("sit_down")["stages"][0]
        ["candidates"][0]["unit_id"]
        == "ACT_BASIC_SIT"
    )
    assert loader.get_behavior_template("defecate") is None
    assert loader.get_behavior_template("go_back") is None
    assert "ACT_POSTURE_SIT" not in loader.action_catalog
    assert "ACT_SNIFF_BOWL_EDGE" not in loader.action_catalog


def test_direct_tree_names_are_preserved_exactly() -> None:
    loader = _load()
    resolver = BehaviorResolver(
        canonical_behaviors=loader.get_behavior_names(),
    )

    for name in (
        "restInPlace",
        "sleepOnSide",
        "respond_owner_call",
        "emergency_stop",
    ):
        ctx = resolver.resolve(ExecutionContext.from_goal(name, {}))
        assert ctx.is_valid
        assert ctx.resolved_behavior_name == name
        assert "alias_source" not in ctx.metadata


def test_legacy_names_and_aliases_are_rejected() -> None:
    loader = _load()
    resolver = BehaviorResolver(
        canonical_behaviors=loader.get_behavior_names(),
    )

    legacy_names = (
        "expressCuriosiexpressCuriosityWithHumanty",
        "seek_food_or_water",
        "expressJoy",
        "defecate",
        "go_back",
        "emergencyStop",
        "wagTailFast",
    )
    for name in legacy_names:
        ctx = resolver.resolve(ExecutionContext.from_goal(name, {}))
        assert not ctx.is_valid
        assert ctx.resolved_behavior_name == name
        assert ctx.error_reason == f"unsupported_behavior: {name!r}"
