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
    "expressCalmInPlaceWithHuman",
    "expressJoyInPlaceWithHuman",
    "expressExcitementInPlaceWithHuman",
    "expressAnxietyInPlaceWithHuman",
    "expressFearInPlaceWithHuman",
    "expressCuriosityInPlaceWithHuman",
    "unhappy",
    "miss_owner",
    "farewell_leave",
    "respond_owner_call",
    "approach_voice_caller",
    "respond_person_fall",
    "respond_stop_gesture",
    "sit_down",
    "lie_down",
    "stand_up",
    "play_alone",
    "walk_to_random_point",
    "go_out_to_play",
    "go_home",
    "approach_owner",
    "back_up",
    "stand_still",
    "hold_position",
    "quiet",
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
# 2026-09-18: sleepOnSide/sleepNow gained a leading ``circle`` stage, and
# barkShortAlert was re-choreographed (circle -> action -> exit -> head_up)
# with the new ACT_RAISE_HEAD.  Totals moved from 268/203 to 269/204, which
# is exactly the one added candidate.
# 2026-09-18 (later): the five 进食 behaviors were replaced by the fixed
# 到达点位后的仪式 (低头 → 抬头 → 趴下 → 站立 → 再低头 → 左右扭腰).  The
# candidate total happens to stay at 269 -- 6+6+6+6+7 replaces 11+8+3+4+5 --
# while the unique-action count drops 204 -> 182 because the rewrite orphaned
# 23 eating-only ACT_* units that were then deleted from every config.
EXPECTED_MAPPING_SHA256 = (
    "9187a5aa84e8b499ab8fb3f47e19b07d2827ed7d5db81b09c281472345a1abef"
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
    ) == 273
    assert len({
        unit_id
        for stages in mapping.values()
        for stage in stages
        for unit_id in stage
    }) == 183

    payload = json.dumps(
        mapping,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    assert hashlib.sha256(payload).hexdigest() == EXPECTED_MAPPING_SHA256


def test_emotion_mobility_contract_separates_inplace_and_alone_routes() -> None:
    loader = _load()
    navigation = loader.navigation_config
    emotion_names = (
        "Calm", "Joy", "Curiosity", "Excitement", "Anxiety", "Fear",
    )

    assert set(navigation["random_navigation_behaviors"]) == {
        *(f"express{name}Alone" for name in emotion_names),
        "walk_to_random_point",
        "go_out_to_play",
    }

    in_place = navigation.get("random_navigation_in_place_probability", {})
    assert set(in_place) == {f"express{name}Alone" for name in emotion_names}
    assert all(
        0.0 <= float(prob) <= 1.0 for prob in in_place.values()
    )
    assert set(in_place) <= set(navigation["random_navigation_behaviors"])
    assert navigation["random_navigation_place"] == "K"
    assert navigation["random_navigation_result_timeout_sec"] == 45.0

    for name in emotion_names:
        human = loader.get_behavior_template(f"express{name}WithHuman")
        alone = loader.get_behavior_template(f"express{name}Alone")
        inplace = loader.get_behavior_template(
            f"express{name}InPlaceWithHuman"
        )

        assert [stage["stage_id"] for stage in human["stages"]] == [
            "target_approach",
            "expression",
        ]
        assert human["stages"][0]["candidates"] == [
            {"unit_id": "ACT_APPROACH_VISUAL_TARGET"}
        ]
        assert human["stages"][0]["failure_policy"] == "abort"
        assert all(
            candidate["unit_id"]
            != "ACT_INTERACT_APPROACH_VOICE_CALLER"
            for stage in human["stages"]
            for candidate in stage["candidates"]
        )
        assert [stage["stage_id"] for stage in alone["stages"]] == [
            "expression"
        ]
        assert [stage["stage_id"] for stage in inplace["stages"]] == [
            "expression"
        ]
        assert inplace["stages"][0].get("motion_state", "active") == "active"
        assert inplace["stages"][0]["candidates"] == (
            human["stages"][1]["candidates"]
        )
        assert all(
            candidate["unit_id"] != "ACT_APPROACH_VISUAL_TARGET"
            for candidate in inplace["stages"][0]["candidates"]
        )
        assert all(
            loader.get_controller_routes("go2")[candidate["unit_id"]] == "go2"
            for candidate in inplace["stages"][0]["candidates"]
        )
        assert any(
            loader.get_controller_routes("lite3").get(
                candidate["unit_id"], "unsupported"
            ) == "lite3"
            for candidate in inplace["stages"][0]["candidates"]
        )
        assert f"express{name}InPlaceWithHuman" not in (
            navigation["random_navigation_behaviors"]
        )
        assert f"express{name}InPlaceWithHuman" not in (
            navigation["behavior_routes"]
        )


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
    assert len(loader.action_catalog) == 183
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
    assert routes["_default"] == "unsupported"
    assert "agv" not in routes.values()
    assert routes["ACT_CONTROL_QUIET"] == "mock"

    for action in (
        "ACT_INTERACT_RESPOND_CALL",
        "ACT_INTERACT_APPROACH_VOICE_CALLER",
        "ACT_PERCEPTION_RESPOND_PERSON_FALL",
        "ACT_PERCEPTION_RESPOND_STOP_GESTURE",
        "ACT_OBJECT_DROP",
        "ACT_OBJECT_BRING",
        "ACT_OBJECT_FETCH",
    ):
        assert loader.action_catalog[action]["requires_controller"] is True
        assert any(
            loader.get_controller_routes(platform).get(action, "unsupported")
            not in {"mock", "unsupported"}
            for platform in ("go2", "lite3")
        )

    go2_routes = loader.get_controller_routes("go2")
    lite3_routes = loader.get_controller_routes("lite3")
    assert "agv" not in go2_routes.values()
    assert "agv" not in lite3_routes.values()
    assert set(loader.action_catalog) <= set(go2_routes)

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

    assert stationary_sleep_actions <= set(loader.navigation_config["stage_actions"])
    for action in (
        "ACT_PERCEPTION_RESPOND_PERSON_FALL",
        "ACT_PERCEPTION_RESPOND_STOP_GESTURE",
    ):
        assert go2_routes[action] == "go2"
        assert lite3_routes[action] == "lite3"

    navigation = loader.navigation_config
    assert set(navigation["waypoints"]) == {"A", "B", "C", "D", "E"}
    assert {
        name: route["waypoint"]
        for name, route in navigation["behavior_routes"].items()
    } == {
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


def test_core_voice_behaviors_keep_distinct_action_contracts() -> None:
    loader = _load()
    expected = {
        "walk_to_random_point": [["ACT_NAV_WALK_RANDOM"]],
        "go_out_to_play": [["ACT_NAV_GO_OUT_TO_PLAY"]],
        "go_home": [["ACT_NAV_GO_HOME"]],
        "approach_owner": [["ACT_INTERACT_APPROACH_OWNER_CLOSER"]],
        "back_up": [["ACT_BASIC_BACK_UP"]],
        "stand_still": [
            ["ACT_BASIC_STAND"],
            ["ACT_CONTROL_HOLD_STANDING"],
        ],
        "hold_position": [["ACT_CONTROL_HOLD_POSITION"]],
        "quiet": [["ACT_CONTROL_QUIET"]],
    }

    mapping = _mapping(loader)
    assert {name: mapping[name] for name in expected} == expected
    assert loader.get_behavior_template("stand_up")["stages"] != (
        loader.get_behavior_template("stand_still")["stages"]
    )
    assert loader.get_behavior_template("hold_position")["stages"][0][
        "motion_state"
    ] == "stationary"
    assert loader.action_catalog["ACT_CONTROL_QUIET"]["unit_type"] == (
        "policy"
    )


def test_visual_safety_behaviors_have_exact_stop_only_routes() -> None:
    loader = _load()
    expected = {
        "respond_person_fall": "ACT_PERCEPTION_RESPOND_PERSON_FALL",
        "respond_stop_gesture": "ACT_PERCEPTION_RESPOND_STOP_GESTURE",
    }

    for behavior_name, action_id in expected.items():
        template = loader.get_behavior_template(behavior_name)
        assert template["behavior_name"] == behavior_name
        assert template["stages"] == [
            {
                "stage_id": "safety_stop",
                "order": 1,
                "selection_policy": "random_one",
                "required": True,
                "candidates": [{"unit_id": action_id}],
            }
        ]
        assert loader.action_catalog[action_id]["unit_id"] == action_id
        assert loader.get_controller_routes("go2")[action_id] == "go2"
        assert loader.get_controller_routes("lite3")[action_id] == "lite3"


def test_owner_visual_behaviors_keep_exact_actions_and_safe_choreography() -> None:
    loader = _load()
    expected = {
        "unhappy": "ACT_OWNER_UNHAPPY",
        "miss_owner": "ACT_EXPRESS_MISS_YOU",
        "farewell_leave": "ACT_OWNER_GOING_OUT",
    }

    for behavior_name, action_id in expected.items():
        stages = loader.get_behavior_template(behavior_name)["stages"]
        assert [stage["stage_id"] for stage in stages] == [
            "target_approach",
            "expression",
        ]
        assert stages[0]["failure_policy"] == "abort"
        assert stages[0]["candidates"] == [
            {"unit_id": "ACT_APPROACH_VISUAL_TARGET"}
        ]
        assert stages[1]["candidates"] == [{"unit_id": action_id}]
        assert loader.action_catalog[action_id]["requires_controller"] is True
        assert loader.action_catalog[action_id]["conditions"] == {}
        assert loader.get_controller_routes("go2")[action_id] == "go2"
        assert loader.get_controller_routes("lite3")[action_id] == "lite3"


def test_direct_tree_names_are_preserved_exactly() -> None:
    loader = _load()
    resolver = BehaviorResolver(
        canonical_behaviors=loader.get_behavior_names(),
    )

    for name in (
        "restInPlace",
        "sleepOnSide",
        "respond_owner_call",
        "approach_voice_caller",
        "respond_person_fall",
        "respond_stop_gesture",
        "walk_to_random_point",
        "go_out_to_play",
        "go_home",
        "approach_owner",
        "back_up",
        "stand_still",
        "hold_position",
        "quiet",
        "unhappy",
        "miss_owner",
        "farewell_leave",
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
