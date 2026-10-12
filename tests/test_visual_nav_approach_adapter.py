from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from marsdog_action_executor.adapters.person_nav_approach_adapter import Ros2PersonApproachTransport
from marsdog_action_executor.adapters.visual_nav_approach_adapter import VisualNavApproachAdapter
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.stage_executor import StageExecutor


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
POSE = {"header": {"frame_id": "map"}, "pose": {
    "position": {"x": 1., "y": 2., "z": 0.},
    "orientation": {"x": 0., "y": 0., "z": 0., "w": 1.}}}
POINT = {"header": {"frame_id": "map"}, "point": {"x": 3., "y": 2., "z": 0.}}


class Transport:
    def __init__(self):
        self.recovery_required = False
        self.calls = []
        self.located = None
        self.nav_result = (True, False, "navigation_succeeded")
        self.on_locate = None
        self.on_navigate = None

    def locate_target(self, target_id, kind, distance, deadline, cancel, **options):
        self.calls.append(("locate", target_id, kind, distance, options))
        if self.on_locate:
            self.on_locate(cancel)
        return self.located or {
            "ok": True, "status": 0, "target_id": target_id, "target_type": kind,
            "vision_epoch": "epoch", "target_point": POINT,
            "navigation_required": True, "navigation_goal": POSE,
        }, "ok"

    valid_pose = staticmethod(Ros2PersonApproachTransport.valid_pose)

    def navigate(self, pose, deadline, cancel):
        self.calls.append(("navigate", pose))
        if self.on_navigate:
            self.on_navigate()
        return self.nav_result

    def cancel(self):
        self.calls.append(("cancel",))


def setup(kind="human", behavior="expressCuriosityWithHuman", *, candidate_confidence=.9, **options):
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    transport, commands, leases, chassis_calls = Transport(), [], [], []
    target_id = "epoch:human:7" if kind == "human" else "epoch:object:7"
    target = {"target_type": kind, "target_id": target_id, "vision_epoch": "epoch", "label": "dog" if kind == "animal" else "toy"}
    ctx = ExecutionContext.from_goal(behavior, {"target": target})
    ctx.resolved_behavior_name = behavior
    candidate = {**target, "track_id": 7, "confidence": candidate_confidence, "detection_confidence": candidate_confidence,
                 "tracking_state": "tracking", "last_seen_age_ms": 0.,
                 "bbox": [.4, .1, .2, .7], "center_x": .5,
                 "geometry_source": "body", "motion_eligible": True}
    packet = {"schema_version": 1, "header": {"stamp": 100., "frame_id": "color"},
              "vision_epoch": "epoch", "sequence": 1, "snapshot_id": "epoch:1"}
    chassis = NS(prepare_navigation=lambda: chassis_calls.append("prepare") or True,
                 finish_navigation=lambda: chassis_calls.append("finish") or True)
    adapter = None

    def start(session, labels, rate, lease):
        leases.append(("start", session))
        adapter.update_object_detection({
            "schema_version": 2, "header": packet["header"],
            "sequence": sum(entry[0] == "start" for entry in leases),
            "source": "stream", "status": "ok", "stream": {"session_id": session},
            "objects": [candidate],
        })
        return True, ""

    adapter = VisualNavApproachAdapter(
        commands.append, transport=transport, chassis_backend=chassis,
        behavior_policies=loader.visual_target_approach_config["behavior_policies"],
        require_object_stream=True, start_object_detection=start,
        stop_object_detection=lambda session: leases.append(("stop", session)),
        acquire_timeout_sec=.05, **options,
    )
    if kind == "human":
        adapter.update_visual({**packet, "human_candidates": [candidate], "active_target": candidate})
    return adapter, transport, ctx, commands, leases, chassis_calls, loader


@pytest.mark.parametrize("kind,behavior,distance", [
    ("human", "respond_wave_gesture", 1.5),
    ("human", "expressCuriosityWithHuman", 1.5),
    ("animal", "greetAnimal", 1.2),
    ("object", "inspectObject", .9),
    ("object", "inspectDogFood", .9),
])
def test_one_map_localization_and_one_nav2_goal_before_expression(kind, behavior, distance):
    adapter, transport, ctx, commands, leases, chassis, loader = setup(kind, behavior)
    result = adapter.execute_task({"unit_id": adapter.ACTION_ID}, ctx, 160.)
    assert result.success
    assert transport.calls[0][0:4] == ("locate", ctx.target["target_id"], kind, distance)
    assert transport.calls[1] == ("navigate", POSE)
    assert len(transport.calls) == 2
    assert chassis == ["prepare", "finish"]
    assert all(command.is_zero for command in commands)
    assert result.metadata["bound_target_id"] == ctx.target["target_id"]
    if kind != "human":
        assert leases[0][0] == "start" and leases[1] == ("stop", leases[0][1])
    assert loader.get_controller_routes("lite3")[adapter.ACTION_ID] == "visual_nav_approach"


@pytest.mark.parametrize("confidence,success", [(.25, True), (.24, False)])
def test_object_confidence_boundary_controls_localization(confidence, success):
    adapter, transport, ctx, commands, leases, chassis, _ = setup(
        "object", "inspectDeliveryBox", candidate_confidence=confidence,
    )
    result = adapter.execute_task({"unit_id": adapter.ACTION_ID}, ctx, 160.)
    assert result.success is success
    if success:
        assert transport.calls[0][4]["min_confidence"] == .25
        assert [call[0] for call in transport.calls] == ["locate", "navigate"]
    else:
        assert transport.calls == []
        assert chassis == []
    assert leases[1] == ("stop", leases[0][1])
    assert all(command.is_zero for command in commands)


@pytest.mark.parametrize("kind,behavior", [
    ("animal", "greetAnimal"),
    ("object", "inspectFamiliarPlayItem"),
])
def test_object_acquisition_after_higher_visual_sequence_reaches_navigation(kind, behavior):
    adapter, transport, ctx, commands, leases, chassis, _ = setup(kind, behavior)
    assert adapter.update_visual({
        "schema_version": 1,
        "header": {"stamp": 100., "frame_id": "color"},
        "vision_epoch": "epoch", "sequence": 6984, "snapshot_id": "epoch:6984",
        "human_candidates": [],
    })
    # The leased object packet starts at sequence 1 in the same vision epoch.
    result = adapter.execute_task({"unit_id": adapter.ACTION_ID}, ctx, 160.)
    assert result.success
    assert [call[0] for call in transport.calls] == ["locate", "navigate"]
    assert transport.calls[0][1] == ctx.target["target_id"]
    assert result.metadata["bound_target_id"] == ctx.target["target_id"]
    assert leases[1] == ("stop", leases[0][1])
    assert chassis == ["prepare", "finish"]
    assert all(command.is_zero for command in commands)


def test_already_close_does_not_navigate_or_switch_chassis_mode():
    adapter, transport, ctx, commands, leases, chassis, _ = setup()
    transport.located = {"ok": True, "status": 0, "target_id": ctx.target["target_id"],
                         "target_type": "human", "vision_epoch": "epoch",
                         "target_point": POINT, "navigation_required": False}
    assert adapter.execute_task({"unit_id": adapter.ACTION_ID}, ctx, 160.).success
    assert [call[0] for call in transport.calls] == ["locate"]
    assert chassis == []


@pytest.mark.parametrize("kind,behavior,distance", [
    ("human", "expressCuriosityWithHuman", 2.0),
    ("animal", "greetAnimal", .8),
    ("object", "inspectFamiliarPlayItem", .55),
    ("object", "inspectObject", .55),
    *[("object", behavior, .2) for behavior in (
        "inspectObject", "inspectFamiliarPlayItem", "inspectTrashCan",
        "inspectDeliveryBox", "inspectTissuePaper", "inspectDoor", "inspectDogFood",
    )],
])
def test_goal_distance_is_forwarded_to_localization(kind, behavior, distance):
    adapter, transport, ctx, _, _, _, _ = setup(kind, behavior)
    original_policy = dict(adapter._policies[behavior])
    ctx.params["stand_off_distance_m"] = distance
    result = adapter.execute_task({"unit_id": adapter.ACTION_ID}, ctx, 160.)
    assert result.success
    assert transport.calls[0][0:4] == ("locate", ctx.target["target_id"], kind, distance)
    assert result.metadata["stand_off_distance_m"] == distance
    assert adapter._policies[behavior] == original_policy


def test_goal_distance_does_not_change_the_next_goals_default():
    adapter, transport, ctx, _, _, _, _ = setup("object", "inspectFamiliarPlayItem")
    ctx.params["stand_off_distance_m"] = .2
    assert adapter.execute_task({"unit_id": adapter.ACTION_ID}, ctx, 160.).success
    del ctx.params["stand_off_distance_m"]
    assert adapter.execute_task({"unit_id": adapter.ACTION_ID}, ctx, 160.).success
    assert [call[3] for call in transport.calls if call[0] == "locate"] == [.2, .9]


@pytest.mark.parametrize("kind,behavior,distance,reason", [
    ("human", "expressCuriosityWithHuman", 1.49, "unsafe_stand_off_distance"),
    ("animal", "greetAnimal", .79, "unsafe_stand_off_distance"),
    ("animal", "testAnimalBoundary", .9, "unsafe_stand_off_distance"),
    ("object", "inspectObject", .19, "unsafe_stand_off_distance"),
    ("object", "inspectObject", 0., "unsafe_stand_off_distance"),
    ("object", "inspectObject", -1., "unsafe_stand_off_distance"),
    ("object", "inspectObject", float("nan"), "invalid_stand_off_distance"),
    ("object", "inspectObject", float("inf"), "invalid_stand_off_distance"),
    ("object", "inspectObject", "bad", "invalid_stand_off_distance"),
    ("object", "inspectObject", None, "invalid_stand_off_distance"),
    ("object", "inspectObject", True, "invalid_stand_off_distance"),
])
def test_invalid_goal_distance_never_localizes_or_navigates(kind, behavior, distance, reason):
    adapter, transport, ctx, commands, leases, chassis, _ = setup(kind, behavior)
    ctx.params["stand_off_distance_m"] = distance
    result = adapter.execute_task({"unit_id": adapter.ACTION_ID}, ctx, 160.)
    assert not result.success and result.reason == reason
    assert transport.calls == [] and chassis == []
    assert all(command.is_zero for command in commands)
    if leases:
        assert leases[-1] == ("stop", leases[0][1])


@pytest.mark.parametrize("changes", [
    {"target_id": "epoch:human:8"}, {"vision_epoch": "other"}, {"target_type": "animal"},
    {"status": 1}, {"target_point": {}}, {"navigation_goal": {}},
    {"navigation_required": "false"}, {"ok": False},
])
def test_invalid_localization_never_starts_motion(changes):
    adapter, transport, ctx, commands, _, chassis, _ = setup()
    transport.located = {"ok": True, "status": 0, "target_id": ctx.target["target_id"],
                         "target_type": "human", "vision_epoch": "epoch",
                         "target_point": POINT, "navigation_required": True, "navigation_goal": POSE,
                         **changes}
    assert not adapter.execute_task({"unit_id": adapter.ACTION_ID}, ctx, 160.).success
    assert [call[0] for call in transport.calls] == ["locate"]
    assert chassis == [] and all(cmd.is_zero for cmd in commands)


def test_cancel_during_localization_never_sends_a_nav2_goal():
    adapter, transport, ctx, _, _, chassis, _ = setup()
    transport.on_locate = lambda cancel: adapter.cancel_task()
    assert adapter.execute_task({"unit_id": adapter.ACTION_ID}, ctx, 160.).canceled
    assert not any(call[0] == "navigate" for call in transport.calls)
    assert chassis == []


def test_unknown_nav2_terminal_retains_recovery_and_skips_chassis_finish():
    adapter, transport, ctx, _, _, chassis, _ = setup()
    transport.on_navigate = lambda: setattr(transport, "recovery_required", True)
    transport.nav_result = (False, False, "nav2_terminal_unknown:operator_recovery_required")
    assert not adapter.execute_task({"unit_id": adapter.ACTION_ID}, ctx, 160.).success
    assert chassis == ["prepare"] and adapter.recovery_required
    assert adapter.execute_task({"unit_id": adapter.ACTION_ID}, ctx, 160.).reason == "operator_recovery_required"
    assert len([call for call in transport.calls if call[0] == "navigate"]) == 1


def test_navigation_exception_requests_cancel_and_keeps_chassis_locked():
    adapter, transport, ctx, commands, _, chassis, _ = setup()

    def fail():
        raise RuntimeError("result transport unavailable")

    transport.on_navigate = fail
    result = adapter.execute_task({"unit_id": adapter.ACTION_ID}, ctx, 160.)
    assert not result.success and adapter.recovery_required
    assert transport.calls[-1] == ("cancel",)
    assert chassis == ["prepare"]
    assert all(command.is_zero for command in commands)


def test_food_exploration_has_no_fixed_kitchen_navigation():
    _, _, _, _, _, _, loader = setup("object", "inspectDogFood")
    assert "inspectDogFood" not in loader.navigation_config["behavior_routes"]
    assert loader.navigation_config["behavior_routes"]["eatNormally"]["waypoint"] == "C"


@pytest.mark.parametrize("kind,behavior", [
    ("object", "inspectObject"), ("object", "inspectFamiliarPlayItem"),
    ("human", "respond_wave_gesture"),
])
def test_failed_navigation_prevents_expression_stage(kind, behavior):
    adapter, transport, ctx, _, _, _, loader = setup(kind, behavior)
    transport.nav_result = (False, False, "navigation_status_6")
    executor = StageExecutor(action_catalog=loader.action_catalog,
                             controller_routes=loader.get_controller_routes("lite3"),
                             controller_adapters={"visual_nav_approach": adapter})
    stages = loader.get_behavior_template(behavior)["stages"]
    result = executor.execute_stage(stages[0], ctx)
    assert not result.success
    assert ctx.executed_units == [] and ctx.completed_stages == []
    assert result.message == "navigation_status_6"


def test_wave_approach_success_precedes_go2_left_right_twist():
    import json
    from itertools import groupby
    from marsdog_action_executor.adapters.go2_sport_backend import Go2ChassisBackend

    adapter, transport, ctx, _, _, _, loader = setup("human", "respond_wave_gesture")
    requests = []
    clock = NS(now=0.0)
    backend = Go2ChassisBackend(
        request_publisher=lambda request: requests.append(request) or True,
        action_sequences=loader.get_go2_action_sequences(),
        action_plans=loader.get_go2_action_plans(),
        monotonic=lambda: clock.now,
        sleep=lambda seconds: setattr(clock, "now", clock.now + seconds),
    )
    executor = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes("go2"),
        controller_adapters={"visual_nav_approach": adapter, "go2": backend},
    )
    stages = loader.get_behavior_template("respond_wave_gesture")["stages"]
    assert executor.execute_stage(stages[0], ctx).success
    assert [call[0] for call in transport.calls] == ["locate", "navigate"]
    assert transport.calls[0][1:4] == ("epoch:human:7", "human", 1.5)
    assert requests == []  # No expression before localization/Nav2 success.
    assert executor.execute_stage(stages[1], ctx).success
    assert ctx.executed_units == ["ACT_APPROACH_VISUAL_TARGET", "ACT_TWIST_WAIST_LEFT_RIGHT"]
    assert ctx.completed_stages == ["target_approach", "expression"]
    yaw = [json.loads(request.parameter)["z"] for request in requests if request.api_id == 1007]
    assert [value for value, _ in groupby(yaw)] == [.2, -.2, 0.0]
    assert not any(request.api_id in {1008, 1016} for request in requests)
    assert ctx.metadata["go2_action"]["physical_completion_confirmed"] is False


@pytest.mark.parametrize("changed_epoch", [False, True])
def test_wave_never_navigates_to_replacement_person(changed_epoch):
    adapter, transport, ctx, _, _, _, _ = setup("human", "respond_wave_gesture")
    ctx.target["identity"] = "owner"
    epoch = "restarted" if changed_epoch else "epoch"
    replacement = {**ctx.target, "vision_epoch": epoch, "target_id": epoch + ":human:8",
                   "track_id": 8, "confidence": .9, "tracking_state": "tracking",
                   "last_seen_age_ms": 0.0, "bbox": [.4, .1, .2, .7],
                   "geometry_source": "body", "motion_eligible": True}
    assert adapter.update_visual({
        "schema_version": 1, "header": {"stamp": 101., "frame_id": "color"},
        "vision_epoch": epoch, "sequence": 2, "snapshot_id": epoch + ":2",
        "active_target": replacement, "human_candidates": [replacement],
    })
    result = adapter.execute_task({"unit_id": adapter.ACTION_ID}, ctx, 160.)
    assert not result.success
    assert transport.calls == []
