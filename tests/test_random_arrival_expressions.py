"""Physical short sequences used after K navigation or solo in-place choice."""

from __future__ import annotations

from itertools import groupby
from pathlib import Path

import pytest

from marsdog_action_executor.adapters.go2_sport_backend import (
    Go2ChassisBackend, Go2SportRequest,
)
from marsdog_action_executor.adapters.lite3_backend import (
    CMD_STAND_LIE_TOGGLE, Lite3ChassisBackend, Lite3RobotStatus,
)
from marsdog_action_executor.adapters.velocity import TwistCommand
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.stage_executor import StageExecutor


UNITS = (
    "ACT_ROAM_LOOK_DOWN_UP", "ACT_ROAM_SCAN_HEAD_UP",
    "ACT_ROAM_STRETCH_SCAN", "ACT_ROAM_TWIST_STRETCH",
    "ACT_ROAM_LIE_STAND", "ACT_ROAM_LOWER_HEAD_PAUSE",
    "ACT_ROAM_OBSERVE_PAUSE", "ACT_ROAM_HEAD_UP_TWIST",
)
POSE_AXES = {0x21010130, 0x21010135, 0x21010102}


@pytest.fixture(scope="module")
def loader():
    result = ConfigLoader(Path(__file__).resolve().parents[1] / "config")
    result.load_all()
    return result


class Clock:
    def __init__(self):
        self.now = 0.0
        self.on_sleep = lambda: None

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        assert duration >= 0
        self.now += duration
        self.on_sleep()


def _lite3(loader, clock, commands, twists, *, publisher=None):
    cfg = loader.lite3_action_config
    backend = Lite3ChassisBackend(
        command_publisher=publisher or (lambda cmd: commands.append(cmd) or True),
        twist_publisher=twists.append,
        action_plans=loader.get_lite3_action_plans(),
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=cfg["accepted_unverified_actions"],
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    def feed_status():
        toggles = sum(cmd.cmd_code == CMD_STAND_LIE_TOGGLE for cmd in commands)
        backend.update_status(Lite3RobotStatus(
            basic_state=1 if toggles % 2 else 6,
            gait_state=0, motion_state=0, battery_level=80.0,
            ultrasound_forward=3.0, ultrasound_backward=3.0,
        ))

    clock.on_sleep = feed_status
    feed_status()
    return backend


def _go2(loader, clock, requests, *, publisher=None):
    return Go2ChassisBackend(
        request_publisher=publisher or (lambda req: requests.append(req) or True),
        action_sequences=loader.get_go2_action_sequences(),
        action_plans=loader.get_go2_action_plans(),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )


def _ctx(behavior="walk_to_random_point"):
    ctx = ExecutionContext.from_goal(behavior, {})
    ctx.resolved_behavior_name = behavior
    ctx.current_posture = "standing"
    return ctx


@pytest.mark.parametrize("unit", UNITS)
def test_lite3_combo_executes_every_gesture_and_finishes_standing(loader, unit):
    commands, twists = [], []
    backend = _lite3(loader, Clock(), commands, twists)
    ctx = _ctx()
    cfg = loader.get_action_config(unit)
    assert backend.execute_step(cfg, ctx, cfg["timeout_sec"])

    actual = [
        (cmd.cmd_code, cmd.value) for cmd in commands
        if cmd.cmd_code in POSE_AXES and cmd.value != 0
    ]
    actual = [gesture for gesture, _ in groupby(actual)]
    expected = [
        (step["cmd_code"], step["value"])
        for step in loader.get_lite3_action_plans()[unit]["sequence"]
        if step["type"] == "pose"
    ]
    assert actual == expected
    assert ctx.metadata["lite3_action"]["physical_posture"] == "standing"
    assert twists and all(twist == TwistCommand() for twist in twists)
    assert sum(cmd.cmd_code == CMD_STAND_LIE_TOGGLE for cmd in commands) == (
        2 if unit == "ACT_ROAM_LIE_STAND" else 0
    )


@pytest.mark.parametrize("unit", UNITS)
def test_go2_combo_executes_the_complete_sequence_in_place(loader, unit):
    requests = []
    backend = _go2(loader, Clock(), requests)
    cfg = loader.get_action_config(unit)
    assert backend.execute_step(cfg, _ctx(), cfg["timeout_sec"])
    sequence = loader.get_go2_action_sequences()[unit]
    expected = {"euler": 1007, "stand_down": 1005,
                "stand_up": 1004, "stop_move": 1003, "stretch": 1017}
    preparation = [1003, 1004, 1002] if sequence[0]["command"] in {"euler", "stretch"} else []
    assert [req.api_id for req in requests[:-4] if req.api_id != 1002] == [api for api in preparation if api != 1002] + [
        expected[item["command"]] for item in sequence
    ]
    assert requests[-4:] == [Go2SportRequest(1003)] * 3 + [Go2SportRequest(1002)]


@pytest.mark.parametrize("behavior", ["walk_to_random_point", "go_out_to_play"])
def test_arrival_stage_uses_real_backend_after_navigation_confirmation(loader, behavior):
    requests = []
    executor = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes("go2"),
        controller_adapters={"go2": _go2(loader, Clock(), requests)},
    )
    stage = loader.get_behavior_template(behavior)["stages"][1]
    assert stage["stage_id"] == "arrival_expression"
    ctx = _ctx(behavior)
    ctx.completed_stages.append("navigation")
    result = executor.execute_stage(stage, ctx, seed=7)
    assert result.success
    assert result.unit_id in UNITS
    assert ctx.completed_stages == ["navigation", "arrival_expression"]
    assert len(requests) > 3


def test_arrival_stage_cannot_report_mock_success_without_controller(loader):
    executor = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes("lite3"),
    )
    ctx = _ctx()
    stage = loader.get_behavior_template("walk_to_random_point")["stages"][1]
    assert not executor.execute_stage(stage, ctx, seed=1).success
    assert ctx.executed_units == []
    assert ctx.metadata["unit_failure_reason"] == "controller_required:lite3"


def test_lite3_cancellation_while_lying_does_not_send_the_stand_toggle(loader):
    commands, twists = [], []
    clock = Clock()
    backend = _lite3(loader, clock, commands, twists)
    feed_status = clock.on_sleep
    ctx = _ctx()
    samples = 0

    def cancel_after_lie_confirmation():
        nonlocal samples
        feed_status()
        if sum(cmd.cmd_code == CMD_STAND_LIE_TOGGLE for cmd in commands) == 1:
            samples += 1
            # Five fresh lie samples confirm the transition; cancel in the hold.
            if samples >= 8:
                ctx.cancel_requested = True

    clock.on_sleep = cancel_after_lie_confirmation
    assert not backend.execute_step({"unit_id": "ACT_ROAM_LIE_STAND"}, ctx, 20)
    assert sum(cmd.cmd_code == CMD_STAND_LIE_TOGGLE for cmd in commands) == 1
    assert twists[-5:] == [TwistCommand()] * 5


def test_lite3_failed_first_gesture_does_not_run_later_gestures(loader):
    commands, twists = [], []

    def reject_pitch(cmd):
        commands.append(cmd)
        return not (cmd.cmd_code == 0x21010130 and cmd.value > 0)

    backend = _lite3(loader, Clock(), commands, twists, publisher=reject_pitch)
    assert not backend.execute_step({"unit_id": "ACT_ROAM_LOOK_DOWN_UP"}, _ctx(), 20)
    assert not any(cmd.cmd_code == 0x21010130 and cmd.value < 0 for cmd in commands)
    assert twists[-5:] == [TwistCommand()] * 5


@pytest.mark.parametrize("cancel", [True, False])
def test_go2_cancel_or_failure_resets_euler_and_skips_later_gestures(loader, cancel):
    requests = []
    clock = Clock()
    ctx = _ctx()

    def publish(req):
        requests.append(req)
        # Refuse the first neutral command, after a successfully held low pitch.
        return cancel or req != Go2SportRequest(1007, '{"x":0.0,"y":0.0,"z":0.0}')

    backend = _go2(loader, clock, requests, publisher=publish)
    if cancel:
        def cancel_after_euler():
            if any(req.api_id == 1007 for req in requests):
                ctx.cancel_requested = True
        clock.on_sleep = cancel_after_euler
    assert not backend.execute_step({"unit_id": "ACT_ROAM_LOOK_DOWN_UP"}, ctx, 20)
    assert requests[-4] == Go2SportRequest(1007, '{"x":0.0,"y":0.0,"z":0.0}')
    assert all('"y":-0.12' not in req.parameter for req in requests)
    assert requests[-3:] == [Go2SportRequest(1003)] * 3
    assert not backend._standing_prepared and not backend._balance_prepared


def test_new_combos_are_distinct_target_free_and_keep_hardware_gate(loader):
    plans = loader.get_lite3_action_plans()
    signatures = set()
    for unit in UNITS:
        cfg = loader.get_action_config(unit)
        assert cfg["requires_controller"] is True
        assert not cfg.get("conditions")
        assert plans[unit]["verified"] is False
        signatures.add(repr(plans[unit]["sequence"]))
    assert len(signatures) == len(UNITS)
    assert loader.lite3_action_config["allow_unverified"] is False
    gated = Lite3ChassisBackend(
        command_publisher=lambda cmd: True, twist_publisher=lambda cmd: None,
        action_plans=plans, allow_proxies=True, allow_unverified=False,
        accepted_unverified_actions=list(UNITS),
    )
    assert all(not gated.can_execute(unit) for unit in UNITS)


TOY_ARRIVAL_UNITS = (
    "ACT_ROAM_LOOK_DOWN_UP", "ACT_ROAM_SCAN_HEAD_UP", "ACT_ROAM_STRETCH_SCAN",
)


@pytest.mark.parametrize("chassis", ["lite3", "go2"])
@pytest.mark.parametrize("seed,expected_unit", [
    (1, "ACT_ROAM_LOOK_DOWN_UP"),
    (0, "ACT_ROAM_SCAN_HEAD_UP"),
    (5, "ACT_ROAM_STRETCH_SCAN"),
])
def test_toy_arrival_runs_a_complete_combo_before_original_interaction(
    loader, chassis, seed, expected_unit,
):
    stages = loader.get_behavior_template("inspectFamiliarPlayItem")["stages"]
    assert [stage["stage_id"] for stage in stages] == [
        "target_approach", "arrival_expression", "inspect",
    ]
    assert stages[0]["required"] and stages[0]["failure_policy"] == "abort"
    assert stages[1]["required"] and stages[1]["failure_policy"] == "abort"
    assert [c["unit_id"] for c in stages[1]["candidates"]] == list(TOY_ARRIVAL_UNITS)
    commands, twists, requests = [], [], []
    clock = Clock()
    backend = (_lite3(loader, clock, commands, twists) if chassis == "lite3"
               else _go2(loader, clock, requests))
    budgets = []
    execute_step = backend.execute_step

    def run_step(cfg, ctx, duration):
        budgets.append(duration)
        return execute_step(cfg, ctx, duration)

    backend.execute_step = run_step
    executor = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes(chassis),
        controller_adapters={chassis: backend},
    )
    ctx = _ctx("inspectFamiliarPlayItem")
    ctx.target = {"target_type": "object", "target_id": "epoch:object:25"}
    ctx.completed_stages.append("target_approach")
    result = executor.execute_stage(stages[1], ctx, seed=seed)
    assert result.success, result.message
    assert result.unit_id == expected_unit
    assert budgets == [12.0]  # Stage override, not the 20 s roaming allowance.
    assert ctx.completed_stages == ["target_approach", "arrival_expression"]
    assert ctx.target["target_id"] == "epoch:object:25"
    if chassis == "lite3":
        actual = [
            (cmd.cmd_code, cmd.value) for cmd in commands
            if cmd.cmd_code in POSE_AXES and cmd.value != 0
        ]
        actual = [gesture for gesture, _ in groupby(actual)]
        expected = [
            (step["cmd_code"], step["value"])
            for step in loader.get_lite3_action_plans()[expected_unit]["sequence"]
        ]
        assert actual == expected and len(actual) >= 2
        assert ctx.metadata["lite3_action"]["physical_posture"] == "standing"
        assert twists and all(twist == TwistCommand() for twist in twists)
        assert not any(cmd.cmd_code == CMD_STAND_LIE_TOGGLE for cmd in commands)
    else:
        assert requests[:2] == [Go2SportRequest(1003), Go2SportRequest(1004)]
        assert all(req.api_id in {1002, 1007, 1017} for req in requests[2:-4])
        assert sum(req.api_id == 1017 for req in requests) == (expected_unit == "ACT_ROAM_STRETCH_SCAN")
        assert requests[-4:] == [Go2SportRequest(1003)] * 3 + [Go2SportRequest(1002)]
        assert len(requests) > 6
    result = executor.execute_stage(stages[2], ctx, seed=1)
    assert result.success, result.message
    if chassis == "go2":
        assert result.unit_id in {candidate["unit_id"] for candidate in stages[2]["candidates"]}
        assert ctx.metadata["go2_action"]["fidelity"] in {"exact", "proxy"}
    else:
        assert result.unit_id == "ACT_BITE_AND_SHAKE_SLIPPERS_SOCKS_OR_TOY"
    assert ctx.completed_stages == ["target_approach", "arrival_expression", "inspect"]
    assert ctx.executed_units == [expected_unit, result.unit_id]


def test_toy_arrival_respects_remaining_behavior_budget(loader, monkeypatch):
    clock = Clock()
    commands, twists = [], []
    backend = _lite3(loader, clock, commands, twists)
    monkeypatch.setattr(
        "marsdog_action_executor.units.unit_executors.time.monotonic",
        clock.monotonic,
    )
    executor = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes("lite3"),
        controller_adapters={"lite3": backend},
    )
    ctx = _ctx("inspectFamiliarPlayItem")
    ctx.runtime_deadline_monotonic = 1.0
    ctx.completed_stages.append("target_approach")
    stage = loader.get_behavior_template("inspectFamiliarPlayItem")["stages"][1]
    result = executor.execute_stage(stage, ctx, seed=1)
    assert not result.success
    assert ctx.completed_stages == ["target_approach"]
    assert ctx.executed_units == []
    assert not any(cmd.cmd_code == 0x21010130 and cmd.value < 0 for cmd in commands)
    assert twists[-5:] == [TwistCommand()] * 5
    assert any(cmd.cmd_code == 0x21010130 and cmd.value == 0 for cmd in commands)
