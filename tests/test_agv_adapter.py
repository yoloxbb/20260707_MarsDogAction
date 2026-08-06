"""Tests for ACT_* to AGV Twist motion-group execution."""

from __future__ import annotations

import math
from pathlib import Path

import yaml

from marsdog_action_executor.adapters.agv_adapter import (
    AgvMotionAdapter,
    TwistCommand,
)
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.stage_executor import StageExecutor


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


class FakeTime:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration


def _agv_config() -> dict:
    return yaml.safe_load(
        (CONFIG_DIR / "agv_motion_groups.yaml").read_text(encoding="utf-8")
    )


def _adapter(
    commands: list[TwistCommand],
    fake_time: FakeTime,
    *,
    should_stop=None,
) -> AgvMotionAdapter:
    config = _agv_config()
    limits = config["limits"]
    return AgvMotionAdapter(
        publish_twist=commands.append,
        motion_groups=config["motion_groups"],
        action_motion_groups=config["action_motion_groups"],
        publish_rate_hz=config["publish_rate_hz"],
        max_linear_x=limits["max_linear_x"],
        max_linear_y=limits["max_linear_y"],
        max_angular_z=limits["max_angular_z"],
        stop_publish_count=config["stop_publish_count"],
        should_stop=should_stop,
        monotonic=fake_time.monotonic,
        sleep=fake_time.sleep,
    )


def _moving_transitions(commands: list[TwistCommand]) -> list[TwistCommand]:
    transitions: list[TwistCommand] = []
    for command in commands:
        if command.is_zero:
            continue
        if not transitions or command != transitions[-1]:
            transitions.append(command)
    return transitions


def test_forward_group_publishes_at_rate_clamps_and_stops() -> None:
    commands: list[TwistCommand] = []
    fake_time = FakeTime()
    adapter = _adapter(commands, fake_time)
    ctx = ExecutionContext.from_goal("come_to_owner", {})
    ctx.speed_scale = 2.0

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_APPROACH_OWNER"},
        ctx,
    )

    moving = [command for command in commands if not command.is_zero]
    assert moving
    # all velocities clamped to configured limits
    assert all(abs(command.linear_x) <= 0.25 for command in moving)
    assert all(command.linear_y == 0.0 for command in moving)
    assert all(abs(command.angular_z) <= 1.2 for command in moving)
    # at least one forward segment (clamped to max)
    assert any(command.linear_x == 0.25 for command in moving)
    assert commands[-3:] == [TwistCommand(), TwistCommand(), TwistCommand()]


def test_roll_over_proxy_uses_both_rotation_directions_and_stops() -> None:
    commands: list[TwistCommand] = []
    adapter = _adapter(commands, FakeTime())

    assert adapter.execute_step(
        {"unit_id": "ACT_TRICK_ROLL_OVER"},
        ExecutionContext.from_goal("roll_over", {}),
    )

    moving = [command for command in commands if not command.is_zero]
    angular = [command.angular_z for command in moving]
    assert moving
    assert all(command.linear_x == 0.0 for command in moving)
    assert all(abs(command.angular_z) == 1.0 for command in moving)
    assert any(value > 0.0 for value in angular)
    assert any(value < 0.0 for value in angular)
    assert commands[-1].is_zero


def test_sit_turns_left_30_then_right_30_and_stops() -> None:
    commands: list[TwistCommand] = []
    fake_time = FakeTime()
    adapter = _adapter(commands, fake_time)

    assert adapter.execute_step(
        {"unit_id": "ACT_BASIC_SIT"},
        ExecutionContext.from_goal("sit_down", {}),
    )

    moving = _moving_transitions(commands)
    assert moving == [
        TwistCommand(angular_z=0.8),
        TwistCommand(angular_z=-0.8),
        TwistCommand(angular_z=0.8),
    ]
    assert math.isclose(fake_time.now, 3.5, abs_tol=1e-9)
    assert commands[-3:] == [TwistCommand(), TwistCommand(), TwistCommand()]


def test_wait_turns_right_90_then_left_90_and_stops() -> None:
    commands: list[TwistCommand] = []
    fake_time = FakeTime()
    adapter = _adapter(commands, fake_time)

    assert adapter.execute_step(
        {"unit_id": "ACT_BASIC_WAIT"},
        ExecutionContext.from_goal("wait_in_place", {}),
    )

    moving = _moving_transitions(commands)
    assert moving[:2] == [
        TwistCommand(angular_z=-0.8),
        TwistCommand(angular_z=0.8),
    ]
    assert math.isclose(fake_time.now, 4.8, abs_tol=1e-9)
    assert commands[-1].is_zero


def test_play_dead_spins_clockwise_and_stops() -> None:
    commands: list[TwistCommand] = []
    fake_time = FakeTime()
    adapter = _adapter(commands, fake_time)

    assert adapter.execute_step(
        {"unit_id": "ACT_TRICK_PLAY_DEAD"},
        ExecutionContext.from_goal("play_dead", {}),
    )

    assert _moving_transitions(commands) == [
        TwistCommand(angular_z=-1.2),
    ]
    assert math.isclose(fake_time.now, 5.236, abs_tol=1e-9)
    assert commands[-1].is_zero


def test_return_owner_turns_around_then_moves_forward_and_stops() -> None:
    commands: list[TwistCommand] = []
    fake_time = FakeTime()
    adapter = _adapter(commands, fake_time)

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RETURN_OWNER"},
        ExecutionContext.from_goal("return_to_owner", {}),
    )

    moving = _moving_transitions(commands)
    assert moving[:2] == [
        TwistCommand(angular_z=1.2),
        TwistCommand(linear_x=0.18),
    ]
    assert math.isclose(fake_time.now, 5.0, abs_tol=1e-9)
    assert commands[-1].is_zero


def test_follow_owner_handoff_does_not_publish_preset_motion() -> None:
    commands: list[TwistCommand] = []
    fake_time = FakeTime()
    def publish(command: TwistCommand) -> None:
        commands.append(command)

    config = _agv_config()
    limits = config["limits"]
    adapter = AgvMotionAdapter(
        publish_twist=publish,
        motion_groups=config["motion_groups"],
        action_motion_groups=config["action_motion_groups"],
        publish_rate_hz=config["publish_rate_hz"],
        max_linear_x=limits["max_linear_x"],
        max_linear_y=limits["max_linear_y"],
        max_angular_z=limits["max_angular_z"],
        stop_publish_count=3,
        should_stop=lambda: False,
        monotonic=fake_time.monotonic,
        sleep=fake_time.sleep,
    )

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_FOLLOW_OWNER"},
        ExecutionContext.from_goal("follow_owner", {}),
    )
    assert not [command for command in commands if not command.is_zero]
    assert commands[-3:] == [TwistCommand(), TwistCommand(), TwistCommand()]


def test_emergency_stop_only_publishes_zero() -> None:
    commands: list[TwistCommand] = []
    adapter = _adapter(commands, FakeTime())

    assert adapter.execute_step(
        {"unit_id": "ACT_SYSTEM_EMERGENCY_STOP"},
        ExecutionContext.from_goal("emergency_stop", {}),
    )
    assert commands == [TwistCommand(), TwistCommand(), TwistCommand()]


def test_sleeping_stage_forces_stationary_then_wakeup_reenables_motion() -> None:
    commands: list[TwistCommand] = []
    fake_time = FakeTime()
    adapter = _adapter(commands, fake_time)
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    executor = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes(),
        controller_adapters={"agv": adapter},
    )
    ctx = ExecutionContext.from_goal("sleepNow", {})
    ctx.resolved_behavior_name = "sleepNow"

    sleeping = executor.execute_stage(
        {
            "stage_id": "sleeping",
            "motion_state": "stationary",
            "selection_policy": "first",
            "required": True,
            "candidates": [{"unit_id": "ACT_FLIP_BODY"}],
        },
        ctx,
    )

    assert sleeping.success
    assert ctx.motion_state == "stationary"
    assert math.isclose(fake_time.now, 5.0, abs_tol=1e-9)
    assert commands
    assert all(command.is_zero for command in commands)

    commands.clear()
    wakeup = executor.execute_stage(
        {
            "stage_id": "wakeup",
            "selection_policy": "first",
            "required": True,
            "candidates": [{"unit_id": "ACT_GETUP_CRAWL"}],
        },
        ctx,
    )

    assert wakeup.success
    assert ctx.motion_state == "active"
    assert math.isclose(fake_time.now, 8.5, abs_tol=1e-9)
    assert any(not command.is_zero for command in commands)
    assert commands[-1].is_zero


def test_collision_reduced_emotion_motion_profiles_match_contract() -> None:
    config = _agv_config()
    groups = config["motion_groups"]
    mappings = config["action_motion_groups"]

    expected = {
        "ACT_CHECK_OWNER": (
            "strafe_osc_4s",
            [(0.0, 0.12, 0.0, 2.0), (0.0, -0.12, 0.0, 2.0)],
        ),
        "ACT_SLEEP_BY_FEET": (
            "strafe_osc_4s",
            [(0.0, 0.12, 0.0, 2.0), (0.0, -0.12, 0.0, 2.0)],
        ),
        "ACT_GUARD_DOOR": (
            "turn_left_4s",
            [(0.0, 0.0, 0.7, 4.0)],
        ),
        "ACT_YAWN": (
            "turn_right_4s",
            [(0.0, 0.0, -0.7, 4.0)],
        ),
        "ACT_STRETCH": (
            "strafe_left_right_each_1s",
            [(0.0, 0.12, 0.0, 1.0), (0.0, -0.12, 0.0, 1.0)],
        ),
        "ACT_PATROL": (
            "turn_left_right_each_1s",
            [(0.0, 0.0, 0.6, 1.0), (0.0, 0.0, -0.6, 1.0)],
        ),
        "ACT_SPLOOT": (
            "forward_backward_each_1s",
            [(0.1, 0.0, 0.0, 1.0), (-0.1, 0.0, 0.0, 1.0)],
        ),
    }

    for action, (expected_group, expected_segments) in expected.items():
        assert mappings[action] == expected_group
        actual_segments = [
            (
                segment.get("linear_x", 0.0),
                segment.get("linear_y", 0.0),
                segment.get("angular_z", 0.0),
                segment["duration_sec"],
            )
            for segment in groups[expected_group]
        ]
        assert actual_segments == expected_segments

    navigation = yaml.safe_load(
        (CONFIG_DIR / "navigation_waypoints.yaml").read_text(encoding="utf-8")
    )
    assert navigation["action_motion_groups"]["ACT_YAWN"] == "turn_right_4s"


def test_stage_executor_routes_composite_action_to_agv_adapter() -> None:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    calls: list[str] = []

    class RecordingAdapter:
        def execute_step(self, unit_config, ctx, duration) -> bool:
            del ctx, duration
            calls.append(unit_config["unit_id"])
            return True

    executor = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes(),
        controller_adapters={"agv": RecordingAdapter()},
    )
    ctx = ExecutionContext.from_goal("roll_over", {})
    result = executor.execute_stage(
        {
            "stage_id": "action",
            "selection_policy": "random_one",
            "required": True,
            "candidates": [{"unit_id": "ACT_TRICK_ROLL_OVER"}],
        },
        ctx,
        seed=1,
    )

    assert result.success
    assert result.unit_id == "ACT_TRICK_ROLL_OVER"
    assert calls == ["ACT_TRICK_ROLL_OVER"]
    assert ctx.executed_units == ["ACT_TRICK_ROLL_OVER"]
