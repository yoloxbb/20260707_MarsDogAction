"""Tests for the fail-closed Lite3 chassis backend."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from marsdog_action_executor.adapters.velocity import TwistCommand
from marsdog_action_executor.adapters.lite3_backend import (
    BLOCKED_BEHAVIOR_COMMANDS,
    CMD_MOONWALK,
    CMD_POSE_ROLL,
    CMD_SOFT_EMERGENCY_STOP,
    CMD_STAND_LIE_TOGGLE,
    CMD_SWITCH_IN_PLACE_MODE,
    CMD_SWITCH_JOYSTICK_MODE,
    CMD_SWITCH_MOVING_MODE,
    CMD_SWITCH_VISION_MODE,
    CMD_VOICE,
    Lite3ChassisBackend,
    Lite3RobotStatus,
    Lite3SimpleCommand,
    _ros2_control_conflicts,
)
from marsdog_action_executor.adapters.navigation_adapter import (
    BehaviorMobilityAdapter,
)
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.posture_manager import PostureManager
from marsdog_action_executor.stage_executor import StageExecutor
from marsdog_action_executor.units.unit_executors import AtomicActionExecutor


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


class FakeTime:
    def __init__(self) -> None:
        self.now = 0.0
        self.on_sleep = None

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration
        if self.on_sleep is not None:
            self.on_sleep(self.now)


class DeadlineCrossingTime(FakeTime):
    """Clock that advances between consecutive monotonic reads."""

    def __init__(self, advance_per_read: float = 0.01) -> None:
        super().__init__()
        self.advance_per_read = advance_per_read
        self.sleep_durations: list[float] = []

    def monotonic(self) -> float:
        current = self.now
        self.now += self.advance_per_read
        return current

    def sleep(self, duration: float) -> None:
        assert duration >= 0.0
        self.sleep_durations.append(duration)
        super().sleep(duration)


def _loader() -> ConfigLoader:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    return loader


def _status(
    basic: int = 6,
    *,
    gait: int = 0,
    motion: int = 0,
    battery: float = 80.0,
    forward: float = 3.0,
    backward: float = 3.0,
) -> Lite3RobotStatus:
    return Lite3RobotStatus(
        basic_state=basic,
        gait_state=gait,
        motion_state=motion,
        battery_level=battery,
        ultrasound_forward=forward,
        ultrasound_backward=backward,
    )


def _dog_that_stands_only_when_toggled(
    backend: Lite3ChassisBackend,
    commands: list[Lite3SimpleCommand],
    fake_time: FakeTime,
) -> None:
    """Feed status samples the way the real dog answers a posture toggle.

    The dog stays lying until the single directionless toggle arrives and only
    then reports standing.  A fake that stands up on *any* sleep is not a
    faithful stand-in: the mode switches inside ``_run_target_posture`` also
    sleep, and they would report the dog already standing before the toggle was
    ever sent.
    """
    toggle = Lite3SimpleCommand(CMD_STAND_LIE_TOGGLE)

    def on_sleep(_now: float) -> None:
        backend.update_status(
            _status(6) if toggle in commands else _status(1)
        )

    fake_time.on_sleep = on_sleep


def test_sleep_until_skips_an_elapsed_deadline() -> None:
    commands: list[Lite3SimpleCommand] = []
    fake_time = DeadlineCrossingTime()
    backend = _backend(commands, [], fake_time)

    fake_time.now = 1.01
    backend._sleep_until(1.0, 0.05)

    assert fake_time.sleep_durations == []


def test_sleep_until_never_passes_a_negative_duration_at_boundary() -> None:
    commands: list[Lite3SimpleCommand] = []
    fake_time = DeadlineCrossingTime(advance_per_read=0.02)
    backend = _backend(commands, [], fake_time)

    fake_time.now = 0.99
    backend._sleep_until(1.0, 0.05)

    assert fake_time.sleep_durations == [pytest.approx(0.01)]


def _backend(
    commands: list[Lite3SimpleCommand],
    twists: list[TwistCommand],
    fake_time: FakeTime,
    *,
    allow_proxies: bool = False,
    allow_unverified: bool = False,
    accepted_unverified_actions: list[str] | None = None,
    action_plans: dict | None = None,
    command_result: bool = True,
    control_conflict: str = "",
) -> Lite3ChassisBackend:
    loader = _loader()
    config = loader.lite3_action_config
    limits = config["limits"]
    return Lite3ChassisBackend(
        command_publisher=lambda command: commands.append(command)
        or command_result,
        twist_publisher=twists.append,
        action_plans=action_plans or loader.get_lite3_action_plans(),
        allow_proxies=allow_proxies,
        allow_unverified=allow_unverified,
        accepted_unverified_actions=accepted_unverified_actions or [],
        status_timeout_sec=config["status_timeout_sec"],
        min_battery_percent=config["min_battery_percent"],
        minimum_backward_clearance_m=config[
            "minimum_backward_clearance_m"
        ],
        mode_settle_sec=config["mode_settle_sec"],
        navigation_settle_timeout_sec=config[
            "navigation_settle_timeout_sec"
        ],
        navigation_stable_samples=config["navigation_stable_samples"],
        posture_recovery_timeout_sec=config["posture_recovery_timeout_sec"],
        publish_rate_hz=config["publish_rate_hz"],
        max_linear_x=limits["max_linear_x"],
        max_linear_y=limits["max_linear_y"],
        max_angular_z=limits["max_angular_z"],
        stop_publish_count=config["stop_publish_count"],
        command_repeat=config["command_repeat"],
        command_repeat_interval_sec=config["command_repeat_interval_sec"],
        control_ownership_checker=lambda: control_conflict,
        monotonic=fake_time.monotonic,
        sleep=fake_time.sleep,
    )


def test_effective_lite3_routes_fail_closed() -> None:
    loader = _loader()
    routes = loader.get_controller_routes("lite3")

    assert routes["ACT_BASIC_STAND"] == "lite3"
    assert routes["ACT_BASIC_BACK_UP"] == "lite3"
    assert routes["ACT_BASIC_SIT"] == "lite3"
    assert routes["ACT_OBJECT_DROP"] == "lite3"
    assert routes["ACT_OBJECT_BRING"] == "lite3"
    assert routes["ACT_OBJECT_FETCH"] == "lite3"
    assert routes["ACT_INTERACT_APPROACH_OWNER"] == "person_nav_approach"
    assert routes["ACT_INTERACT_APPROACH_OWNER_CLOSER"] == "person_nav_approach"
    assert routes["ACT_INTERACT_RETURN_OWNER"] == "person_nav_approach"
    assert routes["ACT_INTERACT_FOLLOW_OWNER"] == "uwb_follow"
    assert routes["ACT_NAV_WALK_RANDOM"] == "behavior_mobility"
    assert "agv" not in routes.values()
    assert "perception_navigation_manipulation" not in routes.values()
    # 太空步 is deliberately unmapped since 2026-09-18 (unbounded gait, see
    # BLOCKED_BEHAVIOR_COMMANDS).  Every other unit must still resolve.
    assert {
        unit_id
        for unit_id in loader.action_catalog
        if routes.get(unit_id, routes["_default"]) == "unsupported"
    } == {"ACT_BONE_DANCE", "ACT_JUMP_PAW"}


def test_full_proxy_mapping_covers_every_behavior_stage() -> None:
    loader = _loader()
    routes = loader.get_controller_routes("lite3")
    plans = loader.get_lite3_action_plans()
    external_routes = {
        "behavior_mobility",
        "mock",
        "target_approach",
        "person_nav_approach",
        "uwb_follow",
        "uwb_roam",
        "visual_target_approach",
        "wake_orientation",
    }
    unsupported_behaviors: set[str] = set()
    for behavior_name, behavior in loader.behavior_tree_templates.items():
        for stage in behavior["stages"]:
            candidates = [
                candidate["unit_id"]
                for candidate in stage.get("candidates", [])
            ]
            if not any(
                routes.get(unit_id) in external_routes
                or (routes.get(unit_id) == "lite3" and unit_id in plans)
                for unit_id in candidates
            ):
                unsupported_behaviors.add(behavior_name)

    assert unsupported_behaviors == set()
    # 190, not 192: ACT_BONE_DANCE and ACT_JUMP_PAW were removed on 2026-09-18
    # because 太空步 is an unbounded gait.  No behavior stage depended on them
    # as its only candidate, which is why the coverage assertion above still
    # holds.
    # 169 = 191 + ACT_TWIST_WAIST_LEFT_RIGHT (the eating ritual's new 扭腰)
    # - the 23 eating-only units the ritual rewrite orphaned.  The coverage
    # assertion above still holds because the ritual's six stages are all
    # already-backed candidates: 低头 (the pitch anchor), 抬头, 趴下, 站立
    # and the new 扭腰.
    assert len(plans) == 169


def test_lite3_plans_use_physical_motion_except_true_hold_semantics() -> None:
    loader = _loader()
    plans = loader.get_lite3_action_plans()
    hold_only = {
        unit_id
        for unit_id, plan in plans.items()
        if all(step.get("type") == "hold" for step in plan["sequence"])
    }
    assert hold_only == {
        "ACT_BASIC_WAIT",
        "ACT_CONTROL_HOLD_POSITION",
        "ACT_CONTROL_HOLD_STANDING",
        "ACT_PERCEPTION_RESPOND_PERSON_FALL",
        "ACT_PERCEPTION_RESPOND_STOP_GESTURE",
        "ACT_SYSTEM_EMERGENCY_STOP",
    }
    assert plans["ACT_BASIC_LIE_DOWN"]["sequence"] == [
        {
            "type": "target_posture",
            "target": "lie",
            "completion_timeout_sec": 4.5,
            "stable_samples": 5,
        }
    ]
    assert plans["ACT_BASIC_STAND"]["sequence"] == [
        {
            "type": "target_posture",
            "target": "stand",
            "completion_timeout_sec": 4.5,
            "stable_samples": 5,
        }
    ]
    assert all(
        plan.get("semantic_effect") == "proxy_motion"
        for plan in plans.values()
        if plan.get("fidelity") == "proxy"
    )


def test_all_unverified_actions_resolve_to_an_accepted_physical_plan() -> None:
    loader = _loader()
    plans = loader.get_lite3_action_plans()
    config = loader.lite3_action_config
    accepted = config["accepted_unverified_actions"]
    backend = _backend(
        [],
        [],
        FakeTime(),
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=accepted,
    )

    unverified = {
        unit_id
        for unit_id, plan in plans.items()
        if not plan.get("verified", False)
    }
    # 181, not 183: the two moonwalk units were removed on 2026-09-18.
    # 160 = 182 + ACT_TWIST_WAIST_LEFT_RIGHT - the 23 orphaned eating units.
    assert len(unverified) == 160
    assert all(backend.can_execute(unit_id) for unit_id in unverified)


def test_config_contains_no_blocked_behavior_command() -> None:
    plans = _loader().get_lite3_action_plans()
    codes = {
        int(step["cmd_code"])
        for plan in plans.values()
        for step in plan["sequence"]
        if "cmd_code" in step
    }
    assert not (codes & BLOCKED_BEHAVIOR_COMMANDS)


def test_moonwalk_is_unreachable() -> None:
    """太空步 is an unbounded gait, so no unit may resolve to it at all.

    Measured on the real dog: 0x2101030C puts the chassis into the moonwalk
    gait and it keeps stepping until something explicitly stops it, so
    ``gait_state`` never returns to 0 and every terminal-state contract times
    out.  The dog ended in 失控保护 and then sat with gait_state=12 stuck, which
    makes the backend reject *every* later action as ``lite3_gait_busy``.
    """
    commands: list[Lite3SimpleCommand] = []
    backend = _backend(
        commands,
        [],
        FakeTime(),
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_BONE_DANCE", "ACT_JUMP_PAW"],
    )
    backend.update_status(_status(6))
    for unit_id in ("ACT_BONE_DANCE", "ACT_JUMP_PAW"):
        assert not backend.execute_step(
            {"unit_id": unit_id},
            ExecutionContext.from_goal("dance", {}),
            5.0,
        )
        assert backend.last_error == f"lite3_action_unmapped:unit_id={unit_id}"
    assert Lite3SimpleCommand(CMD_MOONWALK) not in commands


def test_moonwalk_is_refused_even_if_a_plan_asks_for_it() -> None:
    """Defence in depth: the blocklist holds even for an injected plan."""
    commands: list[Lite3SimpleCommand] = []
    backend = _backend(
        commands,
        [],
        FakeTime(),
        allow_unverified=True,
        accepted_unverified_actions=["ACT_TEST"],
        action_plans={
            "ACT_TEST": {
                "fidelity": "exact",
                "verified": True,
                "sequence": [{"type": "simple", "cmd_code": CMD_MOONWALK}],
            }
        },
    )
    backend.update_status(_status(6))
    assert not backend.execute_step(
        {"unit_id": "ACT_TEST"},
        ExecutionContext.from_goal("dance", {}),
        5.0,
    )
    # A blocked command used to return False without recording anything, which
    # the caller then reported as the useless "adapter returned failure".
    assert backend.last_error == "lite3_command_blocked:cmd_code=0x2101030C"
    assert not commands


def test_directionless_posture_toggle_is_blocked_as_a_raw_simple_step() -> None:
    commands: list[Lite3SimpleCommand] = []
    backend = _backend(
        commands,
        [],
        FakeTime(),
        action_plans={
            "ACT_TEST": {
                "fidelity": "exact",
                "verified": True,
                "sequence": [
                    {
                        "type": "simple",
                        "cmd_code": CMD_STAND_LIE_TOGGLE,
                    }
                ],
            }
        },
    )
    backend.update_status(_status(6))

    assert not backend.execute_step(
        {"unit_id": "ACT_TEST"},
        ExecutionContext.from_goal("stand_up", {}),
        2.0,
    )
    assert Lite3SimpleCommand(CMD_STAND_LIE_TOGGLE) not in commands


def test_proxy_and_unverified_actions_are_gated_independently() -> None:
    backend = _backend([], [], FakeTime())
    assert backend.can_execute("ACT_BASIC_BACK_UP")
    assert not backend.can_execute("ACT_BASIC_STAND")
    assert not backend.can_execute("ACT_INTERACT_GIVE_PAW")

    proxies = _backend([], [], FakeTime(), allow_proxies=True)
    assert not proxies.can_execute("ACT_INTERACT_GIVE_PAW")
    assert not proxies.can_execute("ACT_OWNER_UNHAPPY")

    accepted_shared_plan = _backend(
        [],
        [],
        FakeTime(),
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_INTERACT_GIVE_PAW"],
    )
    assert accepted_shared_plan.can_execute("ACT_INTERACT_GIVE_PAW")

    unverified_plan = {
        "ACT_TEST": {
            "fidelity": "exact",
            "verified": False,
            "sequence": [{"type": "hold", "duration_sec": 0.1}],
        }
    }
    gated = _backend(
        [], [], FakeTime(), allow_unverified=True,
        action_plans=unverified_plan,
    )
    accepted = _backend(
        [], [], FakeTime(), allow_unverified=True,
        accepted_unverified_actions=["ACT_TEST"],
        action_plans=unverified_plan,
    )
    assert not gated.can_execute("ACT_TEST")
    assert accepted.can_execute("ACT_TEST")


def test_target_stand_is_idempotent_when_already_standing() -> None:
    commands: list[Lite3SimpleCommand] = []
    twists: list[TwistCommand] = []
    fake_time = FakeTime()
    backend = _backend(
        commands,
        twists,
        fake_time,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_BASIC_STAND"],
    )
    backend.update_status(_status(6))

    assert backend.execute_step(
        {"unit_id": "ACT_BASIC_STAND"},
        ExecutionContext.from_goal("stand_up", {}),
        5.0,
    )
    assert commands == []
    assert twists[-5:] == [TwistCommand()] * 5


def test_target_stand_sends_one_toggle_and_confirms_stable_terminal() -> None:
    commands: list[Lite3SimpleCommand] = []
    fake_time = FakeTime()
    backend = _backend(
        commands,
        [],
        fake_time,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_BASIC_STAND"],
    )
    backend.update_status(_status(1))
    states = iter([4, 5, 6, 6, 6, 6, 6])

    def update_transition(_now: float) -> None:
        if Lite3SimpleCommand(CMD_STAND_LIE_TOGGLE) in commands:
            backend.update_status(_status(next(states, 6)))
        else:
            backend.update_status(_status(1))

    fake_time.on_sleep = update_transition
    ctx = ExecutionContext.from_goal("stand_up", {})

    assert backend.execute_step(
        {"unit_id": "ACT_BASIC_STAND"},
        ctx,
        5.0,
    )
    assert commands.count(Lite3SimpleCommand(CMD_STAND_LIE_TOGGLE)) == 1
    assert ctx.metadata["lite3_action"]["physical_posture"] == "standing"


def test_target_lie_sends_one_toggle_and_confirms_stable_terminal() -> None:
    commands: list[Lite3SimpleCommand] = []
    fake_time = FakeTime()
    backend = _backend(
        commands,
        [],
        fake_time,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_BASIC_LIE_DOWN"],
    )
    backend.update_status(_status(6))
    states = iter([7, 1, 1, 1, 1, 1])

    def update_transition(_now: float) -> None:
        if Lite3SimpleCommand(CMD_STAND_LIE_TOGGLE) in commands:
            backend.update_status(_status(next(states, 1)))
        else:
            backend.update_status(_status(6))

    fake_time.on_sleep = update_transition
    ctx = ExecutionContext.from_goal("lie_down", {})

    assert backend.execute_step(
        {"unit_id": "ACT_BASIC_LIE_DOWN"},
        ctx,
        5.0,
    )
    assert commands.count(Lite3SimpleCommand(CMD_STAND_LIE_TOGGLE)) == 1
    assert ctx.metadata["lite3_action"]["physical_posture"] == "lying"


def test_target_posture_timeout_never_retries_directionless_toggle() -> None:
    commands: list[Lite3SimpleCommand] = []
    fake_time = FakeTime()
    backend = _backend(
        commands,
        [],
        fake_time,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_BASIC_LIE_DOWN"],
    )
    backend.update_status(_status(6))

    def remain_standing(_now: float) -> None:
        backend.update_status(_status(6))

    fake_time.on_sleep = remain_standing

    assert not backend.execute_step(
        {"unit_id": "ACT_BASIC_LIE_DOWN"},
        ExecutionContext.from_goal("lie_down", {}),
        5.0,
    )
    assert commands.count(Lite3SimpleCommand(CMD_STAND_LIE_TOGGLE)) == 1
    assert backend.last_error.startswith(
        "lite3_posture_transition_timeout:target=lie"
    )


def test_simple_action_rejects_missing_stale_or_low_battery_status() -> None:
    commands: list[Lite3SimpleCommand] = []
    backend = _backend(
        commands,
        [],
        FakeTime(),
        allow_unverified=True,
        accepted_unverified_actions=["ACT_BASIC_STAND"],
    )
    ctx = ExecutionContext.from_goal("stand_up", {})

    assert not backend.execute_step({"unit_id": "ACT_BASIC_STAND"}, ctx, 5.0)
    assert commands == []
    assert backend.last_error == "lite3_status_missing_or_stale"

    backend.update_status(_status(1, battery=10.0))
    assert not backend.execute_step({"unit_id": "ACT_BASIC_STAND"}, ctx, 5.0)
    assert commands == []
    assert backend.last_error == (
        "lite3_low_battery:battery=10.0,minimum=25.0"
    )


def test_rejected_simple_action_never_reaches_the_motion_host() -> None:
    commands: list[Lite3SimpleCommand] = []
    backend = _backend(
        commands,
        [],
        FakeTime(),
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_INTERACT_GIVE_PAW"],
    )
    backend.update_status(_status(1, motion=1))

    assert not backend.execute_step(
        {"unit_id": "ACT_INTERACT_GIVE_PAW"},
        ExecutionContext.from_goal("give_paw", {}),
        5.0,
    )
    # A control-mode switch is itself a command, so a rejection must send
    # nothing at all -- not even the Joystick-mode handshake.
    assert commands == []
    assert backend.last_error == "lite3_motion_busy:motion_state=1"


def test_terminal_wait_timeout_reports_the_observed_status() -> None:
    """A step that never reaches its terminal state names the status it saw.

    Uses the twist-body branch of ACT_INTERACT_GIVE_PAW (0x21010204) driven from
    a standing posture, so the plan's hello step is filtered out by
    ``when_initial_basic_states``.
    """
    commands: list[Lite3SimpleCommand] = []
    fake_time = FakeTime()
    backend = _backend(
        commands,
        [],
        fake_time,
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_INTERACT_GIVE_PAW"],
    )
    backend.update_status(_status(6))

    def never_settles(_now: float) -> None:
        if Lite3SimpleCommand(0x21010204) in commands:
            backend.update_status(_status(6, gait=0, motion=2))
        else:
            backend.update_status(_status(6))

    fake_time.on_sleep = never_settles

    assert not backend.execute_step(
        {"unit_id": "ACT_INTERACT_GIVE_PAW"},
        ExecutionContext.from_goal("express_excitement", {}),
        15.0,
    )
    assert Lite3SimpleCommand(0x21010204) in commands
    assert backend.last_error == (
        "lite3_action_terminal_timeout:cmd_code=0x21010204,"
        "observed_active=True,"
        "last_status=basic_state=6,gait_state=0,motion_state=2"
    )


def test_control_mode_switch_failure_names_the_command() -> None:
    commands: list[Lite3SimpleCommand] = []
    backend = _backend(
        commands,
        [],
        FakeTime(),
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_INTERACT_GIVE_PAW"],
        command_result=False,
    )
    backend.update_status(_status(6))

    assert not backend.execute_step(
        {"unit_id": "ACT_INTERACT_GIVE_PAW"},
        ExecutionContext.from_goal("give_paw", {}),
        5.0,
    )
    # A refused control-mode switch used to leave last_error empty, which the
    # unit executor then reported as the useless "adapter returned failure".
    assert backend.last_error == (
        "lite3_joystick_mode_switch_failed:cmd_code=0x21000C02"
    )


def test_terminal_wait_names_an_unobserved_active_state() -> None:
    commands: list[Lite3SimpleCommand] = []
    fake_time = FakeTime()
    backend = _backend(
        commands,
        [],
        fake_time,
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_INTERACT_GIVE_PAW"],
    )
    backend.update_status(_status(6))

    # The dog moves and settles, but never reports motion_state=2, so the
    # declared active-state set is never satisfied.
    fake_time.on_sleep = lambda _now: backend.update_status(_status(6))

    assert not backend.execute_step(
        {"unit_id": "ACT_INTERACT_GIVE_PAW"},
        ExecutionContext.from_goal("give_paw", {}),
        5.0,
    )
    assert Lite3SimpleCommand(0x21010204) in commands
    assert backend.last_error == (
        "lite3_action_terminal_active_state_unobserved:"
        "cmd_code=0x21010204,"
        "last_status=basic_state=6,gait_state=0,motion_state=0"
    )


def test_atomic_executor_propagates_lite3_rejection_reason() -> None:
    backend = _backend([], [], FakeTime())
    backend.update_status(_status(6, motion=1))
    ctx = ExecutionContext.from_goal("wait", {})

    result = AtomicActionExecutor(adapter=backend).execute(
        {"unit_id": "ACT_BASIC_WAIT", "timeout_sec": 1.0},
        ctx,
    )

    assert result.message == "lite3_motion_busy:motion_state=1"


def test_social_proxy_uses_tested_twist_body_when_initially_standing() -> None:
    commands: list[Lite3SimpleCommand] = []
    fake_time = FakeTime()
    twists: list[TwistCommand] = []
    backend = _backend(
        commands,
        twists,
        fake_time,
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_INTERACT_GIVE_PAW"],
    )
    backend.update_status(_status(6))

    def complete_twist_body(now: float) -> None:
        if now >= 0.18:
            backend.update_status(_status(6, motion=0))
        elif now >= 0.12:
            backend.update_status(_status(6, motion=2))
        else:
            backend.update_status(_status(6, motion=0))

    fake_time.on_sleep = complete_twist_body
    ctx = ExecutionContext.from_goal("give_paw", {})
    assert backend.execute_step(
        {"unit_id": "ACT_INTERACT_GIVE_PAW"},
        ctx,
        5.0,
    )
    assert Lite3SimpleCommand(0x21010204) in commands
    assert Lite3SimpleCommand(0x21010507) not in commands
    assert twists
    assert ctx.metadata["lite3_action"]["executed_plan"] == (
        "ACT_INTERACT_GIVE_PAW"
    )
    assert ctx.metadata["lite3_action"]["semantic_effect"] == "proxy_motion"
    assert ctx.metadata["lite3_action"]["physical_posture"] == "standing"


def test_social_proxy_uses_native_hello_when_initially_lying() -> None:
    commands: list[Lite3SimpleCommand] = []
    fake_time = FakeTime()
    backend = _backend(
        commands,
        [],
        fake_time,
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_INTERACT_GIVE_PAW"],
    )
    backend.update_status(_status(1))

    def complete_hello(now: float) -> None:
        if now >= 0.18:
            backend.update_status(_status(6))
        elif now >= 0.12:
            backend.update_status(_status(20))
        else:
            backend.update_status(_status(1))

    fake_time.on_sleep = complete_hello
    ctx = ExecutionContext.from_goal("give_paw", {})

    assert backend.execute_step(
        {"unit_id": "ACT_INTERACT_GIVE_PAW"},
        ctx,
        5.0,
    )
    assert Lite3SimpleCommand(0x21010507) in commands
    assert Lite3SimpleCommand(0x21010204) not in commands
    assert ctx.metadata["lite3_action"]["physical_posture"] == "standing"


def test_lying_dog_is_stood_up_before_a_standing_pose_step() -> None:
    """A lying dog must not wedge every later action.

    平静行为 picks one candidate at random: ACT_SPLOOT lies the dog down, the
    other four (ACT_GUARD_DOOR / ACT_YAWN / ACT_STRETCH / ACT_PATROL) need a
    standing dog.  Without entry-time recovery the dog stayed down and all four
    failed with ``lite3_posture_mismatch`` forever, because no step anywhere
    stood it back up.
    """
    commands: list[Lite3SimpleCommand] = []
    fake_time = FakeTime()
    backend = _backend(
        commands,
        [],
        fake_time,
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_GUARD_DOOR", "ACT_STRETCH"],
    )
    backend.update_status(_status(1))
    # The real dog answers the single directionless toggle by standing up.
    _dog_that_stands_only_when_toggled(backend, commands, fake_time)
    ctx = ExecutionContext.from_goal("calm", {})

    assert backend.execute_step({"unit_id": "ACT_GUARD_DOOR"}, ctx, 6.0)
    assert Lite3SimpleCommand(CMD_STAND_LIE_TOGGLE) in commands
    assert commands.index(Lite3SimpleCommand(CMD_STAND_LIE_TOGGLE)) < commands.index(
        Lite3SimpleCommand(0x21010135, 10500)
    )
    assert ctx.metadata["lite3_action"]["physical_posture"] == "standing"

    # The same recovery has to happen for every calm candidate that needs a
    # standing dog, not just the first one.
    commands.clear()
    backend.update_status(_status(1))
    assert backend.execute_step({"unit_id": "ACT_STRETCH"}, ctx, 6.0)
    assert Lite3SimpleCommand(CMD_STAND_LIE_TOGGLE) in commands
    assert Lite3SimpleCommand(0x21010102, -21000) in commands


def test_standing_dog_is_never_toggled_by_the_recovery_path() -> None:
    commands: list[Lite3SimpleCommand] = []
    fake_time = FakeTime()
    backend = _backend(
        commands,
        [],
        fake_time,
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_GUARD_DOOR"],
    )
    backend.update_status(_status(6))
    fake_time.on_sleep = lambda _now: backend.update_status(_status(6))
    ctx = ExecutionContext.from_goal("calm", {})

    assert backend.execute_step({"unit_id": "ACT_GUARD_DOOR"}, ctx, 6.0)
    assert Lite3SimpleCommand(CMD_STAND_LIE_TOGGLE) not in commands


def test_recovery_does_not_apply_to_the_opposite_posture_requirement() -> None:
    """Nothing else may be silently repaired by the recovery path."""
    fake_time = FakeTime()
    commands: list[Lite3SimpleCommand] = []
    # The mirror image is *not* repaired: a standing dog asked for ``lie`` is
    # still rejected, because only ``stand`` needs the dog's posture changed
    # for it and only that direction has a recovery path.
    backend = _backend(
        commands,
        [],
        fake_time,
        action_plans={
            "ACT_TEST_REQUIRE_LIE": {
                "fidelity": "exact",
                "verified": True,
                "sequence": [
                    {"type": "hold", "require": "lie", "duration_sec": 0.1}
                ],
            }
        },
    )
    backend.update_status(_status(6))
    fake_time.on_sleep = lambda _now: backend.update_status(_status(6))

    assert not backend.execute_step(
        {"unit_id": "ACT_TEST_REQUIRE_LIE"},
        ExecutionContext.from_goal("x", {}),
        6.0,
    )
    assert backend.last_error.startswith("lite3_posture_mismatch:")
    assert commands == []


def test_recovery_does_not_mask_other_precondition_failures() -> None:
    """A non-posture reason is reported, not papered over with a stand-up."""
    fake_time = FakeTime()
    commands: list[Lite3SimpleCommand] = []
    backend = _backend(
        commands,
        [],
        fake_time,
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_GUARD_DOOR"],
    )
    # Lying and asking for standing, but the battery is below the floor: the
    # battery is checked first and must be what the operator sees.
    backend.update_status(_status(1, battery=10.0))
    _dog_that_stands_only_when_toggled(backend, commands, fake_time)

    assert not backend.execute_step(
        {"unit_id": "ACT_GUARD_DOOR"}, ExecutionContext.from_goal("calm", {}), 6.0
    )
    assert backend.last_error.startswith("lite3_low_battery:")
    assert commands == []


def test_recovery_is_bounded_by_the_step_deadline() -> None:
    commands: list[Lite3SimpleCommand] = []
    fake_time = FakeTime()
    backend = _backend(
        commands,
        [],
        fake_time,
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_GUARD_DOOR"],
    )
    backend.update_status(_status(1))
    # The dog never stands up, so the recovered step must give up inside its
    # own budget instead of running until the recovery timeout.
    fake_time.on_sleep = lambda _now: backend.update_status(_status(1))
    ctx = ExecutionContext.from_goal("calm", {})

    assert not backend.execute_step({"unit_id": "ACT_GUARD_DOOR"}, ctx, 0.5)
    assert backend.last_error.startswith("lite3_posture_transition_timeout:")
    assert Lite3SimpleCommand(0x21010135, 10500) not in commands


def test_lying_dog_is_stood_up_before_a_twist_step() -> None:
    commands: list[Lite3SimpleCommand] = []
    twists: list[TwistCommand] = []
    fake_time = FakeTime()
    backend = _backend(commands, twists, fake_time)
    backend.update_status(_status(1))
    _dog_that_stands_only_when_toggled(backend, commands, fake_time)
    ctx = ExecutionContext.from_goal("back_up", {})

    assert backend.execute_step({"unit_id": "ACT_BASIC_BACK_UP"}, ctx, 8.0)
    assert Lite3SimpleCommand(CMD_STAND_LIE_TOGGLE) in commands
    assert any(twist.linear_x < 0.0 for twist in twists)


def test_lying_dog_is_stood_up_before_the_navigation_preflight() -> None:
    """The random *navigation* route needs the same entry-time recovery.

    expressCalmAlone is in ``random_navigation_behaviors``, so with
    ``navigation_enabled:=true`` the behavior is decided at
    ``prepare_navigation`` -- before any step runs.  Recovery at the step
    entries alone therefore never fired on this route: the stage failed at the
    preflight with ``lite3_posture_mismatch`` and left the dog down for good,
    re-failing the same way every time the behavior was selected.
    """
    commands: list[Lite3SimpleCommand] = []
    fake_time = FakeTime()
    backend = _backend(commands, [], fake_time)
    backend.update_status(_status(1))
    _dog_that_stands_only_when_toggled(backend, commands, fake_time)

    assert backend.prepare_navigation()
    assert Lite3SimpleCommand(CMD_STAND_LIE_TOGGLE) in commands
    # The dog is up before the chassis is handed to Nav2, not after.
    assert commands.index(
        Lite3SimpleCommand(CMD_STAND_LIE_TOGGLE)
    ) < commands.index(Lite3SimpleCommand(CMD_SWITCH_VISION_MODE))
    assert backend.last_error == ""


def test_follow_preflight_also_stands_a_lying_dog_up() -> None:
    """The UWB chain must not turn a lying dog into a permanent refusal.

    The recovery reads chassis status only; ownership is the *preflight's*
    job, and the follow is the one caller allowed to share the chassis with
    that chain.
    """
    commands: list[Lite3SimpleCommand] = []
    fake_time = FakeTime()
    backend = _backend(
        commands,
        [],
        fake_time,
        control_conflict="active_nodes=/uwb_behavior_controller_node=1",
    )
    backend.update_status(_status(1))
    _dog_that_stands_only_when_toggled(backend, commands, fake_time)

    assert backend.prepare_navigation(allow_uwb_chain=True)
    assert Lite3SimpleCommand(CMD_STAND_LIE_TOGGLE) in commands
    assert backend.last_error == ""


def test_raw_motion_check_stays_strict_for_a_lying_dog() -> None:
    """Recovery lives at the entry points, never inside ``_motion_ready``.

    That check is shared with ``publish_velocity``'s per-tick path, where a
    posture change in the middle of an action must abort it instead of
    standing the dog up mid-motion.  This is the line the navigation fix
    deliberately did *not* cross -- and the exact reason the preflight failed
    in the first place.
    """
    commands: list[Lite3SimpleCommand] = []
    backend = _backend(commands, [], FakeTime())
    backend.update_status(_status(1))

    assert not backend._motion_ready()
    assert backend.last_error == (
        "lite3_posture_mismatch:require=stand,basic_state=1"
    )


def test_sleep_circle_is_a_bounded_360_degree_turn() -> None:
    loader = _loader()
    plan = loader.get_lite3_action_plans()["ACT_CIRCLE_AROUND"]
    assert plan["fidelity"] == "proxy"
    assert plan["verified"] is False
    sequence = plan["sequence"]
    assert len(sequence) == 1

    step = sequence[0]
    assert step["type"] == "twist"
    # In place: the turn must not also translate the chassis.
    assert step["linear_x"] == 0.0
    assert step["linear_y"] == 0.0
    full_turn_sec = 2 * math.pi / abs(step["angular_z"])
    assert step["duration_sec"] == pytest.approx(full_turn_sec, abs=0.1)
    # The unit budget is the deadline the twist runs under.  Anything below the
    # turn duration would clip the circle short and leave the dog part-turned.
    assert loader.action_catalog["ACT_CIRCLE_AROUND"]["timeout_sec"] > full_turn_sec


def test_sleep_behaviors_circle_in_place_before_lying_down() -> None:
    loader = _loader()
    for behavior_name in ("sleepOnSide", "sleepNow"):
        stages = loader.behavior_tree_templates[behavior_name]["stages"]
        orders = {stage["stage_id"]: stage["order"] for stage in stages}
        # 到点 -> 原地转一圈 -> 趴下: the circle runs first and on its own,
        # so it cannot be skipped by the random pick in ``prepare``.
        assert orders["circle"] == 1
        assert min(orders.values()) == 1
        assert orders["circle"] < orders["prepare"]
        assert orders["circle"] < orders["sleep_pose"]
        circle = next(s for s in stages if s["stage_id"] == "circle")
        assert [c["unit_id"] for c in circle["candidates"]] == [
            "ACT_CIRCLE_AROUND"
        ]
        prepare = next(s for s in stages if s["stage_id"] == "prepare")
        assert "ACT_CIRCLE_AROUND" not in {
            candidate["unit_id"] for candidate in prepare["candidates"]
        }


def test_sleep_behaviors_are_registered_with_the_circle_stage() -> None:
    """The navigation route lists the stages that actually run."""
    loader = _loader()
    routes = loader.navigation_config["behavior_routes"]
    for behavior_name in ("sleepOnSide", "sleepNow"):
        assert routes[behavior_name]["stages"] == [
            "circle",
            "prepare",
            "sleep_pose",
            "sleeping",
            "wakeup",
        ]


def test_toilet_circle_is_a_bounded_three_lap_walk_around() -> None:
    loader = _loader()
    plan = loader.get_lite3_action_plans()["ACT_SNIFF_AND_CIRCLE_AT_TOILET_SPOT"]
    sequence = plan["sequence"]
    assert len(sequence) == 1

    step = sequence[0]
    assert step["type"] == "twist"
    # 绕点走圈, not a spin in place: the forward channel is what makes it an
    # orbit rather than a pirouette.
    assert step["linear_x"] > 0.0
    assert step["angular_z"] > 0.0
    laps = abs(step["angular_z"]) * step["duration_sec"] / (2 * math.pi)
    assert laps == pytest.approx(3.0, abs=0.05)
    # The orbit radius is linear_x / angular_z.  It was 0.5 m / 47.1 s until
    # 2026-09-18, when the operator asked for 3x the lap rate (1.20 rad/s) and a
    # 0.2 m radius; 0.24 m/s is what 0.2 m * 1.20 rad/s comes to.
    assert step["linear_x"] / step["angular_z"] == pytest.approx(0.2, abs=0.01)
    assert step["duration_sec"] == pytest.approx(3 * 2 * math.pi / 1.20, abs=0.05)
    # Forward motion is guarded for the whole arc: the dog sweeps every heading
    # while orbiting, so the backward-clearance default does not cover it.
    assert step["minimum_forward_clearance_m"] > 0.0
    assert (
        loader.action_catalog["ACT_SNIFF_AND_CIRCLE_AT_TOILET_SPOT"]["timeout_sec"]
        > step["duration_sec"]
    )


def test_no_lite3_twist_plan_is_above_the_velocity_clamp() -> None:
    """A plan above ``limits`` is clamped, not rejected.

    ``publish_velocity`` clamps every Twist to ``limits``, so an oversized plan
    does not fail -- it silently runs slower, and for the curved 上厕所 orbit
    that changes the radius and the lap time.  Nothing else catches that, so
    every twist segment is checked against the same numbers the clamp uses.
    """
    loader = _loader()
    limits = loader.lite3_action_config["limits"]
    checked = 0
    for plan_name, plan in loader.get_lite3_action_plans().items():
        for step in plan.get("sequence", []):
            if step.get("type") != "twist":
                continue
            checked += 1
            assert abs(step.get("linear_x", 0.0)) <= limits["max_linear_x"], (
                f"{plan_name} linear_x exceeds the clamp"
            )
            assert abs(step.get("linear_y", 0.0)) <= limits["max_linear_y"], (
                f"{plan_name} linear_y exceeds the clamp"
            )
            assert abs(step.get("angular_z", 0.0)) <= limits["max_angular_z"], (
                f"{plan_name} angular_z exceeds the clamp"
            )
    assert checked > 0


def test_launch_lite3_clamps_match_this_file() -> None:
    """The launch default wins over the config, so the two must not drift.

    ``ros_node`` declares ``lite3_max_linear_x`` / ``lite3_max_angular_z`` from
    the config ``limits``, but the launch file always passes an explicit value,
    and the parameter overrides the declaration.  If only one file is updated
    the backend silently clamps to the older number.
    """
    loader = _loader()
    limits = loader.lite3_action_config["limits"]
    launch_text = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "action_executor.launch.py"
    ).read_text(encoding="utf-8")
    for argument, key in (
        ("lite3_max_linear_x", "max_linear_x"),
        ("lite3_max_linear_y", "max_linear_y"),
        ("lite3_max_angular_z", "max_angular_z"),
    ):
        marker = f'"{argument}",\n                default_value="'
        start = launch_text.index(marker) + len(marker)
        default = float(launch_text[start : launch_text.index('"', start)])
        assert default == pytest.approx(float(limits[key])), (
            f"{argument} default {default} != limits.{key} {limits[key]}"
        )


def test_raise_head_is_the_mirror_of_the_head_down_proxies() -> None:
    loader = _loader()
    plans = loader.get_lite3_action_plans()
    raise_step = plans["ACT_RAISE_HEAD"]["sequence"][0]
    # The reference is the 低头 anchor of the pitch family, not a family
    # member: the eating rewrite deleted ACT_LOWER_HEAD_AND_APPROACH_BOWL
    # (the old reference here) while the anchor itself survives -- the ritual
    # uses it for both of its 低头 steps, and twelve other units point at it
    # through ``plan_id``.
    lower_step = plans["ACT_SNIFF_BOWL_AND_WAIT_FOR_FOOD"]["sequence"][0]

    # Protocol §1.2.3.1: 0x21010130 is the pitch axis and positive 取正值时低头.
    assert raise_step["cmd_code"] == 0x21010130
    assert lower_step["cmd_code"] == 0x21010130
    assert lower_step["value"] > 0
    assert raise_step["value"] < 0
    assert abs(raise_step["value"]) == lower_step["value"]
    # The axis ignores |value| <= 6553, so being merely outside the dead zone
    # is not enough: at 7500 -- 947 above the floor, the same margin the
    # clearly visible 偏航 10500 has -- the operator could not see either
    # direction on the real dog.  This holds a floor a long way off the dead
    # zone; it is a floor on the number, not a claim that stroke position
    # predicts visibility.
    dead_zone, full_scale = 6553, 32767
    stroke_position = (abs(raise_step["value"]) - dead_zone) / (
        full_scale - dead_zone
    )
    assert stroke_position > 0.4
    assert raise_step["require"] == "stand"
    # A completion marker has to read as a deliberate gesture, so it is held
    # longer than the transient 低头 it mirrors.
    assert raise_step["duration_sec"] > lower_step["duration_sec"]


def test_every_pose_hold_fits_inside_its_unit_budget() -> None:
    """A hold longer than its unit budget is a silently clipped gesture.

    A pose plan pays, in order: the entry stand-up recovery, a mode switch with
    its settle time and one control period per step, the holds themselves, and
    finally the settle wait.  Recovery and settle are clamped by the step
    deadline, so an undersized budget does not fail loudly -- the deadline
    lands mid-hold and the operator just sees a shorter 低头 / 扭腰.  The 偏航
    family went 0.8 -> 1.6 s and the pitch holds 1.0 -> 2.0 s on 2026-09-18,
    together with the unit timeouts that had to rise with them, and the eating
    ritual added the first multi-step pose plan.
    """
    loader = _loader()
    config = loader.lite3_action_config
    fixed_overhead = (
        2 * float(config["mode_settle_sec"])
        + 1.0 / float(config["publish_rate_hz"])
    )
    # The recovery is budgeted at ``posture_recovery_timeout_sec`` (6.0) but
    # only spends 2.2 s when it succeeds; the cap is what a failing stand-up
    # burns, and that fails the step for its own reason anyway.
    entry_recovery_sec = 2.2

    checked = 0
    for unit_id, plan in loader.get_lite3_action_plans().items():
        sequence = plan.get("sequence", [])
        if not sequence or any(
            step.get("type") != "pose" for step in sequence
        ):
            continue
        # The five ACT_IGNORE_* units are ``unit_type: policy``: they report
        # success with no physical action and their 0.0 timeout is deliberate.
        if loader.action_catalog[unit_id].get("unit_type") == "policy":
            continue
        checked += 1
        # Every pose step pays for its own mode switches and control period --
        # the dog is put back into in-place mode and then into moving mode
        # around each hold.  The entry stand-up recovery is paid once, by the
        # first step, because the steps after it start from standing.
        needed = (
            sum(float(step["duration_sec"]) for step in sequence)
            + len(sequence) * fixed_overhead
            + entry_recovery_sec
        )
        budget = float(loader.action_catalog[unit_id]["timeout_sec"])
        assert budget >= needed, (
            f"{unit_id} unit budget {budget} is below its pose holds plus "
            f"recovery ({needed})"
        )
    # 126: every all-pose plan except the five policy units.  The eating ritual
    # added the only multi-step one (ACT_TWIST_WAIST_LEFT_RIGHT, two 1.6 s
    # 偏航 holds, 5.9 s of 8.0 s budget) and deleted 23 single-step ones.
    assert checked == 126


def test_toilet_marks_completion_with_a_final_head_up() -> None:
    loader = _loader()
    stages = loader.behavior_tree_templates["barkShortAlert"]["stages"]
    assert [stage["stage_id"] for stage in stages] == [
        "circle",
        "action",
        "exit",
        "head_up",
    ]
    assert [stage["order"] for stage in stages] == [1, 2, 3, 4]
    # 到点 -> 绕点转3圈 comes straight after navigation, and 抬头 is the last
    # thing the dog does.  Both are singles so the random picks cannot skip them.
    assert [c["unit_id"] for c in stages[0]["candidates"]] == [
        "ACT_SNIFF_AND_CIRCLE_AT_TOILET_SPOT"
    ]
    assert [c["unit_id"] for c in stages[-1]["candidates"]] == [
        "ACT_RAISE_HEAD"
    ]
    assert loader.navigation_config["behavior_routes"]["barkShortAlert"][
        "stages"
    ] == ["circle", "action", "exit", "head_up"]


def test_forward_twist_is_refused_without_forward_clearance() -> None:
    plans = {
        "ACT_TEST_FORWARD": {
            "fidelity": "exact",
            "verified": True,
            "sequence": [
                {
                    "type": "twist",
                    "linear_x": 0.20,
                    "linear_y": 0.0,
                    "angular_z": 0.40,
                    "duration_sec": 2.0,
                    "minimum_forward_clearance_m": 0.60,
                }
            ],
        }
    }
    commands: list[Lite3SimpleCommand] = []
    twists: list[TwistCommand] = []
    fake_time = FakeTime()
    backend = _backend(commands, twists, fake_time, action_plans=plans)
    backend.update_status(_status(6, forward=0.30))

    assert not backend.execute_step(
        {"unit_id": "ACT_TEST_FORWARD"}, ExecutionContext.from_goal("x", {}), 3.0
    )
    assert backend.last_error.startswith("lite3_forward_clearance:")
    assert Lite3SimpleCommand(CMD_SWITCH_VISION_MODE) not in commands


def test_forward_motion_without_a_declared_threshold_is_unchanged() -> None:
    """The forward guard is opt-in, so no existing twist plan changes."""
    plans = {
        "ACT_TEST_FORWARD": {
            "fidelity": "exact",
            "verified": True,
            "sequence": [
                {
                    "type": "twist",
                    "linear_x": 0.20,
                    "linear_y": 0.0,
                    "angular_z": 0.0,
                    "duration_sec": 0.5,
                }
            ],
        }
    }
    commands: list[Lite3SimpleCommand] = []
    twists: list[TwistCommand] = []
    fake_time = FakeTime()
    backend = _backend(commands, twists, fake_time, action_plans=plans)
    backend.update_status(_status(6, forward=0.30))
    fake_time.on_sleep = lambda _now: backend.update_status(_status(6, forward=0.30))

    assert backend.execute_step(
        {"unit_id": "ACT_TEST_FORWARD"}, ExecutionContext.from_goal("x", {}), 3.0
    )
    assert any(twist.linear_x > 0.0 for twist in twists)


def test_task_shaped_object_action_uses_observable_shared_proxy() -> None:
    fake_time = FakeTime()
    commands: list[Lite3SimpleCommand] = []
    backend = _backend(
        commands,
        [],
        fake_time,
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_GUARD_DOOR"],
    )
    backend.update_status(_status(6))
    fake_time.on_sleep = lambda _now: backend.update_status(_status(6))
    ctx = ExecutionContext.from_goal("fetch_object", {})

    assert backend.execute_task(
        {"unit_id": "ACT_OBJECT_FETCH"}, ctx, timeout=5.0
    )
    result = ctx.metadata["lite3_action"]
    assert result["requested_act"] == "ACT_OBJECT_FETCH"
    assert result["executed_plan"] == "ACT_GUARD_DOOR"
    assert result["semantic_effect"] == "proxy_motion"
    assert Lite3SimpleCommand(0x21010135, 10500) in commands


def test_velocity_switches_modes_clamps_and_releases_remote_control() -> None:
    commands: list[Lite3SimpleCommand] = []
    twists: list[TwistCommand] = []
    backend = _backend(commands, twists, FakeTime())
    backend.update_status(_status(6))

    backend.publish_velocity(
        TwistCommand(linear_x=9.0, linear_y=-9.0, angular_z=9.0)
    )
    assert commands == [Lite3SimpleCommand(CMD_SWITCH_VISION_MODE)]
    limits = _loader().lite3_action_config["limits"]
    assert twists[-1] == TwistCommand(
        limits["max_linear_x"], -limits["max_linear_y"], limits["max_angular_z"]
    )

    backend.publish_velocity(TwistCommand())
    assert twists[-1] == TwistCommand()
    assert commands[-1] == Lite3SimpleCommand(CMD_SWITCH_JOYSTICK_MODE)

    assert not backend.finish_navigation()
    assert twists[-5:] == [TwistCommand()] * 5


def test_finish_navigation_waits_for_new_consecutive_idle_samples() -> None:
    commands: list[Lite3SimpleCommand] = []
    twists: list[TwistCommand] = []
    fake_time = FakeTime()
    backend = _backend(commands, twists, fake_time)
    backend.update_status(_status(6, motion=1))

    def settle_after_stop(now: float) -> None:
        backend.update_status(_status(6, motion=0 if now >= 0.20 else 1))

    fake_time.on_sleep = settle_after_stop

    assert backend.finish_navigation()
    assert fake_time.now >= 0.39
    assert backend.last_error == ""
    assert twists[-5:] == [TwistCommand()] * 5


def test_finish_navigation_confirms_stable_status_after_cancel() -> None:
    fake_time = FakeTime()
    backend = _backend([], [], fake_time)
    backend.update_status(_status(6, motion=1))
    backend._cancel_requested.set()
    backend._should_stop = lambda: True

    def settle_after_stop(now: float) -> None:
        backend.update_status(_status(6, motion=0 if now >= 0.20 else 1))

    fake_time.on_sleep = settle_after_stop

    assert backend.finish_navigation()
    assert fake_time.now >= 0.39
    assert backend.last_error == ""


def test_finish_navigation_cancel_still_requires_fresh_stable_status() -> None:
    backend = _backend([], [], FakeTime())
    backend.update_status(_status(6))
    backend._cancel_requested.set()
    backend._should_stop = lambda: True

    assert not backend.finish_navigation()
    assert backend.last_error == (
        "lite3_navigation_settle_timeout:"
        "last_status=lite3_waiting_for_new_status"
    )


def test_finish_navigation_reports_busy_state_when_settle_times_out() -> None:
    backend = _backend([], [], FakeTime())
    backend.update_status(_status(6, motion=1))

    assert not backend.finish_navigation()
    assert backend.last_error == (
        "lite3_navigation_settle_timeout:"
        "last_status=lite3_motion_busy:motion_state=1"
    )


def test_navigation_preflight_rejects_competing_lite3_helpers() -> None:
    commands: list[Lite3SimpleCommand] = []
    twists: list[TwistCommand] = []
    backend = _backend(
        commands,
        twists,
        FakeTime(),
        control_conflict=(
            "active_nodes=/lite3_action=2,/lite3_twist_bridge=2"
        ),
    )
    backend.update_status(_status(6))

    assert not backend.prepare_navigation()
    assert backend.last_error == (
        "lite3_control_conflict:"
        "active_nodes=/lite3_action=2,/lite3_twist_bridge=2"
    )
    assert commands == []
    assert twists[-5:] == [TwistCommand()] * 5


def test_ros2_control_conflict_report_counts_only_root_lite3_helpers() -> None:
    class FakeNode:
        @staticmethod
        def get_node_names_and_namespaces():
            return [
                ("lite3_twist_bridge", "/"),
                ("lite3_twist_bridge", "/"),
                ("lite3_action", "/"),
                ("lite3_action", "/maintenance"),
                ("action_executor_node", "/"),
            ]

    assert _ros2_control_conflicts(FakeNode()) == (
        "active_nodes=/lite3_action=1,/lite3_twist_bridge=2"
    )


def test_ros2_control_conflict_report_always_names_the_uwb_chain() -> None:
    class FakeNode:
        @staticmethod
        def get_node_names_and_namespaces():
            return [
                ("uwb_behavior_controller_node", "/"),
                ("uwb_behavior_controller_node", "/go2"),
                ("action_executor_node", "/"),
            ]

    # Reported unconditionally: this is the honest inventory.  Who is allowed
    # to keep it running is decided by the caller, not hidden here.
    assert _ros2_control_conflicts(FakeNode()) == (
        "active_nodes=/uwb_behavior_controller_node=1"
    )


def test_idle_uwb_chain_can_be_narrowed_to_driving_only() -> None:
    """``uwb_driving`` re-reads the UWB entry as "is it driving right now"."""

    class FakeNode:
        @staticmethod
        def get_node_names_and_namespaces():
            return [
                ("uwb_behavior_controller_node", "/"),
                ("action_executor_node", "/"),
            ]

    # Up but between goals.  With publish_idle_velocity: false the chain is
    # silent on /cmd_vel, so it must not block a wake turn or Nav2.
    assert (
        _ros2_control_conflicts(FakeNode(), uwb_driving=lambda: False) == ""
    )

    # Driving: same loud verdict as the unconditional inventory.
    assert _ros2_control_conflicts(FakeNode(), uwb_driving=lambda: True) == (
        "active_nodes=/uwb_behavior_controller_node=1"
    )


def test_uwb_driving_callback_failure_keeps_the_conflict() -> None:
    """A checker that raises must fail closed, never open."""

    class FakeNode:
        @staticmethod
        def get_node_names_and_namespaces():
            return [("uwb_behavior_controller_node", "/")]

    def boom() -> bool:
        raise RuntimeError("no session state")

    assert _ros2_control_conflicts(FakeNode(), uwb_driving=boom) == (
        "active_nodes=/uwb_behavior_controller_node=1"
    )


def test_narrowed_uwb_chain_still_counts_the_lite3_helpers() -> None:
    """Narrowing the UWB entry must not silence the helper nodes."""

    class FakeNode:
        @staticmethod
        def get_node_names_and_namespaces():
            return [
                ("uwb_behavior_controller_node", "/"),
                ("lite3_twist_bridge", "/"),
            ]

    assert _ros2_control_conflicts(
        FakeNode(), uwb_driving=lambda: False
    ) == "active_nodes=/lite3_twist_bridge=1"


def test_uwb_chain_blocks_twist_actions_but_not_the_follow_preflight() -> None:
    chain = "active_nodes=/uwb_behavior_controller_node=1"

    commands: list[Lite3SimpleCommand] = []
    twists: list[TwistCommand] = []
    refused = _backend(commands, twists, FakeTime(), control_conflict=chain)
    refused.update_status(_status(6))

    # Every other Lite3 motion path keeps failing loudly: the chain publishes
    # /cmd_vel at 20 Hz from startup and would chop an open-loop twist apart.
    assert not refused.prepare_navigation()
    assert refused.last_error.startswith("lite3_control_conflict:")

    commands = []
    twists = []
    allowed = _backend(commands, twists, FakeTime(), control_conflict=chain)
    allowed.update_status(_status(6))

    # Following *is* handing /cmd_vel to that chain, so the follow adapter is
    # the one caller that may proceed -- otherwise its own preflight would be
    # blocked by the very node it depends on.
    assert allowed.prepare_navigation(allow_uwb_chain=True)
    assert allowed.last_error == ""


def test_uwb_chain_exemption_still_reports_other_competing_nodes() -> None:
    commands: list[Lite3SimpleCommand] = []
    twists: list[TwistCommand] = []
    backend = _backend(
        commands,
        twists,
        FakeTime(),
        control_conflict=(
            "active_nodes=/lite3_twist_bridge=1,"
            "/uwb_behavior_controller_node=1"
        ),
    )
    backend.update_status(_status(6))

    assert not backend.prepare_navigation(allow_uwb_chain=True)
    assert backend.last_error == (
        "lite3_control_conflict:active_nodes=/lite3_twist_bridge=1"
    )


def test_pose_axes_switch_to_in_place_mode_exceed_dead_zone_and_reset() -> None:
    commands: list[Lite3SimpleCommand] = []
    fake_time = FakeTime()
    pose_plan = {
        "ACT_TEST_POSE": {
            "fidelity": "proxy",
            "verified": False,
            "sequence": [
                {
                    "type": "pose",
                    "cmd_code": 0x21010130,
                    "value": 7500,
                    "require": "stand",
                    "duration_sec": 0.2,
                }
            ],
        }
    }
    backend = _backend(
        commands,
        [],
        fake_time,
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_TEST_POSE"],
        action_plans=pose_plan,
    )
    backend.update_status(_status(6))
    fake_time.on_sleep = lambda _now: backend.update_status(_status(6))

    assert backend.execute_step(
        {"unit_id": "ACT_TEST_POSE"},
        ExecutionContext.from_goal("unhappy", {}),
        4.0,
    )
    assert Lite3SimpleCommand(CMD_SWITCH_IN_PLACE_MODE) in commands
    assert Lite3SimpleCommand(0x21010130, 7500) in commands
    assert commands[-2:] == [
        Lite3SimpleCommand(0x21010130, 0),
        Lite3SimpleCommand(CMD_SWITCH_MOVING_MODE),
    ]
    assert all(command.cmd_code != CMD_VOICE for command in commands)


def test_pose_waits_for_busy_state_to_return_to_stable_idle() -> None:
    commands: list[Lite3SimpleCommand] = []
    fake_time = FakeTime()
    backend = _backend(
        commands,
        [],
        fake_time,
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_TEST_POSE"],
        action_plans={
            "ACT_TEST_POSE": {
                "fidelity": "proxy",
                "verified": False,
                "sequence": [
                    {
                        "type": "pose",
                        "cmd_code": 0x21010130,
                        "value": 7500,
                        "require": "stand",
                        "duration_sec": 0.2,
                    }
                ],
            }
        },
    )
    backend.update_status(_status(6))
    settle_samples: list[int] = []

    def update_pose_status(_now: float) -> None:
        if Lite3SimpleCommand(CMD_SWITCH_MOVING_MODE) not in commands:
            backend.update_status(_status(6))
            return
        motion = 1 if len(settle_samples) < 2 else 0
        settle_samples.append(motion)
        backend.update_status(_status(6, motion=motion))

    fake_time.on_sleep = update_pose_status

    assert backend.execute_step(
        {"unit_id": "ACT_TEST_POSE"},
        ExecutionContext.from_goal("unhappy", {}),
        4.0,
    )
    assert 1 in settle_samples
    assert settle_samples[-5:] == [0] * 5


def test_pose_axis_value_inside_dead_zone_is_rejected_without_axis_output() -> None:
    commands: list[Lite3SimpleCommand] = []
    backend = _backend(
        commands,
        [],
        FakeTime(),
        allow_proxies=True,
        allow_unverified=True,
        accepted_unverified_actions=["ACT_TEST_POSE"],
        action_plans={
            "ACT_TEST_POSE": {
                "fidelity": "proxy",
                "verified": False,
                "sequence": [
                    {
                        "type": "pose",
                        "cmd_code": 0x21010130,
                        "value": 2000,
                        "require": "stand",
                        "duration_sec": 0.2,
                    }
                ],
            }
        },
    )
    backend.update_status(_status(6))

    assert not backend.execute_step(
        {"unit_id": "ACT_TEST_POSE"},
        ExecutionContext.from_goal("unhappy", {}),
        4.0,
    )
    assert not any(command.cmd_code == 0x21010130 for command in commands)
    assert backend.last_error.startswith("lite3_pose_axis_invalid:")


def test_emergency_stop_never_sends_motor_disabling_command() -> None:
    commands: list[Lite3SimpleCommand] = []
    twists: list[TwistCommand] = []
    backend = _backend(commands, twists, FakeTime())
    backend.update_status(_status(6))
    backend.publish_velocity(TwistCommand(linear_x=0.1))

    backend.emergency_stop()

    assert Lite3SimpleCommand(CMD_SOFT_EMERGENCY_STOP) not in commands
    assert commands[-1] == Lite3SimpleCommand(CMD_SWITCH_JOYSTICK_MODE)
    assert twists[-5:] == [TwistCommand()] * 5


def test_reverse_motion_requires_and_continuously_checks_rear_clearance() -> None:
    commands: list[Lite3SimpleCommand] = []
    twists: list[TwistCommand] = []
    fake_time = FakeTime()
    backend = _backend(commands, twists, fake_time)
    backend.update_status(_status(backward=0.30))

    assert not backend.execute_step(
        {"unit_id": "ACT_BASIC_BACK_UP"},
        ExecutionContext.from_goal("back_up", {}),
        3.0,
    )
    assert all(command.is_zero for command in twists)

    commands.clear()
    twists.clear()
    backend.update_status(_status(backward=1.0))

    def obstacle_appears(now: float) -> None:
        backend.update_status(
            _status(backward=0.30 if now >= 0.20 else 1.0)
        )

    fake_time.on_sleep = obstacle_appears
    assert not backend.execute_step(
        {"unit_id": "ACT_BASIC_BACK_UP"},
        ExecutionContext.from_goal("back_up", {}),
        3.0,
    )
    assert any(not command.is_zero for command in twists)
    assert twists[-5:] == [TwistCommand()] * 5


def test_runtime_blocks_dangerous_simple_commands_even_if_config_is_injected() -> None:
    commands: list[Lite3SimpleCommand] = []
    backend = _backend(
        commands,
        [],
        FakeTime(),
        action_plans={
            "ACT_TEST_DANGEROUS": {
                "fidelity": "exact",
                "verified": True,
                "sequence": [
                    {
                        "type": "simple",
                        "cmd_code": 0x21010205,
                        "require": "lie",
                        "completion_timeout_sec": 1.0,
                    }
                ],
            }
        },
    )
    backend.update_status(_status(1))

    assert not backend.execute_step(
        {"unit_id": "ACT_TEST_DANGEROUS"},
        ExecutionContext.from_goal("dangerous", {}),
        2.0,
    )
    assert commands == []


def test_stage_uses_lite3_physical_posture_instead_of_proxy_catalog_posture() -> None:
    posture = PostureManager()
    posture.set_posture("standing")

    class ProxyAdapter:
        def can_execute(self, unit_id: str) -> bool:
            return unit_id == "ACT_GETUP_SIT"

        def execute_step(self, unit_config, ctx, duration) -> bool:
            del duration
            ctx.metadata["lite3_action"] = {
                "requested_act": unit_config["unit_id"],
                "fidelity": "proxy",
                "semantic_effect": "simulated",
                "physical_posture": "standing",
            }
            return True

    executor = StageExecutor(
        posture_manager=posture,
        action_catalog={
            "ACT_GETUP_SIT": {
                "unit_id": "ACT_GETUP_SIT",
                "unit_type": "atomic_action",
                "timeout_sec": 1.0,
                "to_posture": "sitting",
            }
        },
        controller_routes={"ACT_GETUP_SIT": "lite3"},
        controller_adapters={"lite3": ProxyAdapter()},
    )

    result = executor.execute_stage(
        {
            "stage_id": "wake",
            "selection_policy": "fixed",
            "candidates": [{"unit_id": "ACT_GETUP_SIT"}],
        },
        ExecutionContext.from_goal("wake", {}),
    )

    assert result.success
    assert posture.current == "standing"


def test_navigation_calls_platform_mode_hooks_around_nav2() -> None:
    events: list[str] = []

    class MotionBackend:
        def prepare_navigation(self) -> bool:
            events.append("prepare")
            return True

        def finish_navigation(self) -> bool:
            events.append("finish")
            return True

    adapter = BehaviorMobilityAdapter(
        navigate_waypoint=lambda name, pose, timeout: events.append("navigate")
        or True,
        motion_adapter=MotionBackend(),
        waypoints={
            "A": {
                "x": 0.0,
                "y": 0.0,
                "orientation_z": 0.0,
                "orientation_w": 1.0,
            }
        },
        behavior_routes={"go_home": {"waypoint": "A", "stages": ["navigation"]}},
        stage_actions=[],
    )

    assert adapter.navigate_for_behavior("go_home", timeout_sec=5.0)
    assert events == ["prepare", "navigate", "finish"]


def test_navigation_fails_closed_when_chassis_does_not_settle() -> None:
    events: list[str] = []

    class MotionBackend:
        def prepare_navigation(self) -> bool:
            events.append("prepare")
            return True

        def finish_navigation(self) -> bool:
            events.append("finish")
            return False

    adapter = BehaviorMobilityAdapter(
        navigate_waypoint=lambda name, pose, timeout: events.append("navigate")
        or True,
        motion_adapter=MotionBackend(),
        waypoints={
            "A": {
                "x": 0.0,
                "y": 0.0,
                "orientation_z": 0.0,
                "orientation_w": 1.0,
            }
        },
        behavior_routes={
            "go_home": {"waypoint": "A", "stages": ["navigation"]}
        },
        stage_actions=[],
    )

    assert not adapter.navigate_for_behavior("go_home", timeout_sec=5.0)
    assert events == ["prepare", "navigate", "finish"]
    assert adapter.last_error == "chassis_navigation_settle_failed"
