"""Tests for wake-angle propagation and Nav2 Spin routing."""

from __future__ import annotations

import math
import ast
from pathlib import Path

from marsdog_action_executor.adapters.wake_orientation_adapter import (
    WakeOrientationAdapter,
)
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.ros_node import _dispatch_visual_event
from marsdog_action_executor.stage_executor import StageExecutor


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
LAUNCH_FILE = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "action_executor.launch.py"
)


class RecordingSpin:
    def __init__(self, result: bool = True, on_call=None) -> None:
        self.result = result
        self.on_call = on_call
        self.calls: list[tuple[float, float, float]] = []
        self.cancel_count = 0

    def __call__(
        self,
        target_yaw_rad: float,
        result_timeout_sec: float,
        time_allowance_sec: float,
    ) -> bool:
        self.calls.append(
            (target_yaw_rad, result_timeout_sec, time_allowance_sec)
        )
        if self.on_call is not None:
            self.on_call(len(self.calls))
        return self.result

    def cancel_spin(self) -> None:
        self.cancel_count += 1


def _launch_default(argument_name: str) -> str | None:
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    launch_default = None
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not (
            isinstance(node.func, ast.Name)
            and node.func.id == "DeclareLaunchArgument"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == argument_name
        ):
            continue
        for keyword in node.keywords:
            if keyword.arg == "default_value":
                launch_default = ast.literal_eval(keyword.value)
                break

    return launch_default


def test_launch_defaults_match_packaged_wake_calibration() -> None:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    assert _launch_default("wake_angle_zero_offset_deg") == str(
        loader.wake_orientation_config["angle_zero_offset_deg"]
    )
    assert _launch_default("wake_angle_direction_sign") == str(
        loader.wake_orientation_config["angle_direction_sign"]
    )
    assert _launch_default(
        "wake_linear_array_back_search_enabled"
    ) == str(
        loader.wake_orientation_config[
            "linear_array_back_search_enabled"
        ]
    ).lower()
    for launch_name, config_name in (
        ("wake_visual_confirm_timeout_sec", "visual_confirm_timeout_sec"),
        ("wake_visual_min_confidence", "visual_min_confidence"),
        ("wake_visual_max_age_ms", "visual_max_age_ms"),
    ):
        assert float(_launch_default(launch_name)) == float(
            loader.wake_orientation_config[config_name]
        )


def _context(angle_deg: float = 90.0) -> ExecutionContext:
    return ExecutionContext.from_goal(
        "respond_owner_call",
        {
            "use_wake_angle": True,
            "wake_angle_deg": angle_deg,
            "wake_confidence": 1205.0,
            "wake_frame_id": "microphone_array",
        },
    )


def test_execution_context_parses_wake_contract_fields() -> None:
    ctx = _context(35.5)

    assert ctx.use_wake_angle is True
    assert ctx.wake_angle_deg == 35.5
    assert ctx.wake_confidence == 1205.0
    assert ctx.wake_frame_id == "microphone_array"
    assert not {
        "use_wake_angle",
        "wake_angle_deg",
        "wake_confidence",
        "wake_frame_id",
    } & set(ctx.metadata)


def test_dual_mic_center_is_inside_deadband_without_nav2_spin() -> None:
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(spin)

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"},
        _context(90.0),
    )

    assert spin.calls == []


def test_angle_is_calibrated_normalised_and_direction_can_flip() -> None:
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(
        spin,
        angle_zero_offset_deg=10.0,
        angle_direction_sign=-1.0,
        angle_deadband_deg=0.0,
    )

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"},
        _context(100.0),
    )

    assert math.isclose(spin.calls[0][0], -math.pi / 2.0)

    wrapping_spin = RecordingSpin()
    wrapping_adapter = WakeOrientationAdapter(
        wrapping_spin,
        angle_deadband_deg=0.0,
    )
    assert wrapping_adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"},
        _context(350.0),
    )
    assert math.isclose(
        wrapping_spin.calls[0][0],
        math.radians(100.0),
    )


def test_installed_dual_mic_center_splits_left_and_right_signs() -> None:
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(spin, angle_deadband_deg=0.0)

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"}, _context(50.0)
    )
    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"}, _context(120.0)
    )

    assert math.isclose(spin.calls[0][0], math.radians(40.0))
    assert math.isclose(spin.calls[1][0], math.radians(-30.0))


def test_linear_array_visual_confirmation_avoids_rear_search() -> None:
    adapter = None

    def publish_human(call_count: int) -> None:
        assert call_count == 1
        assert adapter is not None
        adapter.update_visual({
            "active_target": {
                "target_type": "human",
                "confidence": 0.9,
                "tracking_state": "tracking",
                "last_seen_age_ms": 10.0,
            }
        })

    spin = RecordingSpin(on_call=publish_human)
    adapter = WakeOrientationAdapter(
        spin,
        linear_array_back_search_enabled=True,
        visual_confirm_timeout_sec=0.01,
    )

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"}, _context(50.0)
    )
    assert len(spin.calls) == 1
    assert math.isclose(spin.calls[0][0], math.radians(40.0))


def test_linear_array_missing_human_checks_mirrored_rear_heading() -> None:
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(
        spin,
        linear_array_back_search_enabled=True,
        visual_confirm_timeout_sec=0.001,
    )

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"}, _context(50.0)
    )
    assert len(spin.calls) == 2
    assert math.isclose(spin.calls[0][0], math.radians(40.0))
    # First heading is +40 degrees; its rear mirror is +140 degrees,
    # therefore the second relative Spin is +100 degrees.
    assert math.isclose(spin.calls[1][0], math.radians(100.0))


def test_goal_budget_caps_spin_timeouts() -> None:
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(
        spin,
        result_timeout_sec=15.0,
        time_allowance_sec=12.0,
    )

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"},
        _context(50.0),
        duration=0.05,
    )

    assert len(spin.calls) == 1
    _, result_timeout, time_allowance = spin.calls[0]
    assert 0.0 < result_timeout <= 0.05
    assert 0.0 < time_allowance <= 0.05


def test_exhausted_goal_budget_prevents_rear_search() -> None:
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(
        spin,
        linear_array_back_search_enabled=True,
        visual_confirm_timeout_sec=0.05,
    )

    assert not adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"},
        _context(50.0),
        duration=0.002,
    )
    assert len(spin.calls) == 1


def test_linear_array_rear_search_accepts_fresh_human_on_second_heading() -> None:
    adapter = None

    def publish_human(call_count: int) -> None:
        if call_count != 2:
            return
        assert adapter is not None
        adapter.update_visual({
            "active_target": {
                "target_type": "human",
                "confidence": 0.95,
                "tracking_state": "tracking",
                "last_seen_age_ms": 5.0,
            }
        })

    spin = RecordingSpin(on_call=publish_human)
    adapter = WakeOrientationAdapter(
        spin,
        linear_array_back_search_enabled=True,
        visual_confirm_timeout_sec=0.001,
    )

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"}, _context(120.0)
    )
    assert len(spin.calls) == 2
    assert math.isclose(spin.calls[0][0], math.radians(-30.0))
    assert math.isclose(spin.calls[1][0], math.radians(-120.0))


def test_cancel_after_first_heading_prevents_rear_search() -> None:
    canceled = False

    def cancel_after_first(call_count: int) -> None:
        nonlocal canceled
        if call_count == 1:
            canceled = True

    spin = RecordingSpin(on_call=cancel_after_first)
    adapter = WakeOrientationAdapter(
        spin,
        linear_array_back_search_enabled=True,
        visual_confirm_timeout_sec=0.01,
        should_stop=lambda: canceled,
    )

    assert not adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"}, _context(50.0)
    )
    assert len(spin.calls) == 1


def test_visual_dispatch_forwards_human_snapshot_to_wake_search() -> None:
    received = []

    class RecordingWakeAdapter:
        def update_visual(self, payload) -> None:
            received.append(payload)

    assert _dispatch_visual_event(
        '{"active_target":{"target_type":"human"}}',
        wake_orientation_adapter=RecordingWakeAdapter(),
    )
    assert received == [{"active_target": {"target_type": "human"}}]


def test_deadband_succeeds_without_sending_spin_goal() -> None:
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(spin, angle_deadband_deg=5.0)

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"},
        _context(94.0),
    )
    assert not spin.calls


def test_missing_angle_or_wrong_frame_fails_without_motion() -> None:
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(spin)

    missing = ExecutionContext.from_goal(
        "respond_owner_call",
        {
            "use_wake_angle": True,
            "wake_frame_id": "microphone_array",
        },
    )
    wrong_frame = ExecutionContext.from_goal(
        "respond_owner_call",
        {
            "use_wake_angle": True,
            "wake_angle_deg": 45.0,
            "wake_frame_id": "map",
        },
    )

    assert not adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"},
        missing,
    )
    assert not adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"},
        wrong_frame,
    )
    assert not spin.calls


def test_stage_executor_routes_exact_respond_action_to_wake_adapter() -> None:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(spin)
    executor = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes(),
        controller_adapters={"wake_orientation": adapter},
    )
    ctx = _context(60.0)
    ctx.resolved_behavior_name = "respond_owner_call"

    result = executor.execute_stage(
        loader.get_behavior_template("respond_owner_call")["stages"][0],
        ctx,
        seed=1,
    )

    assert result.success
    assert result.unit_id == "ACT_INTERACT_RESPOND_CALL"
    assert ctx.executed_units == ["ACT_INTERACT_RESPOND_CALL"]
    assert math.isclose(spin.calls[0][0], math.radians(30.0))


class RecordingMotionHooks:
    """Stand-in for the chassis prepare/finish hand-over pair."""

    def __init__(self, prepare_result: bool = True) -> None:
        self.prepare_result = prepare_result
        self.events: list[str] = []

    def prepare(self) -> bool:
        self.events.append("prepare")
        return self.prepare_result

    def finish(self) -> bool:
        self.events.append("finish")
        return True


def test_motion_hooks_bracket_the_nav2_turn() -> None:
    hooks = RecordingMotionHooks()
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(
        spin,
        angle_deadband_deg=0.0,
        prepare_motion=hooks.prepare,
        finish_motion=hooks.finish,
    )

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"}, _context(50.0)
    )

    assert hooks.events == ["prepare", "finish"]
    assert len(spin.calls) == 1


def test_motion_hooks_release_the_chassis_when_the_turn_fails() -> None:
    """A rejected or timed-out Spin must not leave the dog owned."""
    hooks = RecordingMotionHooks()
    adapter = WakeOrientationAdapter(
        RecordingSpin(result=False),
        angle_deadband_deg=0.0,
        prepare_motion=hooks.prepare,
        finish_motion=hooks.finish,
    )

    assert not adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"}, _context(50.0)
    )

    assert hooks.events == ["prepare", "finish"]


def test_rejected_prepare_skips_the_turn_and_never_releases() -> None:
    hooks = RecordingMotionHooks(prepare_result=False)
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(
        spin,
        angle_deadband_deg=0.0,
        prepare_motion=hooks.prepare,
        finish_motion=hooks.finish,
    )

    assert not adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"}, _context(50.0)
    )

    assert hooks.events == ["prepare"]
    assert spin.calls == []


def test_source_already_ahead_leaves_the_chassis_alone() -> None:
    """No turn and no rear search means no hand-over at all."""
    hooks = RecordingMotionHooks()
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(
        spin,
        prepare_motion=hooks.prepare,
        finish_motion=hooks.finish,
    )

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"}, _context(90.0)
    )

    assert hooks.events == []
    assert spin.calls == []


def test_centered_source_with_rear_search_still_holds_the_chassis() -> None:
    """The mirrored rear candidate turns 180deg, so the hook must cover it."""
    hooks = RecordingMotionHooks()
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(
        spin,
        linear_array_back_search_enabled=True,
        visual_confirm_timeout_sec=0.01,
        prepare_motion=hooks.prepare,
        finish_motion=hooks.finish,
    )

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"}, _context(90.0)
    )

    assert hooks.events == ["prepare", "finish"]
    assert math.isclose(spin.calls[0][0], -math.pi)


def test_cancel_forwards_to_active_spin_client() -> None:
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(spin)

    adapter.cancel_step()

    assert spin.cancel_count == 1


def test_wake_orientation_config_matches_controller_route() -> None:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()

    assert loader.controller_routes["routes"][
        "ACT_INTERACT_RESPOND_CALL"
    ] == "wake_orientation"
    assert loader.wake_orientation_config["action_name"] == "/spin"
    assert (
        loader.wake_orientation_config["required_frame_id"]
        == "microphone_array"
    )
