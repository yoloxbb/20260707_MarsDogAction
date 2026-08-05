"""Tests for wake-angle propagation and Nav2 Spin routing."""

from __future__ import annotations

import math
from pathlib import Path

from marsdog_action_executor.adapters.wake_orientation_adapter import (
    WakeOrientationAdapter,
)
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.stage_executor import StageExecutor


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


class RecordingSpin:
    def __init__(self, result: bool = True) -> None:
        self.result = result
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
        return self.result

    def cancel_spin(self) -> None:
        self.cancel_count += 1


def _context(angle_deg: float = 90.0) -> ExecutionContext:
    return ExecutionContext.from_goal(
        "respond_owner_call",
        {
            "use_wake_angle": True,
            "wake_angle_deg": angle_deg,
            "wake_confidence": 1205.0,
            "wake_frame_id": "base_link",
        },
    )


def test_execution_context_parses_wake_contract_fields() -> None:
    ctx = _context(35.5)

    assert ctx.use_wake_angle is True
    assert ctx.wake_angle_deg == 35.5
    assert ctx.wake_confidence == 1205.0
    assert ctx.wake_frame_id == "base_link"
    assert not {
        "use_wake_angle",
        "wake_angle_deg",
        "wake_confidence",
        "wake_frame_id",
    } & set(ctx.metadata)


def test_positive_angle_invokes_nav2_spin_in_radians() -> None:
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(spin)

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"},
        _context(90.0),
    )

    assert len(spin.calls) == 1
    target_yaw, result_timeout, time_allowance = spin.calls[0]
    assert math.isclose(target_yaw, math.pi / 2.0)
    assert result_timeout == 15.0
    assert time_allowance == 12.0


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
        math.radians(-10.0),
    )


def test_deadband_succeeds_without_sending_spin_goal() -> None:
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(spin, angle_deadband_deg=5.0)

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_RESPOND_CALL"},
        _context(4.0),
    )
    assert not spin.calls


def test_missing_angle_or_wrong_frame_fails_without_motion() -> None:
    spin = RecordingSpin()
    adapter = WakeOrientationAdapter(spin)

    missing = ExecutionContext.from_goal(
        "respond_owner_call",
        {
            "use_wake_angle": True,
            "wake_frame_id": "base_link",
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
    ctx = _context(-30.0)
    ctx.resolved_behavior_name = "respond_owner_call"

    result = executor.execute_stage(
        loader.get_behavior_template("respond_owner_call")["stages"][0],
        ctx,
        seed=1,
    )

    assert result.success
    assert result.unit_id == "ACT_INTERACT_RESPOND_CALL"
    assert ctx.executed_units == ["ACT_INTERACT_RESPOND_CALL"]
    assert math.isclose(spin.calls[0][0], math.radians(-30.0))


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
        == "base_link"
    )
