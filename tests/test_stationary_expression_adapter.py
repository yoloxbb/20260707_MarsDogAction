"""The stationary expression route must never emit chassis movement."""

from __future__ import annotations

from marsdog_action_executor.adapters import StationaryExpressionAdapter
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.interrupt_manager import InterruptManager
from marsdog_action_executor.stage_executor import StageExecutor


def _is_zero(command) -> bool:
    return (
        command.linear_x == 0.0
        and command.linear_y == 0.0
        and command.angular_z == 0.0
    )


def test_success_cancel_and_emergency_publish_only_zero_twist() -> None:
    commands = []
    adapter = StationaryExpressionAdapter(
        commands.append,
        publish_rate_hz=10.0,
        stop_publish_count=3,
        sleep=lambda _: None,
    )
    ctx = ExecutionContext.from_goal(
        "expressJoyInPlaceWithHuman",
        {"mobility_policy": "stationary"},
    )

    assert adapter.execute_step({}, ctx, 0.21)
    adapter.cancel_step()
    adapter.emergency_stop()

    assert commands
    assert all(_is_zero(command) for command in commands)


def test_cancel_during_execution_stops_immediately_and_leaves_zero_twist() -> None:
    commands = []
    adapter = None

    def cancel_on_first_sleep(_: float) -> None:
        assert adapter is not None
        adapter.cancel_step()

    adapter = StationaryExpressionAdapter(
        commands.append,
        publish_rate_hz=10.0,
        stop_publish_count=3,
        sleep=cancel_on_first_sleep,
    )

    assert not adapter.execute_step({}, None, 2.0)
    assert len(commands) < 20
    assert commands
    assert all(_is_zero(command) for command in commands)


def test_canceled_sequence_never_starts_next_unit() -> None:
    interrupt = InterruptManager()
    executed = []

    class CancelingAdapter:
        def execute_step(self, unit_config, ctx, duration):
            del ctx, duration
            executed.append(unit_config["unit_id"])
            interrupt.request_cancel()
            return False

    stage_executor = StageExecutor(
        interrupt_manager=interrupt,
        action_catalog={
            "FIRST": {
                "unit_id": "FIRST",
                "unit_type": "atomic_action",
                "interrupt_policy": "immediate",
                "timeout_sec": 1.0,
            },
            "SECOND": {
                "unit_id": "SECOND",
                "unit_type": "atomic_action",
                "interrupt_policy": "immediate",
                "timeout_sec": 1.0,
            },
        },
        controller_routes={
            "FIRST": "stationary_expression",
            "SECOND": "stationary_expression",
        },
        controller_adapters={"stationary_expression": CancelingAdapter()},
    )
    result = stage_executor.execute_stage(
        {
            "stage_id": "expression",
            "selection_policy": "sequence",
            "required": True,
            "candidates": [{"unit_id": "FIRST"}, {"unit_id": "SECOND"}],
        },
        ExecutionContext.from_goal("test", {}),
    )

    assert not result.success
    assert executed == ["FIRST"]
