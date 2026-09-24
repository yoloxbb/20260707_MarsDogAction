"""Regression tests for the end-to-end ExecuteBehavior time budget."""

from __future__ import annotations

import time

from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.ros_node import (
    _normalise_goal_timeout, _outer_runtime_deadline,
)
from marsdog_action_executor.units.base_unit_executor import UnitResult, UnitState
from marsdog_action_executor.units.unit_executors import (
    AtomicActionExecutor,
    CompositeActionExecutor,
    TaskExecutor,
)


def _context(deadline_offset_sec: float) -> ExecutionContext:
    ctx = ExecutionContext.from_goal("test", {})
    ctx.runtime_deadline_monotonic = time.monotonic() + deadline_offset_sec
    return ctx


def test_atomic_action_receives_only_remaining_goal_budget() -> None:
    class CapturingAdapter:
        def __init__(self) -> None:
            self.duration = None

        def execute_step(self, unit_config, ctx, duration):
            del unit_config, ctx
            self.duration = duration
            return True

    adapter = CapturingAdapter()
    result = AtomicActionExecutor(adapter=adapter).execute(
        {"unit_id": "ACT_TEST", "timeout_sec": 10.0},
        _context(0.05),
    )

    assert result.state == UnitState.SUCCESS
    assert adapter.duration is not None
    assert 0.0 < adapter.duration <= 0.05


def test_atomic_action_cannot_report_success_after_goal_deadline() -> None:
    class SlowAdapter:
        def execute_step(self, unit_config, ctx, duration):
            del unit_config, ctx, duration
            time.sleep(0.01)
            return True

    result = AtomicActionExecutor(adapter=SlowAdapter()).execute(
        {"unit_id": "ACT_TEST", "timeout_sec": 10.0},
        _context(0.001),
    )

    assert result.state == UnitState.TIMEOUT
    assert result.message == "behavior_goal_timeout"


def test_task_unit_result_success_cannot_bypass_goal_deadline() -> None:
    class SlowTaskAdapter:
        def execute_task(self, unit_config, ctx, timeout):
            del unit_config, ctx, timeout
            time.sleep(0.01)
            return UnitResult("ACT_TEST", UnitState.SUCCESS)

    result = TaskExecutor(adapter=SlowTaskAdapter()).execute(
        {"unit_id": "ACT_TEST", "timeout_sec": 10.0},
        _context(0.001),
    )

    assert result.state == UnitState.TIMEOUT
    assert result.message == "behavior_goal_timeout"


def test_composite_propagates_sub_action_timeout() -> None:
    class TimeoutExecutor:
        def execute(self, unit_config, ctx):
            del unit_config, ctx
            return UnitResult(
                "ACT_SUB",
                UnitState.TIMEOUT,
                message="behavior_goal_timeout",
            )

    result = CompositeActionExecutor(
        sub_executors={"atomic_action": TimeoutExecutor()}
    ).execute(
        {
            "unit_id": "ACT_COMPOSITE",
            "sub_actions": [{"unit_id": "ACT_SUB"}],
        },
        _context(1.0),
    )

    assert result.state == UnitState.TIMEOUT
    assert result.message == "behavior_goal_timeout"


def test_invalid_goal_timeouts_fail_closed_to_zero() -> None:
    assert _normalise_goal_timeout("bad") == 0.0
    assert _normalise_goal_timeout(float("nan")) == 0.0
    assert _normalise_goal_timeout(float("inf")) == 0.0
    assert _normalise_goal_timeout(-1.0) == 0.0
    assert _normalise_goal_timeout(2.5) == 2.5


def test_zero_outer_timeout_keeps_internal_unit_budget_finite() -> None:
    ctx = _context(0.05)
    ctx.runtime_deadline_monotonic = _outer_runtime_deadline(time.monotonic(), 0.0)
    assert ctx.remaining_runtime_sec(time.monotonic()) is None
    assert _outer_runtime_deadline(10.0, 2.5) == 12.5

    class Capture:
        def execute_step(self, unit_config, ctx, duration):
            assert duration == 7.0
            return True

    result = AtomicActionExecutor(adapter=Capture()).execute(
        {"unit_id": "ACT_TEST", "timeout_sec": 7.0}, ctx,
    )
    assert result.state == UnitState.SUCCESS
