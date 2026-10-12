"""Random combinations vary in length/order and stop on incomplete execution."""
from types import SimpleNamespace

import pytest

from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.stage_executor import StageExecutor


def stage():
    return {
        "stage_id": "expression", "selection_policy": "random_n", "required": True,
        "selection_count": {"min": 2, "max": 3},
        "candidates": [{"unit_id": f"ACT_{i}"} for i in range(8)],
    }


def executor(adapter):
    return StageExecutor(controller_routes={"_default": "go2"},
                         controller_adapters={"go2": adapter})


def test_random_combination_varies_count_order_and_selection_reproducibly():
    obj = executor(SimpleNamespace(execute_step=lambda *args: True))
    sequences = []
    for seed in range(64):
        ctx = ExecutionContext.from_goal("test", {})
        assert obj.execute_stage(stage(), ctx, seed=seed).success
        sequences.append(tuple(ctx.executed_units))
        repeated = ExecutionContext.from_goal("test", {})
        assert obj.execute_stage(stage(), repeated, seed=seed).success
        assert tuple(repeated.executed_units) == sequences[-1]
    assert {len(s) for s in sequences} == {2, 3}
    assert len(set(sequences)) > 40
    assert any(s != tuple(sorted(s)) for s in sequences)


def test_random_combination_filters_unavailable_and_ineligible_before_sampling():
    adapter = SimpleNamespace(can_execute=lambda unit: unit not in {"ACT_0", "ACT_1"},
                              execute_step=lambda *args: True)
    config = stage()
    config["candidates"][2]["conditions"] = ["owner_visible"]
    obj = executor(adapter)
    for seed in range(16):
        ctx = ExecutionContext.from_goal("test", {})
        assert obj.execute_stage(config, ctx, seed=seed).success
        assert len(ctx.executed_units) in {2, 3}
        assert not set(ctx.executed_units) & {"ACT_0", "ACT_1", "ACT_2"}


@pytest.mark.parametrize("available", [set(), {"ACT_0"}])
def test_insufficient_eligible_candidates_cannot_execute_partial_combination(available):
    adapter = SimpleNamespace(can_execute=lambda unit: unit in available,
                              execute_step=lambda *args: pytest.fail("must not execute"))
    ctx = ExecutionContext.from_goal("test", {})
    result = executor(adapter).execute_stage(stage(), ctx, seed=1)
    assert not result.success
    assert ctx.executed_units == ctx.completed_stages == []


@pytest.mark.parametrize("interrupted", ["failure", "cancel", "deadline"])
def test_incomplete_combination_stops_remaining_actions(interrupted):
    calls = []

    def execute(config, ctx, duration):
        calls.append(config["unit_id"])
        if interrupted == "cancel":
            ctx.cancel_requested = True
        elif interrupted == "deadline":
            ctx.runtime_deadline_monotonic = 0.0
        return interrupted != "failure" and not ctx.cancel_requested

    obj = executor(SimpleNamespace(execute_step=execute, last_error="injected_failure"))
    ctx = ExecutionContext.from_goal("test", {})
    result = obj.execute_stage(stage(), ctx, seed=1)
    assert not result.success
    assert len(calls) == 1
    assert ctx.completed_stages == []
