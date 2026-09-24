"""Fail-closed contract tests for voice-WAITING in-place expressions."""

from __future__ import annotations

from pathlib import Path

import pytest

from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.interrupt_manager import InterruptManager
from marsdog_action_executor.result_evaluator import ResultEvaluator
from marsdog_action_executor.stage_executor import StageExecutor
from marsdog_action_executor.stationary_policy import (
    INPLACE_WITH_HUMAN_BEHAVIORS,
    validate_inplace_plan,
)


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


@pytest.fixture()
def loader() -> ConfigLoader:
    config = ConfigLoader(CONFIG_DIR)
    config.load_all()
    return config


def _check(loader: ConfigLoader, name: str, params: dict[str, object]):
    template = loader.get_behavior_template(name)
    assert template is not None
    return validate_inplace_plan(
        name,
        params,
        template["stages"],
        loader.get_controller_routes(),
        loader.navigation_config,
    )


@pytest.mark.parametrize("behavior_name", sorted(INPLACE_WITH_HUMAN_BEHAVIORS))
def test_all_inplace_behaviors_require_and_accept_inplace_policy(
    loader: ConfigLoader,
    behavior_name: str,
) -> None:
    missing = _check(loader, behavior_name, {"interaction_id": "voice-42"})
    assert not missing.valid
    assert missing.reason == "inplace_policy_required"

    valid = _check(
        loader,
        behavior_name,
        {"mobility_policy": "in_place", "interaction_id": "voice-42"},
    )
    assert valid.valid


def test_inplace_plan_rejects_approach_unit_before_execution(
    loader: ConfigLoader,
) -> None:
    stages = [{
        "stage_id": "expression",
        "candidates": [{"unit_id": "ACT_APPROACH_VISUAL_TARGET"}],
    }]
    check = validate_inplace_plan(
        "expressJoyInPlaceWithHuman",
        {"mobility_policy": "in_place"},
        stages,
        loader.get_controller_routes(),
        loader.navigation_config,
    )
    assert not check.valid
    assert check.reason == "inplace_policy_violation"


def test_interaction_id_is_preserved_but_does_not_change_inplace_plan(
    loader: ConfigLoader,
) -> None:
    params = {
        "interaction_id": "voice-session-7",
        "mobility_policy": "in_place",
        "target": {
            "target_type": "human",
            "vision_epoch": "epoch-1",
            "target_id": "person-3",
        },
    }
    ctx = ExecutionContext.from_goal(
        "expressCuriosityInPlaceWithHuman", params
    )
    assert ctx.interaction_id == "voice-session-7"
    assert ctx.mobility_policy == "in_place"
    assert _check(loader, ctx.requested_behavior_name, ctx.params).valid


@pytest.mark.parametrize("behavior_name", sorted(INPLACE_WITH_HUMAN_BEHAVIORS))
def test_legal_inplace_plan_executes_expression_successfully(
    loader: ConfigLoader,
    behavior_name: str,
) -> None:
    template = loader.get_behavior_template(behavior_name)
    assert template is not None
    executed = []

    class _Go2ExpressionAdapter:
        def execute_step(self, unit_config, ctx, duration):
            executed.append((unit_config["unit_id"], ctx.current_stage, duration))
            return True

    executor = StageExecutor(
        interrupt_manager=InterruptManager(),
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes(),
        controller_adapters={"go2": _Go2ExpressionAdapter()},
    )
    ctx = ExecutionContext.from_goal(
        behavior_name,
        {
            "mobility_policy": "in_place",
            "interaction_id": "voice-session-success",
        },
    )

    result = executor.execute_stage(template["stages"][0], ctx)

    assert result.success
    assert ctx.completed_stages == ["expression"]
    assert len(ctx.executed_units) == 1
    assert len(executed) == 1
    assert executed[0][0] in {
        candidate["unit_id"]
        for candidate in template["stages"][0]["candidates"]
    }


def test_canceled_inplace_context_cannot_be_reported_as_success() -> None:
    ctx = ExecutionContext.from_goal(
        "expressJoyInPlaceWithHuman",
        {
            "mobility_policy": "in_place",
            "interaction_id": "voice-session-canceled",
        },
    )
    ctx.resolved_behavior_name = ctx.requested_behavior_name
    ctx.cancel_requested = True

    result = ResultEvaluator().evaluate(
        ctx,
        {"expression": False},
        "all_required_stages_completed",
    )

    assert not result.success
    assert result.status == "canceled"
    assert result.result_code == "CANCELED"
