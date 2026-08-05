import json

from marsdog_action_executor.ros_node import _make_result


def test_execute_behavior_result_carries_recharge_energy_metadata() -> None:
    result = _make_result(
        "goal-1",
        "behavior-1",
        "recharge",
        status="SUCCESS",
        result="completed",
        metadata={"energyValue": 88},
    )
    assert result.status == "SUCCESS"
    assert result.behavior_name == "recharge"
    assert json.loads(result.metadata_json) == {"energyValue": 88}
