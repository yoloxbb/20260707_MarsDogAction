from __future__ import annotations

import time
from pathlib import Path

import pytest

from marsdog_action_executor.adapters.velocity import TwistCommand
from marsdog_action_executor.adapters.target_approach_adapter import (
    TargetApproachAdapter,
    TargetApproachResult,
)
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.stage_executor import StageExecutor
from marsdog_action_executor.interrupt_manager import InterruptManager
from marsdog_action_executor.result_evaluator import ResultEvaluator
from marsdog_action_executor.ros_node import _dispatch_visual_event
from marsdog_action_executor.units.unit_executors import TaskExecutor, UnitState


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


class FakeClock:
    def __init__(self) -> None:
        self.now = 1.0
        self.on_sleep = None

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration
        if self.on_sleep is not None:
            self.on_sleep(self.now)


def _target_context(
    *,
    allow_bbox_fallback: bool = False,
    policy_overrides: dict | None = None,
) -> ExecutionContext:
    policy = {
        "interaction_id": "voice-session-1",
        "strict_target_lock": True,
        "allow_target_switch": False,
        "desired_distance_m": 1.2,
        "minimum_safe_distance_m": 0.8,
        "target_max_age_ms": 300.0,
        "target_lost_timeout_sec": 0.3,
        "speaker_allow_bbox_distance_fallback": allow_bbox_fallback,
    }
    policy.update(policy_overrides or {})
    ctx = ExecutionContext.from_goal(
        "approach_voice_caller",
        {
            **policy,
            "target": {
                "target_type": "human",
                "vision_epoch": "vision-epoch-1",
                "target_id": "human-17",
            },
        },
    )
    ctx.resolved_behavior_name = "approach_voice_caller"
    return ctx


def _payload(
    *,
    distance_m=2.0,
    center_x: float = 0.5,
    epoch: str = "vision-epoch-1",
    target_id: str = "human-17",
    confidence: float = 0.9,
    bbox_height: float = 0.25,
    extras: list[dict] | None = None,
    sequence: int = 1,
    tracking_state: str = "tracking",
    range_valid: bool | None = None,
) -> dict:
    if range_valid is None:
        range_valid = distance_m is not None
    candidate = {
        "vision_epoch": epoch,
        "target_id": target_id,
        "target_type": "human",
        "tracking_state": tracking_state,
        "confidence": confidence,
        "last_seen_age_ms": 0.0,
        "body_center": [center_x, 0.5],
        "bbox": [max(0.0, center_x - 0.1), 0.1, 0.2, bbox_height],
        "range_valid": range_valid,
    }
    if distance_m is not None:
        candidate["distance_m"] = distance_m
    return {
        "schema_version": 1,
        "header": {
            "stamp": 1000.0 + sequence,
            "frame_id": "camera_link",
        },
        "vision_epoch": epoch,
        "sequence": sequence,
        "snapshot_id": f"{epoch}:{sequence}",
        "human_candidates": [candidate, *(extras or [])],
    }


def _sequence_for(now: float) -> int:
    return max(2, int(round((now - 1.0) * 10.0)) + 1)


def _adapter(
    commands: list[TwistCommand],
    clock: FakeClock,
    **overrides,
) -> TargetApproachAdapter:
    options = {
        "publish_rate_hz": 10.0,
        "visual_timeout_sec": 0.3,
        "acquire_timeout_sec": 0.4,
        "lost_timeout_sec": 0.3,
        "approach_timeout_sec": 2.0,
        "arrival_hold_sec": 0.2,
        "stop_publish_count": 2,
        "max_linear_accel": 10.0,
        "max_angular_accel": 10.0,
        "monotonic": clock.monotonic,
        "sleep": clock.sleep,
    }
    options.update(overrides)
    return TargetApproachAdapter(commands.append, **options)


def test_metric_target_is_locked_approached_stopped_and_reports_ready() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock)
    adapter.update_visual(_payload(distance_m=2.0), now=clock.now)

    def refresh(now: float) -> None:
        distance = 2.0 if now < 1.4 else 1.2
        distractor = {
            "vision_epoch": "vision-epoch-1",
            "target_id": "human-99",
            "target_type": "human",
            "tracking_state": "tracking",
            "confidence": 0.99,
            "last_seen_age_ms": 0.0,
            "body_center": [0.9, 0.5],
            "bbox": [0.8, 0.1, 0.2, 0.25],
            "distance_m": 0.8,
            "range_valid": True,
        }
        adapter.update_visual(
            _payload(
                distance_m=distance,
                extras=[distractor],
                sequence=_sequence_for(now),
            ),
            now=now,
        )

    clock.on_sleep = refresh
    ctx = _target_context()
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        ctx,
        timeout_sec=2.0,
    )

    assert outcome.success
    assert outcome.reason == "target_reached"
    assert outcome.metadata == {
        "state": "ready",
        "ready": True,
        "reason": "target_reached",
        "vision_epoch": "vision-epoch-1",
        "target_id": "human-17",
        "distance_m": 1.2,
        "distance_source": "metric",
        "bbox_distance_fallback": False,
        "demo_distance_fallback": False,
        "effective_policy": {
            "desired_distance_m": 1.2,
            "minimum_safe_distance_m": 0.8,
            "target_max_age_ms": 300.0,
            "target_lost_timeout_sec": 0.3,
            "target_min_confidence": 0.5,
            "bbox_distance_fallback": False,
            "arrival_hold_sec": 0.2,
            "linear_gain": 0.5,
            "max_linear_x": 0.15,
            "max_linear_accel": 10.0,
            "bbox_target_height": 0.68,
            "bbox_height_deadband": 0.05,
        },
    }
    moving = [command for command in commands if not command.is_zero]
    assert moving
    assert any(command.linear_x > 0.0 for command in moving)
    # The higher-confidence distractor is never used for steering.
    assert all(command.angular_z == 0.0 for command in moving)
    assert commands[-2:] == [TwistCommand(), TwistCommand()]
    assert ctx.metadata["target_approach"]["ready"] is True


def test_large_heading_error_rotates_without_forward_motion() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock, approach_timeout_sec=0.35)
    adapter.update_visual(_payload(distance_m=3.0, center_x=0.9), now=clock.now)
    clock.on_sleep = lambda now: adapter.update_visual(
        _payload(
            distance_m=3.0,
            center_x=0.9,
            sequence=_sequence_for(now),
        ),
        now=now,
    )

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID}, _target_context(), 0.35
    )

    assert outcome.timed_out
    moving = [command for command in commands if not command.is_zero]
    assert moving
    assert all(command.linear_x == 0.0 for command in moving)
    assert all(command.angular_z < 0.0 for command in moving)
    assert commands[-1].is_zero


def test_epoch_change_fails_closed_and_never_reuses_track_id() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock)
    adapter.update_visual(_payload(distance_m=2.0), now=clock.now)

    def restart_vision(now: float) -> None:
        adapter.update_visual(
            _payload(
                distance_m=2.0,
                epoch="vision-epoch-2",
                target_id="human-17",
                sequence=1,
            ),
            now=now,
        )

    clock.on_sleep = restart_vision
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID}, _target_context(), 2.0
    )

    assert not outcome.success
    assert outcome.reason == "vision_epoch_changed"
    assert commands[-1].is_zero


def test_bbox_distance_is_rejected_by_default() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock)
    adapter.update_visual(_payload(distance_m=None), now=clock.now)
    clock.on_sleep = lambda now: adapter.update_visual(
        _payload(distance_m=None, sequence=_sequence_for(now)), now=now
    )

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID}, _target_context(), 1.0
    )

    assert not outcome.success
    assert outcome.reason == "metric_distance_unavailable"
    assert not [command for command in commands if not command.is_zero]


def test_bbox_height_fallback_requires_explicit_flag() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(
        commands,
        clock,
        allow_bbox_distance_fallback=True,
        demo_target_height=0.6,
    )
    adapter.update_visual(
        _payload(distance_m=None, bbox_height=0.25), now=clock.now
    )
    clock.on_sleep = lambda now: adapter.update_visual(
        _payload(
            distance_m=None,
            bbox_height=0.6 if now >= 1.4 else 0.25,
            sequence=_sequence_for(now),
        ),
        now=now,
    )

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _target_context(allow_bbox_fallback=True),
        2.0,
    )

    assert outcome.success
    assert outcome.metadata["distance_source"] == "bbox_height"
    assert outcome.metadata["demo_distance_fallback"] is True
    assert outcome.metadata["bbox_distance_fallback"] is True
    assert any(command.linear_x > 0.0 for command in commands)


def test_cancel_stops_immediately() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock)
    adapter.update_visual(_payload(distance_m=3.0), now=clock.now)

    def cancel(now: float) -> None:
        adapter.update_visual(
            _payload(
                distance_m=3.0,
                sequence=_sequence_for(now),
            ),
            now=now,
        )
        adapter.cancel_task()

    clock.on_sleep = cancel
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID}, _target_context(), 2.0
    )

    assert outcome.canceled
    assert outcome.reason == "target_approach_canceled"
    assert commands[-1].is_zero


def test_task_executor_calls_real_adapter_and_preserves_metadata() -> None:
    class StubTargetApproach:
        def execute_task(self, unit_config, ctx, timeout):
            assert unit_config["unit_id"] == (
                "ACT_INTERACT_APPROACH_VOICE_CALLER"
            )
            assert timeout == 160.0
            metadata = {"state": "ready", "ready": True}
            ctx.metadata["target_approach"] = metadata
            return TargetApproachResult(True, "target_reached", metadata)

    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    executor = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes(),
        controller_adapters={"person_nav_approach": StubTargetApproach()},
    )
    ctx = _target_context()

    result = executor.execute_stage(
        loader.get_behavior_template("approach_voice_caller")["stages"][0],
        ctx,
    )

    assert result.success
    assert result.unit_id == "ACT_INTERACT_APPROACH_VOICE_CALLER"
    assert ctx.executed_units == ["ACT_INTERACT_APPROACH_VOICE_CALLER"]
    assert ctx.metadata["target_approach"]["ready"] is True


def test_exact_behavior_action_and_route_contract() -> None:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()

    template = loader.get_behavior_template("approach_voice_caller")
    assert template["stages"][0]["candidates"] == [
        {"unit_id": "ACT_INTERACT_APPROACH_VOICE_CALLER"}
    ]
    assert (
        loader.action_catalog["ACT_INTERACT_APPROACH_VOICE_CALLER"][
            "unit_type"
        ]
        == "task"
    )
    assert (
        loader.controller_routes["routes"][
            "ACT_INTERACT_APPROACH_VOICE_CALLER"
        ]
        == "person_nav_approach"
    )


def test_target_contract_requires_epoch_and_target_id() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock)
    ctx = ExecutionContext.from_goal(
        "approach_voice_caller",
        {"target": {"target_type": "human", "target_id": "human-17"}},
    )

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID}, ctx, timeout_sec=1.0
    )

    assert not outcome.success
    assert outcome.reason == "target_ref_requires_vision_epoch_and_target_id"
    assert commands[-1].is_zero


@pytest.mark.parametrize(
    "mutation",
    [
        "wrong_id",
        "temporarily_lost",
        "low_confidence",
        "invalid_range",
        "wrong_type",
    ],
)
def test_new_invalid_snapshot_stops_before_update_returns(mutation: str) -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock, approach_timeout_sec=0.7)
    adapter.update_visual(_payload(distance_m=3.0), now=clock.now)
    stop_return_indexes: list[int] = []

    def invalidate(now: float) -> None:
        payload = _payload(
            distance_m=3.0,
            sequence=_sequence_for(now),
        )
        candidate = payload["human_candidates"][0]
        if mutation == "wrong_id":
            candidate["target_id"] = "human-99"
        elif mutation == "temporarily_lost":
            candidate["tracking_state"] = "temporarily_lost"
        elif mutation == "low_confidence":
            candidate["confidence"] = 0.1
        elif mutation == "invalid_range":
            candidate["range_valid"] = False
        elif mutation == "wrong_type":
            candidate["target_type"] = "animal"
        adapter.update_visual(payload, now=now)
        assert commands[-1].is_zero
        stop_return_indexes.append(len(commands))

    clock.on_sleep = invalidate
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _target_context(),
        0.7,
    )

    assert not outcome.success
    assert stop_return_indexes
    assert all(
        command.is_zero for command in commands[stop_return_indexes[0] :]
    )


def test_repeated_invalid_snapshots_coalesce_stop_requests() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(
        commands,
        clock,
        approach_timeout_sec=0.7,
        reacquire_min_consecutive_frames=1,
    )
    adapter.update_visual(_payload(distance_m=3.0), now=clock.now)

    def invalidate(now: float) -> None:
        adapter.update_visual(
            _payload(
                distance_m=3.0,
                sequence=_sequence_for(now),
                tracking_state=(
                    "tracking" if now < 1.15 else "temporarily_lost"
                ),
            ),
            now=now,
        )

    clock.on_sleep = invalidate
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _target_context(),
        0.7,
    )

    assert outcome.reason == "target_lost"
    assert any(not command.is_zero for command in commands)
    # One redundant pair at start, one on the moving->stopped transition and
    # one at the terminal boundary. Repeated callbacks/control ticks add none.
    assert sum(command.is_zero for command in commands) == 6


def test_motion_resumes_only_after_three_new_valid_frames() -> None:
    published: list[tuple[float, TwistCommand]] = []
    clock = FakeClock()
    adapter = _adapter(
        [],
        clock,
        approach_timeout_sec=0.8,
        reacquire_min_consecutive_frames=3,
    )
    adapter._publish_twist = lambda command: published.append(
        (clock.now, command)
    )
    adapter.update_visual(_payload(distance_m=3.0), now=clock.now)

    def refresh(now: float) -> None:
        temporarily_lost = 1.25 < now < 1.35
        adapter.update_visual(
            _payload(
                distance_m=3.0,
                sequence=_sequence_for(now),
                tracking_state=(
                    "temporarily_lost" if temporarily_lost else "tracking"
                ),
            ),
            now=now,
        )

    clock.on_sleep = refresh
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _target_context(policy_overrides={"target_lost_timeout_sec": 0.7}),
        0.8,
    )

    assert outcome.timed_out
    move_times = [now for now, command in published if not command.is_zero]
    assert any(1.19 <= now <= 1.21 for now in move_times)
    assert not any(1.3 < now < 1.6 for now in move_times)
    assert any(now >= 1.6 for now in move_times)


def test_bbox_arrival_hysteresis_ignores_small_threshold_jitter() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(
        commands,
        clock,
        allow_bbox_distance_fallback=True,
        demo_target_height=0.68,
        demo_height_deadband=0.05,
        bbox_height_hysteresis=0.03,
        reacquire_min_consecutive_frames=1,
    )
    adapter.update_visual(
        _payload(distance_m=None, bbox_height=0.64), now=clock.now
    )
    clock.on_sleep = lambda now: adapter.update_visual(
        _payload(
            distance_m=None,
            bbox_height=0.62,
            sequence=_sequence_for(now),
        ),
        now=now,
    )

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _target_context(allow_bbox_fallback=True),
        1.0,
    )

    assert outcome.success
    assert outcome.reason == "target_reached"
    assert not [command for command in commands if not command.is_zero]


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("schema_version", 2),
        ("header", {}),
        ("snapshot_id", ""),
        ("sequence", 0),
    ],
)
def test_invalid_visual_envelope_fails_closed(
    field: str,
    value,
) -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock, approach_timeout_sec=0.6)
    adapter.update_visual(_payload(distance_m=3.0), now=clock.now)
    invalidated_at: list[int] = []

    def invalidate(now: float) -> None:
        payload = _payload(
            distance_m=3.0,
            sequence=_sequence_for(now),
        )
        payload[field] = value
        assert adapter.update_visual(payload, now=now) is False
        assert commands[-1].is_zero
        invalidated_at.append(len(commands))

    clock.on_sleep = invalidate
    adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _target_context(),
        0.6,
    )

    assert invalidated_at
    assert all(command.is_zero for command in commands[invalidated_at[0] :])


@pytest.mark.parametrize("raw_data", ["{not-json", "[]", "null"])
def test_malformed_visual_event_stops_active_approach_before_return(
    raw_data: str,
) -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock, approach_timeout_sec=0.6)
    adapter.update_visual(_payload(distance_m=3.0), now=clock.now)
    stopped_at: list[int] = []

    def invalidate(_now: float) -> None:
        assert not _dispatch_visual_event(
            raw_data,
            target_approach_adapter=adapter,
        )
        assert commands[-1].is_zero
        stopped_at.append(len(commands))

    clock.on_sleep = invalidate
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _target_context(),
        0.6,
    )

    assert not outcome.success
    assert stopped_at
    assert all(command.is_zero for command in commands[stopped_at[0] :])


def test_same_epoch_sequence_must_increase_and_retired_epoch_is_ignored() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock)

    assert adapter.update_visual(_payload(sequence=1), now=clock.now)
    assert not adapter.update_visual(_payload(sequence=1), now=clock.now)
    assert adapter.update_visual(
        _payload(epoch="vision-epoch-2", sequence=1),
        now=clock.now,
    )
    assert not adapter.update_visual(
        _payload(epoch="vision-epoch-1", sequence=2),
        now=clock.now,
    )
    assert adapter._stream_epoch == "vision-epoch-2"
    assert adapter._stream_sequence == 1


def test_cancel_barrier_never_publishes_move_after_cancel_returns() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock)
    adapter.update_visual(_payload(distance_m=3.0), now=clock.now)
    clock.on_sleep = lambda now: adapter.update_visual(
        _payload(distance_m=3.0, sequence=_sequence_for(now)),
        now=now,
    )
    original_linear = adapter._linear_command
    linear_calls = 0
    cancel_return_index: int | None = None

    def cancel_between_check_and_publish(*args):
        nonlocal linear_calls, cancel_return_index
        linear_calls += 1
        value = original_linear(*args)
        if linear_calls == 2:
            adapter.cancel_task()
            cancel_return_index = len(commands)
        return value

    adapter._linear_command = cancel_between_check_and_publish
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _target_context(),
        2.0,
    )

    assert outcome.canceled
    assert cancel_return_index is not None
    assert all(
        command.is_zero for command in commands[cancel_return_index:]
    )


def test_goal_policy_uses_only_equal_or_stricter_effective_limits() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(
        commands,
        clock,
        visual_timeout_sec=0.8,
        lost_timeout_sec=1.0,
        allow_bbox_distance_fallback=True,
    )
    adapter.update_visual(_payload(distance_m=1.2), now=clock.now)
    clock.on_sleep = lambda now: adapter.update_visual(
        _payload(distance_m=1.2, sequence=_sequence_for(now)),
        now=now,
    )
    ctx = _target_context(
        allow_bbox_fallback=True,
        policy_overrides={
            "desired_distance_m": 0.5,
            "minimum_safe_distance_m": 0.4,
            "target_max_age_ms": 5000.0,
            "target_lost_timeout_sec": 5.0,
            # These profile fields are configuration-owned and must be
            # ignored at the public Goal boundary.
            "arrival_hold_sec": 50.0,
            "approach_linear_gain": 99.0,
            "approach_max_linear_x": 99.0,
            "approach_max_linear_accel": 99.0,
            "bbox_target_height": 1.0,
            "bbox_height_deadband": 0.0,
        },
    )

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        ctx,
        2.0,
    )

    assert outcome.success
    assert outcome.metadata["effective_policy"] == {
        "desired_distance_m": 1.2,
        "minimum_safe_distance_m": 0.8,
        "target_max_age_ms": 800.0,
        "target_lost_timeout_sec": 1.0,
        "target_min_confidence": 0.5,
        "bbox_distance_fallback": True,
        "arrival_hold_sec": 0.2,
        "linear_gain": 0.5,
        "max_linear_x": 0.15,
        "max_linear_accel": 10.0,
        "bbox_target_height": 0.68,
        "bbox_height_deadband": 0.05,
    }


def test_goal_cannot_enable_bbox_fallback_for_production_adapter() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock, allow_bbox_distance_fallback=False)
    adapter.update_visual(_payload(distance_m=None), now=clock.now)
    clock.on_sleep = lambda now: adapter.update_visual(
        _payload(distance_m=None, sequence=_sequence_for(now)),
        now=now,
    )

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _target_context(allow_bbox_fallback=True),
        1.0,
    )

    assert not outcome.success
    assert outcome.reason == "metric_distance_unavailable"
    assert outcome.metadata["effective_policy"][
        "bbox_distance_fallback"
    ] is False
    assert not [command for command in commands if not command.is_zero]


@pytest.mark.parametrize(
    ("field", "value", "reason"),
    [
        ("strict_target_lock", False, "strict_target_lock_required"),
        ("allow_target_switch", True, "target_switch_must_be_false"),
        ("desired_distance_m", 0, "invalid_desired_distance_m"),
        (
            "minimum_safe_distance_m",
            float("nan"),
            "invalid_minimum_safe_distance_m",
        ),
        ("target_max_age_ms", -1, "invalid_target_max_age_ms"),
        (
            "target_lost_timeout_sec",
            "bad",
            "invalid_target_lost_timeout_sec",
        ),
        (
            "target_min_confidence",
            1.1,
            "invalid_target_min_confidence",
        ),
    ],
)
def test_invalid_goal_policy_is_rejected(
    field: str,
    value,
    reason: str,
) -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock)
    ctx = _target_context(policy_overrides={field: value})

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        ctx,
        1.0,
    )

    assert not outcome.success
    assert outcome.reason == reason
    assert commands[-1].is_zero


def test_interaction_id_and_explicit_human_type_are_required() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock)
    missing_session = _target_context()
    missing_session.interaction_id = None
    missing_type = _target_context()
    missing_type.target.pop("target_type")

    session_outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        missing_session,
        1.0,
    )
    type_outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        missing_type,
        1.0,
    )

    assert session_outcome.reason == "interaction_id_required"
    assert type_outcome.reason == "human_target_required"


def test_short_behavior_goal_budget_caps_real_task_timeout() -> None:
    class CapturingAdapter:
        def __init__(self) -> None:
            self.timeout = None

        def execute_task(self, unit_config, ctx, timeout):
            self.timeout = timeout
            return TargetApproachResult(
                False,
                "target_approach_timeout",
                timed_out=True,
            )

    adapter = CapturingAdapter()
    ctx = _target_context()
    ctx.runtime_deadline_monotonic = time.monotonic() + 0.05
    executor = TaskExecutor(
        adapter=adapter,
        interrupt_manager=InterruptManager(),
    )

    result = executor.execute(
        {
            "unit_id": "ACT_INTERACT_APPROACH_VOICE_CALLER",
            "timeout_sec": 20.0,
            "interrupt_policy": "immediate",
        },
        ctx,
    )

    assert result.state == UnitState.TIMEOUT
    assert adapter.timeout is not None
    assert 0.0 < adapter.timeout <= 0.05


def test_requested_50ms_timeout_is_not_expanded_to_control_period() -> None:
    commands: list[TwistCommand] = []
    adapter = TargetApproachAdapter(
        commands.append,
        publish_rate_hz=2.0,
        visual_timeout_sec=1.0,
        acquire_timeout_sec=1.0,
        lost_timeout_sec=1.0,
        approach_timeout_sec=2.0,
        arrival_hold_sec=0.2,
        stop_publish_count=2,
        max_linear_accel=10.0,
        max_angular_accel=10.0,
    )
    adapter.update_visual(_payload(distance_m=3.0))

    started_at = time.monotonic()
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _target_context(),
        timeout_sec=0.05,
    )
    elapsed = time.monotonic() - started_at

    assert outcome.timed_out
    assert outcome.reason == "target_approach_timeout"
    # The controller period is 0.5 s.  The short behavior budget must win.
    assert 0.03 <= elapsed < 0.25
    assert commands[-2:] == [TwistCommand(), TwistCommand()]


def test_stage_and_behavior_result_preserve_task_timeout_reason() -> None:
    class TimeoutAdapter:
        def execute_task(self, unit_config, ctx, timeout):
            return TargetApproachResult(
                False,
                "target_approach_timeout",
                {"state": "timeout"},
                timed_out=True,
            )

    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    executor = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes(),
        controller_adapters={"person_nav_approach": TimeoutAdapter()},
    )
    ctx = _target_context()
    ctx.resolved_behavior_name = "approach_voice_caller"

    stage = executor.execute_stage(
        loader.get_behavior_template("approach_voice_caller")["stages"][0],
        ctx,
    )
    result = ResultEvaluator().evaluate(
        ctx,
        {stage.stage_id: stage.success},
        "all_required_stages_completed",
    )

    assert not stage.success
    assert stage.message == "target_approach_timeout"
    assert result.status == "timeout"
    assert result.reason == "target_approach_timeout"
