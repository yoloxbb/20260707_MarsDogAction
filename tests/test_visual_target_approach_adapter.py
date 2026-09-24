from __future__ import annotations

from pathlib import Path

import pytest

from marsdog_action_executor.adapters.velocity import TwistCommand
from marsdog_action_executor.adapters.visual_target_approach_adapter import (
    VisualTargetApproachAdapter,
)
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.stage_executor import StageExecutor


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


def _policies() -> dict:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    return loader.visual_target_approach_config["behavior_policies"]


def _go2_policies() -> dict:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    policies = dict(loader.visual_target_approach_config["behavior_policies"])
    policies.update(
        loader.visual_target_approach_config["go2_owner_approach_policies"]
    )
    return policies


def test_visual_approach_platforms_apply_tighter_linear_clamps() -> None:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()

    approach = loader.visual_target_approach_config
    assert approach["max_linear_x"] == 0.35
    assert loader.go2_sport_config["limits"]["max_linear_x"] <= (
        approach["max_linear_x"]
    )
    assert loader.lite3_action_config["limits"]["max_linear_x"] <= (
        approach["max_linear_x"]
    )
    assert approach["linear_gain"] == 1.0
    assert approach["max_linear_accel"] == 0.5
    assert approach["bbox_target_height"] == 0.80
    assert approach["bbox_height_deadband"] == 0.04
    assert approach["bbox_height_hysteresis"] == 0.03
    assert approach["distance_hysteresis_m"] == 0.10
    assert approach["reacquire_min_consecutive_frames"] == 3
    assert approach["acquire_timeout_sec"] == 5.0
    assert approach["object_detection_stream"]["confidence"] == 0.35


def _adapter(
    commands: list[TwistCommand],
    clock: FakeClock,
    **overrides,
) -> VisualTargetApproachAdapter:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    config = loader.visual_target_approach_config
    options = {
        "behavior_policies": config["behavior_policies"],
        "publish_rate_hz": 10.0,
        "visual_timeout_sec": 0.3,
        "acquire_timeout_sec": 0.4,
        "lost_timeout_sec": 0.3,
        "approach_timeout_sec": 2.0,
        "arrival_hold_sec": 0.2,
        "stop_publish_count": 2,
        "max_linear_x": config["max_linear_x"],
        "max_linear_accel": 10.0,
        "max_angular_accel": 10.0,
        "allow_bbox_distance_fallback": config[
            "allow_bbox_distance_fallback"
        ],
        "bbox_target_height": config["bbox_target_height"],
        "bbox_height_deadband": config["bbox_height_deadband"],
        # Most adapter tests exercise binding/policy behavior with a single
        # static snapshot. Dedicated tests below cover production reacquisition.
        "reacquire_min_consecutive_frames": 1,
        "monotonic": clock.monotonic,
        "sleep": clock.sleep,
    }
    options.update(overrides)
    return VisualTargetApproachAdapter(commands.append, **options)


def _context(
    behavior: str,
    target_type: str,
    target_id: str,
    **target_fields,
) -> ExecutionContext:
    ctx = ExecutionContext.from_goal(
        behavior,
        {
            "source": "need",
            "target": {
                "target_type": target_type,
                "target_id": target_id,
                **target_fields,
            },
        },
    )
    ctx.resolved_behavior_name = behavior
    return ctx


def _payload(
    target_type: str,
    *,
    distance_m: float = 3.0,
    confidence: float = 0.9,
    sequence: int = 1,
    center_x: float = 0.5,
    target_id: str | None = None,
    track_id: int = 7,
    identity: str | None = None,
    label: str | None = None,
    bbox_height: float = 0.80,
    epoch: str = "vision-1",
) -> dict:
    candidate = {
        "track_id": track_id,
        "tracking_state": "tracking",
        "confidence": confidence,
        "last_seen_age_ms": 0.0,
        "center_x": center_x,
        "distance_m": distance_m,
        "range_valid": True,
        "bbox": [max(0.0, center_x - 0.1), 0.1, 0.2, bbox_height],
    }
    if target_id is not None:
        candidate["target_id"] = target_id
    if identity is not None:
        candidate["identity"] = identity
    if label is not None:
        candidate["label"] = label
    payload = {
        "schema_version": 1,
        "header": {"stamp": 1000.0 + sequence, "frame_id": "camera_link"},
        "vision_epoch": epoch,
        "sequence": sequence,
        "snapshot_id": f"{epoch}:{sequence}",
    }
    if target_type == "human":
        candidate["target_type"] = "human"
        payload["human_candidates"] = [candidate]
    else:
        candidate["target_type"] = target_type
        payload["tracked_objects"] = [candidate]
    return payload


def _sequence(now: float) -> int:
    return max(2, int(round((now - 1.0) * 10.0)) + 1)


def _object_stream_packet(
    session_id: str,
    *,
    sequence: int = 1,
    target_id: str = "vision-1:object:7",
    label: str = "delivery box",
    distance_m: float = 0.9,
) -> dict:
    return {
        "schema_version": 2,
        "header": {
            "stamp": 2000.0 + sequence,
            "frame_id": "camera_link",
        },
        "published_at": 2000.1 + sequence,
        "sequence": sequence,
        "source": "stream",
        "status": "ok",
        "stream": {
            "active": True,
            "session_id": session_id,
        },
        "objects": [{
            "vision_epoch": "vision-1",
            "target_id": target_id,
            "target_type": "object",
            "track_id": 7,
            "label": label,
            "tracking_state": "tracking",
            "confidence": 0.9,
            "last_seen_age_ms": 0.0,
            "center_x": 0.5,
            "bbox": [0.4, 0.4, 0.2, 0.2],
            "range_valid": True,
            "distance_m": distance_m,
        }],
        "error": "",
    }


def test_nonhuman_approach_owns_session_stream_until_stopped() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    calls: list[tuple] = []
    holder: dict[str, VisualTargetApproachAdapter] = {}
    active_session = {"id": ""}

    def start(session_id, labels, rate_hz, lease_sec):
        calls.append(("start", session_id, labels, rate_hz, lease_sec))
        active_session["id"] = session_id
        holder["adapter"].update_object_detection(
            _object_stream_packet(session_id),
            now=clock.now,
        )
        return True, ""

    def stop(session_id):
        calls.append(("stop", session_id))

    adapter = _adapter(
        commands,
        clock,
        require_object_stream=True,
        start_object_detection=start,
        stop_object_detection=stop,
        object_detection_rate_hz=4.0,
    )
    holder["adapter"] = adapter

    def refresh(now: float) -> None:
        adapter.update_object_detection(
            _object_stream_packet(
                active_session["id"],
                sequence=_sequence(now),
            ),
            now=now,
        )

    clock.on_sleep = refresh
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context(
            "inspectObject",
            "object",
            "vision-1:object:7",
            vision_epoch="vision-1",
            label="delivery box",
        ),
        2.0,
    )

    assert outcome.success is True
    assert calls[0][0] == "start"
    assert calls[0][2] == ["delivery box"]
    assert calls[0][3] == 4.0
    assert calls[-1] == ("stop", calls[0][1])
    assert commands and commands[-1] == TwistCommand()


def test_required_object_stream_fails_closed_when_client_is_missing() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(
        commands,
        clock,
        require_object_stream=True,
    )

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context(
            "testAnimalBoundary",
            "animal",
            "cat",
            label="cat",
        ),
        2.0,
    )

    assert outcome.success is False
    assert outcome.reason == "object_detection_stream_unavailable"
    assert commands and all(command == TwistCommand() for command in commands)


def test_cancel_during_object_acquisition_stops_same_stream() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    sessions: list[tuple[str, str]] = []

    def start(session_id, labels, rate_hz, lease_sec):
        del labels, rate_hz, lease_sec
        sessions.append(("start", session_id))
        return True, ""

    def stop(session_id):
        sessions.append(("stop", session_id))

    adapter = _adapter(
        commands,
        clock,
        require_object_stream=True,
        start_object_detection=start,
        stop_object_detection=stop,
    )
    clock.on_sleep = lambda _now: adapter.cancel_task()

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context(
            "testAnimalBoundary",
            "animal",
            "cat",
            label="cat",
        ),
        2.0,
    )

    assert outcome.canceled is True
    assert sessions[0][0] == "start"
    assert sessions[-1] == ("stop", sessions[0][1])
    assert commands and all(command == TwistCommand() for command in commands)


@pytest.mark.parametrize(
    ("behavior", "target_type", "requested_id", "payload_kwargs", "stop"),
    [
        (
            "seekHumanInteraction",
            "human",
            "owner",
            {"target_id": "vision-1:human:7", "identity": "owner"},
            1.2,
        ),
        (
            "greetAnimal",
            "animal",
            "7",
            {"label": "dog"},
            1.2,
        ),
        (
            "inspectObject",
            "object",
            "7",
            {"label": "delivery box"},
            0.9,
        ),
    ],
)
def test_selected_target_is_approached_then_stopped(
    behavior: str,
    target_type: str,
    requested_id: str,
    payload_kwargs: dict,
    stop: float,
) -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock)
    adapter.update_visual(
        _payload(
            target_type,
            distance_m=3.0,
            bbox_height=0.25 if target_type == "human" else 0.80,
            **payload_kwargs,
        ),
        now=clock.now,
    )

    def refresh(now: float) -> None:
        adapter.update_visual(
            _payload(
                target_type,
                distance_m=stop if now >= 1.4 else 3.0,
                bbox_height=(
                    0.80 if target_type == "human" and now >= 1.4
                    else 0.25 if target_type == "human"
                    else 0.80
                ),
                sequence=_sequence(now),
                **payload_kwargs,
            ),
            now=now,
        )

    clock.on_sleep = refresh
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context(behavior, target_type, requested_id),
        2.0,
    )

    assert outcome.success
    assert outcome.reason == "target_reached"
    assert outcome.metadata["target_type"] == target_type
    assert outcome.metadata["requested_target_id"] == requested_id
    assert outcome.metadata["effective_policy"]["desired_distance_m"] == stop
    assert any(command.linear_x > 0.0 for command in commands)
    assert commands[-2:] == [TwistCommand(), TwistCommand()]


def test_animal_approach_distances_are_close_but_safety_guarded() -> None:
    policies = _policies()

    assert policies["testAnimalBoundary"]["desired_distance_m"] == 1.50
    assert policies["testAnimalBoundary"]["minimum_safe_distance_m"] == 1.00
    assert policies["testAnimalBoundary"]["min_confidence"] == 0.40
    assert policies["testAnimalBoundary"]["target_lost_timeout_sec"] == 1.8
    assert policies["greetAnimal"]["desired_distance_m"] == 1.20
    assert policies["greetAnimal"]["minimum_safe_distance_m"] == 0.80
    assert policies["greetAnimal"]["min_confidence"] == 0.40
    assert policies["greetAnimal"]["target_lost_timeout_sec"] == 1.8
    assert policies["inviteAnimalToPlay"]["desired_distance_m"] == 1.20
    assert policies["inviteAnimalToPlay"]["minimum_safe_distance_m"] == 0.80
    assert policies["inviteAnimalToPlay"]["min_confidence"] == 0.40
    assert policies["inviteAnimalToPlay"]["target_lost_timeout_sec"] == 1.8
    assert policies["inspectObject"]["target_lost_timeout_sec"] == 1.8
    assert policies["inspectObject"]["min_confidence"] == 0.35
    assert (
        policies["testAnimalBoundary"]["desired_distance_m"]
        > policies["greetAnimal"]["desired_distance_m"]
    )


def test_owner_behavior_visual_profiles_are_distinct_and_bounded() -> None:
    policies = _policies()

    assert policies["unhappy"]["max_linear_x"] == 0.30
    assert policies["unhappy"]["arrival_hold_sec"] == 1.5
    assert policies["miss_owner"]["max_linear_x"] == 0.30
    assert policies["miss_owner"]["bbox_target_height"] == 0.84
    assert policies["farewell_leave"]["arrival_hold_sec"] == 3.0
    assert policies["farewell_leave"]["target_lost_timeout_sec"] == 1.0

    for behavior_name in ("unhappy", "miss_owner", "farewell_leave"):
        policy = policies[behavior_name]
        assert policy["target_type"] == "human"
        assert policy["required_identity"] == "owner"
        assert policy["distance_mode"] == "bbox_height"
        assert policy["desired_distance_m"] >= policy["minimum_safe_distance_m"]


def test_unhappy_uses_slow_profile_and_farewell_resumes_short_follow() -> None:
    unhappy_commands: list[TwistCommand] = []
    unhappy_clock = FakeClock()
    unhappy = _adapter(
        unhappy_commands,
        unhappy_clock,
        approach_timeout_sec=4.0,
    )
    unhappy.update_visual(
        _payload(
            "human",
            bbox_height=0.25,
            target_id="vision-1:human:7",
            identity="owner",
        ),
        now=unhappy_clock.now,
    )

    def refresh_unhappy(now: float) -> None:
        unhappy.update_visual(
            _payload(
                "human",
                bbox_height=0.82 if now >= 1.4 else 0.25,
                sequence=_sequence(now),
                target_id="vision-1:human:7",
                identity="owner",
            ),
            now=now,
        )

    unhappy_clock.on_sleep = refresh_unhappy
    unhappy_outcome = unhappy.execute_task(
        {"unit_id": unhappy.ACTION_ID},
        _context("unhappy", "human", "owner", identity="owner"),
        4.0,
    )

    assert unhappy_outcome.success
    assert unhappy_outcome.metadata["effective_policy"]["max_linear_x"] == 0.30
    assert unhappy_outcome.metadata["effective_policy"]["arrival_hold_sec"] == 1.5
    assert max(command.linear_x for command in unhappy_commands) <= 0.30

    farewell_commands: list[TwistCommand] = []
    farewell_clock = FakeClock()
    farewell = _adapter(
        farewell_commands,
        farewell_clock,
        approach_timeout_sec=5.0,
    )
    farewell.update_visual(
        _payload(
            "human",
            bbox_height=0.76,
            target_id="vision-1:human:7",
            identity="owner",
        ),
        now=farewell_clock.now,
    )

    def refresh_farewell(now: float) -> None:
        owner_departing = 1.3 <= now < 1.8
        farewell.update_visual(
            _payload(
                "human",
                bbox_height=0.50 if owner_departing else 0.76,
                sequence=_sequence(now),
                target_id="vision-1:human:7",
                identity="owner",
            ),
            now=now,
        )

    farewell_clock.on_sleep = refresh_farewell
    farewell_outcome = farewell.execute_task(
        {"unit_id": farewell.ACTION_ID},
        _context("farewell_leave", "human", "owner", identity="owner"),
        5.0,
    )

    assert farewell_outcome.success
    assert farewell_outcome.metadata["effective_policy"][
        "arrival_hold_sec"
    ] == 3.0
    assert any(command.linear_x > 0.0 for command in farewell_commands)
    assert farewell_commands[-2:] == [TwistCommand(), TwistCommand()]


@pytest.mark.parametrize("behavior", ["unhappy", "miss_owner", "farewell_leave"])
def test_owner_behavior_rejects_non_owner_before_motion(behavior: str) -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock)
    adapter.update_visual(
        _payload(
            "human",
            bbox_height=0.84,
            target_id="vision-1:human:7",
            identity="family_member_1",
        ),
        now=clock.now,
    )

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context(
            behavior,
            "human",
            "family_member_1",
            identity="family_member_1",
        ),
        1.0,
    )

    assert outcome.success is False
    assert outcome.reason == "visual_target_identity_mismatch"
    assert commands and all(command.is_zero for command in commands)


@pytest.mark.parametrize(
    ("behavior", "action_id"),
    [
        ("come_to_owner", "ACT_INTERACT_APPROACH_OWNER"),
        ("approach_owner", "ACT_INTERACT_APPROACH_OWNER_CLOSER"),
        ("return_to_owner", "ACT_INTERACT_RETURN_OWNER"),
    ],
)
def test_go2_owner_action_aliases_reject_non_owner_before_motion(
    behavior: str,
    action_id: str,
) -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(
        commands,
        clock,
        behavior_policies=_go2_policies(),
    )

    outcome = adapter.execute_task(
        {"unit_id": action_id},
        _context(
            behavior,
            "human",
            "vision-1:human:7",
            identity="family_member_1",
            vision_epoch="vision-1",
        ),
        1.0,
    )

    assert adapter.can_execute(action_id)
    assert outcome.success is False
    assert outcome.reason == "visual_target_identity_mismatch"
    assert commands and all(command.is_zero for command in commands)


@pytest.mark.parametrize(
    ("behavior", "action_id"),
    [
        ("come_to_owner", "ACT_INTERACT_APPROACH_OWNER"),
        ("approach_owner", "ACT_INTERACT_APPROACH_OWNER_CLOSER"),
        ("return_to_owner", "ACT_INTERACT_RETURN_OWNER"),
    ],
)
def test_go2_owner_action_aliases_approach_confirmed_owner(
    behavior: str,
    action_id: str,
) -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    policies = _go2_policies()
    adapter = _adapter(
        commands,
        clock,
        behavior_policies=policies,
        approach_timeout_sec=2.0,
    )
    adapter.update_visual(
        _payload(
            "human",
            bbox_height=0.25,
            target_id="vision-1:human:7",
            identity="owner",
        ),
        now=clock.now,
    )

    target_height = policies[behavior]["bbox_target_height"]

    def refresh(now: float) -> None:
        adapter.update_visual(
            _payload(
                "human",
                bbox_height=target_height if now >= 1.3 else 0.25,
                sequence=_sequence(now),
                target_id="vision-1:human:7",
                identity="owner",
            ),
            now=now,
        )

    clock.on_sleep = refresh
    outcome = adapter.execute_task(
        {"unit_id": action_id},
        _context(
            behavior,
            "human",
            "vision-1:human:7",
            identity="owner",
            vision_epoch="vision-1",
        ),
        2.0,
    )

    assert outcome.success
    assert outcome.metadata["behavior_name"] == behavior
    assert any(command.linear_x > 0.0 for command in commands)
    assert commands[-2:] == [TwistCommand(), TwistCommand()]


def test_nonhuman_thresholds_accept_moderate_confidence_without_weakening_humans() -> None:
    animal_commands: list[TwistCommand] = []
    animal_clock = FakeClock()
    animal = _adapter(animal_commands, animal_clock, arrival_hold_sec=0.0)
    animal.update_visual(
        _payload(
            "animal",
            distance_m=1.5,
            confidence=0.45,
            label="cat",
        ),
        now=animal_clock.now,
    )

    animal_outcome = animal.execute_task(
        {"unit_id": animal.ACTION_ID},
        _context("testAnimalBoundary", "animal", "7"),
        1.0,
    )

    assert animal_outcome.success
    assert animal_outcome.metadata["effective_policy"][
        "target_min_confidence"
    ] == 0.40

    object_commands: list[TwistCommand] = []
    object_clock = FakeClock()
    object_adapter = _adapter(
        object_commands,
        object_clock,
        arrival_hold_sec=0.0,
    )
    object_adapter.update_visual(
        _payload(
            "object",
            distance_m=0.9,
            confidence=0.40,
            label="box",
        ),
        now=object_clock.now,
    )

    object_outcome = object_adapter.execute_task(
        {"unit_id": object_adapter.ACTION_ID},
        _context("inspectObject", "object", "7"),
        1.0,
    )

    assert object_outcome.success
    assert object_outcome.metadata["effective_policy"][
        "target_min_confidence"
    ] == 0.35
    assert all(command.is_zero for command in object_commands)

    human_commands: list[TwistCommand] = []
    human_clock = FakeClock()
    human = _adapter(human_commands, human_clock, arrival_hold_sec=0.0)
    human.update_visual(
        _payload(
            "human",
            distance_m=1.2,
            confidence=0.45,
            target_id="vision-1:human:7",
            identity="owner",
        ),
        now=human_clock.now,
    )

    human_outcome = human.execute_task(
        {"unit_id": human.ACTION_ID},
        _context("seekHumanInteraction", "human", "owner", identity="owner"),
        1.0,
    )

    assert not human_outcome.success
    assert human_outcome.reason == "visual_target_not_found"
    assert all(command.is_zero for command in human_commands)


def test_human_bbox_mode_ignores_short_depth_and_approaches() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock)
    adapter.update_visual(
        _payload(
            "human",
            distance_m=0.7,
            bbox_height=0.25,
            target_id="vision-1:human:7",
            identity="owner",
        ),
        now=clock.now,
    )

    def refresh(now: float) -> None:
        adapter.update_visual(
            _payload(
                "human",
                # Deliberately keep publishing the misleading close depth.
                distance_m=0.7,
                bbox_height=0.80 if now >= 1.4 else 0.25,
                sequence=_sequence(now),
                target_id="vision-1:human:7",
                identity="owner",
            ),
            now=now,
        )

    clock.on_sleep = refresh
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context("seekHumanInteraction", "human", "owner"),
        2.0,
    )

    assert outcome.success
    assert outcome.metadata["distance_source"] == "bbox_height"
    assert outcome.metadata["bbox_distance_fallback"] is True
    assert outcome.metadata["configured_distance_mode"] == "bbox_height"
    assert any(command.linear_x > 0.0 for command in commands)
    assert commands[-2:] == [TwistCommand(), TwistCommand()]


def test_animal_boundary_keeps_larger_distance() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock)
    adapter.update_visual(
        _payload("animal", distance_m=1.5, label="cat"), now=clock.now
    )
    clock.on_sleep = lambda now: adapter.update_visual(
        _payload(
            "animal",
            distance_m=1.5,
            label="cat",
            sequence=_sequence(now),
        ),
        now=now,
    )

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context("testAnimalBoundary", "animal", "7"),
        1.0,
    )

    assert outcome.success
    assert outcome.metadata["effective_policy"]["desired_distance_m"] == 1.5
    assert not [command for command in commands if not command.is_zero]


def test_social_unknown_identity_binds_only_current_single_active_track() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock, arrival_hold_sec=0.0)
    payload = _payload(
        "human",
        distance_m=1.2,
        target_id="vision-1:human:7",
        identity="unknown",
        track_id=7,
    )
    payload["human_candidates"].append({
        **payload["human_candidates"][0],
        "target_id": "vision-1:human:8",
        "track_id": 8,
        "tracking_state": "temporarily_lost",
        "last_seen_age_ms": 900.0,
    })
    # The compatibility active_target duplicates track 7 and may omit the
    # formatted target ID in older producers.
    payload["active_target"] = {
        **payload["human_candidates"][0],
        "target_id": "",
    }
    adapter.update_visual(payload, now=clock.now)

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context(
            "seekHumanInteraction",
            "human",
            "unknown",
            identity="unknown",
            count=1,
        ),
        1.0,
    )

    assert outcome.success
    assert outcome.metadata["bound_target_id"] == "7"
    assert not [command for command in commands if not command.is_zero]


def test_social_known_identity_prefers_matching_active_target() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock, arrival_hold_sec=0.0)
    payload = _payload(
        "human",
        distance_m=1.2,
        target_id="vision-1:human:7",
        identity="owner",
        track_id=7,
    )
    payload["human_candidates"].append({
        **payload["human_candidates"][0],
        "target_id": "vision-1:human:8",
        "track_id": 8,
        "identity": "unknown",
    })
    payload["active_target"] = dict(payload["human_candidates"][0])
    adapter.update_visual(payload, now=clock.now)

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context(
            "seekHumanInteraction",
            "human",
            "owner",
            identity="owner",
            count=2,
        ),
        1.0,
    )

    assert outcome.success
    assert outcome.metadata["bound_target_id"] == "7"


def test_temporarily_lost_human_is_reacquired_before_approach() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(
        commands,
        clock,
        acquire_timeout_sec=0.5,
        arrival_hold_sec=0.1,
    )
    lost = _payload(
        "human",
        distance_m=3.0,
        bbox_height=0.25,
        target_id="vision-1:human:7",
        identity="only1",
    )
    lost["human_candidates"][0]["tracking_state"] = "temporarily_lost"
    lost["human_candidates"][0]["last_seen_age_ms"] = 600.0
    lost["active_target"] = dict(lost["human_candidates"][0])
    adapter.update_visual(lost, now=clock.now)
    reacquired_at_command: list[int] = []

    def refresh(now: float) -> None:
        payload = _payload(
            "human",
            distance_m=3.0,
            bbox_height=0.80 if now >= 1.6 else 0.25,
            sequence=_sequence(now),
            target_id="vision-1:human:7",
            identity="only1",
        )
        if now < 1.15:
            payload["human_candidates"][0]["tracking_state"] = (
                "temporarily_lost"
            )
            payload["human_candidates"][0]["last_seen_age_ms"] = 600.0
        elif not reacquired_at_command:
            reacquired_at_command.append(len(commands))
        payload["active_target"] = dict(payload["human_candidates"][0])
        adapter.update_visual(payload, now=now)

    clock.on_sleep = refresh
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context("expressJoyWithHuman", "human", "only1", identity="only1"),
        2.0,
    )

    assert outcome.success
    assert reacquired_at_command
    assert all(
        command.is_zero
        for command in commands[:reacquired_at_command[0]]
    )
    assert any(command.linear_x > 0.0 for command in commands)
    assert outcome.metadata["distance_source"] == "bbox_height"


def test_bound_human_stops_then_resumes_after_short_visual_loss() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(
        commands,
        clock,
        lost_timeout_sec=0.5,
        arrival_hold_sec=0.1,
    )
    adapter.update_visual(
        _payload(
            "human",
            bbox_height=0.25,
            target_id="vision-1:human:7",
            identity="only1",
        ),
        now=clock.now,
    )
    loss_command_range: list[int] = []

    def refresh(now: float) -> None:
        payload = _payload(
            "human",
            bbox_height=0.80 if now >= 1.9 else 0.25,
            sequence=_sequence(now),
            target_id="vision-1:human:7",
            identity="only1",
        )
        if 1.3 <= now < 1.6:
            payload["human_candidates"][0]["tracking_state"] = (
                "temporarily_lost"
            )
            payload["human_candidates"][0]["last_seen_age_ms"] = 600.0
            if not loss_command_range:
                loss_command_range.append(len(commands))
        elif loss_command_range and len(loss_command_range) == 1:
            loss_command_range.append(len(commands))
        payload["active_target"] = dict(payload["human_candidates"][0])
        adapter.update_visual(payload, now=now)

    clock.on_sleep = refresh
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context("seekHumanInteraction", "human", "only1", identity="only1"),
        2.5,
    )

    assert outcome.success
    assert loss_command_range and len(loss_command_range) == 2
    loss_start, reacquired = loss_command_range
    assert any(command.linear_x > 0.0 for command in commands[:loss_start])
    assert all(command.is_zero for command in commands[loss_start:reacquired])
    assert any(command.linear_x > 0.0 for command in commands[reacquired:])
    assert commands[-2:] == [TwistCommand(), TwistCommand()]


def test_bound_animal_waits_stopped_for_one_second_detection_gap() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(
        commands,
        clock,
        lost_timeout_sec=2.0,
        arrival_hold_sec=0.1,
    )
    adapter.update_visual(
        _payload(
            "animal",
            distance_m=3.0,
            confidence=0.45,
            label="cat",
        ),
        now=clock.now,
    )
    loss_command_range: list[int] = []

    def refresh(now: float) -> None:
        payload = _payload(
            "animal",
            distance_m=1.5 if now >= 2.8 else 3.0,
            confidence=0.45,
            label="cat",
            sequence=_sequence(now),
        )
        if 1.3 <= now < 2.4:
            payload["tracked_objects"][0]["tracking_state"] = (
                "temporarily_lost"
            )
            payload["tracked_objects"][0]["last_seen_age_ms"] = 600.0
            if not loss_command_range:
                loss_command_range.append(len(commands))
        elif loss_command_range and len(loss_command_range) == 1:
            loss_command_range.append(len(commands))
        adapter.update_visual(payload, now=now)

    clock.on_sleep = refresh
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context("testAnimalBoundary", "animal", "7"),
        4.0,
    )

    assert outcome.success
    assert outcome.metadata["effective_policy"][
        "target_lost_timeout_sec"
    ] == 1.8
    assert loss_command_range and len(loss_command_range) == 2
    loss_start, reacquired = loss_command_range
    assert any(command.linear_x > 0.0 for command in commands[:loss_start])
    assert all(command.is_zero for command in commands[loss_start:reacquired])
    assert any(command.linear_x > 0.0 for command in commands[reacquired:])
    assert commands[-2:] == [TwistCommand(), TwistCommand()]


def test_cancel_during_target_reacquisition_stops_immediately() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock, acquire_timeout_sec=0.5)
    lost = _payload(
        "human",
        target_id="vision-1:human:7",
        identity="only1",
    )
    lost["human_candidates"][0]["tracking_state"] = "temporarily_lost"
    lost["human_candidates"][0]["last_seen_age_ms"] = 600.0
    lost["active_target"] = dict(lost["human_candidates"][0])
    adapter.update_visual(lost, now=clock.now)
    clock.on_sleep = lambda _now: adapter.cancel_task()

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context("expressJoyWithHuman", "human", "only1", identity="only1"),
        2.0,
    )

    assert outcome.canceled
    assert outcome.reason == "visual_target_acquire_canceled"
    assert clock.now < 1.2
    assert commands and all(command.is_zero for command in commands)


def test_social_unknown_identity_uses_producer_selected_active_target() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock, arrival_hold_sec=0.0)
    payload = _payload(
        "human",
        distance_m=1.2,
        target_id="vision-1:human:7",
        identity="unknown",
        track_id=7,
    )
    payload["human_candidates"].append({
        **payload["human_candidates"][0],
        "target_id": "vision-1:human:8",
        "track_id": 8,
    })
    payload["active_target"] = dict(payload["human_candidates"][0])
    adapter.update_visual(payload, now=clock.now)

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context(
            "seekHumanInteraction",
            "human",
            "unknown",
            identity="unknown",
            count=2,
        ),
        1.0,
    )

    assert outcome.success
    assert outcome.metadata["bound_target_id"] == "7"
    assert outcome.metadata["binding_reason"] == (
        "active_target_check_person_compat"
    )
    assert not [command for command in commands if not command.is_zero]


@pytest.mark.parametrize(
    ("context", "reason"),
    [
        (
            _context("seekHumanInteraction", "human", "missing"),
            "visual_target_not_found",
        ),
        (
            _context("seekHumanInteraction", "object", "7"),
            "visual_target_type_mismatch",
        ),
        (
            _context(
                "inspectObject",
                "object",
                "7",
                vision_epoch="old-vision",
            ),
            "visual_target_epoch_mismatch",
        ),
    ],
)
def test_invalid_target_fails_before_nonzero_motion(
    context: ExecutionContext,
    reason: str,
) -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock)
    adapter.update_visual(
        _payload("human", target_id="human-7", identity="owner"),
        now=clock.now,
    )

    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID}, context, 1.0
    )

    assert not outcome.success
    assert outcome.reason == reason
    assert not [command for command in commands if not command.is_zero]
    assert commands[-1].is_zero


def test_metric_distance_is_required() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock)
    payload = _payload("object", label="box")
    payload["tracked_objects"][0].pop("distance_m")
    payload["tracked_objects"][0]["range_valid"] = False
    adapter.update_visual(payload, now=clock.now)

    def refresh(now: float) -> None:
        updated = _payload("object", label="box", sequence=_sequence(now))
        updated["tracked_objects"][0].pop("distance_m")
        updated["tracked_objects"][0]["range_valid"] = False
        adapter.update_visual(updated, now=now)

    clock.on_sleep = refresh
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context("inspectObject", "object", "7"),
        1.0,
    )

    assert not outcome.success
    assert outcome.reason == "metric_distance_unavailable"
    assert not [command for command in commands if not command.is_zero]


def test_bound_target_loss_never_switches_to_visible_distractor() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock, approach_timeout_sec=0.8)
    adapter.update_visual(
        _payload("object", distance_m=3.0, label="box", track_id=7),
        now=clock.now,
    )
    switch_indexes: list[int] = []

    def replace_with_distractor(now: float) -> None:
        adapter.update_visual(
            _payload(
                "object",
                distance_m=3.0,
                label="box",
                track_id=99,
                sequence=_sequence(now),
            ),
            now=now,
        )
        switch_indexes.append(len(commands))

    clock.on_sleep = replace_with_distractor
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context("inspectObject", "object", "7"),
        0.8,
    )

    assert not outcome.success
    assert outcome.reason == "target_lost"
    assert switch_indexes
    assert all(command.is_zero for command in commands[switch_indexes[0] - 1 :])


def test_cancel_stops_and_returns_canceled() -> None:
    commands: list[TwistCommand] = []
    clock = FakeClock()
    adapter = _adapter(commands, clock)
    adapter.update_visual(
        _payload("object", distance_m=3.0, label="box"), now=clock.now
    )

    def cancel(now: float) -> None:
        adapter.update_visual(
            _payload(
                "object",
                distance_m=3.0,
                label="box",
                sequence=_sequence(now),
            ),
            now=now,
        )
        adapter.cancel_task()

    clock.on_sleep = cancel
    outcome = adapter.execute_task(
        {"unit_id": adapter.ACTION_ID},
        _context("inspectObject", "object", "7"),
        2.0,
    )

    assert outcome.canceled
    assert outcome.reason == "target_approach_canceled"
    assert commands[-1].is_zero


def test_behavior_templates_prepend_required_approach_and_exclude_room_roam() -> None:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    policies = loader.visual_target_approach_config["behavior_policies"]

    for behavior_name in policies:
        stages = loader.get_behavior_template(behavior_name)["stages"]
        # The policy guarantees the approach stage comes first; how many stages
        # follow is the behavior's own business.  inspectDogFood grew from two
        # stages to seven on 2026-09-18 when the 进食 ritual replaced its
        # ``inspect`` stage, while its approach stage stayed where it was.
        assert [stage["order"] for stage in stages] == list(
            range(1, len(stages) + 1)
        )
        assert stages[0]["stage_id"] == "target_approach"
        assert stages[0]["required"] is True
        assert stages[0]["failure_policy"] == "abort"
        assert stages[0]["candidates"] == [
            {"unit_id": "ACT_APPROACH_VISUAL_TARGET"}
        ]

    room = loader.get_behavior_template("exploreRoom")
    assert [stage["stage_id"] for stage in room["stages"]] == ["explore"]
    assert loader.controller_routes["routes"]["ACT_APPROACH_VISUAL_TARGET"] == (
        "visual_target_approach"
    )
    assert loader.action_catalog["ACT_APPROACH_VISUAL_TARGET"][
        "requires_controller"
    ] is True


def test_required_approach_controller_cannot_mock_success() -> None:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    executor = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes={"_default": "mock"},
        controller_adapters={},
    )
    ctx = _context("seekHumanInteraction", "human", "person-7")
    stage = loader.get_behavior_template("seekHumanInteraction")["stages"][0]

    result = executor.execute_stage(stage, ctx)

    assert not result.success
    assert ctx.metadata["unit_failure_state"] == "controller_error"
    assert ctx.metadata["unit_failure_reason"] == "controller_required:mock"


@pytest.mark.parametrize(
    ("behavior_name", "target_type", "target_id"),
    [
        ("inspectObject", "object", "7"),
        ("expressJoyWithHuman", "human", "person-7"),
    ],
)
def test_failed_approach_prevents_following_expression_stage(
    behavior_name: str,
    target_type: str,
    target_id: str,
) -> None:
    class RejectingAdapter:
        def execute_task(self, unit_config, ctx, timeout):
            del unit_config, ctx, timeout
            return type(
                "Outcome",
                (),
                {"success": False, "reason": "visual_target_not_found"},
            )()

    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    executor = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes(),
        controller_adapters={
            "visual_target_approach": RejectingAdapter(),
        },
    )
    ctx = _context(behavior_name, target_type, target_id)
    stages = loader.get_behavior_template(behavior_name)["stages"]

    first = executor.execute_stage(stages[0], ctx)

    assert not first.success
    assert first.message == "visual_target_not_found"
    assert ctx.executed_units == []
    assert ctx.completed_stages == []
