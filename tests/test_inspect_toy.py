"""Voice-driven toy acquisition, one Nav2 arrival, and real Lite3 gestures."""
from pathlib import Path
from types import SimpleNamespace as NS
import threading

import pytest

from marsdog_action_executor.adapters.visual_nav_approach_adapter import VisualNavApproachAdapter
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.stage_executor import StageExecutor
from tests.test_visual_nav_approach_adapter import Transport
from tests.test_random_arrival_expressions import Clock, _lite3, POSE_AXES


@pytest.fixture(scope="module")
def loader():
    result = ConfigLoader(Path(__file__).resolve().parents[1] / "config")
    result.load_all()
    return result


def toy(label="dog toy ball", track=7, confidence=.9, **changes):
    return {
        "target_type": "object", "target_id": f"epoch:object:{track}",
        "vision_epoch": "epoch", "track_id": track, "label": label,
        "confidence": confidence, "tracking_state": "tracking", "last_seen_age_ms": 0.,
        "bbox": [.4, .4, .2, .2], "center_x": .5, **changes,
    }


def rig(loader, objects, *, start_ok=True, foreign_session=False):
    ctx = ExecutionContext.from_goal("inspect_toy", {})
    ctx.resolved_behavior_name = "inspect_toy"
    transport, twists, leases, chassis = Transport(), [], [], []
    adapter = None
    sequence = 0

    def packet(session, candidates):
        nonlocal sequence
        sequence += 1
        return {
            "schema_version": 2, "header": {"stamp": 100., "frame_id": "color"},
            "sequence": sequence, "source": "stream", "status": "ok",
            "stream": {"session_id": "foreign" if foreign_session else session},
            "objects": candidates,
        }

    def start(session, labels, rate, lease):
        leases.append(("start", session, labels, rate, lease))
        if not start_ok:
            return False, "service_unavailable"
        adapter.update_object_detection(packet(session, objects))
        return True, ""

    adapter = VisualNavApproachAdapter(
        twists.append, transport=transport,
        chassis_backend=NS(
            prepare_navigation=lambda: chassis.append("prepare") or True,
            finish_navigation=lambda: chassis.append("finish") or True,
        ),
        behavior_policies=loader.visual_target_approach_config["behavior_policies"],
        require_object_stream=True, start_object_detection=start,
        stop_object_detection=lambda session: leases.append(("stop", session)),
        acquire_timeout_sec=.05,
    )
    return NS(adapter=adapter, ctx=ctx, transport=transport, twists=twists,
              leases=leases, chassis=chassis, packet=packet)


@pytest.mark.parametrize("label", ["dog toy ball", "dog frisbee toy", "dog tug ring toy"])
def test_acquires_each_supported_toy_without_a_tree_target(loader, label):
    r = rig(loader, [toy(label)])
    assert r.ctx.target is None
    result = r.adapter.execute_task({"unit_id": r.adapter.ACTION_ID}, r.ctx, 160.)
    assert result.success, result.reason
    assert r.leases[0][2:] == (["dog toy ball", "dog frisbee toy", "dog tug ring toy"], 4., 30.)
    assert r.ctx.target == {
        "target_type": "object", "target_id": "epoch:object:7",
        "vision_epoch": "epoch", "label": label,
    }
    assert r.transport.calls[0][:4] == ("locate", "epoch:object:7", "object", .6)
    assert [call[0] for call in r.transport.calls] == ["locate", "navigate"]
    assert r.chassis == ["prepare", "finish"]
    assert r.leases[-1] == ("stop", r.leases[0][1])
    assert all(command.is_zero for command in r.twists)


def test_selects_highest_confidence_toy_and_never_switches_after_binding(loader):
    r = rig(loader, [toy("dog bowl", 1, .99), toy(track=8, confidence=.4),
                     toy("dog frisbee toy", 7, .8)])

    def change_detections_during_navigation():
        assert r.adapter.update_object_detection(
            r.packet(r.leases[0][1], [toy(track=9, confidence=.99)]),
        )

    r.transport.on_navigate = change_detections_during_navigation
    result = r.adapter.execute_task({"unit_id": r.adapter.ACTION_ID}, r.ctx, 160.)
    assert result.success
    assert result.metadata["bound_target_id"] == "epoch:object:7"
    assert r.ctx.target["target_id"] == "epoch:object:7"


def test_explicit_toy_distance_below_minimum_never_navigates(loader):
    r = rig(loader, [toy()])
    r.ctx.params["stand_off_distance_m"] = .2
    result = r.adapter.execute_task({"unit_id": r.adapter.ACTION_ID}, r.ctx, 160.)
    assert not result.success and result.reason == "unsafe_stand_off_distance"
    assert r.transport.calls == [] and r.chassis == []


@pytest.mark.parametrize("objects", [
    [], [toy("dog bowl")], [toy("dog")], [toy(confidence=.24)],
    [toy(last_seen_age_ms=501.)], [toy(tracking_state="temporarily_lost")],
    [toy(target_id="ball")], [toy(target_id="other:object:7")],
])
def test_missing_invalid_or_non_toy_targets_never_navigate(loader, objects):
    r = rig(loader, objects)
    result = r.adapter.execute_task({"unit_id": r.adapter.ACTION_ID}, r.ctx, 160.)
    assert not result.success and result.reason == "toy_target_not_found"
    assert r.transport.calls == [] and r.chassis == []
    assert r.ctx.target is None
    assert r.leases[-1] == ("stop", r.leases[0][1])
    assert all(command.is_zero for command in r.twists)


def test_foreign_session_cannot_supply_a_toy(loader):
    r = rig(loader, [toy()], foreign_session=True)
    result = r.adapter.execute_task({"unit_id": r.adapter.ACTION_ID}, r.ctx, 160.)
    assert not result.success
    assert r.transport.calls == []
    assert r.leases[-1] == ("stop", r.leases[0][1])


def test_explicit_non_toy_target_is_rejected(loader):
    r = rig(loader, [toy()])
    r.ctx.target = {"target_type": "object", "target_id": "epoch:object:7", "label": "dog bowl"}
    result = r.adapter.execute_task({"unit_id": r.adapter.ACTION_ID}, r.ctx, 160.)
    assert result.reason == "object_target_label_not_allowed"
    assert r.leases == [] and r.transport.calls == []


def test_stream_start_failure_never_navigates(loader):
    r = rig(loader, [toy()], start_ok=False)
    result = r.adapter.execute_task({"unit_id": r.adapter.ACTION_ID}, r.ctx, 160.)
    assert result.reason == "service_unavailable"
    assert r.transport.calls == [] and r.chassis == []


def test_cancel_during_acquisition_releases_the_stream(loader):
    r = rig(loader, [])
    start = r.adapter._start_object_detection

    def cancel_after_start(*args):
        result = start(*args)
        r.adapter.cancel_task()
        return result

    r.adapter._start_object_detection = cancel_after_start
    result = r.adapter.execute_task({"unit_id": r.adapter.ACTION_ID}, r.ctx, 160.)
    assert result.canceled
    assert not any(call[0] in {"locate", "navigate"} for call in r.transport.calls)
    assert r.leases[-1] == ("stop", r.leases[0][1])


@pytest.mark.parametrize("renew_ok", [True, False])
def test_renews_same_session_during_navigation_and_joins_before_stop(loader, renew_ok):
    r = rig(loader, [toy()])
    renewed = threading.Event()
    start = r.adapter._start_object_detection
    r.adapter._OBJECT_STREAM_RENEW_SEC = .01

    def renew(*args):
        if r.leases:
            assert args[0] == r.leases[0][1]
            if not renew_ok:
                renewed.set()
                return False, "renew_failed"
            result = start(*args)
            renewed.set()
            return result
        return start(*args)

    r.adapter._start_object_detection = renew

    def wait_for_renewal():
        assert renewed.wait(1.), "stream was not renewed during navigation"
        if not renew_ok:
            assert r.adapter._cancel_requested.wait(1.)

    r.transport.on_navigate = wait_for_renewal
    if not renew_ok:
        r.transport.nav_result = (False, True, "navigation_canceled")
    result = r.adapter.execute_task({"unit_id": r.adapter.ACTION_ID}, r.ctx, 160.)
    assert result.success is renew_ok
    if not renew_ok:
        assert result.reason == "renew_failed"
        assert ("cancel",) in r.transport.calls
    assert r.leases[-1] == ("stop", r.leases[0][1])
    assert all(command.is_zero for command in r.twists)


@pytest.mark.parametrize("nav_success", [True, False])
def test_full_plan_requires_arrival_then_executes_three_real_lite3_combos(loader, nav_success):
    r = rig(loader, [toy()])
    if not nav_success:
        r.transport.nav_result = (False, False, "navigation_aborted")
    commands, physical_twists = [], []
    backend = _lite3(loader, Clock(), commands, physical_twists)
    executor = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes("lite3"),
        controller_adapters={"visual_nav_approach": r.adapter, "lite3": backend},
    )
    stages = loader.get_behavior_template("inspect_toy")["stages"]
    for stage in stages:
        result = executor.execute_stage(stage, r.ctx)
        if not result.success:
            break
    assert result.success is nav_success
    if nav_success:
        assert r.ctx.completed_stages == ["target_approach", "look_down_up", "scan_head_up", "stretch_scan"]
        assert r.ctx.executed_units == ["ACT_APPROACH_VISUAL_TARGET", "ACT_ROAM_LOOK_DOWN_UP",
                                        "ACT_ROAM_SCAN_HEAD_UP", "ACT_ROAM_STRETCH_SCAN"]
        from itertools import groupby
        actual = [(cmd.cmd_code, cmd.value) for cmd in commands
                  if cmd.cmd_code in POSE_AXES and cmd.value != 0]
        actual = [gesture for gesture, _ in groupby(actual)]
        expected = [(step["cmd_code"], step["value"])
                    for unit in r.ctx.executed_units[1:]
                    for step in loader.get_lite3_action_plans()[unit]["sequence"]]
        assert actual == expected
        assert r.ctx.metadata["lite3_action"]["physical_posture"] == "standing"
        assert physical_twists and all(command.is_zero for command in physical_twists)
    else:
        assert not r.ctx.completed_stages
        assert commands == [] and physical_twists == []
