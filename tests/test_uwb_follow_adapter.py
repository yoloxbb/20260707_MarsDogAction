from __future__ import annotations

from pathlib import Path

import yaml
import pytest

from marsdog_action_executor.adapters.velocity import TwistCommand
from marsdog_action_executor.adapters.uwb_follow_adapter import UwbFollowAdapter
from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.execution_context import ExecutionContext
from marsdog_action_executor.stage_executor import StageExecutor


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration


class FakeProcess:
    _next_pid = 1000

    def __init__(self) -> None:
        self.pid = FakeProcess._next_pid
        FakeProcess._next_pid += 1
        self.returncode = None

    def poll(self):
        return self.returncode


def _adapter(target_fob_id: str = "", *, auto_resume: bool = False):
    clock = FakeClock()
    commands: list[list[str]] = []
    processes: list[FakeProcess] = []
    stopped: list[int] = []
    twists: list[TwistCommand] = []

    def popen(command, **kwargs):
        assert kwargs == {"start_new_session": True}
        commands.append(command)
        process = FakeProcess()
        processes.append(process)
        return process

    def terminate(process, timeout):
        assert timeout == 3.0
        process.returncode = -15
        stopped.append(process.pid)

    adapter = UwbFollowAdapter(
        enabled=True,
        setup_script="/workspace/install/setup.bash",
        serial_device="/dev/serial/by-id/uwb",
        cmd_vel_topic="/cmd_vel",
        target_fob_id=target_fob_id,
        startup_grace_sec=1.0,
        shutdown_timeout_sec=3.0,
        publish_twist=twists.append,
        popen=popen,
        path_exists=lambda path: True,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
        auto_resume=auto_resume,
        terminate_process=terminate,
    )
    return adapter, commands, processes, stopped, twists, clock


def test_goal_follow_stops_when_go2_target_goes_stale() -> None:
    adapter, _, processes, stopped, twists, clock = _adapter()
    assert adapter.start(wait_for_grace=True)
    adapter.enable_target_monitor()
    adapter.update_target(1.0, 0.0)
    clock.now += 3.1

    assert adapter.poll_health() is False
    assert adapter.last_error == "uwb_target_lost"
    assert stopped == [processes[1].pid, processes[0].pid]
    assert all(command == TwistCommand() for command in twists[-3:])


def test_goal_follow_stops_on_controller_odom_timeout() -> None:
    adapter, _, _, _, _, _ = _adapter()
    assert adapter.start(wait_for_grace=True)
    adapter.enable_target_monitor()
    adapter.update_target(1.0, 0.0)
    adapter.update_controller_state("ODOM_TIMEOUT")

    assert adapter.poll_health() is False
    assert adapter.last_error == "uwb_controller_odom_timeout"
    assert not adapter.active


def test_go2_process_group_termination_is_confirmed() -> None:
    import os
    import signal
    import subprocess

    process = subprocess.Popen(["sleep", "30"], start_new_session=True)
    adapter = UwbFollowAdapter(
        enabled=True,
        setup_script="/workspace/install/setup.bash",
        serial_device="/dev/serial/by-id/uwb",
        shutdown_timeout_sec=1.0,
        path_exists=lambda path: True,
    )
    try:
        adapter._terminate_process(process)
        assert process.poll() is not None
        with pytest.raises(ProcessLookupError):
            os.killpg(process.pid, 0)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)


def test_execute_starts_the_two_commands_from_the_operation_guide() -> None:
    adapter, commands, processes, _, _, clock = _adapter()
    ctx = ExecutionContext.from_goal(
        "follow_owner", {"interaction_id": "voice-1"}
    )
    assert adapter.update_control({
        "enabled": True,
        "mode": "follow_owner",
        "interaction_id": "voice-1",
    })

    assert adapter.execute_step({"unit_id": "ACT_INTERACT_FOLLOW_OWNER"}, ctx)
    assert adapter.active
    assert clock.now == 1.0
    assert len(processes) == 2
    assert commands[0][-5:] == [
        "ros2", "run", "uwb_aoa_pkg", "libAoa_robot_example",
        "/dev/serial/by-id/uwb",
    ]
    assert commands[1][-6:] == [
        "ros2", "launch", "go2_uwb_local_follow",
        "local_follow.launch.py", "enable_motion:=true",
        "cmd_vel_topic:=/cmd_vel",
    ]
    assert all(command[:5] == [
        "/bin/bash", "--noprofile", "--norc", "-c",
        'source "$1" && shift && exec "$@"',
    ] for command in commands)


def test_target_fob_id_is_passed_to_aoa_node() -> None:
    adapter, commands, _, _, _, _ = _adapter(target_fob_id="2271560484")
    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_FOLLOW_OWNER"}, duration=2.0
    )
    assert commands[0][-8:] == [
        "ros2", "run", "uwb_aoa_pkg", "libAoa_robot_example",
        "/dev/serial/by-id/uwb",
        "--ros-args", "-p", "target_fob_id:=2271560484",
    ]


def test_matching_session_disable_stops_both_process_groups_and_base() -> None:
    adapter, _, processes, stopped, twists, _ = _adapter()
    assert adapter.update_control({
        "enabled": True,
        "mode": "follow_owner",
        "interaction_id": "voice-1",
    })

    adapter.update_control({
        "enabled": False,
        "mode": "follow_owner",
        "interaction_id": "voice-1",
    })

    assert not adapter.active
    assert stopped == [processes[1].pid, processes[0].pid]
    assert twists[-3:] == [TwistCommand()] * 3


def test_stale_session_disable_does_not_stop_current_follow() -> None:
    adapter, _, _, stopped, _, _ = _adapter()
    adapter.update_control({
        "enabled": True,
        "mode": "follow_owner",
        "interaction_id": "voice-new",
    })

    adapter.update_control({
        "enabled": False,
        "mode": "follow_owner",
        "interaction_id": "voice-old",
    })

    assert adapter.active
    assert stopped == []


def test_direct_action_runs_a_bounded_standalone_follow() -> None:
    adapter, commands, processes, stopped, twists, clock = _adapter()

    assert adapter.execute_step(
        {"unit_id": "ACT_INTERACT_FOLLOW_OWNER"}, duration=2.0
    )

    assert len(commands) == 2
    assert clock.now == 3.0
    assert stopped == [processes[1].pid, processes[0].pid]
    assert not adapter.active
    assert twists[-3:] == [TwistCommand()] * 3


def test_foreground_behavior_suspends_and_does_not_auto_resume() -> None:
    adapter, _, processes, stopped, _, _ = _adapter()
    adapter.update_control({
        "enabled": True,
        "mode": "follow_owner",
        "interaction_id": "voice-1",
    })

    adapter.suspend()
    assert not adapter.active
    assert stopped == [processes[1].pid, processes[0].pid]

    # Being preempted ends the follow: no process may come back on its own.
    assert adapter.resume()
    assert not adapter.active
    assert len(processes) == 2

    # A suspended session still owns the chassis, so the goal gate keeps
    # treating it as the owner even though nothing is running.
    assert adapter.session_desired
    adapter.cancel_step()
    assert not adapter.session_desired


def test_foreground_behavior_resumes_when_auto_resume_is_enabled() -> None:
    adapter, _, processes, stopped, _, _ = _adapter(auto_resume=True)
    adapter.update_control({
        "enabled": True,
        "mode": "follow_owner",
        "interaction_id": "voice-1",
    })

    adapter.suspend()
    assert not adapter.active
    assert stopped == [processes[1].pid, processes[0].pid]

    assert adapter.resume()
    assert adapter.active
    assert len(processes) == 4


def test_idle_suspend_does_not_run_chassis_motion_finish() -> None:
    finishes: list[str] = []
    adapter = UwbFollowAdapter(
        enabled=True,
        setup_script="/workspace/install/setup.bash",
        serial_device="/dev/serial/by-id/uwb",
        finish_motion=lambda: finishes.append("finish"),
        path_exists=lambda path: True,
    )

    adapter.suspend()

    assert finishes == []


def test_active_follow_finishes_chassis_motion_exactly_once() -> None:
    events: list[str] = []

    def popen(command, **kwargs):
        del command
        assert kwargs == {"start_new_session": True}
        return FakeProcess()

    def terminate(process, timeout):
        assert timeout == 3.0
        process.returncode = -15

    adapter = UwbFollowAdapter(
        enabled=True,
        setup_script="/workspace/install/setup.bash",
        serial_device="/dev/serial/by-id/uwb",
        prepare_motion=lambda: events.append("prepare") or True,
        finish_motion=lambda: events.append("finish"),
        popen=popen,
        path_exists=lambda path: True,
        terminate_process=terminate,
    )
    assert adapter.start(wait_for_grace=False)

    adapter.suspend()
    adapter.suspend()

    assert events == ["prepare", "finish"]


def test_missing_serial_fails_closed_without_starting_processes() -> None:
    twists: list[TwistCommand] = []
    adapter = UwbFollowAdapter(
        enabled=True,
        setup_script="/workspace/install/setup.bash",
        serial_device="/dev/serial/by-id/missing",
        publish_twist=twists.append,
        path_exists=lambda path: not path.endswith("missing"),
    )
    assert not adapter.update_control({
        "enabled": True,
        "mode": "follow_owner",
        "interaction_id": "voice-1",
    })

    with pytest.raises(RuntimeError, match="uwb_serial_device_not_found"):
        adapter.execute_step({"unit_id": "ACT_INTERACT_FOLLOW_OWNER"})
    assert adapter.last_error.startswith("uwb_serial_device_not_found:")
    assert twists == [TwistCommand()] * 6


def test_packaged_follow_contract_routes_to_uwb_adapter() -> None:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    config = yaml.safe_load(
        (CONFIG_DIR / "uwb_follow.yaml").read_text(encoding="utf-8")
    )
    assert config["follow_package"] == "go2_uwb_local_follow"
    assert config["follow_launch_file"] == "local_follow.launch.py"
    assert config["cmd_vel_topic"] == "/cmd_vel"
    assert config["target_fob_id"] == "2271560484"

    fake_adapter, _, _, _, _, _ = _adapter()
    executor = StageExecutor(
        action_catalog=loader.action_catalog,
        controller_routes=loader.get_controller_routes(),
        controller_adapters={"uwb_follow": fake_adapter},
    )
    ctx = ExecutionContext.from_goal(
        "follow_owner", {"interaction_id": "voice-1"}
    )
    ctx.resolved_behavior_name = "follow_owner"
    fake_adapter.update_control({
        "enabled": True,
        "mode": "follow_owner",
        "interaction_id": "voice-1",
    })
    result = executor.execute_stage(
        loader.get_behavior_template("follow_owner")["stages"][0], ctx
    )

    assert result.success
    assert result.unit_id == "ACT_INTERACT_FOLLOW_OWNER"
