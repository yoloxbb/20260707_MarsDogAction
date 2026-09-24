"""Session-scoped process adapter for the Go2 external UWB follow pipeline."""

from __future__ import annotations

import logging
import math
import os
import signal
import subprocess
import threading
import time
from typing import Any, Callable, Mapping, Sequence

from .velocity import TwistCommand

logger = logging.getLogger(__name__)


class UwbFollowAdapter:
    """Own the Go2 external UWB processes for one voice follow session."""

    def __init__(
        self,
        *,
        enabled: bool,
        setup_script: str,
        serial_device: str,
        cmd_vel_topic: str = "/cmd_vel",
        aoa_package: str = "uwb_aoa_pkg",
        aoa_executable: str = "libAoa_robot_example",
        target_fob_id: str = "",
        follow_package: str = "go2_uwb_local_follow",
        follow_launch_file: str = "local_follow.launch.py",
        startup_grace_sec: float = 1.0,
        shutdown_timeout_sec: float = 3.0,
        target_timeout_sec: float = 3.0,
        stop_publish_count: int = 3,
        publish_twist: Callable[[TwistCommand], None] | None = None,
        prepare_motion: Callable[[], bool] | None = None,
        finish_motion: Callable[[], bool | None] | None = None,
        should_stop: Callable[[], bool] | None = None,
        popen: Callable[..., Any] = subprocess.Popen,
        path_exists: Callable[[str], bool] = os.path.exists,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        auto_resume: bool = False,
        terminate_process: Callable[[Any, float], None] | None = None,
    ) -> None:
        self._enabled = bool(enabled)
        self._auto_resume = bool(auto_resume)
        self._setup_script = str(setup_script)
        self._serial_device = str(serial_device)
        self._cmd_vel_topic = str(cmd_vel_topic)
        self._aoa_package = str(aoa_package)
        self._aoa_executable = str(aoa_executable)
        self._target_fob_id = str(target_fob_id)
        self._follow_package = str(follow_package)
        self._follow_launch_file = str(follow_launch_file)
        self._startup_grace_sec = max(0.0, float(startup_grace_sec))
        self._shutdown_timeout_sec = max(0.1, float(shutdown_timeout_sec))
        self._target_timeout_sec = max(0.1, float(target_timeout_sec))
        self._stop_publish_count = max(1, int(stop_publish_count))
        self._publish_twist = publish_twist or (lambda command: None)
        self._prepare_motion = prepare_motion or (lambda: True)
        self._finish_motion = finish_motion or (lambda: None)
        self._should_stop = should_stop or (lambda: False)
        self._popen = popen
        self._path_exists = path_exists
        self._monotonic = monotonic
        self._sleep = sleep
        self._terminate_process_override = terminate_process
        self._lock = threading.RLock()
        self._cancel_requested = threading.Event()
        self._processes: list[Any] = []
        # Set only after this adapter has acquired chassis motion ownership.
        # Idle suspend/stop calls must not run Lite3's settle handshake.
        self._motion_prepared = False
        self._interaction_id = ""
        self._desired_follow = False
        self._session_controlled = False
        self._suspended = False
        self._started_at = 0.0
        self._last_target_at: float | None = None
        self._last_diagnostic_at: float | None = None
        self._controller_state = ""
        self._monitor_target = False
        self._last_error = ""
        self.recovery_required = False

    @property
    def active(self) -> bool:
        with self._lock:
            return bool(self._processes) and self._all_alive_locked()

    @property
    def session_desired(self) -> bool:
        """True while the attention session still asks for following.

        Unlike ``active`` this survives a ``suspend()``: a session parked
        behind a foreground behaviour is still the thing that owns the
        chassis, so the goal gate has to keep rejecting lower-priority
        arrivals for as long as it lasts.
        """
        with self._lock:
            return bool(self._desired_follow)

    @property
    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    def update_control(self, payload: Mapping[str, Any]) -> bool:
        enabled = bool(payload.get("enabled", False))
        mode = str(payload.get("mode", "face_body_centering"))
        interaction_id = str(payload.get("interaction_id", "")).strip()
        if enabled and mode in ("follow", "follow_owner"):
            with self._lock:
                self._desired_follow = True
                self._session_controlled = True
                if interaction_id:
                    self._interaction_id = interaction_id
                if self._suspended:
                    return True
            return self.start(wait_for_grace=False)

        with self._lock:
            current_id = self._interaction_id
        if not interaction_id or not current_id or interaction_id == current_id:
            with self._lock:
                self._desired_follow = False
                self._session_controlled = False
                self._interaction_id = ""
                self._stop_locked("attention_follow_disabled")
        return True

    def execute_step(
        self,
        unit_config: Mapping[str, Any],
        ctx: Any = None,
        duration: float | None = None,
    ) -> bool:
        del unit_config
        self._cancel_requested.clear()
        interaction_id = str(getattr(ctx, "interaction_id", "") or "").strip()
        with self._lock:
            if interaction_id:
                self._interaction_id = interaction_id
            standalone = not self._session_controlled
            if standalone:
                self._desired_follow = True
        if self.start(wait_for_grace=True):
            if standalone:
                deadline = self._monotonic() + max(0.0, float(duration or 0.0))
                while self._monotonic() < deadline:
                    if self._cancel_requested.is_set() or self._should_stop():
                        self.cancel_step()
                        return False
                    if not self.poll_health():
                        raise RuntimeError(
                            self.last_error or "uwb_process_exited"
                        )
                    self._sleep(min(0.05, deadline - self._monotonic()))
                with self._lock:
                    if not self._session_controlled:
                        self._desired_follow = False
                        self._interaction_id = ""
                        self._stop_locked("standalone_follow_completed")
            return True
        raise RuntimeError(self.last_error or "uwb_follow_start_failed")

    def start(self, *, wait_for_grace: bool) -> bool:
        with self._lock:
            if self._processes:
                if self._all_alive_locked():
                    return self._wait_for_grace_locked() if wait_for_grace else True
                self._stop_locked("uwb_process_exited")

            error = self._validate_runtime_paths()
            if error:
                self._last_error = error
                logger.error("UWB follow unavailable: %s", error)
                self._publish_stop_locked()
                return False

            self._cancel_requested.clear()
            if not self._prepare_motion():
                self._last_error = "uwb_motion_preflight_failed"
                self._publish_stop_locked()
                return False
            self._motion_prepared = True
            try:
                for command in self._commands():
                    process = self._popen(command, start_new_session=True)
                    self._processes.append(process)
                    if process.poll() is not None:
                        raise RuntimeError(
                            f"process_exited_during_startup:{process.poll()}"
                        )
            except Exception as exc:
                self._last_error = f"uwb_process_start_failed:{exc}"
                logger.exception("Failed to start UWB follow pipeline")
                self._stop_locked(self._last_error)
                return False

            self._started_at = self._monotonic()
            self._last_target_at = None
            self._last_diagnostic_at = None
            self._controller_state = ""
            self._monitor_target = False
            self._last_error = ""
            logger.info(
                "UWB follow pipeline started: serial=%s cmd_vel=%s",
                self._serial_device,
                self._cmd_vel_topic,
            )
            return self._wait_for_grace_locked() if wait_for_grace else True

    def poll_health(self) -> bool:
        with self._lock:
            if not self._processes:
                return True
            if self._all_alive_locked():
                if not self._monitor_target:
                    return True
                reference = self._last_target_at or self._started_at
                if self._monotonic() - reference > self._target_timeout_sec:
                    self._last_error = "uwb_target_lost"
                elif (self._monotonic() -
                      (self._last_diagnostic_at or self._started_at)
                      > self._target_timeout_sec):
                    self._last_error = "uwb_controller_diagnostics_stale"
                elif self._controller_state in (
                    "TARGET_LOST", "ODOM_TIMEOUT", "OUTPUT_DISABLED",
                ):
                    self._last_error = f"uwb_controller_{self._controller_state.lower()}"
                elif (self._controller_state in ("WAIT_TARGET", "WAIT_ODOM")
                      and self._monotonic() - self._started_at > self._target_timeout_sec):
                    self._last_error = f"uwb_controller_{self._controller_state.lower()}"
                else:
                    return True
                self._stop_locked(self._last_error)
                return False
            self._last_error = "uwb_process_exited"
            logger.error("UWB follow process exited unexpectedly")
            self._stop_locked(self._last_error)
            return False

    def enable_target_monitor(self) -> None:
        with self._lock:
            self._monitor_target = True

    def update_target(self, x: float, y: float) -> None:
        """Record a valid target received from /uwb/target_point."""
        if math.isfinite(x) and math.isfinite(y):
            with self._lock:
                if self._processes:
                    self._last_target_at = self._monotonic()

    def update_controller_state(self, state: str) -> None:
        with self._lock:
            if self._processes:
                self._controller_state = str(state)
                self._last_diagnostic_at = self._monotonic()

    def cancel_step(self, step: Any = None) -> None:
        del step
        self._cancel_requested.set()
        with self._lock:
            self._desired_follow = False
            self._session_controlled = False
            self._interaction_id = ""
            self._stop_locked("canceled")

    def request_cancel(self) -> None:
        """Record cancellation without blocking the Action cancel callback."""
        self._cancel_requested.set()

    def stop_confirmed(self, reason: str = "stopped") -> bool:
        """Terminate the process chain and confirm every child has exited."""
        with self._lock:
            self._desired_follow = False
            self._session_controlled = False
            self._interaction_id = ""
            return self._stop_locked(reason)

    def emergency_stop(self) -> None:
        self._cancel_requested.set()
        with self._lock:
            self._desired_follow = False
            self._session_controlled = False
            self._interaction_id = ""
            self._stop_locked("emergency_stop")

    def suspend(self) -> None:
        """Yield /cmd_vel while another foreground behavior executes."""
        with self._lock:
            self._suspended = True
            self._stop_locked("foreground_behavior")

    def resume(self) -> bool:
        """Release the chassis after a foreground behavior finishes.

        With ``auto_resume`` the still-desired session restarts at once.
        Without it -- the default -- the interruption simply ends: following
        again requires a fresh session message or a fresh ``follow_owner``
        goal, so the robot never starts walking again on its own.

        ``_suspended`` is cleared either way.  Leaving it set would trip the
        ``if self._suspended: return True`` short-circuit in
        ``update_control`` forever, closing the only re-arm path.
        """
        with self._lock:
            self._suspended = False
            desired = self._desired_follow
            auto_resume = self._auto_resume
        if not (desired and auto_resume):
            return True
        return self.start(wait_for_grace=False)

    def stop(self, reason: str = "stopped") -> None:
        with self._lock:
            self._desired_follow = False
            self._session_controlled = False
            self._interaction_id = ""
            self._stop_locked(reason)

    def _validate_runtime_paths(self) -> str:
        if not self._enabled:
            return "uwb_follow_disabled"
        if not self._setup_script or not self._path_exists(self._setup_script):
            return f"uwb_setup_script_not_found:{self._setup_script}"
        if not self._serial_device or not self._path_exists(self._serial_device):
            return f"uwb_serial_device_not_found:{self._serial_device}"
        if not self._cmd_vel_topic:
            return "uwb_cmd_vel_topic_empty"
        return ""

    def _commands(self) -> list[list[str]]:
        aoa_command = [
            "ros2", "run", self._aoa_package,
            self._aoa_executable, self._serial_device,
        ]
        if self._target_fob_id:
            aoa_command += [
                "--ros-args", "-p", f"target_fob_id:={self._target_fob_id}"
            ]
        return [
            self._sourced_command(aoa_command),
            self._sourced_command(
                [
                    "ros2", "launch", self._follow_package,
                    self._follow_launch_file, "enable_motion:=true",
                    f"cmd_vel_topic:={self._cmd_vel_topic}",
                ]
            ),
        ]

    def _sourced_command(self, command: Sequence[str]) -> list[str]:
        return [
            "/bin/bash", "--noprofile", "--norc", "-c",
            'source "$1" && shift && exec "$@"',
            "marsdog-uwb-follow", self._setup_script, *command,
        ]

    def _wait_for_grace_locked(self) -> bool:
        deadline = self._started_at + self._startup_grace_sec
        while self._monotonic() < deadline:
            if self._cancel_requested.is_set() or self._should_stop():
                self._stop_locked("canceled_during_startup")
                return False
            if not self._all_alive_locked():
                self._last_error = "uwb_process_exited_during_startup"
                self._stop_locked(self._last_error)
                return False
            self._sleep(min(0.05, deadline - self._monotonic()))
        return self._all_alive_locked()

    def _all_alive_locked(self) -> bool:
        return bool(self._processes) and all(
            process.poll() is None for process in self._processes
        )

    def _stop_locked(self, reason: str) -> bool:
        processes = list(reversed(self._processes))
        confirmed = True
        for process in processes:
            try:
                if self._terminate_process_override is not None:
                    self._terminate_process_override(
                        process, self._shutdown_timeout_sec
                    )
                else:
                    self._terminate_process(process)
            except Exception as exc:
                confirmed = False
                logger.error(
                    "Failed to terminate UWB process pid=%s: %s",
                    getattr(process, "pid", "unknown"),
                    exc,
                )
            if process.poll() is None:
                confirmed = False
        self._publish_stop_locked()
        if confirmed:
            self._processes.clear()
        else:
            self.recovery_required = True
            self._last_error = "uwb_process_exit_unconfirmed:operator_recovery_required"
        if self._motion_prepared and confirmed:
            # Clear before external cleanup so repeated stop requests cannot
            # run a second mode-release/settle sequence.
            self._motion_prepared = False
            try:
                if self._finish_motion() is False:
                    confirmed = False
            except Exception as exc:
                confirmed = False
                logger.warning("Failed to finish UWB chassis motion: %s", exc)
        if not confirmed:
            self.recovery_required = True
            self._last_error = "uwb_chassis_stop_unconfirmed:operator_recovery_required"
        if processes:
            logger.info("UWB follow pipeline stopped: %s", reason)
        return confirmed

    def _terminate_process(self, process: Any) -> None:
        # The ros2 launch parent may exit while a child in its process group
        # still publishes /cmd_vel. Verify the whole group has disappeared.
        def group_exists() -> bool:
            process.poll()  # reap an exited leader before probing its group
            try:
                os.killpg(process.pid, 0)
                return True
            except ProcessLookupError:
                return False

        if not group_exists():
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        for sig in (signal.SIGKILL,):
            deadline = self._monotonic() + self._shutdown_timeout_sec
            while group_exists() and self._monotonic() < deadline:
                self._sleep(0.05)
            if not group_exists():
                break
            logger.warning("UWB process group ignored SIGTERM; sending SIGKILL")
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
        if process.poll() is None:
            try:
                process.wait(timeout=self._shutdown_timeout_sec)
            except subprocess.TimeoutExpired as exc:
                raise RuntimeError("uwb_process_exit_unconfirmed") from exc
        if group_exists():
            raise RuntimeError("uwb_process_group_exit_unconfirmed")

    def _publish_stop_locked(self) -> None:
        for _ in range(self._stop_publish_count):
            try:
                self._publish_twist(TwistCommand())
            except Exception as exc:
                logger.warning("Failed to publish UWB follow stop: %s", exc)
