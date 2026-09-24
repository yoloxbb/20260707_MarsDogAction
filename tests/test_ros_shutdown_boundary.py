from __future__ import annotations

import pytest

from marsdog_action_executor.ros_node import _shutdown_rclpy_if_ok


class FakeRCLError(Exception):
    pass


class FakeRclpy:
    def __init__(
        self,
        *,
        ok: bool,
        shutdown_error: BaseException | None = None,
    ) -> None:
        self._ok = ok
        self._shutdown_error = shutdown_error
        self.ok_calls = 0
        self.shutdown_calls = 0

    def ok(self) -> bool:
        self.ok_calls += 1
        return self._ok

    def shutdown(self) -> None:
        self.shutdown_calls += 1
        if self._shutdown_error is not None:
            raise self._shutdown_error
        self._ok = False


def test_shutdown_runs_once_when_context_is_ok() -> None:
    rclpy_module = FakeRclpy(ok=True)

    assert _shutdown_rclpy_if_ok(rclpy_module, FakeRCLError)
    assert not _shutdown_rclpy_if_ok(rclpy_module, FakeRCLError)

    assert rclpy_module.ok_calls == 2
    assert rclpy_module.shutdown_calls == 1


def test_already_shutdown_context_is_a_clean_noop() -> None:
    rclpy_module = FakeRclpy(ok=False)

    assert not _shutdown_rclpy_if_ok(rclpy_module, FakeRCLError)

    assert rclpy_module.ok_calls == 1
    assert rclpy_module.shutdown_calls == 0


def test_shutdown_race_swallows_only_rcl_error() -> None:
    rclpy_module = FakeRclpy(
        ok=True,
        shutdown_error=FakeRCLError("rcl_shutdown already called"),
    )

    assert not _shutdown_rclpy_if_ok(rclpy_module, FakeRCLError)
    assert rclpy_module.shutdown_calls == 1


def test_unrelated_shutdown_error_is_not_swallowed() -> None:
    rclpy_module = FakeRclpy(
        ok=True,
        shutdown_error=RuntimeError("unexpected cleanup failure"),
    )

    with pytest.raises(RuntimeError, match="unexpected cleanup failure"):
        _shutdown_rclpy_if_ok(rclpy_module, FakeRCLError)
