"""Zero-motion controller for voice-WAITING in-place expressions."""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Mapping

from .velocity import TwistCommand


class StationaryExpressionAdapter:
    """Execute a timed expression while publishing only redundant zero Twist.

    The current hardware proxy has no independent head/tail controller.  These
    explicit units therefore provide the UI/audio expression interval while
    making chassis immobility the controller's hard invariant.  A future
    posture controller may replace this adapter only if the stationary plan
    validator continues to classify that route as non-chassis.
    """

    def __init__(
        self,
        publish_twist: Callable[[TwistCommand], None] | None = None,
        *,
        publish_rate_hz: float = 10.0,
        stop_publish_count: int = 3,
        should_stop: Callable[[], bool] | None = None,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._publish_twist = publish_twist or (lambda command: None)
        self._period_sec = 1.0 / max(1.0, float(publish_rate_hz))
        self._stop_publish_count = max(1, int(stop_publish_count))
        self._should_stop = should_stop or (lambda: False)
        self._sleep = sleep
        self._cancel_requested = threading.Event()

    def execute_step(
        self,
        unit_config: Mapping[str, Any],
        ctx: Any = None,
        duration: float | None = None,
    ) -> bool:
        del unit_config
        self._cancel_requested.clear()
        requested = max(0.0, float(duration or 0.0))
        elapsed = 0.0
        self._publish_stop()
        try:
            while elapsed < requested:
                if self._cancel_requested.is_set() or self._should_stop():
                    return False
                if getattr(ctx, "cancel_requested", False):
                    return False
                self._publish_twist(TwistCommand())
                interval = min(self._period_sec, requested - elapsed)
                self._sleep(interval)
                elapsed += interval
            return True
        finally:
            self._publish_stop()

    def cancel_step(self, step: Any = None) -> None:
        del step
        self._cancel_requested.set()
        self._publish_stop()

    def emergency_stop(self) -> None:
        self.cancel_step()

    def hold_position(self, duration_sec: float | None = None) -> bool:
        del duration_sec
        self._publish_stop()
        return True

    def _publish_stop(self) -> None:
        for _ in range(self._stop_publish_count):
            self._publish_twist(TwistCommand())
