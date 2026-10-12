"""Live cmd_vel ingress using the same Go2 posture gate as Action motions."""
from __future__ import annotations

import math
import threading
import time
from typing import Callable

from .velocity import TwistCommand


class Go2VelocityBridge:
    """No queued/replayed velocity; watchdog stops a vanished upstream."""

    def __init__(self, backend, timeout_sec: float = 0.5,
                 monotonic: Callable[[], float] = time.monotonic):
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError("cmd_vel timeout must be finite and positive")
        self._backend = backend
        self._timeout = timeout_sec
        self._monotonic = monotonic
        self._last_received = None
        self._lock = threading.Lock()

    def receive(self, command: TwistCommand) -> None:
        with self._lock:
            # A stop is already complete; no watchdog or preparation replay.
            self._last_received = None if command.is_zero else self._monotonic()
            self._backend.publish_external_velocity(command)

    def tick(self) -> None:
        with self._lock:
            if self._last_received is not None and (
                self._monotonic() - self._last_received >= self._timeout
            ):
                self._last_received = None
                self._backend.publish_external_velocity(TwistCommand())
