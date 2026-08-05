"""InterruptManager — handles cancel, preempt, and safe interrupt points."""

from __future__ import annotations

import logging
import time
from enum import Enum

from .execution_context import ExecutionContext

logger = logging.getLogger(__name__)


class InterruptPolicy(Enum):
    IMMEDIATE = "immediate"
    SAFE_POINT = "safe_point"
    NON_INTERRUPTIBLE = "non_interruptible"


class InterruptState(Enum):
    IDLE = "idle"
    ACTIVE = "active"
    CANCEL_REQUESTED = "cancel_requested"
    CANCELED = "canceled"
    CLEANING_UP = "cleaning_up"


class InterruptManager:
    """Manages interrupt lifecycle for a single behavior execution.

    Each action unit declares an ``interrupt_policy``:
      - immediate: stop controller now, cleanup, return canceled.
      - safe_point: set flag, wait for safe point, cleanup, return canceled.
      - non_interruptible: finish current unit, don't start next, cleanup.

    Cleanup includes: stop chassis, stop navigation, stop gimbal,
    stop audio, release objects, clear modifiers, return to safe posture.
    """

    def __init__(self) -> None:
        self._state = InterruptState.IDLE
        self._policy = InterruptPolicy.SAFE_POINT
        self._cancel_requested_at: float = 0.0

    # ── public API ──────────────────────────────────────────────────────

    @property
    def state(self) -> InterruptState:
        return self._state

    @property
    def cancel_requested(self) -> bool:
        return self._state in (
            InterruptState.CANCEL_REQUESTED,
            InterruptState.CANCELED,
            InterruptState.CLEANING_UP,
        )

    @property
    def safe_to_interrupt(self) -> bool:
        """Whether it is currently safe to interrupt."""
        if self._state == InterruptState.IDLE:
            return True
        if self._policy == InterruptPolicy.NON_INTERRUPTIBLE:
            return False
        return True

    def start_unit(self, policy: InterruptPolicy) -> None:
        """Called when a new unit begins execution."""
        self._policy = policy
        if self._state == InterruptState.CANCEL_REQUESTED:
            if policy == InterruptPolicy.IMMEDIATE:
                self._state = InterruptState.CANCELED
            # else: continue with cancel_requested — complete current unit

    def request_cancel(self) -> None:
        """Called by external cancel signal."""
        if self._state in (InterruptState.IDLE, InterruptState.ACTIVE):
            self._state = InterruptState.CANCEL_REQUESTED
            self._cancel_requested_at = time.time()
            logger.info("Cancel requested (policy=%s)", self._policy.value)

    def check_safe_point(self) -> bool:
        """Check if we're at a safe point to stop."""
        if self._state != InterruptState.CANCEL_REQUESTED:
            return False
        if self._policy == InterruptPolicy.IMMEDIATE:
            self._state = InterruptState.CANCELED
            return True
        # safe_point and non_interruptible: signal but don't force
        return True

    def unit_completed(self) -> None:
        """Called when the current unit finishes execution."""
        if self._state == InterruptState.CANCEL_REQUESTED:
            self._state = InterruptState.CANCELED
            logger.info("Unit completed after cancel request — transitioning to CANCELED")

    def start_cleanup(self) -> None:
        self._state = InterruptState.CLEANING_UP

    def finish_cleanup(self) -> None:
        self._state = InterruptState.CANCELED

    def apply_to_context(self, ctx: ExecutionContext) -> None:
        """Sync interrupt state to ExecutionContext."""
        ctx.cancel_requested = self.cancel_requested
        ctx.safe_to_interrupt = self.safe_to_interrupt

    def reset(self) -> None:
        self._state = InterruptState.IDLE
        self._policy = InterruptPolicy.SAFE_POINT
