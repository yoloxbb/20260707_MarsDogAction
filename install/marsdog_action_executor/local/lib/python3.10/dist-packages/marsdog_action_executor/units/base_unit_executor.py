"""BaseUnitExecutor — abstract interface for all unit executors."""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Any

from ..execution_context import ExecutionContext
from ..interrupt_manager import InterruptManager, InterruptPolicy


class UnitState(Enum):
    IDLE = "idle"
    RUNNING = "running"
    SUCCESS = "success"
    FAILURE = "failure"
    TIMEOUT = "timeout"
    CANCELED = "canceled"


@dataclass
class UnitResult:
    """Result of executing a single unit."""

    unit_id: str
    state: UnitState
    elapsed_sec: float = 0.0
    message: str = ""
    metadata: dict[str, Any] | None = None


class BaseUnitExecutor:
    """Abstract base for all unit executors.

    Subclass for atomic_action, composite_action, task, policy, modifier.
    """

    def __init__(
        self,
        interrupt_manager: InterruptManager | None = None,
    ) -> None:
        self._interrupt = interrupt_manager or InterruptManager()

    def execute(
        self,
        unit_config: dict[str, Any],
        ctx: ExecutionContext,
    ) -> UnitResult:
        """Execute a unit.  Override in subclass."""
        raise NotImplementedError

    def get_interrupt_policy(self, unit_config: dict[str, Any]) -> InterruptPolicy:
        raw = unit_config.get("interrupt_policy", "safe_point")
        try:
            return InterruptPolicy(raw)
        except ValueError:
            return InterruptPolicy.SAFE_POINT

    def _make_result(
        self,
        unit_id: str,
        state: UnitState,
        elapsed: float = 0.0,
        message: str = "",
        metadata: dict | None = None,
    ) -> UnitResult:
        return UnitResult(
            unit_id=unit_id,
            state=state,
            elapsed_sec=elapsed,
            message=message,
            metadata=metadata,
        )
