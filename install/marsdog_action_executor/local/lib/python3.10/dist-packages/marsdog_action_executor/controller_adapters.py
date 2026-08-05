"""Controller adapters — abstraction layer for hardware control.

This module defines the interface that the Executor uses to execute individual
ActionSteps. The default implementation is a mock that simulates execution via
sleep, suitable for testing and demos.

Real hardware integrations (future work):
  - MotionControllerAdapter  → /motion/execute_motion
  - GimbalControllerAdapter  → /gimbal/set_target
  - ExpressionControllerAdapter → /expression/play
  - NavigationControllerAdapter → /navigation/navigate_to
"""

from __future__ import annotations

import logging
import time
from typing import Any

from .models import ActionStep

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════════════════
# Abstract base
# ═══════════════════════════════════════════════════════════════════════════════


class BaseControllerAdapter:
    """Abstract interface for executing action steps on hardware.

    Subclass this to implement real hardware control (ROS2 services,
    serial protocols, etc.).
    """

    def execute_step(self, step: ActionStep) -> bool:
        """Execute a single action step.

        Args:
            step: The action step to execute.

        Returns:
            True on success, False on failure.
        """
        raise NotImplementedError

    def cancel_step(self, step: ActionStep) -> None:
        """Cancel a currently-executing step (best-effort).

        Args:
            step: The step to cancel.
        """
        raise NotImplementedError

    def emergency_stop(self) -> None:
        """Immediately stop all motion / output (best-effort)."""
        raise NotImplementedError


# ═══════════════════════════════════════════════════════════════════════════════
# Mock implementation
# ═══════════════════════════════════════════════════════════════════════════════


class MockControllerAdapter(BaseControllerAdapter):
    """Mock adapter that simulates execution via time.sleep().

    Suitable for:
      - Unit / integration tests
      - Standalone demos
      - CI pipelines without ROS2

    Additional keyword arguments can control behaviour:
      - ``fail_on``: optional set of action_ids that should simulate failure.
      - ``delay_factor``: multiplier for step duration (default 1.0).
    """

    def __init__(self, **options: Any) -> None:
        self._fail_on: set[str] = set(options.get("fail_on", []) or [])
        self._delay_factor: float = float(options.get("delay_factor", 1.0))
        self._cancelled: bool = False

    def execute_step(self, step: ActionStep) -> bool:
        """Simulate execution by sleeping for step.duration_sec."""
        self._cancelled = False

        if step.action_id in self._fail_on:
            logger.warning(
                "MockControllerAdapter: simulated failure for %s", step.action_id
            )
            return False

        delay = step.duration_sec * self._delay_factor
        # Sleep in small increments so cancel can be noticed promptly
        tick = 0.05
        elapsed = 0.0
        while elapsed < delay:
            if self._cancelled:
                logger.info(
                    "MockControllerAdapter: cancelled during %s", step.action_id
                )
                return False
            time.sleep(min(tick, delay - elapsed))
            elapsed += tick

        logger.debug("MockControllerAdapter: completed %s (%.2fs)", step.action_id, delay)
        return True

    def cancel_step(self, step: ActionStep) -> None:
        """Signal cancellation of the current step."""
        self._cancelled = True
        logger.info("MockControllerAdapter: cancel requested for %s", step.action_id)

    def emergency_stop(self) -> None:
        """Signal emergency stop."""
        self._cancelled = True
        logger.info("MockControllerAdapter: emergency stop requested")


# ═══════════════════════════════════════════════════════════════════════════════
# Factory for future hardware adapters
# ═══════════════════════════════════════════════════════════════════════════════


_ADAPTER_REGISTRY: dict[str, type[BaseControllerAdapter]] = {
    "mock": MockControllerAdapter,
}


def register_adapter(name: str, cls: type[BaseControllerAdapter]) -> None:
    """Register a controller adapter class."""
    _ADAPTER_REGISTRY[name] = cls


def create_adapter(name: str = "mock", **options: Any) -> BaseControllerAdapter:
    """Create a controller adapter by name.

    Args:
        name: Adapter name (``"mock"`` by default).
        **options: Passed to the adapter constructor.

    Returns:
        A BaseControllerAdapter instance.

    Raises:
        KeyError: If the adapter name is not registered.
    """
    cls = _ADAPTER_REGISTRY.get(name)
    if cls is None:
        raise KeyError(
            f"Unknown controller adapter: {name!r}. "
            f"Available: {sorted(_ADAPTER_REGISTRY.keys())}"
        )
    return cls(**options)
