"""PostureManager — tracks and validates posture transitions during execution."""

from __future__ import annotations

import logging
from typing import Any

logger = logging.getLogger(__name__)

_VALID_POSTURES = {
    "standing", "sitting", "lying", "lying_side", "lying_back",
    "lying_belly", "sleep_curled", "moving", "unknown",
}


class PostureManager:
    """Tracks the robot's current posture and validates transitions.

    Each action unit declares ``from_postures`` and ``to_posture``.
    If the current posture doesn't match, the manager looks for a
    transition or rejects the candidate.
    """

    def __init__(
        self,
        transitions: dict[str, dict[str, list[str]]] | None = None,
    ) -> None:
        self._posture: str = "unknown"
        self._transitions: dict[str, dict[str, list[str]]] = transitions or {}

    @property
    def current(self) -> str:
        return self._posture

    def set_posture(self, posture: str) -> None:
        if posture not in _VALID_POSTURES:
            logger.warning("Unknown posture %r — setting anyway", posture)
        self._posture = posture

    def can_transition(self, from_posture: str, to_posture: str) -> bool:
        """Check if a transition is allowed."""
        if from_posture == to_posture:
            return True
        if to_posture == "same":
            return True
        if from_posture == "unknown" or to_posture == "unknown":
            return True
        # Check transition map
        allowed = self._transitions.get(from_posture, {}).get(to_posture)
        return allowed is not None

    def get_transition_actions(
        self, from_posture: str, to_posture: str,
    ) -> list[str]:
        """Return action IDs needed to transition between postures."""
        if from_posture == to_posture or to_posture == "same":
            return []
        if from_posture == "unknown":
            return []
        transitions = self._transitions.get(from_posture, {})
        return list(transitions.get(to_posture, []))

    def is_posture_valid_for(
        self, from_postures: list[str],
    ) -> bool:
        """Check if current posture is in the allowed list."""
        if not from_postures:
            return True
        if self._posture == "unknown":
            return True
        return self._posture in from_postures

    def apply_unit_to_posture(self, to_posture: str | None) -> None:
        """Update posture after executing a unit."""
        if to_posture is None or to_posture == "same" or to_posture == "":
            return
        self.set_posture(to_posture)

    def reset(self) -> None:
        self._posture = "unknown"
