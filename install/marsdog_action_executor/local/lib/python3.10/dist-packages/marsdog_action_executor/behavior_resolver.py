"""Strict behavior-name validation for the behavior-tree action contract.

Only exact names loaded from ``config/behavior_tree_actions.yaml`` are
executable.
"""

from __future__ import annotations

import logging

from .execution_context import ExecutionContext

logger = logging.getLogger(__name__)


class BehaviorResolver:
    """Validate a requested behavior against the configured direct-name set."""

    def __init__(self, canonical_behaviors: set[str]) -> None:
        self._canonical_behaviors = set(canonical_behaviors)

    def resolve(self, ctx: ExecutionContext) -> ExecutionContext:
        """Validate the exact requested name and preserve it unchanged."""
        name = ctx.requested_behavior_name
        if name not in self._canonical_behaviors:
            ctx.resolved_behavior_name = name
            ctx.is_valid = False
            ctx.error_reason = f"unsupported_behavior: {name!r}"
            logger.error("Unsupported behavior-tree name: %r", name)
            return ctx

        ctx.resolved_behavior_name = name
        return ctx

    def get_canonical_behaviors(self) -> set[str]:
        """Return the exact directly executable behavior-name set."""
        return set(self._canonical_behaviors)
