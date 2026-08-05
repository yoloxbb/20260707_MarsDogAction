"""GoalParser — converts raw ROS2 Action goal into ExecutionContext.

Single entry point for all upstream parameter handling.
No other module should parse params_json directly.
"""

from __future__ import annotations

import logging
from typing import Any

from .execution_context import ExecutionContext

logger = logging.getLogger(__name__)


class GoalParser:
    """Parses incoming behavior goals into validated ExecutionContext objects.

    Usage::

        parser = GoalParser()
        ctx = parser.parse(behavior_name="eatNormally", params_json='{"level":"MID"}')
        if not ctx.is_valid:
            return error_result(ctx.error_reason)
    """

    def parse(
        self,
        behavior_name: str,
        params_json: str | dict | None = None,
        seed: int | None = None,
        priority_level: int = 5,
        timeout_sec: float = 60.0,
    ) -> ExecutionContext:
        """Parse a raw goal into an ExecutionContext.

        Args:
            behavior_name: As received in the Action goal.
            params_json: Raw JSON string or dict from params_json field.
            seed: Optional random seed override.
            priority_level: Priority from the goal.
            timeout_sec: Timeout from the goal.

        Returns:
            ExecutionContext. Check ``.is_valid`` — if False, read
            ``.error_reason`` for details.
        """
        ctx = ExecutionContext.from_goal(
            behavior_name=behavior_name,
            params_json=params_json,
            seed=seed,
        )

        # Inject goal-level fields that aren't in params_json
        ctx.priority_level = priority_level
        ctx.metadata["timeout_sec"] = timeout_sec

        if not ctx.is_valid:
            logger.error(
                "GoalParser: invalid params for %r: %s",
                behavior_name, ctx.error_reason,
            )

        return ctx

    def parse_from_ros_goal(self, goal_request: Any) -> ExecutionContext:
        """Parse from a ROS2 Action goal object.

        Uses duck-typing so we don't need to import the action type:
        reads ``goal_id``, ``behavior_name``, ``params_json``,
        ``priority_level``, ``timeout_sec``.
        """
        return self.parse(
            behavior_name=getattr(goal_request, "behavior_name", "unknown"),
            params_json=getattr(goal_request, "params_json", "{}"),
            priority_level=getattr(goal_request, "priority_level", 5),
            timeout_sec=getattr(goal_request, "timeout_sec", 60.0),
        )
