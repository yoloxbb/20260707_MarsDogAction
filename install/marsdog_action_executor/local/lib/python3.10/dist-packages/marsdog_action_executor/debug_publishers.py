"""Debug / visualisation topic publishers.

These topics are **debug-only** and should not be relied upon for
business logic. The official interface is the ``/execute_behavior``
ROS2 Action Server.

Debug topics (all under ``/debug/execute_behavior/``):
  - ``/debug/execute_behavior/goal``     — goal received
  - ``/debug/execute_behavior/feedback`` — emitted after each Stage completes
  - ``/debug/execute_behavior/result``   — terminal result

For official visualisation / logging, prefer:
  - ``/behavior/execution_event``  (marsdog_interfaces/msg/BehaviorExecutionEvent)
  - ``/behavior/result_event``     (marsdog_interfaces/msg/BehaviorResultEvent)

If the old topic names (without ``/debug`` prefix) were needed for
compatibility, they can be enabled by setting the environment variable
``MARSDOG_LEGACY_DEBUG_TOPICS=1`` — but this is deprecated and will be
removed in a future release.
"""

from __future__ import annotations

import json
import os

from .models import ExecutionFeedback, ExecutionResult, BehaviorGoal

# ── Conditionally import ROS2 types ───────────────────────────────────────────

from .ros2_compat import HAS_ROS2

if HAS_ROS2:
    from std_msgs.msg import String  # noqa: F811


# ── Topic name constants ──────────────────────────────────────────────────────

DEBUG_TOPIC_GOAL = "/debug/execute_behavior/goal"
DEBUG_TOPIC_FEEDBACK = "/debug/execute_behavior/feedback"
DEBUG_TOPIC_RESULT = "/debug/execute_behavior/result"

# Deprecated — kept for backward compat, off by default
_LEGACY = os.environ.get("MARSDOG_LEGACY_DEBUG_TOPICS", "") in ("1", "true", "yes")
LEGACY_TOPIC_GOAL = "/execute_behavior/goal"
LEGACY_TOPIC_FEEDBACK = "/execute_behavior/feedback"
LEGACY_TOPIC_RESULT = "/execute_behavior/result"


# ═══════════════════════════════════════════════════════════════════════════════
# ROS2 publisher helper
# ═══════════════════════════════════════════════════════════════════════════════


class DebugPublishers:
    """Manages debug topic publishers for a ROS2 Node.

    Usage inside a Node::

        debug = DebugPublishers(node, enable_legacy=False)
        debug.publish_goal(goal)
        debug.publish_feedback(feedback)
        debug.publish_result(result)

    Args:
        node: The ROS2 Node to create publishers on.
        enable_legacy: If True, also publish to deprecated
            ``/execute_behavior/*`` topics (default False).
    """

    def __init__(self, node, enable_legacy: bool = False) -> None:
        self._node = node
        self._enable_legacy = enable_legacy or _LEGACY

        if not HAS_ROS2:
            self._goal_pub = None
            self._fb_pub = None
            self._result_pub = None
            self._legacy_goal_pub = None
            self._legacy_fb_pub = None
            self._legacy_result_pub = None
            return

        self._goal_pub = node.create_publisher(String, DEBUG_TOPIC_GOAL, 10)
        self._fb_pub = node.create_publisher(String, DEBUG_TOPIC_FEEDBACK, 10)
        self._result_pub = node.create_publisher(String, DEBUG_TOPIC_RESULT, 10)

        if self._enable_legacy:
            node.get_logger().warning(
                "Legacy debug topics enabled "
                "(/execute_behavior/goal|feedback|result). "
                "These are DEPRECATED — use /debug/execute_behavior/* instead."
            )
            self._legacy_goal_pub = node.create_publisher(
                String, LEGACY_TOPIC_GOAL, 10
            )
            self._legacy_fb_pub = node.create_publisher(
                String, LEGACY_TOPIC_FEEDBACK, 10
            )
            self._legacy_result_pub = node.create_publisher(
                String, LEGACY_TOPIC_RESULT, 10
            )
        else:
            self._legacy_goal_pub = None
            self._legacy_fb_pub = None
            self._legacy_result_pub = None

    def publish_goal(self, goal: BehaviorGoal) -> None:
        """Publish goal to debug topic(s)."""
        data = goal.to_json()
        self._pub(self._goal_pub, data)
        self._pub(self._legacy_goal_pub, data)

    def publish_feedback(self, fb: ExecutionFeedback) -> None:
        """Publish feedback to debug topic(s)."""
        data = fb.to_json()
        self._pub(self._fb_pub, data)
        self._pub(self._legacy_fb_pub, data)

    def publish_result(self, result: ExecutionResult) -> None:
        """Publish result to debug topic(s)."""
        data = result.to_json()
        self._pub(self._result_pub, data)
        self._pub(self._legacy_result_pub, data)

    @staticmethod
    def _pub(publisher, data: str) -> None:
        if publisher is not None:
            publisher.publish(String(data=data))
