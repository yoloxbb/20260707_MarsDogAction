"""ROS2 compatibility layer — conditional imports and fallbacks.

This module allows the rest of the package to gracefully degrade when
ROS2 (rclpy) is not installed, enabling pure-Python testing and demos.

Usage::

    from .ros2_compat import HAS_ROS2, get_execute_behavior_action

    if HAS_ROS2:
        ExecuteBehavior = get_execute_behavior_action()
        ...
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)

# ── rclpy availability ────────────────────────────────────────────────────────

try:
    import rclpy  # noqa: F401

    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

# ── ExecuteBehavior action message ────────────────────────────────────────────
#
# Priority order:
#   1. marsdog_interfaces.action.ExecuteBehavior  (official public interface)
#   2. marsdog_action_executor.action.ExecuteBehavior  (historical, deprecated)

_EXECUTE_BEHAVIOR_ACTION = None
_EXECUTE_BEHAVIOR_SOURCE = None  # track which package provided it


def _try_import_action():
    """Attempt to import ExecuteBehavior from the preferred packages."""
    global _EXECUTE_BEHAVIOR_ACTION, _EXECUTE_BEHAVIOR_SOURCE

    if not HAS_ROS2:
        return

    # 1) Preferred: marsdog_interfaces (public interface package)
    try:
        from marsdog_interfaces.action import ExecuteBehavior  # type: ignore[import-untyped]

        _EXECUTE_BEHAVIOR_ACTION = ExecuteBehavior
        _EXECUTE_BEHAVIOR_SOURCE = "marsdog_interfaces"
        logger.debug("Using ExecuteBehavior from marsdog_interfaces")
        return
    except ImportError:
        pass

    # 2) Fallback: local definition (deprecated, for transition)
    try:
        from marsdog_action_executor.action import ExecuteBehavior  # type: ignore[import-untyped]

        _EXECUTE_BEHAVIOR_ACTION = ExecuteBehavior
        _EXECUTE_BEHAVIOR_SOURCE = "marsdog_action_executor (deprecated)"
        logger.warning(
            "ExecuteBehavior imported from marsdog_action_executor.action — "
            "this is DEPRECATED. Migrate to marsdog_interfaces."
        )
    except ImportError:
        _EXECUTE_BEHAVIOR_ACTION = None
        _EXECUTE_BEHAVIOR_SOURCE = None


def get_execute_behavior_action():
    """Return the ExecuteBehavior action class, or None if unavailable.

    Attempts to import from:
      1. ``marsdog_interfaces.action.ExecuteBehavior`` (official)
      2. ``marsdog_action_executor.action.ExecuteBehavior`` (deprecated fallback)

    Returns:
        The action class, or None.
    """
    if _EXECUTE_BEHAVIOR_ACTION is None:
        _try_import_action()
    return _EXECUTE_BEHAVIOR_ACTION


def get_action_source() -> str | None:
    """Return which package provided the ExecuteBehavior action."""
    if _EXECUTE_BEHAVIOR_ACTION is None:
        _try_import_action()
    return _EXECUTE_BEHAVIOR_SOURCE
