"""Backward-compatibility shim — delegates to ros_node.py.

.. deprecated::
    This module is kept so existing launch scripts and ``setup.py``
    entry points that reference ``marsdog_action_executor.node:main``
    continue to work.  New code should import from
    ``marsdog_action_executor.ros_node``.
"""

from .ros_node import main  # noqa: F401
