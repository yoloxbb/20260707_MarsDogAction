"""MarsDog Action Executor — bionic robot dog behavior execution engine.

Receives behavior goals (``behavior_name``) from the upstream behavior tree via
the ``/execute_behavior`` ROS2 Action Server, resolves them into staged action
plans, and drives execution through a pluggable controller-adapter layer.

Architecture (v2)::

    Behavior Tree (marsdog_behavior)
        │  /execute_behavior Action goal
        ▼
    ros_node.py              ← ROS2 Action Server shell
        │
        ▼
    GoalParser → ExecutionContext
        │
        ▼
    BehaviorResolver → exact behavior-name validation
        │
        ▼
    StageExecutor → filter → select → execute units per stage
        │
        ▼
    ResultEvaluator → BehaviorResult

Public API::

    from marsdog_action_executor import (
        # Models
        BehaviorGoal, ActionStep, ActionStage, ActionPlan,
        ExecutionFeedback, ExecutionResult, ExecutionState,
        # v2 Pipeline
        GoalParser, BehaviorResolver, ConfigLoader,
        StageExecutor, ResultEvaluator, BehaviorResult,
        # Context
        ExecutionContext,
        # Support
        EligibilityChecker, PostureManager, InterruptManager,
        # Controller adapters
        AgvMotionAdapter, Ros2TwistPublisher, TwistCommand,
        BaseControllerAdapter, MockControllerAdapter, create_adapter,
        # ROS2 (conditional)
        HAS_ROS2, ActionExecutorNode, main as ros_main,
        # Backward-compat aliases
        Goal, Feedback, Result, TaskState,
    )
"""

from .adapters.agv_adapter import (
    AgvMotionAdapter,
    Ros2TwistPublisher,
    TwistCommand,
)
from .adapters.navigation_adapter import (
    BehaviorMobilityAdapter,
    Ros2Nav2Client,
)
from .controller_adapters import (
    BaseControllerAdapter,
    MockControllerAdapter,
    create_adapter,
)
from .models import (
    ActionPlan,
    ActionStage,
    ActionStep,
    BehaviorGoal,
    ExecutionFeedback,
    ExecutionResult,
    ExecutionState,
    # Backward-compat
    Feedback,
    Goal,
    Result,
)
from .ros2_compat import HAS_ROS2

# ActionExecutorNode is importable directly from .ros_node when ROS2 is
# fully available (rclpy + all transitive deps).  We intentionally do NOT
# re-export it from __init__ to avoid import errors in venvs that lack
# numpy etc. — users who need it can do:
#   from marsdog_action_executor.ros_node import ActionExecutorNode

__all__ = [
    # Models
    "ActionPlan",
    "ActionStage",
    "ActionStep",
    "BehaviorGoal",
    "ExecutionFeedback",
    "ExecutionResult",
    "ExecutionState",
    # Controller adapters
    "AgvMotionAdapter",
    "Ros2TwistPublisher",
    "TwistCommand",
    "BehaviorMobilityAdapter",
    "Ros2Nav2Client",
    "BaseControllerAdapter",
    "MockControllerAdapter",
    "create_adapter",
    # ROS2 compat
    "HAS_ROS2",
    # Backward-compat
    "Feedback",
    "Goal",
    "Result",
]
