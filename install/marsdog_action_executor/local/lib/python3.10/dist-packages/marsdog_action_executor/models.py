"""Core data models for the MarsDog Action Executor.

Defines the canonical data structures used throughout the system:
  BehaviorGoal      — inbound goal from behavior tree
  ActionStep         — a single executable action
  ActionStage        — a stage with candidate actions and selection policy
  ActionPlan         — resolved plan with selected steps
  ExecutionFeedback  — periodic progress feedback
  ExecutionResult    — terminal outcome
  ExecutionState     — task lifecycle enum

Backward-compatibility aliases (Goal, Feedback, Result) are exported so
existing code that imported from ``interfaces`` keeps working.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Any


# ═══════════════════════════════════════════════════════════════════════════════
# Execution state enum
# ═══════════════════════════════════════════════════════════════════════════════


class ExecutionState(Enum):
    """Lifecycle states for a single behavior execution."""

    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    TIMEOUT = "TIMEOUT"
    CANCELED = "CANCELED"


# ═══════════════════════════════════════════════════════════════════════════════
# Core data structures
# ═══════════════════════════════════════════════════════════════════════════════


@dataclass
class BehaviorGoal:
    """Goal dispatched by the behavior tree to /execute_behavior.

    Attributes:
        goal_id: Unique identifier (UUID from client).
        behavior_name: Exact behavior-tree name (e.g. ``"eatNormally"``).
        priority_level: Priority 0-6 (higher = more urgent).
        params: Optional key-value parameters for the behavior.
        timeout_sec: Maximum wall-clock duration before automatic timeout.
        timestamp: Epoch seconds when the goal was created.
    """

    goal_id: str
    behavior_name: str
    behavior_id: str = ""  # upstream behavior_id (marsdog_behavior passes this as goal_id)
    priority_level: int = 0
    params: dict[str, Any] = field(default_factory=dict)
    timeout_sec: float = 60.0
    timestamp: float = 0.0

    def __post_init__(self) -> None:
        if not self.timestamp:
            self.timestamp = time.time()

    def to_json(self) -> str:
        return json.dumps(asdict(self))

    @classmethod
    def from_json(cls, data: str | dict) -> BehaviorGoal:
        if isinstance(data, str):
            data = json.loads(data)
        return cls(**data)


@dataclass
class ActionStep:
    """A single executable action within a behavior plan.

    Attributes:
        action_id: Canonical action identifier (``ACT_<CATEGORY>_<DESCRIPTION>``).
        category: Action category (POSTURE | LOCO | HEAD | TAIL | MOUTH | PAW |
                  VOCAL | EAR | GIMBAL | LIGHT | NAV).
        description: Human-readable description.
        duration_sec: How long this step should take.
        safe_to_interrupt: Whether preemption is safe during this step.
        params: Optional parameters for the controller adapter.
    """

    action_id: str
    category: str = ""
    description: str = ""
    duration_sec: float = 1.0
    safe_to_interrupt: bool = True
    params: dict[str, Any] = field(default_factory=dict)


@dataclass
class ActionStage:
    """A stage within a behavior definition containing candidate actions.

    Attributes:
        stage_name: Human-readable stage name.
        candidates: Pool of ActionSteps to choose from.
        selection_policy: How to pick, such as ``"random_one"``.
        required: Whether this stage must complete for the behavior to succeed.
    """

    stage_name: str
    candidates: list[ActionStep] = field(default_factory=list)
    selection_policy: str = "random_one"
    required: bool = True


@dataclass
class ActionPlan:
    """A fully-resolved execution plan for a single behavior goal.

    Built by the Planner: stages are looked up from the catalog, one
    ActionStep is selected per stage, and durations are resolved.

    Attributes:
        goal_id: Matching goal identifier.
        behavior_name: Behavior this plan executes.
        stages: Original stage definitions (all candidates).
        selected_steps: The concrete ActionSteps chosen for execution.
        total_duration_sec: Sum of selected step durations.
        created_at: Epoch seconds when the plan was created.
    """

    goal_id: str
    behavior_name: str
    resolved_behavior_name: str = ""  # exact validated behavior-tree name
    stages: list[ActionStage] = field(default_factory=list)
    selected_steps: list[ActionStep] = field(default_factory=list)
    total_duration_sec: float = 0.0
    created_at: float = 0.0

    def __post_init__(self) -> None:
        if not self.created_at:
            self.created_at = time.time()


@dataclass
class ExecutionFeedback:
    """Periodic progress update published during execution.

    Published after each Stage completes via Action feedback and debug topic.

    Attributes:
        goal_id: Matching goal identifier.
        behavior_name: Behavior being executed.
        status: Always ``"RUNNING"`` during feedback.
        progress: 0.0 – 1.0 fraction complete.
        current_stage: Name of the current stage.
        current_action: Current action_id being executed.
        safe_to_interrupt: Whether preemption is safe right now.
        message: Human-readable status message.
        timestamp: Epoch seconds.
    """

    goal_id: str
    behavior_id: str = ""
    behavior_name: str = ""
    status: str = "RUNNING"
    progress: float = 0.0
    current_stage: str = ""
    current_action: str = ""
    safe_to_interrupt: bool = True
    message: str = ""
    timestamp: float = 0.0

    def __post_init__(self) -> None:
        if not self.timestamp:
            self.timestamp = time.time()

    def to_json(self) -> str:
        return json.dumps(asdict(self))

    @classmethod
    def from_json(cls, data: str | dict) -> ExecutionFeedback:
        if isinstance(data, str):
            data = json.loads(data)
        return cls(**data)


@dataclass
class ExecutionResult:
    """Terminal result published when execution finishes.

    Published once per goal via Action result and optional debug topic.

    Attributes:
        goal_id: Matching goal identifier.
        behavior_name: Behavior that was executed.
        status: ``"SUCCESS"`` | ``"FAILED"`` | ``"CANCELED"`` | ``"TIMEOUT"``.
        result: Short outcome label (``"completed"``, ``"canceled"``, …).
        reason: Human-readable explanation.
        duration_sec: Actual wall-clock duration.
        failed_action: Action that failed (if any).
        interrupted_by: What caused interruption (if any).
        reward: Numeric reward signal (-1.0 to 1.0).
        timestamp: Epoch seconds.
    """

    goal_id: str
    behavior_id: str = ""
    behavior_name: str = ""
    status: str = "SUCCESS"
    result: str = "completed"
    reason: str = ""
    duration_sec: float = 0.0
    failed_action: str = ""
    interrupted_by: str = ""
    reward: float = 1.0
    emotion_delta_json: str = "{}"
    need_delta_json: str = "{}"
    metadata_json: str = "{}"
    timestamp: float = 0.0

    def __post_init__(self) -> None:
        if not self.timestamp:
            self.timestamp = time.time()

    def to_json(self) -> str:
        return json.dumps(asdict(self))

    @classmethod
    def from_json(cls, data: str | dict) -> ExecutionResult:
        if isinstance(data, str):
            data = json.loads(data)
        return cls(**data)


# ═══════════════════════════════════════════════════════════════════════════════
# Backward-compatibility aliases
# ═══════════════════════════════════════════════════════════════════════════════

# These allow existing code that does ``from .interfaces import Goal`` to
# continue working after interfaces.py re-exports from here.
Goal = BehaviorGoal
Feedback = ExecutionFeedback
Result = ExecutionResult
