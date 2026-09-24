"""Controller adapters — abstraction layer for hardware control.

Each adapter routes specific unit types to a backend:
  - velocity: shared body-frame velocity command and ROS2 Twist bridge
  - motion_adapter: servo/gait control
  - gimbal_adapter: camera/head gimbal
  - audio_adapter: speaker/sound output
  - navigation_adapter: autonomous navigation
  - perception_adapter: object/person detection
  - expression_adapter: LED/display output
"""

from .velocity import Ros2TwistPublisher, TwistCommand
from .go2_sport_backend import (
    Go2ChassisBackend,
    Go2SportRequest,
    Ros2Go2SportPublisher,
)
from .lite3_backend import (
    Lite3ChassisBackend,
    Lite3RobotStatus,
    Lite3SimpleCommand,
    Ros2Lite3SimpleCommandPublisher,
    Ros2Lite3StatusSubscriber,
    make_ros2_lite3_backend,
)
from .attention_tracking_controller import AttentionTrackingController
from .navigation_adapter import BehaviorMobilityAdapter, Ros2Nav2Client
from .waypoint_nav_adapter import (
    Ros2WaypointNavClient,
    WaypointNavOutcome,
    WaypointNavProtocolError,
    WaypointNavStatus,
    parse_waypoint_status,
)
from .wake_orientation_adapter import (
    Ros2Nav2SpinClient,
    WakeOrientationAdapter,
)
from .target_approach_adapter import (
    TargetApproachAdapter,
    TargetApproachResult,
)
from .visual_target_approach_adapter import VisualTargetApproachAdapter
from .stationary_expression_adapter import StationaryExpressionAdapter
from .uwb_follow_adapter import UwbFollowAdapter
from .uwb_follow_action_adapter import (
    Ros2FollowUwbClient,
    Ros2SetBehaviorClient,
    UwbFollowActionAdapter,
)
from .go2_utrack_follow_adapter import (
    Go2UtrackFollowAdapter,
    Ros2Go2UtrackClient,
    UtrackSwitchResult,
)
from .mock_adapters import (
    MockAudioAdapter,
    MockExpressionAdapter,
    MockGimbalAdapter,
    MockMotionAdapter,
    MockNavigationAdapter,
    MockPerceptionAdapter,
)

__all__ = [
    "Ros2TwistPublisher",
    "TwistCommand",
    "Go2ChassisBackend",
    "Go2SportRequest",
    "Ros2Go2SportPublisher",
    "Lite3ChassisBackend",
    "Lite3RobotStatus",
    "Lite3SimpleCommand",
    "Ros2Lite3SimpleCommandPublisher",
    "Ros2Lite3StatusSubscriber",
    "make_ros2_lite3_backend",
    "AttentionTrackingController",
    "BehaviorMobilityAdapter",
    "Ros2Nav2Client",
    "Ros2WaypointNavClient",
    "WaypointNavOutcome",
    "WaypointNavProtocolError",
    "WaypointNavStatus",
    "parse_waypoint_status",
    "Ros2Nav2SpinClient",
    "WakeOrientationAdapter",
    "TargetApproachAdapter",
    "TargetApproachResult",
    "VisualTargetApproachAdapter",
    "StationaryExpressionAdapter",
    "UwbFollowAdapter",
    "UwbFollowActionAdapter",
    "Ros2FollowUwbClient",
    "Ros2SetBehaviorClient",
    "Go2UtrackFollowAdapter",
    "Ros2Go2UtrackClient",
    "UtrackSwitchResult",
    "MockMotionAdapter",
    "MockGimbalAdapter",
    "MockAudioAdapter",
    "MockNavigationAdapter",
    "MockPerceptionAdapter",
    "MockExpressionAdapter",
]
