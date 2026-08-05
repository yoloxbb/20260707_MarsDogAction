"""Controller adapters — abstraction layer for hardware control.

Each adapter routes specific unit types to a backend:
  - agv_adapter: ROS2 Twist motion groups on /cmd_vel
  - motion_adapter: servo/gait control
  - gimbal_adapter: camera/head gimbal
  - audio_adapter: speaker/sound output
  - navigation_adapter: autonomous navigation
  - perception_adapter: object/person detection
  - expression_adapter: LED/display output
"""

from .agv_adapter import AgvMotionAdapter, Ros2TwistPublisher, TwistCommand
from .attention_tracking_controller import AttentionTrackingController
from .navigation_adapter import BehaviorMobilityAdapter, Ros2Nav2Client
from .wake_orientation_adapter import (
    Ros2Nav2SpinClient,
    WakeOrientationAdapter,
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
    "AgvMotionAdapter",
    "Ros2TwistPublisher",
    "TwistCommand",
    "AttentionTrackingController",
    "BehaviorMobilityAdapter",
    "Ros2Nav2Client",
    "Ros2Nav2SpinClient",
    "WakeOrientationAdapter",
    "MockMotionAdapter",
    "MockGimbalAdapter",
    "MockAudioAdapter",
    "MockNavigationAdapter",
    "MockPerceptionAdapter",
    "MockExpressionAdapter",
]
