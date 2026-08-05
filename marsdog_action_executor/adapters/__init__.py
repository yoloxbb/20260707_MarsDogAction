"""Controller adapters — abstraction layer for hardware control.

Each adapter routes specific unit types to a backend:
  - motion_adapter: servo/gait control
  - gimbal_adapter: camera/head gimbal
  - audio_adapter: speaker/sound output
  - navigation_adapter: autonomous navigation
  - perception_adapter: object/person detection
  - expression_adapter: LED/display output
"""

from .mock_adapters import (
    MockAudioAdapter,
    MockExpressionAdapter,
    MockGimbalAdapter,
    MockMotionAdapter,
    MockNavigationAdapter,
    MockPerceptionAdapter,
)

__all__ = [
    "MockMotionAdapter",
    "MockGimbalAdapter",
    "MockAudioAdapter",
    "MockNavigationAdapter",
    "MockPerceptionAdapter",
    "MockExpressionAdapter",
]
