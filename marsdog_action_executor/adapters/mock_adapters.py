"""Mock controller adapters for development and testing.

Each mock adapter simulates hardware behaviour without real hardware.
Tasks support RUNNING/SUCCESS/FAILURE/TIMEOUT/CANCELED lifecycle.
"""

from __future__ import annotations

import logging
import time
from typing import Any

logger = logging.getLogger(__name__)


class MockMotionAdapter:
    """Mock motion controller — simulates servo/gait execution."""

    def execute(self, unit_config: dict[str, Any], duration: float) -> bool:
        logger.debug("MockMotion: %s (%.1fs)", unit_config.get("unit_id", "?"), duration)
        time.sleep(min(duration, 0.5))  # fast-forward for dev
        return True

    def emergency_stop(self) -> None:
        logger.info("MockMotion: emergency stop")


class MockGimbalAdapter:
    """Mock gimbal controller — simulates camera/head movement."""

    def execute(self, unit_config: dict[str, Any], duration: float) -> bool:
        logger.debug("MockGimbal: %s (%.1fs)", unit_config.get("unit_id", "?"), duration)
        time.sleep(min(duration, 0.3))
        return True

    def emergency_stop(self) -> None:
        logger.info("MockGimbal: emergency stop")


class MockAudioAdapter:
    """Mock audio controller — simulates speaker output."""

    def execute(self, unit_config: dict[str, Any], duration: float) -> bool:
        logger.debug("MockAudio: %s (%.1fs)", unit_config.get("unit_id", "?"), duration)
        time.sleep(min(duration, 0.2))
        return True

    def emergency_stop(self) -> None:
        logger.info("MockAudio: emergency stop")


class MockNavigationAdapter:
    """Mock navigation controller — simulates autonomous navigation.

    Supports RUNNING/SUCCESS/FAILURE/TIMEOUT lifecycle for tasks.
    """

    def execute(self, unit_config: dict[str, Any], duration: float) -> bool:
        logger.debug("MockNav: %s (%.1fs)", unit_config.get("unit_id", "?"), duration)
        time.sleep(min(duration, 1.0))
        return True

    def emergency_stop(self) -> None:
        logger.info("MockNav: emergency stop")


class MockPerceptionAdapter:
    """Mock perception controller — simulates object/person detection."""

    def execute(self, unit_config: dict[str, Any], duration: float) -> bool:
        logger.debug("MockPerception: %s (%.1fs)", unit_config.get("unit_id", "?"), duration)
        time.sleep(min(duration, 0.3))
        return True


class MockExpressionAdapter:
    """Mock expression controller — simulates LED/display output."""

    def execute(self, unit_config: dict[str, Any], duration: float) -> bool:
        logger.debug("MockExpression: %s (%.1fs)", unit_config.get("unit_id", "?"), duration)
        time.sleep(min(duration, 0.1))
        return True
