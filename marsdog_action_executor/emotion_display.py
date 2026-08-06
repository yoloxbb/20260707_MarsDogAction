"""Friendly animated Qt5 virtual-pet display for dog emotion and behavior.

Usage::

    ros2 run marsdog_action_executor emotion_display

Requires: PySide2 (``sudo apt install python3-pyside2.qtwidgets``)

The main face is drawn and animated in real time. Legacy category images remain
installed for compatibility and fallback consumers.
"""

from __future__ import annotations

import json
import logging
import math
import os
import queue
import random
import sys
import threading
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

# --- Qt bindings ---

try:
    from PySide2.QtCore import QPointF, QRectF, Qt, QTimer
    from PySide2.QtGui import (
        QBrush,
        QColor,
        QFont,
        QLinearGradient,
        QPainter,
        QPainterPath,
        QPalette,
        QPen,
    )
    from PySide2.QtWidgets import (
        QApplication, QLabel, QProgressBar,
        QVBoxLayout, QHBoxLayout, QSizePolicy, QWidget,
    )
    HAS_QT = True
except ImportError:
    HAS_QT = False
    # Keep pure classification/image helpers importable in headless tests.
    QWidget = object

from .ros2_compat import HAS_ROS2

if HAS_ROS2:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String


# --- Image path resolution ---

def _resolve_image_dirs() -> tuple[Path, ...]:
    """Return all image roots in priority order, including source fallback."""
    candidates: list[Path] = []
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("marsdog_action_executor"))
        candidates.append(share / "config" / "emotion_images")
    except Exception:
        pass
    candidates.extend((
        Path(__file__).resolve().parent.parent / "config" / "emotion_images",
        Path("config") / "emotion_images",
    ))

    unique: list[Path] = []
    for candidate in candidates:
        if candidate not in unique:
            unique.append(candidate)
    return tuple(unique)


_IMAGE_DIRS = _resolve_image_dirs()


# --- Emotion -> image + color ---

EMOTION_MAP: dict[str, dict[str, str]] = {
    "calm":       {"emoji": "\U0001f60c", "label": "平静", "color": "#87CEEB", "image": "平静.png"},
    "joy":        {"emoji": "\U0001f60a", "label": "喜悦", "color": "#FFD700", "image": "愉悦.png"},
    "excitement": {"emoji": "\U0001f929", "label": "兴奋", "color": "#FF6B6B", "image": "兴奋.png"},
    "curiosity":  {"emoji": "\U0001f9d0", "label": "好奇", "color": "#C3B1E1", "image": "好奇.png"},
    "anxiety":    {"emoji": "\U0001f630", "label": "焦虑", "color": "#FFB347", "image": "焦虑.png"},
    "fear":       {"emoji": "\U0001f628", "label": "恐惧", "color": "#A9A9A9", "image": "恐惧.png"},
    "eat":        {"emoji": "\U0001f37d", "label": "进食", "color": "#FF8C42", "image": "进食.png"},
    "sleep":      {"emoji": "\U0001f634", "label": "休息", "color": "#7B68EE", "image": "睡觉.png"},
    "social":     {"emoji": "\U0001f43e", "label": "社交", "color": "#98FB98", "image": "社交.png"},
    "explore":    {"emoji": "\U0001f50d", "label": "探索", "color": "#20B2AA", "image": "探索.png"},
    "command":    {"emoji": "\U0001f3af", "label": "指令", "color": "#6495ED", "image": "指令.png"},
    "emergency":  {"emoji": "\U0001f6d1", "label": "急停", "color": "#FF0000", "image": "急停.png"},
    "clean":      {"emoji": "\U0001f9f9", "label": "清洁", "color": "#E0E0E0", "image": "清洁.png"},
    "toilet":     {"emoji": "\U0001f4a9", "label": "排泄", "color": "#8B4513", "image": "排泄.png"},
    "charge":     {"emoji": "\U0001f50b", "label": "充电", "color": "#FFD700", "image": "充电.png"},
    "unknown":    {"emoji": "\U0001f415", "label": "就绪", "color": "#B0B0B0", "image": "就绪.png"},
}

_BEHAVIOR_EMOTION: list[tuple[str, str]] = [
    ("expressCalm", "calm"),
    ("expressJoy", "joy"),
    ("expressExcitement", "excitement"),
    ("expressCuriosity", "curiosity"),
    ("expressAnxiety", "anxiety"),
    ("expressFear", "fear"),
    ("eat", "eat"),
    ("seekFood", "eat"),
    ("sleep", "sleep"),
    ("recharge", "charge"),
    ("restInPlace", "charge"),
    ("rest", "sleep"),
    ("greet", "social"),
    ("invite", "social"),
    ("seekHumanInteraction", "social"),
    ("seekInteraction", "social"),
    ("explore", "explore"),
    ("inspect", "explore"),
    ("lickPaws", "clean"),
    ("barkShortAlert", "toilet"),
    ("emergency_stop", "emergency"),
    ("sit_down", "command"),
    ("lie_down", "command"),
    ("stand_up", "command"),
    ("wait_in_place", "command"),
    ("come_to_owner", "command"),
    ("follow_owner", "command"),
    ("give_paw", "command"),
    ("high_five", "command"),
    ("roll_over", "command"),
    ("spin_around", "command"),
    ("return_to_owner", "command"),
    ("drop_object", "command"),
    ("play_dead", "command"),
    ("bring_object", "command"),
    ("fetch_object", "command"),
    ("respond_owner_call", "command"),
    ("testAnimalBoundary", "social"),
]


_PARAM_EMOTION_CATEGORY = {
    "calm": "calm",
    "joy": "joy",
    "excite": "excitement",
    "excitement": "excitement",
    "curious": "curiosity",
    "curiosity": "curiosity",
    "anxiety": "anxiety",
    "fear": "fear",
}


def _classify(name: str, params: dict[str, Any] | None = None) -> str:
    """Resolve a visual category from behavior name and goal metadata."""
    normalized_name = str(name or "").casefold()
    for prefix, cat in _BEHAVIOR_EMOTION:
        if prefix.casefold() in normalized_name:
            return cat

    params = params if isinstance(params, dict) else {}
    emotion = str(
        params.get("source_emotion") or params.get("emotion") or ""
    ).casefold()
    if emotion in _PARAM_EMOTION_CATEGORY:
        return _PARAM_EMOTION_CATEGORY[emotion]

    source = str(params.get("source", "")).casefold()
    trigger_event = str(params.get("trigger_event", "")).upper()
    if source == "audio_direct" or trigger_event.startswith("EVT_VOICE_"):
        return "command"

    category = str(params.get("category", "")).casefold()
    if category == "external_interaction":
        return "command"
    if category == "emotion_expression" and emotion:
        return _PARAM_EMOTION_CATEGORY.get(emotion, "unknown")
    return "unknown"


def _emoji(cat: str) -> str:
    return EMOTION_MAP.get(cat, EMOTION_MAP["unknown"])["emoji"]


def _label(cat: str) -> str:
    return EMOTION_MAP.get(cat, EMOTION_MAP["unknown"])["label"]


def _color(cat: str) -> str:
    return EMOTION_MAP.get(cat, EMOTION_MAP["unknown"])["color"]


def _image_path(cat: str) -> str:
    """Return the full path to the emotion image, or empty string."""
    fname = EMOTION_MAP.get(cat, EMOTION_MAP["unknown"]).get("image", "")
    if not fname:
        return ""
    for image_dir in _IMAGE_DIRS:
        path = image_dir / fname
        if path.is_file():
            return str(path)
    return ""


# --- Event type constants ---

_EV_GOAL = "goal"
_EV_FEEDBACK = "feedback"
_EV_RESULT = "result"


# --- Qt main window ---

WINDOW_BG = "#f8f2ea"


_FACE_PROFILES: dict[str, dict[str, float]] = {
    "unknown":    {"eyes": 0.82, "smile": 0.10, "ears": 0.0, "bob": 1.5, "shake": 0.0, "tilt": 0.0, "speed": 0.7},
    "calm":       {"eyes": 0.42, "smile": 0.35, "ears": -2.0, "bob": 2.5, "shake": 0.0, "tilt": 0.0, "speed": 0.5},
    "joy":        {"eyes": 0.58, "smile": 0.95, "ears": 8.0, "bob": 8.0, "shake": 0.0, "tilt": 2.0, "speed": 1.3},
    "excitement": {"eyes": 1.00, "smile": 0.80, "ears": 14.0, "bob": 13.0, "shake": 1.5, "tilt": 3.0, "speed": 1.8},
    "curiosity":  {"eyes": 0.95, "smile": 0.25, "ears": 9.0, "bob": 3.0, "shake": 0.0, "tilt": 10.0, "speed": 0.9},
    "anxiety":    {"eyes": 0.92, "smile": -0.45, "ears": -12.0, "bob": 2.0, "shake": 3.5, "tilt": -3.0, "speed": 1.5},
    "fear":       {"eyes": 1.00, "smile": -0.85, "ears": -18.0, "bob": 1.0, "shake": 5.5, "tilt": -5.0, "speed": 1.8},
    "eat":        {"eyes": 0.72, "smile": 0.55, "ears": 3.0, "bob": 5.0, "shake": 0.0, "tilt": 0.0, "speed": 1.2},
    "sleep":      {"eyes": 0.05, "smile": 0.20, "ears": -15.0, "bob": 5.0, "shake": 0.0, "tilt": -7.0, "speed": 0.35},
    "social":     {"eyes": 0.88, "smile": 0.75, "ears": 10.0, "bob": 6.0, "shake": 0.0, "tilt": 4.0, "speed": 1.1},
    "explore":    {"eyes": 0.90, "smile": 0.15, "ears": 12.0, "bob": 4.0, "shake": 0.0, "tilt": 8.0, "speed": 1.0},
    "command":    {"eyes": 0.72, "smile": 0.05, "ears": 13.0, "bob": 2.0, "shake": 0.0, "tilt": 0.0, "speed": 1.0},
    "emergency":  {"eyes": 1.00, "smile": -0.70, "ears": 15.0, "bob": 0.0, "shake": 4.0, "tilt": 0.0, "speed": 2.2},
    "clean":      {"eyes": 0.62, "smile": 0.35, "ears": -2.0, "bob": 3.0, "shake": 0.0, "tilt": -4.0, "speed": 0.8},
    "toilet":     {"eyes": 0.75, "smile": -0.10, "ears": 2.0, "bob": 2.0, "shake": 0.0, "tilt": 0.0, "speed": 0.8},
    "charge":     {"eyes": 0.35, "smile": 0.20, "ears": -8.0, "bob": 3.0, "shake": 0.0, "tilt": 0.0, "speed": 0.55},
}


def _face_profile(category: str) -> dict[str, float]:
    """Return a copy of the animation profile for a visual category."""
    return dict(_FACE_PROFILES.get(category, _FACE_PROFILES["unknown"]))


if HAS_QT:

    class DigitalDogFace(QWidget):
        """Animated, code-drawn pet face with soft, friendly styling."""

        def __init__(self, parent: QWidget | None = None) -> None:
            super().__init__(parent)
            self.setMinimumHeight(310)
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self._category = "unknown"
            self._behavior_name = ""
            self._status = "READY"
            self._stage = ""
            self._progress = 0.0
            self._intensity = 50.0
            self._phase = 0.0
            self._blink_frames = 0
            self._next_blink = 90
            self._frame = 0
            self._animation_timer = QTimer(self)
            self._animation_timer.timeout.connect(self._tick)
            self._animation_timer.start(33)

        def set_emotion(
            self,
            category: str,
            behavior_name: str = "",
            intensity: float | None = None,
        ) -> None:
            self._category = category if category in EMOTION_MAP else "unknown"
            self._behavior_name = behavior_name
            if intensity is None:
                self._intensity = 50.0
            else:
                try:
                    self._intensity = min(100.0, max(0.0, float(intensity)))
                except (TypeError, ValueError):
                    self._intensity = 50.0
            self._phase = 0.0
            self.update()

        def set_execution(
            self,
            status: str,
            progress: float | None = None,
            stage: str | None = None,
        ) -> None:
            self._status = str(status or "READY").upper()
            if progress is not None:
                self._progress = min(1.0, max(0.0, float(progress)))
            if stage is not None:
                self._stage = stage
            self.update()

        def _tick(self) -> None:
            profile = _face_profile(self._category)
            intensity_scale = 0.7 + self._intensity / 125.0
            self._phase += 0.055 * profile["speed"] * intensity_scale
            self._frame += 1
            if self._blink_frames > 0:
                self._blink_frames -= 1
            elif self._category != "sleep" and self._frame >= self._next_blink:
                self._blink_frames = 4
                self._next_blink = self._frame + random.randint(70, 170)
            self.update()

        @staticmethod
        def _color(hex_color: str, alpha: int = 255) -> QColor:
            color = QColor(hex_color)
            color.setAlpha(alpha)
            return color

        @staticmethod
        def _draw_heart(
            painter: QPainter,
            center: QPointF,
            size: float,
            color: QColor,
        ) -> None:
            """Draw one soft decorative heart around happy pets."""
            x = center.x()
            y = center.y()
            heart = QPainterPath()
            heart.moveTo(x, y + size * 0.78)
            heart.cubicTo(
                x - size * 1.18, y + size * 0.05,
                x - size * 0.82, y - size * 0.72,
                x, y - size * 0.22,
            )
            heart.cubicTo(
                x + size * 0.82, y - size * 0.72,
                x + size * 1.18, y + size * 0.05,
                x, y + size * 0.78,
            )
            heart.closeSubpath()
            painter.setPen(Qt.NoPen)
            painter.setBrush(color)
            painter.drawPath(heart)

        def paintEvent(self, event) -> None:
            del event
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing, True)
            width = max(1, self.width())
            height = max(1, self.height())
            accent_hex = _color(self._category)
            accent = self._color(accent_hex)

            # Warm pastel card instead of a dark instrument-panel background.
            pastel = QColor(accent_hex).lighter(178)
            background = QLinearGradient(0, 0, width, height)
            background.setColorAt(0.0, QColor("#fffaf5"))
            background.setColorAt(0.58, pastel.lighter(120))
            background.setColorAt(1.0, QColor("#f7eee4"))
            painter.fillRect(self.rect(), QBrush(background))

            # A few slow bubbles keep the scene alive without looking technical.
            painter.setPen(Qt.NoPen)
            for index in range(9):
                x_ratio = 0.08 + ((index * 37) % 84) / 100.0
                y_ratio = 0.10 + ((index * 29) % 72) / 100.0
                drift = math.sin(self._phase * (0.35 + index * 0.025) + index) * 9.0
                lift = math.cos(self._phase * (0.28 + index * 0.018) + index) * 7.0
                radius = 5.0 + (index % 4) * 3.0
                painter.setBrush(self._color(accent_hex, 20 + (index % 3) * 8))
                painter.drawEllipse(
                    QPointF(width * x_ratio + drift, height * y_ratio + lift),
                    radius,
                    radius,
                )

            center = QPointF(width * 0.5, height * 0.50)

            profile = _face_profile(self._category)
            scale = min(width / 680.0, height / 430.0)
            bob = math.sin(self._phase * 2.0) * profile["bob"] * scale
            shake = math.sin(self._phase * 12.0) * profile["shake"] * scale
            tilt = profile["tilt"] + math.sin(self._phase * 0.8) * abs(profile["tilt"]) * 0.18

            painter.save()
            painter.translate(center.x() + shake, center.y() + bob)
            painter.rotate(tilt)
            breath_amount = 0.014 if self._category in ("calm", "sleep", "charge") else 0.007
            breath = 1.0 + math.sin(self._phase * 1.45) * breath_amount
            painter.scale(breath, breath)

            head = QRectF(-178 * scale, -108 * scale, 356 * scale, 245 * scale)
            ear_shift = profile["ears"] * scale
            lively_ears = self._category in (
                "joy", "excitement", "curiosity", "social", "explore", "command",
            )
            ear_range = (8.0 if lively_ears else 3.0) * scale
            left_ear_wiggle = math.sin(self._phase * 3.1) * ear_range
            right_ear_wiggle = math.sin(self._phase * 3.1 + 1.1) * ear_range

            outline = QColor("#654536")
            fur = QColor("#dca06a")
            inner_ear = QColor("#f3aaa5")
            muzzle_color = QColor("#f7d7ad")

            # Soft curved ears make the silhouette feel like a pet, not a robot.
            left_ear = QPainterPath()
            left_ear.moveTo(-145 * scale, -63 * scale)
            left_ear.cubicTo(
                -184 * scale, -102 * scale,
                -164 * scale, -178 * scale - ear_shift - left_ear_wiggle,
                -126 * scale, -190 * scale - ear_shift - left_ear_wiggle,
            )
            left_ear.cubicTo(-93 * scale, -168 * scale, -75 * scale, -127 * scale, -67 * scale, -101 * scale)
            left_ear.closeSubpath()
            right_ear = QPainterPath()
            right_ear.moveTo(145 * scale, -63 * scale)
            right_ear.cubicTo(
                184 * scale, -102 * scale,
                164 * scale, -178 * scale - ear_shift - right_ear_wiggle,
                126 * scale, -190 * scale - ear_shift - right_ear_wiggle,
            )
            right_ear.cubicTo(93 * scale, -168 * scale, 75 * scale, -127 * scale, 67 * scale, -101 * scale)
            right_ear.closeSubpath()
            painter.setPen(QPen(outline, max(2.0, 4.0 * scale), Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            painter.setBrush(fur)
            painter.drawPath(left_ear)
            painter.drawPath(right_ear)

            painter.setPen(Qt.NoPen)
            painter.setBrush(inner_ear)
            painter.drawEllipse(QRectF(
                -145 * scale,
                -161 * scale - ear_shift * 0.65 - left_ear_wiggle * 0.65,
                38 * scale,
                76 * scale,
            ))
            painter.drawEllipse(QRectF(
                107 * scale,
                -161 * scale - ear_shift * 0.65 - right_ear_wiggle * 0.65,
                38 * scale,
                76 * scale,
            ))

            # Subtle shadow and a solid, rounded caramel face.
            painter.setBrush(QColor(89, 59, 45, 24))
            painter.drawEllipse(QRectF(-151 * scale, 115 * scale, 302 * scale, 39 * scale))
            painter.setPen(QPen(outline, max(2.0, 4.0 * scale), Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            painter.setBrush(fur)
            painter.drawRoundedRect(head, 67 * scale, 67 * scale)

            # A small forehead patch adds warmth and character.
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor("#efbd86"))
            forehead = QPainterPath()
            forehead.moveTo(-30 * scale, -107 * scale)
            forehead.cubicTo(-17 * scale, -71 * scale, 17 * scale, -71 * scale, 30 * scale, -107 * scale)
            forehead.closeSubpath()
            painter.drawPath(forehead)

            eye_open = profile["eyes"]
            if self._blink_frames > 0:
                eye_open *= 0.06
            if self._category == "sleep":
                eye_open = 0.04
            eye_y = -24 * scale
            gaze_scale = 1.55 if self._category in ("curiosity", "explore") else 1.0
            pupil_shift = math.sin(self._phase * 0.92) * 7.0 * scale * gaze_scale
            pupil_lift = math.sin(self._phase * 0.67 + 1.2) * 3.0 * scale * gaze_scale
            if self._category in ("anxiety", "fear"):
                pupil_shift += math.sin(self._phase * 8.0) * 1.8 * scale

            for eye_x in (-82 * scale, 82 * scale):
                eye_w = 72 * scale
                eye_h = max(4.0 * scale, 54 * scale * eye_open)
                if eye_open <= 0.12:
                    closed_eye = QPainterPath()
                    closed_eye.moveTo((eye_x / scale - 31) * scale, eye_y)
                    closed_eye.cubicTo(
                        (eye_x / scale - 15) * scale, (eye_y / scale + 12) * scale,
                        (eye_x / scale + 15) * scale, (eye_y / scale + 12) * scale,
                        (eye_x / scale + 31) * scale, eye_y,
                    )
                    painter.setBrush(Qt.NoBrush)
                    painter.setPen(QPen(outline, max(3.0, 5.0 * scale), Qt.SolidLine, Qt.RoundCap))
                    painter.drawPath(closed_eye)
                    continue

                eye_rect = QRectF(eye_x - eye_w / 2, eye_y - eye_h / 2, eye_w, eye_h)
                painter.setPen(QPen(outline, max(2.0, 3.5 * scale)))
                painter.setBrush(QColor("#fffaf3"))
                painter.drawRoundedRect(eye_rect, eye_h / 2, eye_h / 2)
                pupil_radius = 14.0 * scale * (1.18 if self._category == "fear" else 1.0)
                painter.setBrush(QColor("#533a30"))
                painter.setPen(Qt.NoPen)
                pupil_center = QPointF(
                    eye_x + pupil_shift,
                    eye_y + 2 * scale + pupil_lift,
                )
                painter.drawEllipse(pupil_center, pupil_radius, pupil_radius)
                painter.setBrush(QColor("#fffdf8"))
                painter.drawEllipse(
                    QPointF(pupil_center.x() - 4 * scale, pupil_center.y() - 5 * scale),
                    4.5 * scale,
                    4.5 * scale,
                )

            # Eyebrows carry anxious and frightened expressions more clearly.
            if self._category in ("anxiety", "fear"):
                painter.setPen(QPen(outline, max(2.0, 4.0 * scale), Qt.SolidLine, Qt.RoundCap))
                painter.drawLine(-112 * scale, -62 * scale, -57 * scale, -50 * scale)
                painter.drawLine(112 * scale, -62 * scale, 57 * scale, -50 * scale)

            # Muzzle, nose and expressive mouth curve.
            painter.setPen(Qt.NoPen)
            painter.setBrush(self._color(accent_hex, 58))
            painter.drawEllipse(QPointF(-130 * scale, 51 * scale), 24 * scale, 13 * scale)
            painter.drawEllipse(QPointF(130 * scale, 51 * scale), 24 * scale, 13 * scale)

            painter.setPen(QPen(QColor("#b87955"), max(1.0, 2.0 * scale)))
            painter.setBrush(muzzle_color)
            painter.drawRoundedRect(QRectF(-82 * scale, 27 * scale, 164 * scale, 91 * scale), 38 * scale, 38 * scale)
            painter.setBrush(QColor("#5a3d32"))
            painter.setPen(Qt.NoPen)
            nose_bounce = math.sin(self._phase * 2.4) * 1.4 * scale
            painter.drawRoundedRect(
                QRectF(-22 * scale, 36 * scale + nose_bounce, 44 * scale, 25 * scale),
                12 * scale,
                12 * scale,
            )
            painter.setBrush(QColor(255, 255, 255, 125))
            painter.drawEllipse(
                QPointF(-8 * scale, 42 * scale + nose_bounce),
                4.5 * scale,
                2.7 * scale,
            )

            mouth = QPainterPath()
            mouth_motion = 0.0
            if self._category in ("joy", "excitement", "social", "eat"):
                mouth_motion = math.sin(self._phase * 3.0) * 4.0
            mouth.moveTo(-52 * scale, (78 + mouth_motion) * scale)
            mouth.cubicTo(
                -24 * scale,
                (78 + profile["smile"] * 34 + mouth_motion * 0.3) * scale,
                24 * scale,
                (78 + profile["smile"] * 34 + mouth_motion * 0.3) * scale,
                52 * scale,
                (78 + mouth_motion) * scale,
            )
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(outline, max(2.0, 4.0 * scale), Qt.SolidLine, Qt.RoundCap))
            painter.drawPath(mouth)

            if self._category == "eat":
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor("#ff7f9f"))
                tongue_y = (91 + math.sin(self._phase * 4.0) * 5.0) * scale
                painter.drawRoundedRect(QRectF(-17 * scale, tongue_y, 34 * scale, 27 * scale), 13 * scale, 13 * scale)

            if self._category == "charge":
                bolt = QPainterPath()
                bolt.moveTo(8 * scale, -57 * scale)
                bolt.lineTo(-10 * scale, -27 * scale)
                bolt.lineTo(4 * scale, -27 * scale)
                bolt.lineTo(-7 * scale, 0 * scale)
                bolt.lineTo(22 * scale, -36 * scale)
                bolt.lineTo(7 * scale, -36 * scale)
                bolt.closeSubpath()
                painter.setBrush(accent)
                painter.setPen(Qt.NoPen)
                painter.drawPath(bolt)

            if self._category == "sleep":
                painter.setFont(QFont("Sans", max(12, int(20 * scale)), QFont.Bold))
                painter.setPen(accent.darker(120))
                z_offset = math.sin(self._phase) * 5.0 * scale
                painter.drawText(QPointF(150 * scale, -105 * scale + z_offset), "Z")
                painter.drawText(QPointF(178 * scale, -142 * scale + z_offset), "z")

            if self._category in ("joy", "excitement", "social"):
                heart_speed = 0.8 if self._category == "joy" else 1.25
                for index, side in enumerate((-1.0, 1.0, -1.0)):
                    rise = (self._phase * 18.0 * heart_speed + index * 46.0) % 135.0
                    heart_x = side * (190.0 + index * 14.0) * scale
                    heart_y = (70.0 - rise) * scale
                    heart_alpha = int(210 - rise * 0.9)
                    self._draw_heart(
                        painter,
                        QPointF(heart_x, heart_y),
                        (10.0 + index * 2.0) * scale,
                        QColor(241, 111, 132, max(65, heart_alpha)),
                    )

            if self._category in ("anxiety", "fear"):
                sweat_sway = math.sin(self._phase * 3.2) * 5.0 * scale
                sweat = QPainterPath()
                sweat.moveTo(166 * scale + sweat_sway, -62 * scale)
                sweat.cubicTo(
                    145 * scale + sweat_sway, -32 * scale,
                    154 * scale + sweat_sway, -10 * scale,
                    169 * scale + sweat_sway, -10 * scale,
                )
                sweat.cubicTo(
                    187 * scale + sweat_sway, -10 * scale,
                    190 * scale + sweat_sway, -33 * scale,
                    166 * scale + sweat_sway, -62 * scale,
                )
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor(116, 184, 222, 205))
                painter.drawPath(sweat)

            # A colored collar and softly pulsing tag retain the virtual-pet cue.
            painter.setPen(QPen(outline, max(1.0, 2.5 * scale)))
            painter.setBrush(accent.lighter(115))
            painter.drawRoundedRect(QRectF(-104 * scale, 116 * scale, 208 * scale, 25 * scale), 11 * scale, 11 * scale)
            tag_radius = (15.0 + math.sin(self._phase * 2.0) * 1.5) * scale
            painter.setBrush(QColor("#fff5dd"))
            painter.drawEllipse(QPointF(0, 143 * scale), tag_radius, tag_radius)
            painter.setPen(Qt.NoPen)
            painter.setBrush(accent.darker(108))
            painter.drawEllipse(QPointF(0, 143 * scale), 5 * scale, 5 * scale)

            painter.restore()


class EmotionWindow(QWidget):
    """Emotion visualization Qt window.

    Receives events from the ROS2 thread via a thread-safe queue.
    A Qt timer drains the queue and updates the UI.
    """

    def __init__(self, event_queue: queue.Queue | None = None) -> None:
        if not HAS_QT:
            raise RuntimeError("PySide2 not installed")
        super().__init__()
        self._queue = event_queue or queue.Queue()
        self._display_generation = 0
        self._active_goal_id = ""
        self._active_behavior_name = ""
        self._setup_ui()
        self._setup_timer()

    def _setup_ui(self) -> None:
        self.setWindowTitle("MarsDog Pet Display")
        self.setMinimumSize(640, 520)

        pal = self.palette()
        pal.setColor(QPalette.Window, QColor(WINDOW_BG))
        self.setPalette(pal)
        self.setAutoFillBackground(True)

        screen = QApplication.primaryScreen().availableGeometry()
        w = min(screen.width(), max(720, int(screen.width() * 0.68)))
        h = min(screen.height(), max(560, int(screen.height() * 0.72)))
        self.setGeometry(
            screen.x() + (screen.width() - w) // 2,
            screen.y() + (screen.height() - h) // 2,
            w, h,
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 18)
        layout.setSpacing(8)

        # --- Animated virtual-pet face ---
        self._face = DigitalDogFace(self)
        self._face.setMinimumHeight(max(300, int(h * 0.54)))
        layout.addWidget(self._face, 1)

        # --- Emotion label ---
        self._emotion_label = QLabel(_label("unknown"))
        self._emotion_label.setAlignment(Qt.AlignCenter)
        self._emotion_label.setStyleSheet(
            "color: #62483b; background: transparent; font-weight: bold; "
            "letter-spacing: 2px;"
        )
        self._emotion_label.setFont(QFont("Sans", max(28, h // 18)))
        layout.addWidget(self._emotion_label)

        # --- Behavior name ---
        self._behavior_label = QLabel("Waiting...")
        self._behavior_label.setAlignment(Qt.AlignCenter)
        self._behavior_label.setStyleSheet(
            "color: #725c50; background: transparent;"
        )
        self._behavior_label.setFont(QFont("Sans", max(20, h // 25)))
        layout.addWidget(self._behavior_label)

        # --- Current action ---
        self._action_label = QLabel("")
        self._action_label.setAlignment(Qt.AlignCenter)
        self._action_label.setStyleSheet(
            "color: #795e50; background: #fffaf5; "
            "border: 1px solid #ead9ca; border-radius: 10px; padding: 6px;"
        )
        self._action_label.setFont(QFont("Monospace", max(16, h // 30)))
        layout.addWidget(self._action_label)

        layout.addSpacing(10)

        # --- Progress bar ---
        self._progress = QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        self._progress.setTextVisible(True)
        self._progress.setStyleSheet("""
            QProgressBar {
                color: #6f5548; border: 1px solid #e1cbbb; border-radius: 9px;
                background: #fffaf5; height: 20px; text-align: center;
            }
            QProgressBar::chunk { background: #efb08d; border-radius: 8px; }
        """)
        layout.addWidget(self._progress)

        # --- Info bar ---
        info_layout = QHBoxLayout()

        self._status_label = QLabel("Ready")
        self._status_label.setStyleSheet(
            "color: #568465; background: #edf6eb; "
            "border-radius: 10px; padding: 5px 10px;"
        )
        self._status_label.setFont(QFont("Sans", max(14, h // 35)))
        info_layout.addWidget(self._status_label)

        info_layout.addStretch()
        self._extra_label = QLabel("")
        self._extra_label.setStyleSheet("color: #92786a; background: transparent;")
        self._extra_label.setFont(QFont("Sans", max(14, h // 35)))
        info_layout.addWidget(self._extra_label)

        info_layout.addStretch()
        self._gid_label = QLabel("")
        self._gid_label.setStyleSheet("color: #b09b8f; background: transparent;")
        self._gid_label.setFont(QFont("Sans", max(12, h // 35)))
        info_layout.addWidget(self._gid_label)

        layout.addLayout(info_layout)

        # --- Message ---
        self._msg_label = QLabel("")
        self._msg_label.setAlignment(Qt.AlignCenter)
        self._msg_label.setWordWrap(True)
        self._msg_label.setStyleSheet("color: #9a8174; background: transparent;")
        self._msg_label.setFont(QFont("Sans", max(14, h // 35)))
        layout.addWidget(self._msg_label)

        # Demo
        self._demo_mode = False
        self._demo_behaviors: list[str] = []
        self._demo_idx = 0
        self._show_visual("unknown", "")

    def _setup_timer(self) -> None:
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._drain_queue)
        self._timer.start(33)  # ~30 FPS

    def keyPressEvent(self, event) -> None:
        """Provide kiosk-friendly full-screen keyboard controls."""
        if event.key() == Qt.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
            event.accept()
            return
        if event.key() == Qt.Key_Escape and self.isFullScreen():
            self.showNormal()
            event.accept()
            return
        super().keyPressEvent(event)

    def _drain_queue(self) -> None:
        drained = 0
        while drained < 50:
            try:
                ev_type, data = self._queue.get_nowait()
            except queue.Empty:
                break
            self._apply(ev_type, data)
            drained += 1

    def _apply(self, ev_type: str, d: dict[str, Any]) -> None:
        if ev_type == _EV_GOAL:
            self._apply_goal(d)
        elif ev_type == _EV_FEEDBACK:
            self._apply_feedback(d)
        elif ev_type == _EV_RESULT:
            self._apply_result(d)

    def _apply_goal(self, d: dict[str, Any]) -> None:
        self._display_generation += 1
        name = str(d.get("behavior_name", "") or "")
        self._active_goal_id = str(d.get("goal_id", "") or "")
        self._active_behavior_name = name
        cat = _classify(name, d.get("params"))
        color = QColor(_color(cat)).darker(145).name()

        intensity = d.get("intensity")
        self._show_visual(cat, name, intensity)
        self._face.set_execution("RUNNING", 0.0, "")

        self._emotion_label.setStyleSheet(
            f"color: {color}; background: transparent; font-weight: bold; "
            "letter-spacing: 2px;"
        )
        self._behavior_label.setText(name)
        self._action_label.setText("")
        self._progress.setValue(0)
        self._status_label.setText("Running")
        self._status_label.setStyleSheet(
            "color: #b47932; background: #fff1d8; "
            "border-radius: 10px; padding: 5px 10px;"
        )
        self._gid_label.setText(d.get("goal_id", ""))
        self._msg_label.setText("")

        parts = []
        if intensity is not None:
            try:
                parts.append(f"Intensity {float(intensity):.0f}")
            except (TypeError, ValueError):
                pass
        level = d.get("level")
        if level:
            parts.append(f"Level {level}")
        self._extra_label.setText("  |  ".join(parts) if parts else "")

    def _show_visual(
        self,
        cat: str,
        behavior_name: str,
        intensity: float | None = None,
    ) -> None:
        """Switch the live virtual-pet face to a behavior category."""
        self._face.set_emotion(cat, behavior_name, intensity)
        if cat == "unknown" and behavior_name:
            self._emotion_label.setText(f"{_emoji(cat)} 执行中")
        else:
            self._emotion_label.setText(f"{_emoji(cat)} {_label(cat)}")

    def _apply_feedback(self, d: dict[str, Any]) -> None:
        event_goal_id = str(d.get("goal_id", "") or "")
        if (
            self._active_goal_id
            and event_goal_id
            and event_goal_id != self._active_goal_id
        ):
            return

        # The display may start after the volatile Goal event was published.
        # Recover the active visual state from Feedback, which also carries the
        # canonical behavior name and goal id.
        if not self._active_behavior_name:
            behavior_name = str(d.get("behavior_name", "") or "")
            if behavior_name:
                self._apply_goal(d)
            else:
                return
        elif not self._active_goal_id and event_goal_id:
            self._active_goal_id = event_goal_id

        stage = d.get("current_stage", "")
        act = d.get("current_action", "")
        progress = float(d.get("progress", 0.0))
        self._action_label.setText(f"{stage}  ->  {act}")
        self._progress.setValue(int(min(100.0, max(0.0, progress * 100.0))))
        self._face.set_execution("RUNNING", progress, stage)
        self._msg_label.setText(d.get("message", ""))

    def _apply_result(self, d: dict[str, Any]) -> None:
        event_goal_id = str(d.get("goal_id", "") or "")
        event_behavior = str(d.get("behavior_name", "") or "")

        # A preempted goal can report CANCELED after the replacement Goal has
        # already started. Never let that stale terminal event reset the new
        # behavior's visual state.
        if self._active_goal_id and event_goal_id:
            if event_goal_id != self._active_goal_id:
                return
        elif self._active_behavior_name and event_behavior:
            if event_behavior != self._active_behavior_name:
                return
        elif not self._active_behavior_name:
            return

        status = d.get("status", "")
        if status == "SUCCESS":
            self._status_label.setText("Completed")
            self._status_label.setStyleSheet(
                "color: #568465; background: #edf6eb; "
                "border-radius: 10px; padding: 5px 10px;"
            )
        elif status in ("CANCELED", "FAILED", "FAILURE", "TIMEOUT"):
            self._status_label.setText(status)
            self._status_label.setStyleSheet(
                "color: #b45454; background: #fde8e5; "
                "border-radius: 10px; padding: 5px 10px;"
            )
        else:
            self._status_label.setText(status)
        self._progress.setValue(100)
        self._face.set_execution(status, 1.0)
        self._msg_label.setText(d.get("reason", ""))
        generation = self._display_generation
        QTimer.singleShot(
            2000,
            lambda: self._reset_to_ready(generation),
        )

    def _reset_to_ready(self, generation: int) -> None:
        """Leave a terminal result visible briefly, then clear stale behavior."""
        if generation != self._display_generation:
            return
        self._active_goal_id = ""
        self._active_behavior_name = ""
        self._show_visual("unknown", "")
        self._emotion_label.setStyleSheet(
            f"color: {QColor(_color('unknown')).darker(145).name()}; background: transparent; "
            "font-weight: bold; letter-spacing: 2px;"
        )
        self._face.set_execution("READY", 0.0, "")
        self._behavior_label.setText("Waiting...")
        self._action_label.setText("")
        self._progress.setValue(0)
        self._status_label.setText("Ready")
        self._status_label.setStyleSheet(
            "color: #568465; background: #edf6eb; "
            "border-radius: 10px; padding: 5px 10px;"
        )
        self._gid_label.setText("")
        self._msg_label.setText("")
        self._extra_label.setText("")

    # --- Demo ---

    def set_demo_mode(self, behaviors: list[str]) -> None:
        self._demo_mode = True
        self._demo_behaviors = behaviors
        self._demo_idx = 0
        self._start_demo()

    def _start_demo(self) -> None:
        if not self._demo_mode:
            return
        if self._demo_behaviors:
            name = self._demo_behaviors[self._demo_idx % len(self._demo_behaviors)]
            self._demo_idx += 1
            self._apply_goal({"behavior_name": name, "intensity": random.uniform(20, 90)})
            self._apply_feedback({
                "current_stage": "expression",
                "current_action": f"ACT_{_classify(name).upper()}_DEMO",
                "progress": random.uniform(0.3, 0.9),
                "message": f"Demo: {name}",
            })
        QTimer.singleShot(random.randint(2500, 5000), self._start_demo)


# --- ROS2 node ---

if HAS_ROS2:

    class EmotionDisplayNode(Node):
        """Subscribe to debug topics, push events to shared queue."""

        _GOAL_TOPIC = "/debug/execute_behavior/goal"
        _FB_TOPIC = "/debug/execute_behavior/feedback"
        _RESULT_TOPIC = "/debug/execute_behavior/result"

        def __init__(self, event_queue: queue.Queue) -> None:
            super().__init__("emotion_display_node")
            self._queue = event_queue
            self._goal_sub = self.create_subscription(
                String, self._GOAL_TOPIC, self._on_goal, 10,
            )
            self._fb_sub = self.create_subscription(
                String, self._FB_TOPIC, self._on_feedback, 10,
            )
            self._result_sub = self.create_subscription(
                String, self._RESULT_TOPIC, self._on_result, 10,
            )
            self.get_logger().info(
                f"Subscribed to {self._GOAL_TOPIC}, "
                f"{self._FB_TOPIC}, {self._RESULT_TOPIC}"
            )

        def _on_goal(self, msg: String) -> None:
            try:
                d: dict[str, Any] = json.loads(msg.data)
            except (json.JSONDecodeError, TypeError):
                return
            params = d.get("params", {})
            if isinstance(params, str):
                try:
                    params = json.loads(params)
                except (json.JSONDecodeError, TypeError):
                    params = {}
            if isinstance(params, dict):
                d["intensity"] = params.get("intensity")
                d["level"] = params.get("level")
            self._queue.put((_EV_GOAL, d))

        def _on_feedback(self, msg: String) -> None:
            try:
                d: dict[str, Any] = json.loads(msg.data)
                self._queue.put((_EV_FEEDBACK, d))
            except (json.JSONDecodeError, TypeError):
                pass

        def _on_result(self, msg: String) -> None:
            try:
                d: dict[str, Any] = json.loads(msg.data)
                self._queue.put((_EV_RESULT, d))
            except (json.JSONDecodeError, TypeError):
                pass


# --- Entry point ---

_DEMO_BEHAVIORS = [
    "expressJoyWithHuman", "expressJoyAlone", "expressExcitementAlone",
    "expressCalmAlone", "expressCuriosityAlone", "eatNormally",
    "sleepOnSide", "expressAnxietyAlone", "expressFearAlone",
    "seekHumanInteraction", "exploreRoom", "sit_down", "lie_down",
    "expressExcitementWithHuman", "restInPlace",
]


def main(args: list[str] | None = None) -> None:
    logging.basicConfig(level=logging.INFO)

    if not HAS_QT:
        raise RuntimeError(
            "PySide2 is required: sudo apt install python3-pyside2.qtwidgets"
        )

    app = QApplication(sys.argv)
    event_queue: queue.Queue = queue.Queue()

    if HAS_ROS2 and os.environ.get("ROS_DOMAIN_ID", "").strip():
        rclpy.init(args=args)
        ros_node = EmotionDisplayNode(event_queue)
        executor = SingleThreadedExecutor()
        executor.add_node(ros_node)

        window = EmotionWindow(event_queue=event_queue)
        window.showFullScreen()

        _stop_ros = threading.Event()

        def _ros_loop() -> None:
            while not _stop_ros.is_set():
                try:
                    executor.spin_once(timeout_sec=0.05)
                except Exception:
                    pass

        ros_thread = threading.Thread(target=_ros_loop, daemon=True, name="ros-loop")
        ros_thread.start()

        try:
            app.exec_()
        finally:
            _stop_ros.set()
            ros_thread.join(timeout=2.0)
            executor.remove_node(ros_node)
            ros_node.destroy_node()
            rclpy.try_shutdown()
    else:
        print("No ROS2 -- running demo mode")
        window = EmotionWindow(event_queue=event_queue)
        window.showFullScreen()
        window.set_demo_mode(_DEMO_BEHAVIORS)
        app.exec_()


if __name__ == "__main__":
    main()
