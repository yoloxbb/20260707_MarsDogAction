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
from dataclasses import dataclass
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

# --- Qt bindings ---

try:
    from PySide2.QtCore import QPointF, QRectF, Qt, QTimer, QPropertyAnimation
    from PySide2.QtGui import (
        QBrush,
        QColor,
        QFont,
        QLinearGradient,
        QPainter,
        QPainterPath,
        QPalette,
        QPen,
        QRadialGradient,
    )
    from PySide2.QtWidgets import (
        QApplication, QLabel, QProgressBar, QGraphicsOpacityEffect,
        QVBoxLayout, QSizePolicy, QWidget,
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
    "fall_alert": {"emoji": "\u26a0", "label": "人员跌倒", "color": "#FF7A00", "image": "急停.png"},
    "stop_alert": {"emoji": "\u270b", "label": "停止手势", "color": "#FF3B30", "image": "急停.png"},
    "emergency":  {"emoji": "\U0001f6d1", "label": "紧急停车", "color": "#C90000", "image": "急停.png"},
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
    ("unhappy", "anxiety"),
    ("miss_owner", "excitement"),
    ("farewell_leave", "social"),
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
    ("play_alone", "command"),
    ("walk_to_random_point", "command"),
    ("go_out_to_play", "command"),
    ("go_home", "command"),
    ("approach_owner", "command"),
    ("back_up", "command"),
    ("stand_still", "command"),
    ("hold_position", "command"),
    ("quiet", "command"),
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
    ("approach_voice_caller", "command"),
    ("respond_person_fall", "fall_alert"),
    ("respond_stop_gesture", "stop_alert"),
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


# --- Presentation state -----------------------------------------------------

_EMOTION_CATEGORY_TO_STATE = {
    "calm": "calm",
    "joy": "joy",
    "excitement": "excited",
    "curiosity": "curious",
    "anxiety": "anxious",
    "fear": "fear",
}

_STATE_EMOTION_TO_CATEGORY = {
    value: key for key, value in _EMOTION_CATEGORY_TO_STATE.items()
}

_ACTIVITY_CATEGORY_TO_STATE = {
    "eat": "eating",
    "sleep": "resting",
    "social": "social",
    "explore": "exploring",
    "clean": "cleaning",
    "toilet": "elimination",
    "charge": "charging",
}

_STATE_ACTIVITY_TO_CATEGORY = {
    "eating": "eat",
    "resting": "sleep",
    "social": "social",
    "exploring": "explore",
    "cleaning": "clean",
    "elimination": "toilet",
    "charging": "charge",
    "following": "command",
    "listening": "command",
    "searching": "explore",
}


@dataclass(frozen=True)
class FaceState:
    """UI-facing state, deliberately separated from legacy visual categories."""

    emotion: str = "calm"
    activity: str = "idle"
    system_state: str = "ready"
    intensity: float = 50.0
    alert_type: str = "none"


@dataclass(frozen=True)
class PresentationState:
    """Friendly text and animation hints derived from Action debug fields."""

    title: str = "在等你"
    subtitle: str = ""
    face_cue: str = "idle"
    animation_cue: str = "gentle"
    show_progress: bool = False


def _clamp_intensity(value: Any) -> float:
    try:
        return min(100.0, max(0.0, float(value)))
    except (TypeError, ValueError):
        return 50.0


def _activity_for(
    category: str,
    behavior_name: str,
    stage: str = "",
    action: str = "",
) -> str:
    """Resolve an activity without changing the legacy category contract."""
    name = str(behavior_name or "").casefold()
    stage_key = str(stage or "").casefold()
    action_key = str(action or "").casefold()
    combined_action = f"{stage_key} {action_key}"

    if (
        "follow_owner" in name
        or "farewell_leave" in name
        or "follow" in action_key
    ):
        return "following"
    if any(token in combined_action for token in ("search", "scan", "look_for")):
        return "searching"
    if (
        "respond_owner_call" in name
        or "approach_voice_caller" in name
        or any(token in combined_action for token in ("listen", "wait", "respond_call"))
    ):
        return "listening"
    return _ACTIVITY_CATEGORY_TO_STATE.get(category, "idle")


def _system_state_for(category: str, status: str) -> str:
    status_key = str(status or "READY").upper()
    if status_key == "EMERGENCY" or (
        category in {"fall_alert", "stop_alert", "emergency"}
        and status_key == "RUNNING"
    ):
        return "emergency"
    if status_key == "RUNNING":
        return "running"
    if status_key == "SUCCESS":
        return "completed"
    if status_key in ("CANCELED", "CANCELLED"):
        return "cancelled"
    if status_key in ("FAILED", "FAILURE", "TIMEOUT"):
        return "failed"
    return "ready"


def _alert_type_for(category: str) -> str:
    if category in {"fall_alert", "stop_alert", "emergency"}:
        return category
    return "none"


def build_face_state(
    category: str,
    behavior_name: str = "",
    stage: str = "",
    action: str = "",
    status: str = "READY",
    intensity: Any = 50.0,
) -> FaceState:
    """Convert legacy behavior/category fields into orthogonal UI state."""
    return FaceState(
        emotion=_EMOTION_CATEGORY_TO_STATE.get(category, "calm"),
        activity=_activity_for(category, behavior_name, stage, action),
        system_state=_system_state_for(category, status),
        intensity=_clamp_intensity(intensity),
        alert_type=_alert_type_for(category),
    )


def _visual_category(state: FaceState) -> str:
    """Map the split state back to the established face profile vocabulary."""
    if state.system_state == "emergency":
        return (
            state.alert_type
            if state.alert_type in {"fall_alert", "stop_alert", "emergency"}
            else "emergency"
        )
    if state.system_state == "ready" and state.activity == "idle":
        return "unknown"
    if state.emotion != "calm":
        return _STATE_EMOTION_TO_CATEGORY.get(state.emotion, "unknown")
    if state.activity != "idle":
        return _STATE_ACTIVITY_TO_CATEGORY.get(state.activity, "unknown")
    return _STATE_EMOTION_TO_CATEGORY.get(state.emotion, "unknown")


def build_presentation_state(
    face_state: FaceState,
    behavior_name: str = "",
    stage: str = "",
    action: str = "",
    progress: float = 0.0,
) -> PresentationState:
    """Build user-facing copy and face cues in one testable mapping layer."""
    name = str(behavior_name or "").casefold()
    stage_key = str(stage or "").casefold()
    action_key = str(action or "").casefold()
    combined_action = f"{stage_key} {action_key}"
    state = face_state.system_state

    if state == "emergency":
        if face_state.alert_type == "fall_alert":
            return PresentationState(
                "检测到有人跌倒",
                "小车已停车，正在关注",
                "fall_alert",
                "urgent",
                False,
            )
        if face_state.alert_type == "stop_alert":
            return PresentationState(
                "检测到停止手势",
                "小车已停止",
                "stop_alert",
                "urgent",
                False,
            )
        return PresentationState(
            "紧急停车",
            "请检查周围环境",
            "emergency",
            "urgent",
            False,
        )
    if state == "completed":
        return PresentationState("做到了", "", "success", "celebrate", False)
    if state == "failed":
        return PresentationState("有点困惑", "稍后再试试", "failed", "soft", False)
    if state == "cancelled":
        return PresentationState("停下来啦", "", "failed", "soft", False)
    if state == "ready":
        return PresentationState()

    if "follow_owner" in name:
        return PresentationState("正在跟着你", "", "follow", "focused", 0.0 < progress < 1.0)
    if "respond_owner_call" in name or "approach_voice_caller" in name:
        return PresentationState("我来啦", "", "call", "responsive", 0.0 < progress < 1.0)
    if "seekinteraction" in name or "invite" in name:
        return PresentationState("想和你玩", "", "social", "playful", 0.0 < progress < 1.0)
    if any(token in combined_action for token in ("search", "scan", "look_for")):
        return PresentationState("我看看……", "", "search", "scanning", 0.0 < progress < 1.0)
    if any(token in combined_action for token in ("listen", "wait", "respond_call")):
        return PresentationState("在听", "", "listening", "attentive", 0.0 < progress < 1.0)

    behavior_copy = (
        (("unhappy",), ("安静陪着你", "anxious", "soft")),
        (("miss_owner",), ("好想你", "social", "playful")),
        (("farewell_leave",), ("送送你", "follow", "focused")),
        (("sit_down",), ("坐下来", "listening", "attentive")),
        (("lie_down", "play_dead"), ("趴下来", "idle", "gentle")),
        (("stand_up",), ("站起来", "listening", "attentive")),
        (("give_paw",), ("握个手", "social", "playful")),
        (("high_five",), ("击个掌", "social", "playful")),
        (("spin_around", "roll_over"), ("转个圈", "social", "playful")),
        (("fetch_object", "bring_object", "return_to_owner"), ("去拿回来", "search", "focused")),
    )
    for names, presentation_values in behavior_copy:
        if any(candidate in name for candidate in names):
            title, cue, animation = presentation_values
            return PresentationState(
                title, "", cue, animation, 0.0 < progress < 1.0
            )

    activity_copy = {
        "charging": ("充电中", "charge", "gentle"),
        "resting": ("休息一下", "sleep", "gentle"),
        "exploring": ("我看看……", "search", "scanning"),
        "eating": ("吃饭中", "eat", "playful"),
        "cleaning": ("整理一下", "clean", "gentle"),
        "elimination": ("稍等一下", "idle", "gentle"),
        "social": ("陪你玩", "social", "playful"),
        "following": ("正在跟着你", "follow", "focused"),
        "listening": ("在听", "listening", "attentive"),
        "searching": ("我看看……", "search", "scanning"),
    }
    if face_state.activity in activity_copy:
        title, cue, animation = activity_copy[face_state.activity]
        return PresentationState(title, "", cue, animation, 0.0 < progress < 1.0)

    emotion_copy = {
        "joy": ("好开心", "social", "playful"),
        "excited": ("好期待", "social", "playful"),
        "curious": ("有点好奇", "search", "scanning"),
        "anxious": ("有点不安", "anxious", "soft"),
        "fear": ("有点害怕", "anxious", "soft"),
        "calm": ("陪着你", "idle", "gentle"),
    }
    title, cue, animation = emotion_copy.get(
        face_state.emotion, ("", "idle", "gentle")
    )
    return PresentationState(title, "", cue, animation, 0.0 < progress < 1.0)


# --- Event type constants ---

_EV_GOAL = "goal"
_EV_FEEDBACK = "feedback"
_EV_RESULT = "result"

# Safety Actions intentionally finish in a few milliseconds after publishing
# zero Twist. Keep their semantic warning visible independently of that short
# controller lifetime; a new Goal still supersedes the hold immediately.
_SAFETY_ALERT_HOLD_MS = {
    "fall_alert": 4000,
    "stop_alert": 2500,
    "emergency": 3000,
}


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
    "fall_alert": {"eyes": 0.96, "smile": -0.58, "ears": 10.0, "bob": 0.0, "shake": 2.4, "tilt": 5.0, "speed": 1.6},
    "stop_alert": {"eyes": 1.00, "smile": -0.62, "ears": 16.0, "bob": 0.0, "shake": 3.4, "tilt": 0.0, "speed": 2.0},
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
            self._face_state = FaceState()
            self._presentation = PresentationState()
            self._visual_profile = _face_profile("unknown")
            self._target_profile = dict(self._visual_profile)
            self._accent_rgb = [176.0, 176.0, 176.0]
            self._target_accent_rgb = list(self._accent_rgb)
            self._emergency_mix = 0.0
            self._cue_started_frame = 0
            self._gaze_x = 0.0
            self._gaze_y = 0.0
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
            value = 50.0 if intensity is None else intensity
            face_state = build_face_state(
                category,
                behavior_name=behavior_name,
                stage=self._stage,
                status=self._status,
                intensity=value,
            )
            presentation = build_presentation_state(
                face_state,
                behavior_name=behavior_name,
                stage=self._stage,
                progress=self._progress,
            )
            self.set_display_state(face_state, presentation, behavior_name)

        def set_display_state(
            self,
            face_state: FaceState,
            presentation: PresentationState,
            behavior_name: str = "",
        ) -> None:
            """Set semantic targets; the frame timer interpolates toward them."""
            previous_cue = self._presentation.face_cue
            self._face_state = face_state
            self._presentation = presentation
            self._category = _visual_category(face_state)
            self._behavior_name = behavior_name
            self._status = face_state.system_state.upper()
            self._intensity = _clamp_intensity(face_state.intensity)
            self._target_profile = self._build_animation_targets()
            target = QColor(_color(self._category))
            self._target_accent_rgb = [
                float(target.red()), float(target.green()), float(target.blue())
            ]
            if previous_cue != presentation.face_cue:
                self._cue_started_frame = self._frame
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

        def _build_animation_targets(self) -> dict[str, float]:
            """Apply bounded, category-specific intensity and action cues."""
            profile = _face_profile(self._category)
            amount = self._intensity / 100.0
            eased = amount * amount * (3.0 - 2.0 * amount)

            # Motion grows within conservative bounds; facial deformation below
            # remains category-specific instead of scaling every field equally.
            amplitude = 0.72 + 0.58 * eased
            profile["bob"] *= amplitude
            profile["shake"] *= 0.55 + 0.90 * eased
            profile["speed"] *= 0.72 + 0.62 * eased
            profile["ears"] *= 0.82 + 0.34 * eased

            if self._category == "joy":
                # The reference face keeps its large, glossy eyes open while
                # smiling; only the very highest intensity becomes a squint.
                profile["eyes"] = 0.96 - 0.30 * eased
                profile["smile"] = 0.38 + 0.62 * eased
            elif self._category == "excitement":
                profile["eyes"] = 0.82 + 0.18 * eased
                profile["smile"] = 0.52 + 0.42 * eased
            elif self._category in ("anxiety", "fear"):
                profile["eyes"] = 0.98 - 0.25 * eased
                profile["smile"] *= 0.65 + 0.45 * eased
            elif self._category == "curiosity":
                profile["eyes"] = 0.82 + 0.18 * eased

            cue = self._presentation.face_cue
            if cue == "follow":
                profile.update(eyes=0.86, smile=0.22, ears=15.0, tilt=2.0)
                profile["bob"] = min(profile["bob"], 3.5)
            elif cue == "call":
                profile.update(eyes=1.0, smile=0.56, ears=18.0)
                profile["speed"] = max(profile["speed"], 1.35)
            elif cue == "search":
                profile.update(eyes=0.96, smile=0.18, ears=13.0, tilt=7.0)
            elif cue == "listening":
                profile.update(eyes=0.78, smile=0.12, ears=17.0, bob=1.2, shake=0.0)
            elif cue == "success":
                profile.update(eyes=0.38, smile=1.0, ears=14.0, bob=8.0, shake=0.0)
            elif cue == "failed":
                profile.update(eyes=0.54, smile=-0.28, ears=-7.0, bob=1.0, shake=0.0, tilt=8.0)
            elif cue == "fall_alert":
                profile.update(
                    eyes=0.98,
                    smile=-0.62,
                    ears=9.0,
                    bob=0.0,
                    tilt=6.0,
                )
                profile["shake"] = 2.2 + eased * 1.8
                profile["speed"] = 1.45 + eased * 0.55
            elif cue == "stop_alert":
                profile.update(eyes=1.0, smile=-0.66, ears=17.0, bob=0.0)
                profile["shake"] = 3.8 + eased * 2.2
                profile["speed"] = 1.7 + eased * 0.7
            elif cue == "emergency":
                profile.update(eyes=1.0, smile=-0.72, ears=17.0, bob=0.0)
                profile["shake"] = 4.5 + eased * 3.0
                profile["speed"] = 1.8 + eased * 0.8
            return profile

        def _tick(self) -> None:
            # 0.12 at 30 FPS settles most facial transitions in about 300 ms.
            for key, target in self._target_profile.items():
                current = self._visual_profile.get(key, target)
                self._visual_profile[key] = current + (target - current) * 0.12
            for index, target in enumerate(self._target_accent_rgb):
                self._accent_rgb[index] += (target - self._accent_rgb[index]) * 0.10
            emergency_target = (
                1.0 if self._face_state.system_state == "emergency" else 0.0
            )
            self._emergency_mix += (
                emergency_target - self._emergency_mix
            ) * 0.12

            self._phase += 0.055 * self._visual_profile["speed"]
            self._frame += 1
            gaze_target_x, gaze_target_y = self._gaze_target()
            self._gaze_x += (gaze_target_x - self._gaze_x) * 0.12
            self._gaze_y += (gaze_target_y - self._gaze_y) * 0.12
            if self._blink_frames > 0:
                self._blink_frames -= 1
            elif self._category != "sleep" and self._frame >= self._next_blink:
                self._blink_frames = 6
                self._next_blink = self._frame + random.randint(70, 170)
            self.update()

        def _gaze_target(self) -> tuple[float, float]:
            """Return a restrained gaze target in face-coordinate pixels."""
            cue = self._presentation.face_cue
            if cue == "search":
                return (
                    math.sin(self._phase * 2.0) * 8.0,
                    math.sin(self._phase * 0.9) * 2.0,
                )
            if cue == "follow":
                return (math.sin(self._phase * 0.52) * 6.5, -1.0)
            if cue == "listening":
                return (0.0, -1.5)
            if self._category in ("anxiety", "fear"):
                return (
                    math.sin(self._phase * 4.5) * 5.0,
                    math.sin(self._phase * 2.8) * 1.8,
                )
            gaze_scale = 1.35 if self._category in ("curiosity", "explore") else 1.0
            return (
                math.sin(self._phase * 0.72) * 3.7 * gaze_scale,
                math.sin(self._phase * 0.51 + 1.2) * 1.4 * gaze_scale,
            )

        def _blink_cover(self, side: float) -> float:
            """Stagger eyelid closure by one frame instead of flattening eyes."""
            if self._category == "sleep":
                return 0.94
            if self._blink_frames <= 0:
                return 0.0
            sequence = (0.0, 0.20, 0.62, 0.96, 0.72, 0.30, 0.0)
            index = max(0, min(6, 6 - self._blink_frames))
            if side > 0:
                index = max(0, index - 1)
            return sequence[index]

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

        def _current_accent(self) -> QColor:
            return QColor(
                int(self._accent_rgb[0]),
                int(self._accent_rgb[1]),
                int(self._accent_rgb[2]),
            )

        def _draw_background(
            self,
            painter: QPainter,
            width: int,
            height: int,
            accent: QColor,
        ) -> None:
            """Draw ambient warmth or the dedicated emergency pulse."""
            emergency_mix = min(1.0, max(0.0, self._emergency_mix))
            pulse = 0.5 + 0.5 * math.sin(self._phase * 4.2)
            pastel = accent.lighter(178)
            normal_colors = (
                QColor("#fffaf5"), pastel.lighter(120), QColor("#f7eee4")
            )
            emergency_colors = (
                QColor("#fff8f3"),
                QColor(255, int(225 - 28 * pulse), int(216 - 22 * pulse)),
                QColor("#f5ddd4"),
            )

            def blended(normal: QColor, emergency: QColor) -> QColor:
                return QColor(
                    int(normal.red() + (emergency.red() - normal.red()) * emergency_mix),
                    int(normal.green() + (emergency.green() - normal.green()) * emergency_mix),
                    int(normal.blue() + (emergency.blue() - normal.blue()) * emergency_mix),
                )

            background = QLinearGradient(0, 0, width, height)
            background.setColorAt(0.0, blended(normal_colors[0], emergency_colors[0]))
            background.setColorAt(0.58, blended(normal_colors[1], emergency_colors[1]))
            background.setColorAt(1.0, blended(normal_colors[2], emergency_colors[2]))
            painter.fillRect(self.rect(), QBrush(background))

            # Intensity changes density in bounded steps, avoiding visual noise.
            bubble_count = 5 + int(round(self._intensity / 25.0))
            painter.setPen(Qt.NoPen)
            for index in range(bubble_count):
                x_ratio = 0.08 + ((index * 37) % 84) / 100.0
                y_ratio = 0.10 + ((index * 29) % 72) / 100.0
                drift = math.sin(self._phase * (0.35 + index * 0.025) + index) * 9.0
                lift = math.cos(self._phase * (0.28 + index * 0.018) + index) * 7.0
                radius = 5.0 + (index % 4) * 3.0
                bubble = QColor(accent)
                bubble.setAlpha(int((20 + (index % 3) * 8) * (1.0 - emergency_mix)))
                painter.setBrush(bubble)
                painter.drawEllipse(
                    QPointF(width * x_ratio + drift, height * y_ratio + lift),
                    radius,
                    radius,
                )

            if emergency_mix > 0.01:
                painter.setBrush(Qt.NoBrush)
                alert_border = QColor(accent)
                alert_border.setAlpha(
                    int((115 + 105 * pulse) * emergency_mix)
                )
                painter.setPen(QPen(
                    alert_border,
                    8.0,
                ))
                painter.drawRoundedRect(
                    QRectF(7.0, 7.0, width - 14.0, height - 14.0), 24.0, 24.0
                )

        @staticmethod
        def _draw_golden_ears(
            painter: QPainter,
            scale: float,
            outline: QColor,
            profile: dict[str, float],
            left_wiggle: float,
            right_wiggle: float,
        ) -> None:
            """Draw tapered ears that rotate gently around their broad roots."""
            del outline
            base_lift = profile["ears"] * 0.10
            for side, movement, asymmetry in (
                (-1.0, left_wiggle, -1.2),
                (1.0, right_wiggle, 1.5),
            ):
                painter.save()
                painter.translate(side * 158 * scale, -82 * scale)
                painter.rotate(side * (-5.0 + base_lift) + movement + asymmetry)
                painter.scale(side, 1.0)

                ear = QPainterPath()
                ear.moveTo(0, 0)
                ear.cubicTo(
                    27 * scale, -12 * scale,
                    67 * scale, 1 * scale,
                    76 * scale, 42 * scale,
                )
                ear.cubicTo(
                    88 * scale, 91 * scale,
                    70 * scale, 151 * scale,
                    38 * scale, 184 * scale,
                )
                ear.cubicTo(
                    31 * scale, 191 * scale,
                    24 * scale, 181 * scale,
                    18 * scale, 187 * scale,
                )
                ear.cubicTo(
                    11 * scale, 179 * scale,
                    3 * scale, 184 * scale,
                    -1 * scale, 172 * scale,
                )
                ear.cubicTo(
                    -9 * scale, 130 * scale,
                    10 * scale, 68 * scale,
                    0, 0,
                )
                ear.closeSubpath()

                root_shadow = QRadialGradient(
                    QPointF(5 * scale, 6 * scale), 84 * scale
                )
                root_shadow.setColorAt(0.0, QColor(143, 81, 33, 115))
                root_shadow.setColorAt(1.0, QColor(143, 81, 33, 0))
                painter.setPen(Qt.NoPen)
                painter.setBrush(QBrush(root_shadow))
                painter.drawEllipse(QRectF(
                    -17 * scale, -23 * scale, 116 * scale, 107 * scale
                ))

                ear_gradient = QLinearGradient(0, 0, 67 * scale, 184 * scale)
                ear_gradient.setColorAt(0.0, QColor("#E4A144"))
                ear_gradient.setColorAt(0.52, QColor("#D98D34"))
                ear_gradient.setColorAt(1.0, QColor("#C87929"))
                painter.setPen(QPen(
                    QColor(128, 73, 30, 78),
                    max(1.0, 1.35 * scale),
                    Qt.SolidLine,
                    Qt.RoundCap,
                    Qt.RoundJoin,
                ))
                painter.setBrush(QBrush(ear_gradient))
                painter.drawPath(ear)

                inner = QPainterPath()
                inner.moveTo(18 * scale, 18 * scale)
                inner.cubicTo(
                    51 * scale, 31 * scale,
                    60 * scale, 89 * scale,
                    37 * scale, 146 * scale,
                )
                inner.cubicTo(
                    27 * scale, 155 * scale,
                    21 * scale, 158 * scale,
                    17 * scale, 147 * scale,
                )
                inner.cubicTo(
                    18 * scale, 103 * scale,
                    27 * scale, 58 * scale,
                    18 * scale, 18 * scale,
                )
                inner.closeSubpath()
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor(244, 172, 85, 68))
                painter.drawPath(inner)

                # Three restrained strands follow the ear rather than hatching it.
                painter.setBrush(Qt.NoBrush)
                painter.setPen(QPen(
                    QColor(139, 78, 30, 72),
                    max(0.8, 1.05 * scale),
                    Qt.SolidLine,
                    Qt.RoundCap,
                ))
                for offset in (0.0, 10.0, 20.0):
                    strand = QPainterPath()
                    strand.moveTo((25 + offset * 0.32) * scale, 61 * scale)
                    strand.cubicTo(
                        (47 + offset * 0.20) * scale, 97 * scale,
                        (41 + offset * 0.25) * scale, 133 * scale,
                        (26 + offset * 0.18) * scale, 162 * scale,
                    )
                    painter.drawPath(strand)
                painter.restore()

        @staticmethod
        def _draw_golden_head(
            painter: QPainter,
            scale: float,
            outline: QColor,
        ) -> None:
            """Draw a wide forehead, inset temples, soft cheeks and narrow chin."""
            del outline
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(89, 59, 45, 24))
            painter.drawEllipse(QRectF(
                -164 * scale, 142 * scale, 328 * scale, 35 * scale
            ))

            head = QPainterPath()
            head.moveTo(0, -148 * scale)
            head.cubicTo(
                79 * scale, -151 * scale,
                140 * scale, -126 * scale,
                158 * scale, -91 * scale,
            )
            head.cubicTo(
                171 * scale, -69 * scale,
                160 * scale, -48 * scale,
                158 * scale, -31 * scale,
            )
            head.cubicTo(
                183 * scale, -4 * scale,
                190 * scale, 42 * scale,
                179 * scale, 75 * scale,
            )
            head.cubicTo(
                169 * scale, 107 * scale,
                143 * scale, 128 * scale,
                111 * scale, 138 * scale,
            )
            head.cubicTo(
                80 * scale, 148 * scale,
                58 * scale, 149 * scale,
                41 * scale, 155 * scale,
            )
            head.cubicTo(
                24 * scale, 159 * scale,
                13 * scale, 161 * scale,
                0, 166 * scale,
            )
            head.cubicTo(
                -14 * scale, 161 * scale,
                -26 * scale, 159 * scale,
                -42 * scale, 154 * scale,
            )
            head.cubicTo(
                -60 * scale, 149 * scale,
                -82 * scale, 148 * scale,
                -113 * scale, 137 * scale,
            )
            head.cubicTo(
                -146 * scale, 126 * scale,
                -171 * scale, 104 * scale,
                -181 * scale, 72 * scale,
            )
            head.cubicTo(
                -191 * scale, 38 * scale,
                -184 * scale, -6 * scale,
                -159 * scale, -33 * scale,
            )
            head.cubicTo(
                -161 * scale, -51 * scale,
                -171 * scale, -71 * scale,
                -158 * scale, -93 * scale,
            )
            head.cubicTo(
                -138 * scale, -128 * scale,
                -78 * scale, -151 * scale,
                0, -148 * scale,
            )
            head.closeSubpath()

            fur_gradient = QRadialGradient(
                QPointF(-24 * scale, -89 * scale), 302 * scale
            )
            fur_gradient.setColorAt(0.0, QColor("#FFD889"))
            fur_gradient.setColorAt(0.43, QColor("#F5BF65"))
            fur_gradient.setColorAt(0.79, QColor("#E6A249"))
            fur_gradient.setColorAt(1.0, QColor("#C97B2C"))
            painter.setPen(QPen(
                QColor(132, 77, 31, 76),
                max(0.9, 1.25 * scale),
                Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin,
            ))
            painter.setBrush(QBrush(fur_gradient))
            painter.drawPath(head)

            # Ear roots and lower jaw receive soft shade rather than contour ink.
            painter.setPen(Qt.NoPen)
            for side in (-1.0, 1.0):
                root = QRadialGradient(
                    QPointF(side * 145 * scale, -56 * scale), 79 * scale
                )
                root.setColorAt(0.0, QColor(154, 83, 28, 65))
                root.setColorAt(1.0, QColor(154, 83, 28, 0))
                painter.setBrush(QBrush(root))
                painter.drawEllipse(QRectF(
                    (side * 145 - 72) * scale,
                    -124 * scale,
                    144 * scale,
                    143 * scale,
                ))

            jaw = QLinearGradient(0, 79 * scale, 0, 166 * scale)
            jaw.setColorAt(0.0, QColor(171, 90, 29, 0))
            jaw.setColorAt(1.0, QColor(171, 90, 29, 58))
            painter.setBrush(QBrush(jaw))
            jaw_path = QPainterPath()
            jaw_path.moveTo(-139 * scale, 82 * scale)
            jaw_path.cubicTo(
                -99 * scale, 139 * scale,
                99 * scale, 139 * scale,
                140 * scale, 80 * scale,
            )
            jaw_path.cubicTo(
                106 * scale, 151 * scale,
                -108 * scale, 151 * scale,
                -139 * scale, 82 * scale,
            )
            jaw_path.closeSubpath()
            painter.drawPath(jaw_path)

        def _draw_eye(
            self,
            painter: QPainter,
            side: float,
            scale: float,
            profile: dict[str, float],
        ) -> None:
            """Draw one vertical soft eye with an animated upper eyelid."""
            x = side * 75.0 * scale
            y = (-35.0 + (1.2 if side > 0 else 0.0)) * scale
            eye_w = 66.0 * scale
            eye_h = 75.0 * scale
            top = y - eye_h / 2.0
            bottom = y + eye_h / 2.0

            closed_expression = self._category == "sleep" or (
                self._category == "joy"
                and self._intensity >= 94.0
                and profile["eyes"] < 0.72
            )
            if closed_expression:
                painter.setBrush(Qt.NoBrush)
                painter.setPen(QPen(
                    QColor(91, 58, 39, 145),
                    max(1.1, 1.55 * scale),
                    Qt.SolidLine,
                    Qt.RoundCap,
                ))
                closed_lid = QPainterPath()
                closed_lid.moveTo(x - eye_w * 0.34, y + 4 * scale)
                closed_lid.cubicTo(
                    x - eye_w * 0.16, y - 5 * scale,
                    x + eye_w * 0.16, y - 5 * scale,
                    x + eye_w * 0.34, y + 4 * scale,
                )
                painter.drawPath(closed_lid)
                return

            eye = QPainterPath()
            eye.addEllipse(QRectF(
                x - eye_w / 2.0, top, eye_w, eye_h
            ))

            sclera = QLinearGradient(0, top, 0, bottom)
            sclera.setColorAt(0.0, QColor("#efe3cc"))
            sclera.setColorAt(0.26, QColor("#fffaf0"))
            sclera.setColorAt(1.0, QColor("#f4e5cb"))
            painter.setPen(QPen(
                QColor(91, 58, 39, 142),
                max(1.0, 1.45 * scale),
                Qt.SolidLine,
                Qt.RoundCap,
                Qt.RoundJoin,
            ))
            painter.setBrush(QBrush(sclera))
            painter.drawPath(eye)

            pupil_x = x + self._gaze_x * scale
            pupil_y = y + 3.0 * scale + self._gaze_y * scale
            iris_rx = 23.5 * scale
            iris_ry = 27.0 * scale
            painter.save()
            painter.setClipPath(eye)
            iris = QRadialGradient(
                QPointF(pupil_x - 4 * scale, pupil_y - 6 * scale),
                31 * scale,
            )
            iris.setColorAt(0.0, QColor("#976334"))
            iris.setColorAt(0.58, QColor("#57351f"))
            iris.setColorAt(1.0, QColor("#2c211c"))
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(iris))
            painter.drawEllipse(QPointF(pupil_x, pupil_y), iris_rx, iris_ry)
            painter.setBrush(QColor("#171411"))
            painter.drawEllipse(
                QPointF(pupil_x, pupil_y), 13.5 * scale, 17.2 * scale
            )
            painter.setBrush(QColor("#fffdf7"))
            painter.drawEllipse(
                QPointF(pupil_x - 5.6 * scale, pupil_y - 8.0 * scale),
                5.8 * scale,
                6.1 * scale,
            )
            painter.setBrush(QColor(255, 235, 184, 170))
            painter.drawEllipse(
                QPointF(pupil_x + 4.7 * scale, pupil_y + 8.0 * scale),
                1.8 * scale,
                2.0 * scale,
            )

            # The lid covers the eye; the eye itself never collapses to a line.
            base_cover = min(0.54, max(0.0, (0.78 - profile["eyes"]) * 0.90))
            cover = min(0.97, base_cover + self._blink_cover(side) * 0.90)
            lid_y = top + eye_h * (0.10 + cover * 0.82)
            lid = QPainterPath()
            lid.moveTo(x - eye_w * 0.55, top - 2 * scale)
            lid.lineTo(x + eye_w * 0.55, top - 2 * scale)
            lid.lineTo(x + eye_w * 0.55, lid_y)
            lid.cubicTo(
                x + eye_w * 0.25, lid_y + 4 * scale,
                x - eye_w * 0.25, lid_y + 4 * scale,
                x - eye_w * 0.55, lid_y,
            )
            lid.closeSubpath()
            lid_fur = QLinearGradient(0, top, 0, lid_y + 8 * scale)
            lid_fur.setColorAt(0.0, QColor("#F5BF65"))
            lid_fur.setColorAt(1.0, QColor("#E6A249"))
            painter.setBrush(QBrush(lid_fur))
            painter.drawPath(lid)
            painter.restore()

            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(
                QColor(86, 53, 35, 130),
                max(0.9, 1.25 * scale),
                Qt.SolidLine,
                Qt.RoundCap,
            ))
            lid_line = QPainterPath()
            lid_line.moveTo(x - eye_w * 0.38, lid_y)
            lid_line.cubicTo(
                x - eye_w * 0.14, lid_y - 4 * scale,
                x + eye_w * 0.18, lid_y - 4 * scale,
                x + eye_w * 0.38, lid_y,
            )
            painter.drawPath(lid_line)

        def _draw_eyebrows(self, painter: QPainter, scale: float) -> None:
            """Draw short, soft brows whose pose follows the emotion state."""
            category = self._category
            for side in (-1.0, 1.0):
                outer_y = -93.0
                inner_y = -91.0
                if category == "joy":
                    outer_y += 4.0
                    inner_y -= 1.0
                elif category == "curiosity":
                    if side < 0:
                        outer_y -= 6.0
                        inner_y -= 4.0
                    else:
                        outer_y += 2.0
                elif category == "anxiety":
                    inner_y -= 9.0
                    outer_y += 1.0
                elif category == "fear":
                    inner_y -= 11.0
                    outer_y -= 7.0
                elif category == "emergency":
                    inner_y -= 14.0
                    outer_y -= 11.0

                inner_x = side * 48.0
                outer_x = side * 101.0
                brow = QPainterPath()
                brow.moveTo(inner_x * scale, inner_y * scale)
                brow.cubicTo(
                    side * 63 * scale, (inner_y - 5.0) * scale,
                    side * 84 * scale, (outer_y - 4.0) * scale,
                    outer_x * scale, outer_y * scale,
                )
                painter.setBrush(Qt.NoBrush)
                painter.setPen(QPen(
                    QColor(143, 83, 35, 164),
                    max(1.1, 2.0 * scale),
                    Qt.SolidLine,
                    Qt.RoundCap,
                ))
                painter.drawPath(brow)

        def _draw_cheeks(self, painter: QPainter, scale: float) -> None:
            visible = self._category in ("joy", "excitement", "social")
            alpha = 68 if visible else 18
            if self._face_state.system_state == "emergency":
                alpha = 82
            for side in (-1.0, 1.0):
                center = QPointF(side * 128 * scale, 55 * scale)
                blush = QRadialGradient(center, 48 * scale)
                blush.setColorAt(0.0, QColor(240, 135, 80, alpha))
                blush.setColorAt(0.60, QColor(240, 135, 80, alpha // 2))
                blush.setColorAt(1.0, QColor(240, 135, 80, 0))
                painter.setPen(Qt.NoPen)
                painter.setBrush(QBrush(blush))
                painter.drawEllipse(center, 49 * scale, 31 * scale)

        @staticmethod
        def _draw_muzzle(painter: QPainter, scale: float) -> None:
            """Build the muzzle from two overlapping cheek lobes, not an oval."""
            painter.setPen(Qt.NoPen)
            for side in (-1.0, 1.0):
                lobe = QPainterPath()
                lobe.moveTo(0, 27 * scale)
                lobe.cubicTo(
                    side * 25 * scale, 20 * scale,
                    side * 76 * scale, 30 * scale,
                    side * 82 * scale, 65 * scale,
                )
                lobe.cubicTo(
                    side * 90 * scale, 100 * scale,
                    side * 55 * scale, 119 * scale,
                    side * 13 * scale, 104 * scale,
                )
                lobe.cubicTo(
                    side * 2 * scale, 91 * scale,
                    side * 3 * scale, 51 * scale,
                    0, 27 * scale,
                )
                lobe.closeSubpath()
                lobe_light = QRadialGradient(
                    QPointF(side * 22 * scale, 42 * scale), 102 * scale
                )
                lobe_light.setColorAt(0.0, QColor("#FFDFA0"))
                lobe_light.setColorAt(0.68, QColor("#F5C873"))
                lobe_light.setColorAt(1.0, QColor("#E8AE58"))
                painter.setBrush(QBrush(lobe_light))
                painter.drawPath(lobe)

            chin = QRadialGradient(QPointF(0, 102 * scale), 58 * scale)
            chin.setColorAt(0.0, QColor(248, 204, 119, 125))
            chin.setColorAt(1.0, QColor(218, 142, 54, 0))
            painter.setBrush(QBrush(chin))
            painter.drawEllipse(QRectF(
                -54 * scale, 83 * scale, 108 * scale, 57 * scale
            ))

        def _draw_nose_and_mouth(
            self,
            painter: QPainter,
            scale: float,
            profile: dict[str, float],
        ) -> None:
            nose_bounce = math.sin(self._phase * 2.4) * 0.8 * scale
            nose = QPainterPath()
            nose.moveTo(-29 * scale, 29 * scale + nose_bounce)
            nose.cubicTo(
                -20 * scale, 20 * scale + nose_bounce,
                20 * scale, 20 * scale + nose_bounce,
                29 * scale, 29 * scale + nose_bounce,
            )
            nose.cubicTo(
                30 * scale, 45 * scale + nose_bounce,
                13 * scale, 57 * scale + nose_bounce,
                0, 61 * scale + nose_bounce,
            )
            nose.cubicTo(
                -13 * scale, 57 * scale + nose_bounce,
                -30 * scale, 45 * scale + nose_bounce,
                -29 * scale, 29 * scale + nose_bounce,
            )
            nose.closeSubpath()
            nose_fill = QLinearGradient(0, 20 * scale, 0, 63 * scale)
            nose_fill.setColorAt(0.0, QColor("#4A3428"))
            nose_fill.setColorAt(1.0, QColor("#30231E"))
            painter.setPen(QPen(
                QColor(48, 34, 29, 110), max(0.8, 1.0 * scale)
            ))
            painter.setBrush(QBrush(nose_fill))
            painter.drawPath(nose)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(255, 244, 223, 105))
            painter.drawEllipse(
                QPointF(-10 * scale, 29 * scale + nose_bounce),
                5.5 * scale,
                3.0 * scale,
            )
            painter.setBrush(QColor(27, 20, 18, 82))
            painter.drawEllipse(
                QPointF(-13 * scale, 43 * scale + nose_bounce),
                4.5 * scale,
                3.0 * scale,
            )
            painter.drawEllipse(
                QPointF(13 * scale, 43 * scale + nose_bounce),
                4.5 * scale,
                3.0 * scale,
            )

            if self._category == "sleep":
                return
            mouth_color = QColor(83, 48, 37, 190)
            philtrum_end = 69.0
            painter.setPen(QPen(
                mouth_color, max(1.0, 1.45 * scale),
                Qt.SolidLine, Qt.RoundCap,
            ))
            painter.drawLine(
                QPointF(0, 60 * scale + nose_bounce),
                QPointF(0, philtrum_end * scale),
            )

            open_mouth = (
                self._category == "excitement"
                or (self._category in ("joy", "social") and self._intensity >= 78)
                or self._category == "eat"
                or (self._category == "fear" and self._intensity >= 62)
            )
            if open_mouth:
                mouth = QPainterPath()
                mouth.moveTo(-37 * scale, 75 * scale)
                mouth.cubicTo(
                    -19 * scale, 84 * scale,
                    19 * scale, 84 * scale,
                    37 * scale, 75 * scale,
                )
                mouth.cubicTo(
                    32 * scale, 110 * scale,
                    16 * scale, 122 * scale,
                    0, 122 * scale,
                )
                mouth.cubicTo(
                    -16 * scale, 122 * scale,
                    -32 * scale, 110 * scale,
                    -37 * scale, 75 * scale,
                )
                mouth.closeSubpath()
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor("#5A3329"))
                painter.drawPath(mouth)
                if self._category in ("excitement", "joy", "social", "eat"):
                    painter.setBrush(QColor("#EF8293"))
                    tongue_y = (
                        100.0 + math.sin(self._phase * 3.8) * 2.5
                    ) * scale
                    painter.drawRoundedRect(
                        QRectF(-16 * scale, tongue_y, 32 * scale, 31 * scale),
                        14 * scale,
                        14 * scale,
                    )
                return

            # Closed mouth uses two small arcs with a hint of asymmetry.
            smile = profile["smile"]
            center_y = 78.0 + max(-8.0, min(10.0, smile * 8.0))
            left_corner_y = 74.0 - max(-4.0, min(5.0, smile * 5.0))
            right_corner_y = left_corner_y - 1.2
            for side, corner_y in ((-1.0, left_corner_y), (1.0, right_corner_y)):
                curve = QPainterPath()
                curve.moveTo(0, philtrum_end * scale)
                curve.cubicTo(
                    side * 10 * scale, center_y * scale,
                    side * 28 * scale, (center_y + 2.0) * scale,
                    side * 42 * scale, corner_y * scale,
                )
                painter.drawPath(curve)

        @staticmethod
        def _draw_fur_accents(painter: QPainter, scale: float) -> None:
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(
                QColor(255, 222, 156, 76),
                max(0.7, 0.95 * scale),
                Qt.SolidLine,
                Qt.RoundCap,
            ))
            for offset in (-9.0, 9.0):
                tuft = QPainterPath()
                tuft.moveTo(offset * scale, -133 * scale)
                tuft.cubicTo(
                    (offset - 3.0) * scale, -119 * scale,
                    (offset + 3.0) * scale, -111 * scale,
                    offset * 0.72 * scale, -101 * scale,
                )
                painter.drawPath(tuft)

            painter.setPen(QPen(
                QColor(172, 96, 31, 56),
                max(0.7, 0.9 * scale),
                Qt.SolidLine,
                Qt.RoundCap,
            ))
            for side in (-1.0, 1.0):
                for y in (69.0, 82.0):
                    cheek = QPainterPath()
                    cheek.moveTo(side * 144 * scale, y * scale)
                    cheek.cubicTo(
                        side * 153 * scale, (y + 4.0) * scale,
                        side * 151 * scale, (y + 9.0) * scale,
                        side * 158 * scale, (y + 12.0) * scale,
                    )
                    painter.drawPath(cheek)

        @staticmethod
        def _draw_bionic_ears(
            painter: QPainter,
            scale: float,
            profile: dict[str, float],
            left_wiggle: float,
            right_wiggle: float,
            accent: QColor,
        ) -> None:
            """Soft actuator ears retain a readable canine silhouette."""
            for side, movement in ((-1.0, left_wiggle), (1.0, right_wiggle)):
                painter.save()
                painter.translate(side * 151 * scale, -78 * scale)
                painter.rotate(side * (-5.0 + profile["ears"] * 0.10) + movement)
                painter.scale(side, 1.0)

                ear = QPainterPath()
                ear.moveTo(-5 * scale, -5 * scale)
                ear.cubicTo(32 * scale, -19 * scale, 72 * scale, 1 * scale, 77 * scale, 39 * scale)
                ear.cubicTo(82 * scale, 84 * scale, 63 * scale, 139 * scale, 31 * scale, 169 * scale)
                ear.cubicTo(19 * scale, 181 * scale, 3 * scale, 170 * scale, 1 * scale, 151 * scale)
                ear.cubicTo(-2 * scale, 111 * scale, 12 * scale, 61 * scale, -5 * scale, -5 * scale)
                ear.closeSubpath()

                ear_fill = QLinearGradient(0, -10 * scale, 70 * scale, 170 * scale)
                ear_fill.setColorAt(0.0, QColor("#59636a"))
                ear_fill.setColorAt(0.48, QColor("#343d44"))
                ear_fill.setColorAt(1.0, QColor("#20272d"))
                painter.setPen(QPen(QColor(20, 27, 32, 125), max(1.0, 1.15 * scale)))
                painter.setBrush(QBrush(ear_fill))
                painter.drawPath(ear)

                insert = QPainterPath()
                insert.moveTo(18 * scale, 21 * scale)
                insert.cubicTo(48 * scale, 22 * scale, 56 * scale, 73 * scale, 35 * scale, 126 * scale)
                insert.cubicTo(26 * scale, 147 * scale, 18 * scale, 144 * scale, 17 * scale, 124 * scale)
                insert.cubicTo(18 * scale, 85 * scale, 27 * scale, 49 * scale, 18 * scale, 21 * scale)
                insert.closeSubpath()
                inner = QLinearGradient(0, 20 * scale, 0, 145 * scale)
                tint = QColor(accent)
                tint.setAlpha(92)
                inner.setColorAt(0.0, QColor(132, 146, 153, 95))
                inner.setColorAt(1.0, tint)
                painter.setPen(Qt.NoPen)
                painter.setBrush(QBrush(inner))
                painter.drawPath(insert)
                painter.restore()

        @staticmethod
        def _draw_bionic_shell(painter: QPainter, scale: float, accent: QColor) -> None:
            """Warm composite shell shaped as a broad canine forehead and jaw."""
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(29, 38, 43, 30))
            painter.drawEllipse(QRectF(-174 * scale, 145 * scale, 348 * scale, 30 * scale))

            shell = QPainterPath()
            shell.moveTo(0, -148 * scale)
            shell.cubicTo(84 * scale, -151 * scale, 143 * scale, -124 * scale, 158 * scale, -84 * scale)
            shell.cubicTo(163 * scale, -67 * scale, 157 * scale, -49 * scale, 160 * scale, -32 * scale)
            shell.cubicTo(184 * scale, -3 * scale, 187 * scale, 54 * scale, 171 * scale, 91 * scale)
            shell.cubicTo(156 * scale, 126 * scale, 116 * scale, 143 * scale, 74 * scale, 149 * scale)
            shell.cubicTo(45 * scale, 153 * scale, 25 * scale, 161 * scale, 0, 166 * scale)
            shell.cubicTo(-25 * scale, 161 * scale, -45 * scale, 153 * scale, -74 * scale, 149 * scale)
            shell.cubicTo(-116 * scale, 143 * scale, -156 * scale, 126 * scale, -171 * scale, 91 * scale)
            shell.cubicTo(-187 * scale, 54 * scale, -184 * scale, -3 * scale, -160 * scale, -32 * scale)
            shell.cubicTo(-157 * scale, -49 * scale, -163 * scale, -67 * scale, -158 * scale, -84 * scale)
            shell.cubicTo(-143 * scale, -124 * scale, -84 * scale, -151 * scale, 0, -148 * scale)
            shell.closeSubpath()

            body = QLinearGradient(-120 * scale, -130 * scale, 145 * scale, 155 * scale)
            body.setColorAt(0.0, QColor("#f8f6ef"))
            body.setColorAt(0.42, QColor("#e8e8e3"))
            body.setColorAt(0.76, QColor("#d9dde0"))
            body.setColorAt(1.0, QColor("#bcc4c9"))
            painter.setPen(QPen(QColor(63, 75, 82, 100), max(1.0, 1.2 * scale)))
            painter.setBrush(QBrush(body))
            painter.drawPath(shell)

            # A recessed brow plate prevents the shell from reading as a plain oval.
            brow_plate = QPainterPath()
            brow_plate.moveTo(-126 * scale, -57 * scale)
            brow_plate.cubicTo(-82 * scale, -91 * scale, 82 * scale, -91 * scale, 126 * scale, -57 * scale)
            brow_plate.cubicTo(103 * scale, -73 * scale, -103 * scale, -73 * scale, -126 * scale, -57 * scale)
            painter.setBrush(Qt.NoBrush)
            seam = QColor(accent)
            seam.setAlpha(88)
            painter.setPen(QPen(seam, max(1.0, 1.35 * scale), Qt.SolidLine, Qt.RoundCap))
            painter.drawPath(brow_plate)

            # Side service seams follow the cheek, keeping detail sparse.
            painter.setPen(QPen(QColor(82, 96, 104, 70), max(0.8, 0.9 * scale), Qt.SolidLine, Qt.RoundCap))
            for side in (-1.0, 1.0):
                seam_path = QPainterPath()
                seam_path.moveTo(side * 154 * scale, 18 * scale)
                seam_path.cubicTo(side * 166 * scale, 56 * scale, side * 151 * scale, 102 * scale, side * 126 * scale, 121 * scale)
                painter.drawPath(seam_path)

        def _draw_bionic_eye(
            self,
            painter: QPainter,
            side: float,
            scale: float,
            profile: dict[str, float],
            accent: QColor,
        ) -> None:
            """Expressive optical pod: canine gaze inside a restrained dark lens."""
            x = side * 72.0 * scale
            y = (-30.0 + (1.0 if side > 0 else 0.0)) * scale
            eye_w = 74.0 * scale
            eye_h = 62.0 * scale

            closed = self._category == "sleep"
            if closed:
                line = QPainterPath()
                line.moveTo(x - 25 * scale, y + 4 * scale)
                line.cubicTo(x - 10 * scale, y - 5 * scale, x + 10 * scale, y - 5 * scale, x + 25 * scale, y + 4 * scale)
                glow = QColor(accent)
                glow.setAlpha(175)
                painter.setPen(QPen(glow, max(2.0, 2.4 * scale), Qt.SolidLine, Qt.RoundCap))
                painter.setBrush(Qt.NoBrush)
                painter.drawPath(line)
                return

            lens = QPainterPath()
            lens.moveTo(x - eye_w * 0.50, y)
            lens.cubicTo(x - eye_w * 0.35, y - eye_h * 0.55, x + eye_w * 0.34, y - eye_h * 0.55, x + eye_w * 0.50, y)
            lens.cubicTo(x + eye_w * 0.34, y + eye_h * 0.52, x - eye_w * 0.35, y + eye_h * 0.52, x - eye_w * 0.50, y)
            lens.closeSubpath()
            glass = QRadialGradient(QPointF(x - 10 * scale, y - 12 * scale), 63 * scale)
            glass.setColorAt(0.0, QColor("#33434a"))
            glass.setColorAt(0.55, QColor("#172229"))
            glass.setColorAt(1.0, QColor("#0b1115"))
            painter.setPen(QPen(QColor(48, 61, 68, 180), max(1.0, 1.3 * scale)))
            painter.setBrush(QBrush(glass))
            painter.drawPath(lens)

            px = x + self._gaze_x * scale
            py = y + self._gaze_y * scale
            ring = QColor(accent)
            ring.setAlpha(220)
            painter.setPen(QPen(ring, max(2.0, 2.6 * scale)))
            painter.setBrush(QColor("#071014"))
            painter.drawEllipse(QPointF(px, py), 17 * scale, 19 * scale)
            painter.setPen(Qt.NoPen)
            core = QColor(accent.lighter(145))
            painter.setBrush(core)
            painter.drawEllipse(QPointF(px, py), 7.5 * scale, 10 * scale)
            painter.setBrush(QColor(239, 255, 255, 230))
            painter.drawEllipse(QPointF(px - 5 * scale, py - 7 * scale), 3.0 * scale, 3.0 * scale)

            # Fur-like lids become shell shutters and retain blink semantics.
            base_cover = min(0.48, max(0.0, (0.75 - profile["eyes"]) * 0.85))
            cover = min(0.94, base_cover + self._blink_cover(side) * 0.88)
            if cover > 0.01:
                painter.save()
                painter.setClipPath(lens)
                shutter_y = y - eye_h * 0.50 + eye_h * cover
                shutter = QLinearGradient(0, y - eye_h / 2, 0, shutter_y)
                shutter.setColorAt(0.0, QColor("#d9dde0"))
                shutter.setColorAt(1.0, QColor("#aeb8bd"))
                painter.setPen(Qt.NoPen)
                painter.setBrush(QBrush(shutter))
                painter.drawRoundedRect(QRectF(x - eye_w / 2, y - eye_h / 2, eye_w, max(1.0, shutter_y - (y - eye_h / 2))), 10 * scale, 10 * scale)
                painter.restore()

        def _draw_bionic_muzzle(
            self,
            painter: QPainter,
            scale: float,
            profile: dict[str, float],
            accent: QColor,
        ) -> None:
            """One soft silicone muzzle module with a readable dog nose and mouth."""
            module = QPainterPath()
            module.moveTo(0, 24 * scale)
            module.cubicTo(73 * scale, 18 * scale, 94 * scale, 51 * scale, 80 * scale, 91 * scale)
            module.cubicTo(68 * scale, 122 * scale, 28 * scale, 132 * scale, 0, 132 * scale)
            module.cubicTo(-28 * scale, 132 * scale, -68 * scale, 122 * scale, -80 * scale, 91 * scale)
            module.cubicTo(-94 * scale, 51 * scale, -73 * scale, 18 * scale, 0, 24 * scale)
            module.closeSubpath()
            silicone = QLinearGradient(0, 20 * scale, 0, 135 * scale)
            silicone.setColorAt(0.0, QColor("#778288"))
            silicone.setColorAt(0.55, QColor("#59646a"))
            silicone.setColorAt(1.0, QColor("#414b51"))
            painter.setPen(QPen(QColor(35, 45, 50, 125), max(1.0, 1.1 * scale)))
            painter.setBrush(QBrush(silicone))
            painter.drawPath(module)

            nose = QPainterPath()
            nose.moveTo(-31 * scale, 38 * scale)
            nose.cubicTo(-22 * scale, 27 * scale, 22 * scale, 27 * scale, 31 * scale, 38 * scale)
            nose.cubicTo(28 * scale, 57 * scale, 12 * scale, 65 * scale, 0, 68 * scale)
            nose.cubicTo(-12 * scale, 65 * scale, -28 * scale, 57 * scale, -31 * scale, 38 * scale)
            nose.closeSubpath()
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor("#11191d"))
            painter.drawPath(nose)
            painter.setBrush(QColor(255, 255, 255, 82))
            painter.drawEllipse(QPointF(-11 * scale, 38 * scale), 5 * scale, 3 * scale)

            if self._category == "sleep":
                return
            open_mouth = self._category in ("excitement", "eat") or (
                self._category in ("joy", "social") and self._intensity >= 76
            )
            mouth_color = QColor(18, 26, 30, 220)
            if open_mouth:
                mouth = QPainterPath()
                mouth.moveTo(-34 * scale, 81 * scale)
                mouth.cubicTo(-17 * scale, 89 * scale, 17 * scale, 89 * scale, 34 * scale, 81 * scale)
                mouth.cubicTo(27 * scale, 112 * scale, 13 * scale, 119 * scale, 0, 119 * scale)
                mouth.cubicTo(-13 * scale, 119 * scale, -27 * scale, 112 * scale, -34 * scale, 81 * scale)
                mouth.closeSubpath()
                painter.setBrush(mouth_color)
                painter.drawPath(mouth)
                painter.setBrush(QColor("#d97886"))
                painter.drawRoundedRect(QRectF(-13 * scale, 100 * scale, 26 * scale, 24 * scale), 11 * scale, 11 * scale)
                return

            smile = profile["smile"]
            painter.setBrush(Qt.NoBrush)
            mouth_pen = QColor(accent)
            mouth_pen.setAlpha(180)
            painter.setPen(QPen(mouth_pen, max(1.2, 1.7 * scale), Qt.SolidLine, Qt.RoundCap))
            mouth = QPainterPath()
            if smile >= 0:
                mouth.moveTo(-31 * scale, 86 * scale)
                mouth.cubicTo(-14 * scale, (94 + smile * 5) * scale, 14 * scale, (94 + smile * 5) * scale, 31 * scale, 86 * scale)
            else:
                mouth.moveTo(-30 * scale, 96 * scale)
                mouth.cubicTo(-13 * scale, (86 + smile * 4) * scale, 13 * scale, (86 + smile * 4) * scale, 30 * scale, 96 * scale)
            painter.drawPath(mouth)

        @staticmethod
        def _draw_bionic_status(painter: QPainter, scale: float, accent: QColor) -> None:
            """A single forehead sensor and cheek light keep the design quiet."""
            light = QColor(accent)
            light.setAlpha(195)
            painter.setPen(Qt.NoPen)
            painter.setBrush(light)
            painter.drawRoundedRect(QRectF(-16 * scale, -119 * scale, 32 * scale, 4 * scale), 2 * scale, 2 * scale)
            for side in (-1.0, 1.0):
                painter.setBrush(QColor(accent.red(), accent.green(), accent.blue(), 95))
                painter.drawEllipse(QPointF(side * 139 * scale, 83 * scale), 4 * scale, 4 * scale)

        def _draw_classic_pet_face(
            self,
            painter: QPainter,
            scale: float,
            profile: dict[str, float],
            accent: QColor,
            left_wiggle: float,
            right_wiggle: float,
        ) -> None:
            """Draw the original caramel virtual-pet face used before redesigns."""
            outline = QColor("#654536")
            fur = QColor("#dca06a")
            inner_ear = QColor("#f3aaa5")
            muzzle_color = QColor("#f7d7ad")
            ear_shift = profile["ears"] * scale

            left_ear = QPainterPath()
            left_ear.moveTo(-145 * scale, -63 * scale)
            left_ear.cubicTo(
                -184 * scale, -102 * scale,
                -164 * scale, -178 * scale - ear_shift - left_wiggle * scale,
                -126 * scale, -190 * scale - ear_shift - left_wiggle * scale,
            )
            left_ear.cubicTo(-93 * scale, -168 * scale, -75 * scale, -127 * scale, -67 * scale, -101 * scale)
            left_ear.closeSubpath()
            right_ear = QPainterPath()
            right_ear.moveTo(145 * scale, -63 * scale)
            right_ear.cubicTo(
                184 * scale, -102 * scale,
                164 * scale, -178 * scale - ear_shift - right_wiggle * scale,
                126 * scale, -190 * scale - ear_shift - right_wiggle * scale,
            )
            right_ear.cubicTo(93 * scale, -168 * scale, 75 * scale, -127 * scale, 67 * scale, -101 * scale)
            right_ear.closeSubpath()
            painter.setPen(QPen(outline, max(2.0, 4.0 * scale), Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            painter.setBrush(fur)
            painter.drawPath(left_ear)
            painter.drawPath(right_ear)

            painter.setPen(Qt.NoPen)
            painter.setBrush(inner_ear)
            painter.drawEllipse(QRectF(-145 * scale, -161 * scale - ear_shift * 0.65 - left_wiggle * scale * 0.65, 38 * scale, 76 * scale))
            painter.drawEllipse(QRectF(107 * scale, -161 * scale - ear_shift * 0.65 - right_wiggle * scale * 0.65, 38 * scale, 76 * scale))

            painter.setBrush(QColor(89, 59, 45, 24))
            painter.drawEllipse(QRectF(-151 * scale, 115 * scale, 302 * scale, 39 * scale))
            painter.setPen(QPen(outline, max(2.0, 4.0 * scale), Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            painter.setBrush(fur)
            painter.drawRoundedRect(QRectF(-178 * scale, -108 * scale, 356 * scale, 245 * scale), 67 * scale, 67 * scale)

            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor("#efbd86"))
            forehead = QPainterPath()
            forehead.moveTo(-30 * scale, -107 * scale)
            forehead.cubicTo(-17 * scale, -71 * scale, 17 * scale, -71 * scale, 30 * scale, -107 * scale)
            forehead.closeSubpath()
            painter.drawPath(forehead)

            eye_y = -24 * scale
            eye_open = profile["eyes"]
            for side in (-1.0, 1.0):
                eye_x = side * 82 * scale
                if self._category == "sleep" or self._blink_cover(side) > 0.72:
                    closed_eye = QPainterPath()
                    closed_eye.moveTo(eye_x - 31 * scale, eye_y)
                    closed_eye.cubicTo(eye_x - 15 * scale, eye_y + 12 * scale, eye_x + 15 * scale, eye_y + 12 * scale, eye_x + 31 * scale, eye_y)
                    painter.setBrush(Qt.NoBrush)
                    painter.setPen(QPen(outline, max(3.0, 5.0 * scale), Qt.SolidLine, Qt.RoundCap))
                    painter.drawPath(closed_eye)
                    continue

                eye_w = 72 * scale
                eye_h = max(12 * scale, 54 * scale * eye_open)
                painter.setPen(QPen(outline, max(2.0, 3.5 * scale)))
                painter.setBrush(QColor("#fffaf3"))
                painter.drawRoundedRect(QRectF(eye_x - eye_w / 2, eye_y - eye_h / 2, eye_w, eye_h), eye_h / 2, eye_h / 2)
                pupil = QPointF(eye_x + self._gaze_x * scale, eye_y + 2 * scale + self._gaze_y * scale)
                radius = 16.5 * scale if self._category == "fear" else 14 * scale
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor("#533a30"))
                painter.drawEllipse(pupil, radius, radius)
                painter.setBrush(QColor("#fffdf8"))
                painter.drawEllipse(QPointF(pupil.x() - 4 * scale, pupil.y() - 5 * scale), 4.5 * scale, 4.5 * scale)

            if self._category in ("anxiety", "fear"):
                painter.setPen(QPen(outline, max(2.0, 4.0 * scale), Qt.SolidLine, Qt.RoundCap))
                painter.drawLine(-112 * scale, -62 * scale, -57 * scale, -50 * scale)
                painter.drawLine(112 * scale, -62 * scale, 57 * scale, -50 * scale)

            cheek = QColor(accent)
            cheek.setAlpha(58)
            painter.setPen(Qt.NoPen)
            painter.setBrush(cheek)
            painter.drawEllipse(QPointF(-130 * scale, 51 * scale), 24 * scale, 13 * scale)
            painter.drawEllipse(QPointF(130 * scale, 51 * scale), 24 * scale, 13 * scale)

            painter.setPen(QPen(QColor("#b87955"), max(1.0, 2.0 * scale)))
            painter.setBrush(muzzle_color)
            painter.drawRoundedRect(QRectF(-82 * scale, 27 * scale, 164 * scale, 91 * scale), 38 * scale, 38 * scale)
            nose_bounce = math.sin(self._phase * 2.4) * 1.4 * scale
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor("#5a3d32"))
            painter.drawRoundedRect(QRectF(-22 * scale, 36 * scale + nose_bounce, 44 * scale, 25 * scale), 12 * scale, 12 * scale)
            painter.setBrush(QColor(255, 255, 255, 125))
            painter.drawEllipse(QPointF(-8 * scale, 42 * scale + nose_bounce), 4.5 * scale, 2.7 * scale)

            mouth_motion = math.sin(self._phase * 3.0) * 4.0 if self._category in ("joy", "excitement", "social", "eat") else 0.0
            mouth = QPainterPath()
            mouth.moveTo(-52 * scale, (78 + mouth_motion) * scale)
            mouth.cubicTo(-24 * scale, (78 + profile["smile"] * 34 + mouth_motion * 0.3) * scale, 24 * scale, (78 + profile["smile"] * 34 + mouth_motion * 0.3) * scale, 52 * scale, (78 + mouth_motion) * scale)
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(outline, max(2.0, 4.0 * scale), Qt.SolidLine, Qt.RoundCap))
            painter.drawPath(mouth)

            if self._category == "eat":
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor("#ff7f9f"))
                tongue_y = (91 + math.sin(self._phase * 4.0) * 5.0) * scale
                painter.drawRoundedRect(QRectF(-17 * scale, tongue_y, 34 * scale, 27 * scale), 13 * scale, 13 * scale)

            painter.setPen(QPen(outline, max(1.0, 2.5 * scale)))
            painter.setBrush(accent.lighter(115))
            painter.drawRoundedRect(QRectF(-104 * scale, 116 * scale, 208 * scale, 25 * scale), 11 * scale, 11 * scale)
            tag_radius = (15.0 + math.sin(self._phase * 2.0) * 1.5) * scale
            painter.setBrush(QColor("#fff5dd"))
            painter.drawEllipse(QPointF(0, 143 * scale), tag_radius, tag_radius)
            painter.setPen(Qt.NoPen)
            painter.setBrush(accent.darker(108))
            painter.drawEllipse(QPointF(0, 143 * scale), 5 * scale, 5 * scale)

        def paintEvent(self, event) -> None:
            del event
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing, True)
            width = max(1, self.width())
            height = max(1, self.height())
            accent = self._current_accent()
            self._draw_background(painter, width, height, accent)

            center = QPointF(width * 0.5, height * 0.50)

            profile = self._visual_profile
            scale = min(width / 680.0, height / 430.0)
            expressive_bob = min(5.0, profile["bob"])
            bob = math.sin(self._phase * 2.0) * expressive_bob * scale
            breathing_y = math.sin(self._phase * 1.35) * 1.7 * scale
            shake = math.sin(self._phase * 12.0) * profile["shake"] * scale
            tilt = profile["tilt"] + math.sin(self._phase * 0.8) * abs(profile["tilt"]) * 0.18
            if self._presentation.face_cue == "success":
                tilt += math.sin(self._phase * 3.4) * 3.0

            painter.save()
            painter.translate(center.x() + shake, center.y() + bob + breathing_y)
            painter.rotate(tilt)
            breath_amount = (
                0.009 if self._category in ("calm", "sleep", "charge") else 0.005
            )
            breath_phase = math.sin(self._phase * 1.45) * breath_amount
            painter.scale(1.0 + breath_phase, 1.0 + breath_phase)

            lively_ears = self._category in (
                "joy", "excitement", "curiosity", "social", "explore", "command",
            )
            ear_range = 3.4 if lively_ears else 1.4
            left_ear_wiggle = math.sin(self._phase * 3.1) * ear_range
            right_ear_wiggle = math.sin(self._phase * 3.1 + 1.1) * ear_range
            cue = self._presentation.face_cue
            if cue == "search":
                left_ear_wiggle = math.sin(self._phase * 4.0) * 5.5
                right_ear_wiggle = math.sin(self._phase * 4.0 + math.pi) * 5.5
            elif cue == "follow":
                tracking = math.sin(self._phase * 0.55) * 2.6
                left_ear_wiggle += tracking
                right_ear_wiggle += tracking
            elif cue == "call":
                elapsed = max(0, self._frame - self._cue_started_frame)
                response = max(0.0, 1.0 - elapsed / 24.0)
                left_ear_wiggle += math.sin(elapsed * 0.9) * response * 7.0
                right_ear_wiggle += math.sin(elapsed * 0.9 + 0.8) * response * 7.0

            self._draw_classic_pet_face(
                painter, scale, profile, accent,
                left_ear_wiggle, right_ear_wiggle,
            )

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

            painter.restore()


if HAS_QT:

    class FriendlyStatusLabel(QLabel):
        """Large normal-mode copy with a short, soft fade on state changes."""

        def __init__(self, parent: QWidget | None = None) -> None:
            super().__init__(parent)
            self._opacity = QGraphicsOpacityEffect(self)
            self.setGraphicsEffect(self._opacity)
            self._fade = QPropertyAnimation(self._opacity, b"opacity", self)
            self._fade.setDuration(260)
            self._opacity.setOpacity(1.0)

        def set_friendly_text(self, text: str) -> None:
            if text == self.text():
                return
            self.setText(text)
            self._fade.stop()
            self._fade.setStartValue(0.35)
            self._fade.setEndValue(1.0)
            self._fade.start()


    class SoftProgressBar(QWidget):
        """Thin breathing light strip used only when progress is meaningful."""

        def __init__(self, parent: QWidget | None = None) -> None:
            super().__init__(parent)
            self.setFixedHeight(18)
            self._value = 0.0
            self._opacity = 0.0
            self._target_opacity = 0.0
            self._timer = QTimer(self)
            self._timer.timeout.connect(self._animate)
            self._timer.start(33)

        def set_progress(self, value: float, visible: bool) -> None:
            self._value = min(1.0, max(0.0, float(value)))
            self._target_opacity = 1.0 if visible else 0.0
            self.update()

        def _animate(self) -> None:
            self._opacity += (self._target_opacity - self._opacity) * 0.16
            if abs(self._target_opacity - self._opacity) < 0.01:
                self._opacity = self._target_opacity
            self.update()

        def paintEvent(self, event) -> None:
            del event
            if self._opacity <= 0.01:
                return
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing, True)
            bar_width = max(120.0, self.width() * 0.48)
            bar_height = 8.0
            rect = QRectF(
                (self.width() - bar_width) / 2.0,
                (self.height() - bar_height) / 2.0,
                bar_width,
                bar_height,
            )
            alpha = int(120 * self._opacity)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(218, 190, 170, alpha))
            painter.drawRoundedRect(rect, 4.0, 4.0)
            if self._value > 0.0:
                fill = QRectF(rect)
                fill.setWidth(max(8.0, rect.width() * self._value))
                pulse = 0.82 + 0.18 * math.sin(self._value * math.pi)
                painter.setBrush(QColor(233, 154, 116, int(225 * self._opacity * pulse)))
                painter.drawRoundedRect(fill, 4.0, 4.0)


class EmotionWindow(QWidget):
    """Emotion visualization Qt window.

    Receives events from the ROS2 thread via a thread-safe queue.
    A Qt timer drains the queue and updates the UI.
    """

    def __init__(
        self,
        event_queue: queue.Queue | None = None,
        ros_connected: bool = False,
    ) -> None:
        if not HAS_QT:
            raise RuntimeError("PySide2 not installed")
        super().__init__()
        self._queue = event_queue or queue.Queue()
        self._display_generation = 0
        self._active_goal_id = ""
        self._active_behavior_name = ""
        self._category = "unknown"
        self._current_stage = ""
        self._current_action = ""
        self._current_progress = 0.0
        self._current_intensity = 50.0
        self._current_status = "READY"
        self._ros_connected = bool(ros_connected)
        self._debug_visible = False
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
        layout.setContentsMargins(14, 12, 14, 14)
        layout.setSpacing(4)

        # --- Animated virtual-pet face ---
        self._face = DigitalDogFace(self)
        self._face.setMinimumHeight(max(340, int(h * 0.70)))
        layout.addWidget(self._face, 1)
        layout.addSpacing(12)

        # Normal mode contains only the living face and friendly presentation.
        self._user_status_label = FriendlyStatusLabel(self)
        self._user_status_label.setAlignment(Qt.AlignCenter)
        self._user_status_label.setStyleSheet(
            "color: #66534A; background: transparent; font-weight: 500; "
            "letter-spacing: 1px;"
        )
        self._user_status_label.setFont(QFont("Sans", max(24, h // 20)))
        layout.addWidget(self._user_status_label)

        self._user_subtitle_label = QLabel("")
        self._user_subtitle_label.setAlignment(Qt.AlignCenter)
        self._user_subtitle_label.setStyleSheet(
            "color: #9a8174; background: transparent;"
        )
        self._user_subtitle_label.setFont(QFont("Sans", max(16, h // 34)))
        layout.addWidget(self._user_subtitle_label)

        self._soft_progress = SoftProgressBar(self)
        layout.addWidget(self._soft_progress)

        # Compatibility widgets retain the old field API but live in the
        # generated Debug Overlay instead of the normal-mode layout.
        self._emotion_label = QLabel(_label("unknown"), self)
        self._behavior_label = QLabel("Waiting...", self)
        self._stage_label = QLabel("", self)
        self._action_label = QLabel("", self)
        self._status_label = QLabel("Ready", self)
        self._extra_label = QLabel("", self)
        self._intensity_label = QLabel("50", self)
        self._level_label = QLabel("", self)
        self._gid_label = QLabel("", self)
        self._feedback_label = QLabel("", self)
        self._result_label = QLabel("", self)
        self._ros_label = QLabel(
            "Connected" if self._ros_connected else "Demo / disconnected", self
        )
        self._msg_label = self._feedback_label

        self._progress = QProgressBar(self)
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        self._progress.setTextVisible(True)
        for legacy_widget in (
            self._emotion_label, self._behavior_label, self._stage_label,
            self._action_label, self._status_label, self._extra_label,
            self._intensity_label, self._level_label, self._gid_label,
            self._feedback_label, self._result_label, self._ros_label,
            self._progress,
        ):
            legacy_widget.hide()

        self._setup_debug_overlay(h)

        # Demo
        self._demo_mode = False
        self._demo_behaviors: list[Any] = []
        self._demo_idx = 0
        self._show_visual("unknown", "")
        self._update_presentation()

    def _setup_debug_overlay(self, height_hint: int) -> None:
        self._debug_overlay = QWidget(self)
        self._debug_overlay.setAttribute(Qt.WA_StyledBackground, True)
        self._debug_overlay.setStyleSheet(
            "background-color: rgba(255, 250, 245, 225); "
            "border: 1px solid rgba(196, 154, 126, 150); "
            "border-radius: 18px;"
        )
        overlay_layout = QVBoxLayout(self._debug_overlay)
        overlay_layout.setContentsMargins(18, 14, 18, 14)
        overlay_layout.setSpacing(6)

        title = QLabel("Debug Overlay  ·  D / F1", self._debug_overlay)
        title.setStyleSheet(
            "color: #6a4c3e; background: transparent; border: none; "
            "font-weight: bold;"
        )
        title.setFont(QFont("Sans", max(14, height_hint // 42), QFont.Bold))
        overlay_layout.addWidget(title)

        self._debug_text = QLabel("", self._debug_overlay)
        self._debug_text.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self._debug_text.setWordWrap(True)
        self._debug_text.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self._debug_text.setStyleSheet(
            "color: #765c50; background: transparent; border: none;"
        )
        self._debug_text.setFont(QFont("Monospace", max(11, height_hint // 54)))
        overlay_layout.addWidget(self._debug_text, 1)
        self._debug_overlay.hide()
        self._refresh_debug_overlay()

    def _refresh_debug_overlay(self) -> None:
        if not hasattr(self, "_debug_text"):
            return
        lines = (
            ("Behavior", self._behavior_label.text()),
            ("Emotion", self._emotion_label.text()),
            ("Intensity", self._intensity_label.text()),
            ("Level", self._level_label.text()),
            ("Stage", self._stage_label.text()),
            ("Action", self._action_label.text()),
            ("Progress", f"{self._progress.value()}%"),
            ("Goal ID", self._gid_label.text()),
            ("Action Status", self._status_label.text()),
            ("Feedback", self._feedback_label.text()),
            ("Result", self._result_label.text()),
            ("ROS", self._ros_label.text()),
        )
        self._debug_text.setText(
            "\n".join(f"{label:<13} {value or '—'}" for label, value in lines)
        )

    def _setup_timer(self) -> None:
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._drain_queue)
        self._timer.start(33)  # ~30 FPS

    def keyPressEvent(self, event) -> None:
        """Provide kiosk-friendly full-screen keyboard controls."""
        if event.key() in (Qt.Key_D, Qt.Key_F1):
            self._debug_visible = not self._debug_visible
            self._debug_overlay.setVisible(self._debug_visible)
            if self._debug_visible:
                self._refresh_debug_overlay()
                self._position_debug_overlay()
                self._debug_overlay.raise_()
            event.accept()
            return
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

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._position_debug_overlay()

    def _position_debug_overlay(self) -> None:
        if not hasattr(self, "_debug_overlay"):
            return
        margin = 22
        overlay_width = min(470, max(320, int(self.width() * 0.36)))
        overlay_height = min(590, max(390, int(self.height() * 0.66)))
        self._debug_overlay.setGeometry(
            self.width() - overlay_width - margin,
            max(margin, (self.height() - overlay_height) // 2),
            overlay_width,
            overlay_height,
        )

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
        self._category = _classify(name, d.get("params"))
        self._current_stage = ""
        self._current_action = ""
        self._current_progress = 0.0
        self._current_status = "RUNNING"
        self._current_intensity = _clamp_intensity(d.get("intensity"))

        self._show_visual(self._category, name, self._current_intensity)
        self._behavior_label.setText(name)
        self._stage_label.setText("")
        self._action_label.setText("")
        self._progress.setValue(0)
        self._status_label.setText("Running")
        self._gid_label.setText(str(d.get("goal_id", "") or ""))
        self._feedback_label.setText("")
        self._result_label.setText("")
        self._intensity_label.setText(f"{self._current_intensity:.0f}")
        level = d.get("level")
        self._level_label.setText(str(level or ""))
        self._extra_label.setText(
            f"Intensity {self._current_intensity:.0f}"
            + (f"  |  Level {level}" if level else "")
        )
        self._update_presentation()
        self._refresh_debug_overlay()

    def _show_visual(
        self,
        cat: str,
        behavior_name: str,
        intensity: float | None = None,
    ) -> None:
        """Switch the live virtual-pet face to a behavior category."""
        self._category = cat if cat in EMOTION_MAP else "unknown"
        if intensity is not None:
            self._current_intensity = _clamp_intensity(intensity)
        if cat == "unknown" and behavior_name:
            self._emotion_label.setText(f"{_emoji(cat)} 执行中")
        else:
            self._emotion_label.setText(f"{_emoji(cat)} {_label(cat)}")

    def _update_presentation(self) -> None:
        face_state = build_face_state(
            self._category,
            behavior_name=self._active_behavior_name,
            stage=self._current_stage,
            action=self._current_action,
            status=self._current_status,
            intensity=self._current_intensity,
        )
        presentation = build_presentation_state(
            face_state,
            behavior_name=self._active_behavior_name,
            stage=self._current_stage,
            action=self._current_action,
            progress=self._current_progress,
        )
        self._face.set_execution(
            self._current_status,
            self._current_progress,
            self._current_stage,
        )
        self._face.set_display_state(
            face_state,
            presentation,
            self._active_behavior_name,
        )
        self._user_status_label.set_friendly_text(presentation.title)
        self._user_subtitle_label.setText(presentation.subtitle)
        self._soft_progress.set_progress(
            self._current_progress,
            presentation.show_progress,
        )

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

        self._current_stage = str(d.get("current_stage", "") or "")
        self._current_action = str(d.get("current_action", "") or "")
        try:
            progress = float(d.get("progress", 0.0))
        except (TypeError, ValueError):
            progress = 0.0
        self._current_progress = min(1.0, max(0.0, progress))
        self._current_status = "RUNNING"
        self._stage_label.setText(self._current_stage)
        self._action_label.setText(self._current_action)
        self._progress.setValue(int(self._current_progress * 100.0))
        self._status_label.setText("Running")
        self._feedback_label.setText(str(d.get("message", "") or ""))
        self._update_presentation()
        self._refresh_debug_overlay()

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

        status = str(d.get("status", "") or "").upper()
        self._current_progress = 1.0
        self._status_label.setText(status)
        self._progress.setValue(100)
        reason = str(d.get("reason", "") or "")
        result = str(d.get("result", "") or "")
        self._result_label.setText(
            "  |  ".join(part for part in (status, result, reason) if part)
        )

        alert_hold_ms = _SAFETY_ALERT_HOLD_MS.get(self._category, 0)
        # The engineering Result remains visible in Debug immediately. Normal
        # mode keeps the safety meaning instead of flashing generic success.
        self._current_status = "RUNNING" if alert_hold_ms else status
        self._update_presentation()
        self._refresh_debug_overlay()
        generation = self._display_generation
        QTimer.singleShot(
            alert_hold_ms or 2000,
            lambda: self._reset_to_ready(generation),
        )

    def _reset_to_ready(self, generation: int) -> None:
        """Leave a terminal result visible briefly, then clear stale behavior."""
        if generation != self._display_generation:
            return
        self._active_goal_id = ""
        self._active_behavior_name = ""
        self._category = "unknown"
        self._current_stage = ""
        self._current_action = ""
        self._current_progress = 0.0
        self._current_intensity = 50.0
        self._current_status = "READY"
        self._show_visual("unknown", "")
        self._behavior_label.setText("Waiting...")
        self._stage_label.setText("")
        self._action_label.setText("")
        self._progress.setValue(0)
        self._status_label.setText("Ready")
        self._gid_label.setText("")
        self._feedback_label.setText("")
        self._result_label.setText("")
        self._intensity_label.setText("50")
        self._level_label.setText("")
        self._extra_label.setText("")
        self._update_presentation()
        self._refresh_debug_overlay()

    # --- Demo ---

    def set_demo_mode(self, behaviors: list[Any]) -> None:
        self._demo_mode = True
        self._demo_behaviors = behaviors
        self._demo_idx = 0
        self._start_demo()

    def _start_demo(self) -> None:
        if not self._demo_mode:
            return
        if self._demo_behaviors:
            scenario = self._demo_behaviors[
                self._demo_idx % len(self._demo_behaviors)
            ]
            self._demo_idx += 1
            if isinstance(scenario, str):
                scenario = {"behavior_name": scenario}

            if scenario.get("ready"):
                self._display_generation += 1
                self._reset_to_ready(self._display_generation)
            else:
                name = str(scenario.get("behavior_name", "expressCalmAlone"))
                goal_id = f"demo-{self._demo_idx:02d}"
                self._apply_goal({
                    "goal_id": goal_id,
                    "behavior_name": name,
                    "intensity": scenario.get("intensity", 55.0),
                })
                self._apply_feedback({
                    "goal_id": goal_id,
                    "behavior_name": name,
                    "current_stage": scenario.get("stage", "expression"),
                    "current_action": scenario.get(
                        "action", f"ACT_{_classify(name).upper()}_DEMO"
                    ),
                    "progress": scenario.get("progress", 0.55),
                    "message": f"Demo: {scenario.get('label', name)}",
                })
                terminal_status = scenario.get("result_status")
                if terminal_status:
                    QTimer.singleShot(
                        1900,
                        lambda gid=goal_id, behavior=name, terminal=terminal_status: self._apply_result({
                            "goal_id": gid,
                            "behavior_name": behavior,
                            "status": terminal,
                            "result": "demo",
                            "reason": "Demo terminal state",
                        }),
                    )
        QTimer.singleShot(4300, self._start_demo)


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
                d["params"] = params
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

_DEMO_BEHAVIORS: list[dict[str, Any]] = [
    {"ready": True, "label": "Ready"},
    {"behavior_name": "expressJoyAlone", "intensity": 32, "label": "Joy low"},
    {"behavior_name": "expressJoyWithHuman", "intensity": 88, "label": "Joy high"},
    {"behavior_name": "expressExcitementAlone", "intensity": 82, "label": "Excited"},
    {
        "behavior_name": "exploreRoom", "intensity": 67,
        "stage": "perception search", "action": "ACT_SEARCH_DEMO",
        "label": "Curious + Exploring",
    },
    {
        "behavior_name": "follow_owner", "intensity": 64,
        "action": "ACT_INTERACT_FOLLOW_OWNER", "label": "Following",
    },
    {"behavior_name": "seekHumanInteraction", "intensity": 62, "label": "Social"},
    {
        "behavior_name": "wait_in_place", "intensity": 42,
        "action": "ACT_WAIT_DEMO", "label": "Listening",
    },
    {"behavior_name": "expressAnxietyAlone", "intensity": 38, "label": "Anxiety low"},
    {"behavior_name": "expressAnxietyAlone", "intensity": 88, "label": "Anxiety high"},
    {"behavior_name": "expressFearAlone", "intensity": 78, "label": "Fear"},
    {"behavior_name": "eatNormally", "intensity": 58, "label": "Eating"},
    {"behavior_name": "sleepOnSide", "intensity": 35, "label": "Sleeping"},
    {"behavior_name": "recharge", "intensity": 35, "label": "Charging"},
    {
        "behavior_name": "expressJoyAlone", "intensity": 72,
        "result_status": "SUCCESS", "label": "Success",
    },
    {
        "behavior_name": "inspectObject", "intensity": 45,
        "result_status": "FAILED", "label": "Failed",
    },
    {
        "behavior_name": "respond_person_fall", "intensity": 92,
        "action": "ACT_PERCEPTION_RESPOND_PERSON_FALL", "label": "Emergency",
    },
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

        window = EmotionWindow(event_queue=event_queue, ros_connected=True)
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
        window = EmotionWindow(event_queue=event_queue, ros_connected=False)
        window.showFullScreen()
        window.set_demo_mode(_DEMO_BEHAVIORS)
        app.exec_()


if __name__ == "__main__":
    main()
