"""Emotion Display - Qt5 visualization node showing dog emotion and behavior.

Usage::

    ros2 run marsdog_action_executor emotion_display

Requires: PySide2 (``sudo apt install python3-pyside2.qtwidgets``)

Images are loaded from ``config/emotion_images/`` relative to the package
share directory, with fallback to a local source tree.
"""

from __future__ import annotations

import json
import logging
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
    from PySide2.QtCore import Qt, QTimer
    from PySide2.QtGui import QFont, QColor, QPalette, QPixmap
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

DARK_BG = "#1a1a2e"


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
        self._pixmap_cache: dict[str, Any] = {}
        self._current_pixmap: Any = None
        self._setup_ui()
        self._setup_timer()

    def _setup_ui(self) -> None:
        self.setWindowTitle("MarsDog Emotion Display")
        self.setMinimumSize(640, 520)

        pal = self.palette()
        pal.setColor(QPalette.Window, QColor(DARK_BG))
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
        layout.setSpacing(6)

        # --- Image label (replaces emoji when image available) ---
        self._image_label = QLabel()
        self._image_label.setAlignment(Qt.AlignCenter)
        self._image_label.setStyleSheet(f"background: {DARK_BG};")
        self._image_label.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Expanding
        )
        self._image_label.setMinimumHeight(max(280, int(h * 0.52)))
        self._image_label.setScaledContents(False)
        self._image_label.setMaximumHeight(max(520, int(h * 0.68)))
        layout.addWidget(self._image_label, 1)

        # --- Emotion label ---
        self._emotion_label = QLabel(_label("unknown"))
        self._emotion_label.setAlignment(Qt.AlignCenter)
        self._emotion_label.setStyleSheet(
            f"color: white; background: {DARK_BG}; font-weight: bold;"
        )
        self._emotion_label.setFont(QFont("Sans", max(28, h // 18)))
        layout.addWidget(self._emotion_label)

        # --- Behavior name ---
        self._behavior_label = QLabel("Waiting...")
        self._behavior_label.setAlignment(Qt.AlignCenter)
        self._behavior_label.setStyleSheet(f"color: #cccccc; background: {DARK_BG};")
        self._behavior_label.setFont(QFont("Sans", max(20, h // 25)))
        layout.addWidget(self._behavior_label)

        # --- Current action ---
        self._action_label = QLabel("")
        self._action_label.setAlignment(Qt.AlignCenter)
        self._action_label.setStyleSheet(f"color: #88ff88; background: {DARK_BG};")
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
                border: 1px solid #444; border-radius: 4px;
                background: #222; height: 18px;
            }
            QProgressBar::chunk { background: #FFD700; border-radius: 3px; }
        """)
        layout.addWidget(self._progress)

        # --- Info bar ---
        info_layout = QHBoxLayout()

        self._status_label = QLabel("Ready")
        self._status_label.setStyleSheet(f"color: #88ff88; background: {DARK_BG};")
        self._status_label.setFont(QFont("Sans", max(14, h // 35)))
        info_layout.addWidget(self._status_label)

        info_layout.addStretch()
        self._extra_label = QLabel("")
        self._extra_label.setStyleSheet(f"color: #aaaaaa; background: {DARK_BG};")
        self._extra_label.setFont(QFont("Sans", max(14, h // 35)))
        info_layout.addWidget(self._extra_label)

        info_layout.addStretch()
        self._gid_label = QLabel("")
        self._gid_label.setStyleSheet(f"color: #666666; background: {DARK_BG};")
        self._gid_label.setFont(QFont("Sans", max(12, h // 35)))
        info_layout.addWidget(self._gid_label)

        layout.addLayout(info_layout)

        # --- Message ---
        self._msg_label = QLabel("")
        self._msg_label.setAlignment(Qt.AlignCenter)
        self._msg_label.setWordWrap(True)
        self._msg_label.setStyleSheet(f"color: #777777; background: {DARK_BG};")
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
        name = d.get("behavior_name", "")
        cat = _classify(name, d.get("params"))
        color = _color(cat)

        self._show_visual(cat, name)

        self._emotion_label.setStyleSheet(
            f"color: {color}; background: {DARK_BG}; font-weight: bold;"
        )
        self._behavior_label.setText(name)
        self._action_label.setText("")
        self._progress.setValue(0)
        self._status_label.setText("Running")
        self._status_label.setStyleSheet(f"color: #FFD700; background: {DARK_BG};")
        self._gid_label.setText(d.get("goal_id", ""))
        self._msg_label.setText("")

        parts = []
        intensity = d.get("intensity")
        if intensity is not None:
            parts.append(f"Intensity {intensity:.0f}")
        level = d.get("level")
        if level:
            parts.append(f"Level {level}")
        self._extra_label.setText("  |  ".join(parts) if parts else "")

    def _show_visual(self, cat: str, behavior_name: str) -> None:
        """Show a category image or a non-empty generated fallback card."""
        img_path = _image_path(cat)
        if img_path:
            pixmap = self._pixmap_cache.get(img_path)
            if pixmap is None:
                pixmap = QPixmap(img_path)
                self._pixmap_cache[img_path] = pixmap
            if not pixmap.isNull():
                self._current_pixmap = pixmap
                self._image_label.setText("")
                self._image_label.setStyleSheet(
                    f"background: {DARK_BG}; border: none;"
                )
                self._rescale_current_pixmap()
                self._emotion_label.setText(f"{_emoji(cat)} {_label(cat)}")
                return

            logger.warning("Cannot load visualization image: %s", img_path)

        # Asset missing/corrupt or an unforeseen category: never leave the
        # main visual region blank.  This card remains useful on minimal
        # deployments where only code, not image assets, was installed.
        self._current_pixmap = None
        self._image_label.clear()
        card_name = behavior_name or "MarsDog"
        self._image_label.setText(
            f"{_emoji(cat)}\n{_label(cat)}\n{card_name}"
        )
        self._image_label.setFont(QFont("Sans", 28))
        self._image_label.setStyleSheet(
            f"color: {_color(cat)}; background: #222238; "
            f"border: 3px solid {_color(cat)}; border-radius: 16px; "
            "font-weight: bold; padding: 16px;"
        )
        self._emotion_label.setText(f"{_emoji(cat)} {_label(cat)}")

    def _rescale_current_pixmap(self) -> None:
        """Fit the current asset to the full image panel without distortion."""
        pixmap = self._current_pixmap
        if pixmap is None or pixmap.isNull():
            return
        scaled = pixmap.scaled(
            max(1, self._image_label.width()),
            max(1, self._image_label.height()),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self._image_label.setPixmap(scaled)

    def resizeEvent(self, event) -> None:
        """Keep the image large and sharp when the window is resized."""
        super().resizeEvent(event)
        self._rescale_current_pixmap()

    def _apply_feedback(self, d: dict[str, Any]) -> None:
        stage = d.get("current_stage", "")
        act = d.get("current_action", "")
        progress = float(d.get("progress", 0.0))
        self._action_label.setText(f"{stage}  ->  {act}")
        self._progress.setValue(int(min(100.0, max(0.0, progress * 100.0))))
        self._msg_label.setText(d.get("message", ""))

    def _apply_result(self, d: dict[str, Any]) -> None:
        status = d.get("status", "")
        if status == "SUCCESS":
            self._status_label.setText("Completed")
            self._status_label.setStyleSheet(f"color: #88ff88; background: {DARK_BG};")
        elif status in ("CANCELED", "FAILED"):
            self._status_label.setText(status)
            self._status_label.setStyleSheet(f"color: #FF6B6B; background: {DARK_BG};")
        else:
            self._status_label.setText(status)
        self._progress.setValue(100)
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
        self._show_visual("unknown", "")
        self._emotion_label.setStyleSheet(
            f"color: {_color('unknown')}; background: {DARK_BG}; "
            "font-weight: bold;"
        )
        self._behavior_label.setText("Waiting...")
        self._action_label.setText("")
        self._progress.setValue(0)
        self._status_label.setText("Ready")
        self._status_label.setStyleSheet(
            f"color: #88ff88; background: {DARK_BG};"
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
        window.show()

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
        window.show()
        window.set_demo_mode(_DEMO_BEHAVIORS)
        app.exec_()


if __name__ == "__main__":
    main()
