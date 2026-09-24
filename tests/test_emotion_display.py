import os
from pathlib import Path

import pytest
import yaml

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from marsdog_action_executor.emotion_display import (
    EMOTION_MAP,
    HAS_QT,
    EmotionWindow,
    _classify,
    _face_profile,
    _image_path,
    _visual_category,
    build_face_state,
    build_presentation_state,
)

if HAS_QT:
    from PySide2.QtCore import QEvent, Qt
    from PySide2.QtGui import QImage, QKeyEvent
    from PySide2.QtWidgets import QApplication


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


def test_every_visual_category_has_an_installed_image() -> None:
    for category, spec in EMOTION_MAP.items():
        assert spec["image"], category
        image_path = Path(_image_path(category))
        assert image_path.is_file(), (category, image_path)
        assert image_path.stat().st_size > 0


def test_every_visual_category_has_a_complete_animation_profile() -> None:
    required = {"eyes", "smile", "ears", "bob", "shake", "tilt", "speed"}

    for category in EMOTION_MAP:
        profile = _face_profile(category)
        assert set(profile) == required
        assert 0.0 <= profile["eyes"] <= 1.0
        assert profile["speed"] > 0.0


def test_every_behavior_template_resolves_to_a_visual_category() -> None:
    raw = yaml.safe_load(
        (CONFIG_DIR / "behavior_tree_actions.yaml").read_text(encoding="utf-8")
    )
    behaviors = raw.get("behaviors", raw)

    unresolved = [name for name in behaviors if _classify(name) == "unknown"]

    assert unresolved == []


def test_goal_metadata_classifies_unfamiliar_names() -> None:
    assert _classify(
        "future_voice_behavior",
        {"source": "audio_direct"},
    ) == "command"
    assert _classify(
        "future_emotion_behavior",
        {"source_emotion": "Excite"},
    ) == "excitement"


def test_safety_behaviors_keep_distinct_visual_categories() -> None:
    assert _classify("respond_person_fall") == "fall_alert"
    assert _classify("respond_stop_gesture") == "stop_alert"
    assert _classify("emergency_stop") == "emergency"


def test_owner_visual_behaviors_have_distinct_friendly_categories() -> None:
    assert _classify("unhappy") == "anxiety"
    assert _classify("miss_owner") == "excitement"
    assert _classify("farewell_leave") == "social"

    farewell = build_face_state(
        "social",
        behavior_name="farewell_leave",
        status="RUNNING",
    )
    assert farewell.activity == "following"
    presentation = build_presentation_state(
        farewell,
        behavior_name="farewell_leave",
        progress=0.5,
    )
    assert presentation.title == "送送你"
    assert presentation.face_cue == "follow"


def test_split_face_state_keeps_emotion_activity_and_system_independent() -> None:
    state = build_face_state(
        "anxiety",
        behavior_name="future_behavior",
        stage="perception search",
        action="ACT_SEARCH",
        status="RUNNING",
        intensity=145,
    )

    assert state.emotion == "anxious"
    assert state.activity == "searching"
    assert state.system_state == "running"
    assert state.intensity == 100.0
    assert _visual_category(state) == "anxiety"


def test_presentation_mapping_uses_friendly_copy_and_face_cues() -> None:
    following = build_face_state(
        "command", behavior_name="follow_owner", status="RUNNING"
    )
    following_ui = build_presentation_state(
        following, behavior_name="follow_owner", progress=0.5
    )
    assert following_ui.title == "正在跟着你"
    assert following_ui.face_cue == "follow"
    assert following_ui.show_progress is True

    listening = build_face_state(
        "command",
        behavior_name="wait_in_place",
        action="ACT_WAIT",
        status="RUNNING",
    )
    listening_ui = build_presentation_state(
        listening,
        behavior_name="wait_in_place",
        action="ACT_WAIT",
    )
    assert listening_ui.title == "在听"
    assert listening_ui.face_cue == "listening"


def test_fall_alert_has_distinct_warning_copy_and_can_complete() -> None:
    running = build_face_state(
        "fall_alert", behavior_name="respond_person_fall", status="RUNNING"
    )
    completed = build_face_state(
        "fall_alert", behavior_name="respond_person_fall", status="SUCCESS"
    )

    assert running.system_state == "emergency"
    assert running.alert_type == "fall_alert"
    assert _visual_category(running) == "fall_alert"
    presentation = build_presentation_state(
        running, behavior_name="respond_person_fall"
    )
    assert presentation.title == "检测到有人跌倒"
    assert presentation.subtitle == "小车已停车，正在关注"
    assert presentation.face_cue == "fall_alert"
    assert completed.system_state == "completed"
    assert build_presentation_state(completed).face_cue == "success"


def test_stop_gesture_and_emergency_stop_have_distinct_copy() -> None:
    stop_gesture = build_face_state("stop_alert", status="RUNNING")
    emergency = build_face_state("emergency", status="RUNNING")

    stop_ui = build_presentation_state(stop_gesture)
    emergency_ui = build_presentation_state(emergency)

    assert _visual_category(stop_gesture) == "stop_alert"
    assert stop_ui.title == "检测到停止手势"
    assert stop_ui.subtitle == "小车已停止"
    assert emergency_ui.title == "紧急停车"
    assert emergency_ui.subtitle == "请检查周围环境"


@pytest.mark.skipif(not HAS_QT, reason="PySide2 is not installed")
def test_short_fall_result_keeps_warning_visible_and_debug_result_exact() -> None:
    app = QApplication.instance() or QApplication([])
    window = EmotionWindow()
    window._apply_goal({
        "goal_id": "fall-1",
        "behavior_name": "respond_person_fall",
        "intensity": 100,
    })

    window._apply_result({
        "goal_id": "fall-1",
        "behavior_name": "respond_person_fall",
        "status": "SUCCESS",
        "result": "completed",
    })

    assert window._category == "fall_alert"
    assert window._current_status == "RUNNING"
    assert window._status_label.text() == "SUCCESS"
    assert "SUCCESS" in window._result_label.text()
    assert window._face._face_state.system_state == "emergency"
    assert window._face._presentation.title == "检测到有人跌倒"
    assert window._face._presentation.face_cue == "fall_alert"
    window.close()
    app.processEvents()


@pytest.mark.skipif(not HAS_QT, reason="PySide2 is not installed")
def test_stale_result_does_not_reset_preempting_goal() -> None:
    app = QApplication.instance() or QApplication([])
    window = EmotionWindow()
    window._apply_goal({"goal_id": "old", "behavior_name": "recharge"})
    window._apply_goal({"goal_id": "new", "behavior_name": "sleepOnSide"})

    window._apply_result({
        "goal_id": "old",
        "behavior_name": "recharge",
        "status": "CANCELED",
    })

    assert window._active_goal_id == "new"
    assert window._active_behavior_name == "sleepOnSide"
    assert "休息" in window._emotion_label.text()
    assert window._status_label.text() == "Running"
    window.close()
    app.processEvents()


@pytest.mark.skipif(not HAS_QT, reason="PySide2 is not installed")
def test_feedback_recovers_a_goal_missed_by_late_display_start() -> None:
    app = QApplication.instance() or QApplication([])
    window = EmotionWindow()

    window._apply_feedback({
        "goal_id": "running-goal",
        "behavior_name": "expressJoyAlone",
        "current_stage": "expression",
        "current_action": "ACT_WAG_TAIL",
        "progress": 0.4,
    })

    assert window._active_goal_id == "running-goal"
    assert window._active_behavior_name == "expressJoyAlone"
    assert "喜悦" in window._emotion_label.text()
    assert window._progress.value() == 40
    window.close()
    app.processEvents()


@pytest.mark.skipif(not HAS_QT, reason="PySide2 is not installed")
def test_unclassified_active_behavior_is_not_labeled_ready() -> None:
    app = QApplication.instance() or QApplication([])
    window = EmotionWindow()

    window._apply_goal({"goal_id": "future", "behavior_name": "futureBehavior"})

    assert "执行中" in window._emotion_label.text()
    assert "就绪" not in window._emotion_label.text()
    window.close()
    app.processEvents()


@pytest.mark.skipif(not HAS_QT, reason="PySide2 is not installed")
def test_normal_mode_hides_engineering_fields_and_debug_key_reveals_overlay() -> None:
    app = QApplication.instance() or QApplication([])
    window = EmotionWindow(ros_connected=True)
    window.show()
    app.processEvents()

    assert window._debug_overlay.isVisible() is False
    assert window._behavior_label.isVisible() is False
    assert window._user_status_label.isVisible() is True

    window.keyPressEvent(QKeyEvent(QEvent.KeyPress, Qt.Key_D, Qt.NoModifier))
    app.processEvents()
    assert window._debug_overlay.isVisible() is True
    assert "ROS           Connected" in window._debug_text.text()

    window.keyPressEvent(QKeyEvent(QEvent.KeyPress, Qt.Key_F1, Qt.NoModifier))
    app.processEvents()
    assert window._debug_overlay.isVisible() is False
    window.close()
    app.processEvents()


@pytest.mark.skipif(not HAS_QT, reason="PySide2 is not installed")
def test_feedback_drives_friendly_copy_face_cue_and_debug_state() -> None:
    app = QApplication.instance() or QApplication([])
    window = EmotionWindow()
    window._apply_goal({
        "goal_id": "follow-1",
        "behavior_name": "follow_owner",
        "intensity": 80,
    })
    window._apply_feedback({
        "goal_id": "follow-1",
        "behavior_name": "follow_owner",
        "current_stage": "tracking",
        "current_action": "ACT_INTERACT_FOLLOW_OWNER",
        "progress": 0.4,
        "message": "tracking owner",
    })

    assert window._user_status_label.text() == "正在跟着你"
    assert window._face._presentation.face_cue == "follow"
    assert window._face._intensity == 80.0
    assert window._stage_label.text() == "tracking"
    assert window._action_label.text() == "ACT_INTERACT_FOLLOW_OWNER"
    assert window._progress.value() == 40
    assert "tracking owner" in window._debug_text.text()
    window.close()
    app.processEvents()


@pytest.mark.skipif(not HAS_QT, reason="PySide2 is not installed")
def test_intensity_changes_bounded_facial_shape_and_motion_targets() -> None:
    app = QApplication.instance() or QApplication([])
    window = EmotionWindow()

    window._apply_goal({"behavior_name": "expressJoyAlone", "intensity": 10})
    low = dict(window._face._target_profile)
    window._apply_goal({"behavior_name": "expressJoyAlone", "intensity": 95})
    high = dict(window._face._target_profile)

    assert high["smile"] > low["smile"]
    assert high["eyes"] < low["eyes"]
    assert high["bob"] > low["bob"]
    assert high["speed"] > low["speed"]
    assert 0.0 <= high["eyes"] <= 1.0
    window.close()
    app.processEvents()


@pytest.mark.skipif(not HAS_QT, reason="PySide2 is not installed")
def test_gaze_and_blink_remain_bounded_and_use_staggered_lids() -> None:
    app = QApplication.instance() or QApplication([])
    window = EmotionWindow()
    window._apply_goal({"behavior_name": "follow_owner", "intensity": 70})

    for phase in (0.0, 1.0, 3.5, 8.0):
        window._face._phase = phase
        gaze_x, gaze_y = window._face._gaze_target()
        assert abs(gaze_x) <= 8.0
        assert abs(gaze_y) <= 2.0

    window._face._category = "unknown"
    window._face._blink_frames = 4
    assert window._face._blink_cover(-1.0) > window._face._blink_cover(1.0)
    window._face._category = "sleep"
    assert window._face._blink_cover(-1.0) == pytest.approx(0.94)
    window.close()
    app.processEvents()


@pytest.mark.skipif(not HAS_QT, reason="PySide2 is not installed")
def test_classic_pet_face_renders_key_expressions_offscreen() -> None:
    app = QApplication.instance() or QApplication([])
    window = EmotionWindow()
    window.resize(960, 640)
    window.show()

    for behavior, intensity in (
        ("follow_owner", 70),
        ("expressJoyAlone", 85),
        ("expressAnxietyAlone", 75),
        ("sleepOnSide", 55),
        ("respond_person_fall", 100),
    ):
        window._apply_goal({"behavior_name": behavior, "intensity": intensity})
        window._face._visual_profile = dict(window._face._target_profile)
        app.processEvents()
        image = window.grab().toImage().convertToFormat(QImage.Format_ARGB32)
        assert image.isNull() is False
        assert image.width() == 960
        assert image.height() == 640

    window.close()
    app.processEvents()
