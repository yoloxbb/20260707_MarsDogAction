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
)

if HAS_QT:
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
