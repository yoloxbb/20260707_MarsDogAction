from pathlib import Path

import yaml

from marsdog_action_executor.emotion_display import (
    EMOTION_MAP,
    _classify,
    _image_path,
)


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


def test_every_visual_category_has_an_installed_image() -> None:
    for category, spec in EMOTION_MAP.items():
        assert spec["image"], category
        image_path = Path(_image_path(category))
        assert image_path.is_file(), (category, image_path)
        assert image_path.stat().st_size > 0


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
