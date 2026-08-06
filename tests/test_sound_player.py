"""Tests for exact behavior-to-audio routing and playback lifecycle."""

from __future__ import annotations

from pathlib import Path
from urllib.parse import unquote

from marsdog_action_executor.config_loader import ConfigLoader
from marsdog_action_executor.sound_player import (
    BehaviorSoundController,
    SoundPlayer,
)


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


class RecordingPlayer:
    instances: list["RecordingPlayer"] = []

    def __init__(self, filepath: str | Path) -> None:
        self.filepath = Path(filepath)
        self.play_count = 0
        self.stop_count = 0
        self.instances.append(self)

    def play(self) -> bool:
        self.play_count += 1
        return True

    def stop(self) -> None:
        self.stop_count += 1


def _controller() -> BehaviorSoundController:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()
    RecordingPlayer.instances.clear()
    return BehaviorSoundController(
        loader.sound_config,
        loader.config_dir,
        player_factory=RecordingPlayer,
    )


def test_emotion_behavior_sound_mapping_matches_file_semantics() -> None:
    controller = _controller()
    expected = {
        "expressCuriosityWithHuman": "好奇.mp3",
        "expressCuriosityAlone": "好奇.mp3",
        "expressFearWithHuman": "恐惧.wav",
        "expressFearAlone": "恐惧.wav",
        "expressAnxietyWithHuman": "焦虑.wav",
        "expressAnxietyAlone": "焦虑.wav",
        "expressExcitementWithHuman": "兴奋愉悦.wav",
        "expressExcitementAlone": "兴奋愉悦.wav",
        "expressJoyWithHuman": "兴奋愉悦.wav",
        "expressJoyAlone": "兴奋愉悦.wav",
    }

    for behavior_name, file_name in expected.items():
        path = controller.sound_path_for(behavior_name)
        assert path is not None
        assert path.name == file_name
        assert path.is_file()

    assert controller.sound_path_for("sleepOnSide") is None


def test_new_behavior_sound_replaces_and_stop_terminates_active_audio() -> None:
    controller = _controller()

    first_path = controller.play_for("expressCuriosityAlone")
    first = RecordingPlayer.instances[-1]
    second_path = controller.play_for("expressFearAlone")
    second = RecordingPlayer.instances[-1]

    assert first_path is not None and first_path.name == "好奇.mp3"
    assert second_path is not None and second_path.name == "恐惧.wav"
    assert first.play_count == 1
    assert first.stop_count == 1
    assert second.play_count == 1
    assert second.stop_count == 0

    controller.stop()
    assert second.stop_count == 1


def test_player_commands_support_packaged_mp3_and_wav() -> None:
    mp3 = SoundPlayer(CONFIG_DIR / "sounds" / "好奇.mp3")
    wav = SoundPlayer(CONFIG_DIR / "sounds" / "恐惧.wav")

    assert mp3._build_command(["gst-launch-1.0"])[2] == "playbin"
    assert "好奇.mp3" in unquote(mp3._build_command(["gst-launch-1.0"])[3])
    assert wav._build_command(["ffmpeg"])[5].endswith("恐惧.wav")
