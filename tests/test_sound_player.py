"""Tests for exact behavior-to-audio routing and playback lifecycle."""

from __future__ import annotations

from pathlib import Path
from urllib.parse import unquote

import pytest

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
        "expressJoyInPlaceWithHuman": "兴奋愉悦.wav",
        "expressExcitementInPlaceWithHuman": "兴奋愉悦.wav",
        "expressAnxietyInPlaceWithHuman": "焦虑.wav",
        "expressFearInPlaceWithHuman": "恐惧.wav",
        "expressCuriosityInPlaceWithHuman": "好奇.mp3",
    }

    for behavior_name, file_name in expected.items():
        path = controller.sound_path_for(behavior_name)
        assert path is not None
        assert path.name == file_name
        assert path.is_file()

    assert controller.sound_path_for("sleepOnSide") is None
    assert controller.sound_path_for("expressCalmInPlaceWithHuman") is None


def test_feeding_behaviors_share_the_eating_sound() -> None:
    controller = _controller()

    for behavior_name in (
        "eatNormally",
        "eatExcitedly",
        "seekFood",
        "seekFoodUrgently",
        "inspectDogFood",
    ):
        path = controller.sound_path_for(behavior_name)
        assert path is not None, behavior_name
        assert path.name == "进食.mp3"
        assert path.is_file()


def test_stage_sound_covers_exactly_the_two_stages_dog_is_down() -> None:
    controller = _controller()

    for behavior_name in ("sleepOnSide", "sleepNow"):
        for stage_id in ("sleep_pose", "sleeping"):
            path = controller.stage_sound_path(behavior_name, stage_id)
            assert path is not None, f"{behavior_name}.{stage_id}"
            assert path.name == "睡觉呼噜.mp3"
            assert path.is_file()

        # The dog is on its feet and moving through these, so it must not
        # snore: circle is the pre-sleep turn and wakeup is the way back up.
        for stage_id in ("circle", "prepare", "wakeup"):
            assert controller.stage_sound_path(behavior_name, stage_id) is None

    # Sleep is deliberately absent from behavior_sounds: snoring is a Stage
    # property, not a behavior-wide one. Listing it there would also make the
    # dog snore through circle / prepare / wakeup. The two requirements use
    # two different mechanisms on purpose.
    assert controller.sound_path_for("sleepOnSide") is None
    assert controller.sound_path_for("sleepNow") is None


def test_stage_sound_survives_config_validation() -> None:
    loader = ConfigLoader(CONFIG_DIR)
    loader.load_all()

    assert loader.sound_config["stage_sounds"]["sleepNow"]["sleep_pose"] == (
        "sounds/睡觉呼噜.mp3"
    )


def test_stage_sound_replaces_behavior_audio_and_stops_on_exit() -> None:
    controller = _controller()

    assert controller.play_for("expressJoyAlone") is not None
    behavior_player = RecordingPlayer.instances[-1]

    with controller.stage_sound("sleepOnSide", "sleep_pose") as path:
        assert path is not None and path.name == "睡觉呼噜.mp3"
        stage_player = RecordingPlayer.instances[-1]
        # The single-stream rule holds: the Stage sound replaced the
        # behavior-level player instead of layering a second process.
        assert stage_player is not behavior_player
        assert behavior_player.stop_count == 1
        assert stage_player.play_count == 1
        assert stage_player.stop_count == 0

    assert stage_player.stop_count == 1


def test_stage_sound_stops_when_the_stage_body_raises() -> None:
    controller = _controller()

    with pytest.raises(RuntimeError):
        with controller.stage_sound("sleepNow", "sleep_pose"):
            raise RuntimeError("stage blew up")

    assert RecordingPlayer.instances[-1].stop_count == 1


def test_stage_sound_leaves_a_replaced_stream_alone() -> None:
    controller = _controller()

    with controller.stage_sound("sleepOnSide", "sleep_pose"):
        stage_player = RecordingPlayer.instances[-1]
        # Something else takes the stream mid-Stage — quiet, a preemption, or a
        # behavior-level start. The Stage exit must not stop that stream: the
        # identity check, not a path comparison, is what makes this safe.
        controller.play_for("expressFearAlone")
        replacement = RecordingPlayer.instances[-1]

    assert stage_player.stop_count == 1
    assert replacement.stop_count == 0


def test_unconfigured_stage_sound_yields_none_without_playing() -> None:
    controller = _controller()

    with controller.stage_sound("sleepOnSide", "wakeup") as path:
        assert path is None
    with controller.stage_sound("expressJoyAlone", "sleep_pose") as path:
        assert path is None

    assert RecordingPlayer.instances == []


def test_disabled_switch_silences_behavior_and_stage_audio(tmp_path: Path) -> None:
    controller = BehaviorSoundController(
        {
            "enabled": False,
            "file": "sounds/好奇.mp3",
            "voice_command_behaviors": ["stand_up"],
            "behavior_sounds": {"eatNormally": "sounds/进食.mp3"},
            "stage_sounds": {
                "sleepOnSide": {"sleep_pose": "sounds/睡觉呼噜.mp3"}
            },
        },
        tmp_path,
    )

    assert controller.sound_path_for("eatNormally") is None
    assert controller.sound_path_for("stand_up") is None
    assert controller.stage_sound_path("sleepOnSide", "sleep_pose") is None
    with controller.stage_sound("sleepOnSide", "sleep_pose") as path:
        assert path is None


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


def test_quiet_stops_active_audio_without_starting_confirmation_sound() -> None:
    controller = _controller()
    assert controller.play_for("stand_up") is not None
    active = RecordingPlayer.instances[-1]

    assert controller.apply_control_behavior("quiet")
    assert active.stop_count == 1
    assert controller.sound_path_for("quiet") is None
    assert len(RecordingPlayer.instances) == 1


def test_player_commands_support_packaged_mp3_and_wav() -> None:
    mp3 = SoundPlayer(CONFIG_DIR / "sounds" / "好奇.mp3")
    wav = SoundPlayer(CONFIG_DIR / "sounds" / "恐惧.wav")

    assert mp3._build_command(["gst-launch-1.0"])[2] == "playbin"
    assert "好奇.mp3" in unquote(mp3._build_command(["gst-launch-1.0"])[3])
    assert wav._build_command(["ffmpeg"])[5].endswith("恐惧.wav")
