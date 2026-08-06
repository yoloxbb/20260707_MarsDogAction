"""Asynchronous audio playback for behavior execution.

Uses GStreamer (``gst-launch-1.0``) for non-blocking MP3/WAV playback.
Falls back to ``ffmpeg`` if GStreamer is unavailable.
"""

from __future__ import annotations

import logging
import os
import shutil
import subprocess
import threading
from pathlib import Path
from typing import Any, Callable, Mapping

logger = logging.getLogger(__name__)

def _find_player() -> list[str] | None:
    """Return the first available CLI audio-player command.

    Returns a prefix list (e.g. ``["gst-launch-1.0", "playbin", "uri=file://…"]``
    is built by the caller).  We only check that the binary exists.
    """
    if shutil.which("gst-launch-1.0"):
        return ["gst-launch-1.0"]
    if shutil.which("ffmpeg"):
        return ["ffmpeg"]
    return None


class SoundPlayer:
    """Plays a sound file asynchronously and optionally stops it early.

    Usage::

        player = SoundPlayer("/path/to/bark.mp3")
        player.play()
        # ... behavior executes ...
        player.stop()
    """

    def __init__(self, filepath: str | Path) -> None:
        self._filepath = str(filepath)
        self._process: subprocess.Popen | None = None
        self._lock = threading.Lock()

    # ── public API ──────────────────────────────────────────────────────

    @property
    def filepath(self) -> str:
        return self._filepath

    def play(self) -> bool:
        """Start playback and return without waiting for audio completion."""
        if not os.path.exists(self._filepath):
            logger.error("Sound file not found: %s", self._filepath)
            return False

        player_prefix = _find_player()
        if player_prefix is None:
            logger.warning(
                "No audio player found (gst-launch-1.0 or ffmpeg); "
                "cannot play behavior sound."
            )
            return False

        self.stop()
        cmd = self._build_command(player_prefix)
        logger.debug("Sound player: %s", " ".join(cmd))
        try:
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            logger.exception("Sound playback failed")
            return False

        with self._lock:
            self._process = process

        def _run() -> None:
            try:
                process.wait()
            except Exception:
                logger.exception("Sound playback failed")
            finally:
                with self._lock:
                    if self._process is process:
                        self._process = None

        thread = threading.Thread(target=_run, daemon=True, name="sound-player")
        thread.start()
        return True

    def stop(self) -> None:
        """Terminate playback early (best-effort, no-op if idle)."""
        with self._lock:
            proc = self._process
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=2.0)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass

    # ── internal ─────────────────────────────────────────────────────────

    def _build_command(self, player_prefix: list[str]) -> list[str]:
        """Build the full CLI command for the detected player."""
        if player_prefix[0] == "gst-launch-1.0":
            return [
                "gst-launch-1.0",
                "-q",  # quiet
                "playbin",
                f"uri={Path(self._filepath).resolve().as_uri()}",
            ]
        # ffmpeg fallback
        return [
            "ffmpeg",
            "-nostdin",
            "-loglevel", "quiet",
            "-i", self._filepath,
            "-f", "pulse",
            "-v", "0",
            "default",
        ]


class BehaviorSoundController:
    """Select and own the single audio stream for an executing behavior."""

    def __init__(
        self,
        config: Mapping[str, Any] | None,
        config_dir: str | Path,
        *,
        player_factory: Callable[[str | Path], SoundPlayer] = SoundPlayer,
    ) -> None:
        self._config = dict(config or {})
        self._config_dir = Path(config_dir)
        self._player_factory = player_factory
        self._active_player: SoundPlayer | None = None
        self._lock = threading.Lock()

    def sound_path_for(self, behavior_name: str) -> Path | None:
        """Resolve the configured sound for an exact behavior name."""
        if not self._config.get("enabled", False):
            return None

        behavior_sounds = self._config.get("behavior_sounds", {})
        configured = behavior_sounds.get(behavior_name)
        if configured is None:
            voice_behaviors = set(
                self._config.get("voice_command_behaviors", [])
            )
            if behavior_name in voice_behaviors:
                configured = self._config.get("file")
        if not configured:
            return None

        path = Path(str(configured)).expanduser()
        if not path.is_absolute():
            path = self._config_dir / path
        return path

    def play_for(self, behavior_name: str) -> Path | None:
        """Start the configured behavior sound, replacing any previous one."""
        path = self.sound_path_for(behavior_name)
        if path is None:
            return None

        player = self._player_factory(path)
        with self._lock:
            previous = self._active_player
            self._active_player = None
            if previous is not None:
                previous.stop()
            if not player.play():
                return None
            self._active_player = player
        return path

    def stop(self) -> None:
        """Stop and release the active behavior sound, if any."""
        with self._lock:
            player = self._active_player
            self._active_player = None
        if player is not None:
            player.stop()
