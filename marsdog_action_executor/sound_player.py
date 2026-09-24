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
from collections.abc import Iterator, Mapping
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Callable

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
        if behavior_name in behavior_sounds and configured is None:
            return None
        if configured is None:
            voice_behaviors = set(
                self._config.get("voice_command_behaviors", [])
            )
            if behavior_name in voice_behaviors:
                configured = self._config.get("file")
        if not configured:
            return None

        return self._resolve(configured)

    def stage_sound_path(self, behavior_name: str, stage_id: str) -> Path | None:
        """Resolve the configured sound for one exact Stage of a behavior.

        Unlike :meth:`sound_path_for` this never falls back to the generic
        voice-command file: an unlisted behavior or Stage is intentionally
        silent, and a Stage entry is always an explicit file.
        """
        if not self._config.get("enabled", False):
            return None

        stage_sounds = self._config.get("stage_sounds", {})
        if not isinstance(stage_sounds, Mapping):
            return None
        per_behavior = stage_sounds.get(behavior_name)
        if not isinstance(per_behavior, Mapping):
            return None
        configured = per_behavior.get(stage_id)
        if not configured:
            return None

        return self._resolve(configured)

    def play_for(self, behavior_name: str) -> Path | None:
        """Start the configured behavior sound, replacing any previous one."""
        path = self.sound_path_for(behavior_name)
        if path is None:
            return None
        if self._start(path) is None:
            return None
        return path

    @contextmanager
    def stage_sound(
        self, behavior_name: str, stage_id: str
    ) -> Iterator[Path | None]:
        """Own the audio stream for the duration of a single Stage.

        Yields the resolved path, or ``None`` when the Stage has no configured
        sound — or has one that cannot be played (missing file, no audio
        backend), in which case the Stage still runs silently.

        The stream started here is stopped on exit, including when the body
        raises. A stream that was replaced meanwhile — by a behavior-level
        :meth:`play_for`, ``quiet``, or a preemption — is left untouched, which
        is why the check is object identity and not a file comparison.
        """
        path = self.stage_sound_path(behavior_name, stage_id)
        if path is None:
            yield None
            return

        player = self._start(path)
        if player is None:
            yield None
            return

        try:
            yield path
        finally:
            self._stop_if_active(player)

    def apply_control_behavior(self, behavior_name: str) -> bool:
        """Apply configured audio-only control and report whether it matched."""
        stop_behaviors = set(self._config.get("stop_behaviors", []))
        if behavior_name not in stop_behaviors:
            return False
        self.stop()
        return True

    def stop(self) -> None:
        """Stop and release the active behavior sound, if any."""
        with self._lock:
            player = self._active_player
            self._active_player = None
        if player is not None:
            player.stop()

    # ── internal ─────────────────────────────────────────────────────────

    def _resolve(self, configured: Any) -> Path:
        """Turn a configured value into a path. Does not check ``enabled``.

        Relative values resolve from the config directory, so packaged assets
        travel with the package. Every public entry point checks ``enabled``
        itself — this helper must stay a pure path computation or the global
        switch would be bypassed.
        """
        path = Path(str(configured)).expanduser()
        if not path.is_absolute():
            path = self._config_dir / path
        return path

    def _start(self, path: Path) -> SoundPlayer | None:
        """Replace the active stream with ``path`` and start playback."""
        player = self._player_factory(path)
        with self._lock:
            previous = self._active_player
            self._active_player = None
            if previous is not None:
                previous.stop()
            if not player.play():
                return None
            self._active_player = player
        return player

    def _stop_if_active(self, player: SoundPlayer) -> bool:
        """Stop ``player`` only while it is still the active stream."""
        with self._lock:
            if self._active_player is not player:
                return False
            self._active_player = None
        player.stop()
        return True
