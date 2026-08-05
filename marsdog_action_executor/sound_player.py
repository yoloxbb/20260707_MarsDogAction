"""Async dog-bark sound player for voice-command behaviors.

Uses GStreamer (``gst-launch-1.0``) for non-blocking MP3 playback.
Falls back to ``ffmpeg`` if GStreamer is unavailable.
"""

from __future__ import annotations

import logging
import os
import shutil
import subprocess
import threading
from pathlib import Path

logger = logging.getLogger(__name__)

# ── Voice-command behaviors that trigger a bark sound ──────────────────────

VOICE_COMMAND_BEHAVIORS: set[str] = {
    "respond_owner_call",
    "sit_down",
    "lie_down",
    "stand_up",
    "wait_in_place",
    "come_to_owner",
    "follow_owner",
    "give_paw",
    "high_five",
    "roll_over",
    "spin_around",
    "return_to_owner",
    "drop_object",
    "play_dead",
    "bring_object",
    "fetch_object",
    "emergency_stop",
}


def _find_player() -> list[str] | None:
    """Return the first available CLI audio-player command for MP3.

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

    def play(self) -> None:
        """Start playback in a daemon thread (non-blocking)."""
        if not os.path.exists(self._filepath):
            logger.error("Sound file not found: %s", self._filepath)
            return

        player_prefix = _find_player()
        if player_prefix is None:
            logger.warning(
                "No audio player found (gst-launch-1.0 or ffmpeg); "
                "cannot play bark sound."
            )
            return

        def _run() -> None:
            cmd = self._build_command(player_prefix)
            logger.debug("Sound player: %s", " ".join(cmd))
            try:
                with self._lock:
                    self._process = subprocess.Popen(
                        cmd,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                self._process.wait()
            except Exception:
                logger.exception("Sound playback failed")
            finally:
                with self._lock:
                    self._process = None

        thread = threading.Thread(target=_run, daemon=True, name="sound-player")
        thread.start()

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
                f"uri=file://{os.path.abspath(self._filepath)}",
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
