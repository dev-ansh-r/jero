#!/usr/bin/env python3
"""Show controller: the dashboard's command buttons make Jero speak and move.

Follows the dashboard's events (brain/eyes.py, /api/events). For each button:
  say hi        speaks the greeting, shows it in the "Jero says" card, nods
  dance         "Would you like to see me dance?", a pause for the crowd, then the dance
                ("Let's dance!" ... "Ta-da!")
  capture       the dashboard takes the photo; Jero says "Say cheese!" and "Got it!"
  stop          stops any motion at once

Speech: ElevenLabs (~/.config/jero/elevenlabs.env), every line cached in ~/.cache/jero/show-tts so a
line said once plays offline after; played with pw-play/aplay. Motion: over the Jero link to the walk
(robot/jero.sh running, Cross pressed to unpause). Runs in jero-speech's environment:

    ~/jero-speech/.venv/bin/python ~/Jero/brain/show.py              # dashboard buttons -> robot
    ~/jero-speech/.venv/bin/python ~/Jero/brain/show.py --dry-run    # speaks + text, motions only logged
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import logging
import os
import shutil
import subprocess
import sys
import threading
import time
import urllib.request
import wave
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(1, str(HERE.parent / "jero_link" / "src"))

from show_api import Dashboard
from skills import SkillExecutor, plan_for
from tts_elevenlabs import ElevenLabsTTS

log = logging.getLogger("jero.show")
CACHE = Path.home() / ".cache" / "jero" / "show-tts"
LINES = {
    "hi": "Hi everyone! I'm Jero. Nice to meet you!",
    "dance_ask": "Would you like to see me dance?",
    "cheese": "Say cheese!",
    "got_it": "Got it!",
    "lets_dance": "Let's dance!",
    "tada": "Ta-da!",
}
CROWD_PAUSE_S = 2.0  # after "would you like to see me dance?"


class Voice:
    """Text -> cached WAV (ElevenLabs) -> speaker. Never raises: no audio is better than no show."""

    def __init__(self, tts: ElevenLabsTTS | None, cache: Path = CACHE, player: list | None = None):
        self.tts, self.cache = tts, Path(cache)
        self.player = player if player is not None else self._find_player()
        self._proc: subprocess.Popen | None = None

    @staticmethod
    def _find_player() -> list | None:
        for cmd in (["pw-play"], ["paplay"], ["aplay", "-q"]):
            if shutil.which(cmd[0]):
                return cmd
        return None

    def path(self, text: str) -> Path:
        key = f"{getattr(self.tts, 'voice_id', '')}|{getattr(self.tts, 'model_id', '')}|{text}"
        return self.cache / f"{hashlib.sha1(key.encode()).hexdigest()[:16]}.wav"

    def render(self, text: str) -> Path | None:
        p = self.path(text)
        if p.exists():
            return p
        if self.tts is None:
            return None
        try:
            return Path(self.tts.synthesize_to_wav(text, p))
        except Exception as exc:  # noqa: BLE001
            log.warning("no voice for %r: %s", text, exc)
            return None

    def speak(self, text: str) -> float:
        """Start speaking; returns the line's length in seconds (0 if silent)."""
        p = self.render(text)
        if p is None or not self.player:
            return 0.0
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
        try:
            self._proc = subprocess.Popen([*self.player, str(p)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            with wave.open(str(p)) as wf:
                return wf.getnframes() / wf.getframerate()
        except (OSError, wave.Error) as exc:
            log.warning("can't play %s: %s", p, exc)
            return 0.0

    def prerender(self, texts) -> int:
        return sum(1 for t in texts if self.render(t) is not None)


class Show:
    def __init__(self, executor: SkillExecutor, voice: Voice, dashboard: Dashboard, sleep=time.sleep):
        self.executor, self.voice, self.dashboard, self.sleep = executor, voice, dashboard, sleep
        self._cancel = threading.Event()
        self._job: threading.Thread | None = None

    def say(self, text: str) -> float:
        self.dashboard.say(text)
        return self.voice.speak(text)

    def handle(self, ev: dict) -> None:
        kind = ev.get("type")
        if kind == "command":
            name = ev.get("name")
            log.info("button: %s", name)
            if name == "stop":
                self.stop()
            elif name == "hi":
                self.stop(quiet=True)
                self.dashboard.mode("greeting")
                self.executor.run(dataclasses.replace(plan_for("greet"), say=LINES["hi"]))
            elif name == "dance":
                self.stop(quiet=True)
                self._start(self._dance)
        elif kind == "countdown":
            self.say(LINES["cheese"])
        elif kind == "photo":
            self.say(LINES["got_it"])

    def _start(self, fn) -> None:
        self._cancel.clear()
        self._job = threading.Thread(target=fn, name="jero-show", daemon=True)
        self._job.start()

    def _wait(self, seconds: float) -> bool:
        """Sleep in small steps; False if stop was pressed."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self._cancel.is_set():
                return False
            self.sleep(min(0.1, max(0.0, end - time.monotonic())))
        return not self._cancel.is_set()

    def _dance(self) -> None:
        self.dashboard.mode("dancing")
        took = self.say(LINES["dance_ask"])
        if not self._wait(took + CROWD_PAUSE_S):
            return
        plan = plan_for("dance")
        self.executor.run(dataclasses.replace(plan, say=LINES["lets_dance"], say_after=LINES["tada"]))
        if self._wait(plan.duration + 0.5):
            self.dashboard.mode("standby")

    def stop(self, quiet: bool = False) -> None:
        self._cancel.set()
        self.executor.stop()
        if not quiet:
            self.dashboard.mode("standby")


def sse_events(lines):
    """Parse a text/event-stream (iterable of decoded lines) into event dicts."""
    data = []
    for line in lines:
        line = line.rstrip("\r\n")
        if line.startswith("data:"):
            data.append(line[5:].lstrip())
        elif not line and data:
            try:
                yield json.loads("\n".join(data))
            except ValueError:
                pass
            data = []


def follow(base: str, handle, stop: threading.Event | None = None) -> None:
    """Handle the dashboard's events forever (reconnects if it restarts)."""
    stop = stop or threading.Event()
    while not stop.is_set():
        try:
            with urllib.request.urlopen(f"{base}/api/events", timeout=30) as resp:
                log.info("following %s/api/events", base)
                for ev in sse_events(line.decode() for line in resp):
                    handle(ev)
        except Exception as exc:  # noqa: BLE001
            log.warning("dashboard events: %s (retrying in 2 s)", exc)
            time.sleep(2.0)


class LogClient:
    """--dry-run: motions are logged, not sent."""

    def __init__(self):
        self.last = None

    def drive(self, vx=0.0, vy=0.0, wz=0.0):
        cmd = (round(vx, 2), round(vy, 2), round(wz, 2))
        if cmd != self.last:
            log.info("drive vx=%.2f vy=%.2f wz=%.2f", *cmd)
            self.last = cmd

    def head(self, **kw):
        pass

    def stop(self):
        self.drive()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dashboard", default="http://127.0.0.1:8080")
    p.add_argument("--host", default="127.0.0.1", help="robot running jero_walk.py with the link on")
    p.add_argument("--port", type=int, default=5005)
    p.add_argument("--key", default=os.path.expanduser("~/.config/jero/link.key"))
    p.add_argument("--dry-run", action="store_true", help="speak and show text, only log motions")
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    voice = Voice(ElevenLabsTTS.from_settings())
    if voice.tts is None:
        log.warning("no ElevenLabs key (~/.config/jero/elevenlabs.env): lines are shown, not spoken")
    log.info("voice: %d/%d lines ready (cached in %s)", voice.prerender(LINES.values()), len(LINES), voice.cache)
    if args.dry_run:
        client = LogClient()
    else:
        from jero_link import JeroClient, load_key

        client = JeroClient(args.host, port=args.port, key=load_key(args.key)).start()
    dashboard = Dashboard(args.dashboard)
    show = Show(None, voice, dashboard)
    show.executor = SkillExecutor(client, say=show.say).start()
    try:
        follow(args.dashboard, show.handle)
    except KeyboardInterrupt:
        pass
    finally:
        show.stop(quiet=True)
        show.executor.close()
        if hasattr(client, "close"):
            client.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
