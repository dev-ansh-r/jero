#!/usr/bin/env python3
"""Voice control: jero-speech events -> skills -> Jero link, with ElevenLabs as the voice.

Runs on the RB3 Gen 2 next to the walk (robot/jero.sh keeps the Wi-Fi link on), in jero-speech's
own Python environment, with jero_link installed into it:

    ~/jero-speech/.venv/bin/pip install -e ~/Jero/jero_link
    ~/jero-speech/.venv/bin/python ~/Jero/brain/voice.py              # mic -> robot
    ~/jero-speech/.venv/bin/python ~/Jero/brain/voice.py --dry-run    # print skills, don't move

Run it as a script (never ``python -m brain.voice``): jero-speech's package is also called
``brain``, so its repo root goes first on sys.path. ElevenLabs settings: brain/tts_elevenlabs.py.
The walk must be unpaused (Cross on the pad) before voice commands move the robot.
"""

from __future__ import annotations

import argparse
import copy
import logging
import os
import sys
import threading
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(1, str(HERE.parent / "jero_link" / "src"))

from show_api import Dashboard
from skills import SkillExecutor, plan_for
from tts_elevenlabs import ElevenLabsTTS

log = logging.getLogger("jero.voice")
EL_CACHE = Path.home() / ".cache" / "jero" / "tts-elevenlabs"


class PrintClient:
    """--dry-run: log what would be sent instead of driving the robot."""

    def __init__(self):
        self.last = None

    def drive(self, vx=0.0, vy=0.0, wz=0.0):
        cmd = (round(vx, 3), round(vy, 3), round(wz, 3))
        if cmd != self.last:
            log.info("drive vx=%.2f vy=%.2f wz=%.2f", *cmd)
            self.last = cmd

    def head(self, **kw):
        pass

    def stop(self):
        self.drive()


def handle(ev: dict, executor: SkillExecutor) -> None:
    """One jero-speech event."""
    kind = ev.get("type")
    if kind == "stop":
        executor.stop()  # the speech service already says "Stopping." (safety priority)
    elif kind == "intent":
        plan = plan_for(ev.get("intent", "none"), ev.get("slots") or {})
        log.info("heard %r -> %s %s", ev.get("text"), ev.get("intent"), plan.name if plan else "(nothing)")
        if plan is not None:
            executor.run(plan)


def setup_tts(svc, config: dict, prerender) -> str:
    """Put ElevenLabs in front of the service's Piper voice. Returns a short status."""
    tts = ElevenLabsTTS.from_settings(fallback=svc.tts)
    if tts is None:
        return "Piper (no ElevenLabs key in ~/.config/jero/elevenlabs.env)"
    lines = config.get("canned_lines") or []
    try:  # re-render the canned lines in the ElevenLabs voice (cached; Piper where offline)
        prerender(tts, lines, config["tts"]["cache_dir"])
    except Exception as exc:  # noqa: BLE001  keep the Piper renders
        log.warning("ElevenLabs pre-render failed: %s", exc)
    svc.tts = tts
    return f"ElevenLabs voice {tts.voice_id} ({tts.failures} fallbacks to Piper while pre-rendering)"


def load_speech(speech_dir: Path):
    """jero-speech's SpeechService and prerender, from its repo root (ahead of this repo's brain/)."""
    speech_dir = Path(speech_dir).expanduser().resolve()
    if not (speech_dir / "brain" / "speech").is_dir():
        sys.exit(f"jero-speech not found at {speech_dir} (--speech-dir or JERO_SPEECH_DIR)")
    sys.path.insert(0, str(speech_dir))
    for name in [m for m in sys.modules if m == "brain" or m.startswith("brain.")]:
        del sys.modules[name]  # a different `brain` imported earlier must not shadow it
    from brain.speech import SpeechService
    from brain.speech.prerender import prerender

    return SpeechService, prerender


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--speech-dir", default=os.environ.get("JERO_SPEECH_DIR", str(Path.home() / "jero-speech")))
    p.add_argument("--config", default=None, help="jero-speech config.yaml (default: <speech-dir>/config.yaml)")
    p.add_argument("--host", default="127.0.0.1", help="robot running jero_walk.py with the link on")
    p.add_argument("--port", type=int, default=5005)
    p.add_argument("--key", default=os.path.expanduser("~/.config/jero/link.key"))
    p.add_argument("--tts", choices=("elevenlabs", "piper"), default="elevenlabs")
    p.add_argument("--dry-run", action="store_true", help="print skills instead of moving the robot")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    speech_dir = Path(args.speech_dir).expanduser().resolve()
    SpeechService, prerender = load_speech(speech_dir)
    import yaml

    config = yaml.safe_load(Path(args.config or speech_dir / "config.yaml").read_text()) or {}
    if args.tts == "elevenlabs":
        config = copy.deepcopy(config)  # own cache: canned lines in the ElevenLabs voice
        config.setdefault("tts", {})["cache_dir"] = str(EL_CACHE)
    svc = SpeechService(config=config)
    voice = setup_tts(svc, config, prerender) if args.tts == "elevenlabs" else "Piper"
    log.info("voice: %s", voice)

    if args.dry_run:
        client = PrintClient()
    else:
        from jero_link import JeroClient, load_key

        client = JeroClient(args.host, port=args.port, key=load_key(args.key)).start()
    dashboard = Dashboard()  # the speech bubble on brain/eyes.py's page (skipped if it isn't running)

    def say(text: str) -> None:
        dashboard.say(text)
        svc.say(text)

    executor = SkillExecutor(client, say=say).start()
    threading.Thread(target=svc.run_mic_loop, name="jero-mic", daemon=True).start()
    log.info("listening (trigger: %s). Ctrl+C to quit", config.get("trigger", "vad"))
    try:
        for ev in svc.events():
            handle(ev, executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.close()
        svc.close()
        if hasattr(client, "close"):
            client.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
