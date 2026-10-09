"""ElevenLabs text-to-speech for jero-speech, with the offline Piper voice as fallback.

Drop-in for jero-speech's PiperTTS where SpeechService uses it: ``synthesize_to_wav(text, path)``
and ``sample_rate``. Any failure (no key, no network, HTTP error, timeout) falls back to the Piper
engine passed in, so the robot always speaks; event Wi-Fi can't be trusted.

Key and voice come from ~/.config/jero/elevenlabs.env (chmod 600; never put them in a repo):

    ELEVENLABS_API_KEY=...
    ELEVENLABS_VOICE_ID=21m00Tcm4TlvDq8ikWAM     # optional; default "Rachel" (premade)
    ELEVENLABS_MODEL_ID=eleven_flash_v2_5         # optional; low latency
    ELEVENLABS_OUTPUT_FORMAT=pcm_22050            # optional; raw 16-bit PCM, written as WAV

Environment variables of the same names override the file.
"""

from __future__ import annotations

import json
import logging
import os
import urllib.request
import wave
from pathlib import Path

log = logging.getLogger("jero.tts.elevenlabs")

ENV_FILE = Path.home() / ".config" / "jero" / "elevenlabs.env"
API = "https://api.elevenlabs.io/v1/text-to-speech/{voice_id}?output_format={fmt}"
DEFAULTS = {
    "ELEVENLABS_VOICE_ID": "21m00Tcm4TlvDq8ikWAM",
    "ELEVENLABS_MODEL_ID": "eleven_flash_v2_5",
    "ELEVENLABS_OUTPUT_FORMAT": "pcm_22050",
}


def load_settings(env_file: Path = ENV_FILE, environ=os.environ) -> dict:
    settings = dict(DEFAULTS)
    try:
        for line in Path(env_file).read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                settings[k.strip()] = v.strip().strip('"').strip("'")
    except FileNotFoundError:
        pass
    for k in ("ELEVENLABS_API_KEY", *DEFAULTS):
        if environ.get(k):
            settings[k] = environ[k]
    return settings


class ElevenLabsTTS:
    def __init__(
        self,
        api_key: str,
        voice_id: str = DEFAULTS["ELEVENLABS_VOICE_ID"],
        model_id: str = DEFAULTS["ELEVENLABS_MODEL_ID"],
        output_format: str = DEFAULTS["ELEVENLABS_OUTPUT_FORMAT"],
        timeout_s: float = 3.0,
        fallback=None,
        urlopen=urllib.request.urlopen,
    ):
        if not output_format.startswith("pcm_"):
            raise ValueError("use a pcm_<rate> output format (written straight to WAV, no decoder needed)")
        self.api_key, self.voice_id, self.model_id = api_key, voice_id, model_id
        self.output_format = output_format
        self.sample_rate = int(output_format.split("_", 1)[1])
        self.timeout_s = timeout_s
        self.fallback = fallback
        self._urlopen = urlopen
        self.failures = 0

    @classmethod
    def from_settings(cls, fallback=None, settings: dict | None = None, **kw) -> ElevenLabsTTS | None:
        """None if no API key is configured (the caller keeps Piper)."""
        s = settings if settings is not None else load_settings()
        key = s.get("ELEVENLABS_API_KEY")
        if not key:
            return None
        return cls(
            key,
            voice_id=s["ELEVENLABS_VOICE_ID"],
            model_id=s["ELEVENLABS_MODEL_ID"],
            output_format=s["ELEVENLABS_OUTPUT_FORMAT"],
            fallback=fallback,
            **kw,
        )

    def _request(self, text: str) -> bytes:
        req = urllib.request.Request(
            API.format(voice_id=self.voice_id, fmt=self.output_format),
            data=json.dumps({"text": text, "model_id": self.model_id}).encode(),
            headers={"xi-api-key": self.api_key, "Content-Type": "application/json", "Accept": "audio/*"},
            method="POST",
        )
        with self._urlopen(req, timeout=self.timeout_s) as resp:
            pcm = resp.read()
        if len(pcm) < 2 or len(pcm) % 2:
            raise ValueError(f"unexpected audio length {len(pcm)} bytes")
        return pcm

    def synthesize_to_wav(self, text: str, out_path) -> str:
        out_path = Path(out_path)
        try:
            pcm = self._request(text)
        except Exception as exc:  # any failure -> offline voice
            self.failures += 1
            log.warning("ElevenLabs failed for %r (%s): %s", text, type(exc).__name__,
                        "using Piper" if self.fallback else "no fallback")
            if self.fallback is None:
                raise
            return self.fallback.synthesize_to_wav(text, out_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = out_path.with_suffix(".part")  # never leave a half-written WAV in the cache
        with wave.open(str(tmp), "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(self.sample_rate)
            wf.writeframes(pcm)
        os.replace(tmp, out_path)
        return str(out_path)
