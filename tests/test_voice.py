"""Voice control: intent -> plan mapping, the skill executor, ElevenLabs TTS with Piper fallback."""

import io
import sys
import wave
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "brain"))

import skills
import tts_elevenlabs as el
import voice


class FakeClient:
    def __init__(self):
        self.cmds, self.heads, self.stops = [], [], 0

    def drive(self, vx=0.0, vy=0.0, wz=0.0):
        self.cmds.append((vx, vy, wz))

    def head(self, **kw):
        self.heads.append(kw)

    def stop(self):
        self.stops += 1
        self.drive()


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


# -- plans ----------------------------------------------------------------------------------
def test_walk_plans():
    p = skills.plan_for("walk", {})
    assert p.segments[0].vx == skills.SPEED["normal"] and p.duration == 3.0
    assert skills.plan_for("walk", {"direction": "back"}).segments[0].vx < 0
    assert skills.plan_for("walk", {"direction": "left"}).segments[0].vy > 0
    assert skills.plan_for("walk", {"steps": 5}).duration == pytest.approx(5 * skills.STEP_S)
    assert skills.plan_for("walk", {"speed": "fast", "duration_s": 99}).duration == skills.MAX_MOVE_S
    assert skills.plan_for("walk", {"speed": "fast"}).segments[0].vx <= 0.15  # jero_link limit


def test_turn_dance_stop_none():
    t = skills.plan_for("turn", {"direction": "right", "angle_deg": 180})
    assert t.segments[0].wz == -skills.TURN_RATE
    assert t.duration == pytest.approx(3.14159 / skills.TURN_RATE, abs=0.01)
    d = skills.plan_for("dance", {})
    assert d.say == "Let's dance!" and d.say_after == "Ta-da!" and d.duration == 8.0
    assert all(abs(s.vy) <= 0.2 and abs(s.head["head_yaw"]) <= 0.5 for s in d.segments)
    assert skills.plan_for("stop", {}).name == "stop" and skills.plan_for("cancel", {}).name == "stop"
    assert skills.plan_for("none", {}) is None and skills.plan_for("unclear", {}) is None


# -- executor -------------------------------------------------------------------------------
def make_executor():
    client, clock, said = FakeClient(), Clock(), []
    return skills.SkillExecutor(client, say=said.append, clock=clock), client, clock, said


def test_executor_runs_and_finishes_a_plan():
    ex, client, clock, said = make_executor()
    ex.run(skills.plan_for("walk", {"duration_s": 2}))
    assert said == ["Walking forward."] and client.cmds[-1][0] == skills.SPEED["normal"]
    clock.t += 1.0
    ex.tick()
    assert client.cmds[-1][0] > 0
    clock.t += 1.5
    ex.tick()
    assert client.cmds[-1] == (0.0, 0.0, 0.0) and ex.active is None


def test_stop_zeroes_at_once_mid_skill():
    ex, client, clock, _ = make_executor()
    ex.run(skills.plan_for("walk", {"duration_s": 5}))
    clock.t += 0.5
    ex.run(skills.plan_for("stop", {}))  # what a "stop" intent does
    assert client.cmds[-1] == (0.0, 0.0, 0.0) and ex.active is None
    clock.t += 0.1
    ex.tick()
    assert client.cmds[-1] == (0.0, 0.0, 0.0)  # nothing resumes


def test_new_command_replaces_the_running_one():
    ex, client, clock, said = make_executor()
    ex.run(skills.plan_for("walk", {"duration_s": 5}))
    clock.t += 1.0
    ex.run(skills.plan_for("turn", {"direction": "left"}))
    assert ex.active == "turn left" and client.cmds[-1] == (0.0, 0.0, skills.TURN_RATE)
    assert said[-1] == "Turning left."


def test_dance_says_tada_when_done():
    ex, _client, clock, said = make_executor()
    ex.run(skills.plan_for("dance", {}))
    clock.t += 8.5
    ex.tick()
    assert said == ["Let's dance!", "Ta-da!"]


def test_handle_stop_event():
    ex, client, _clock, _ = make_executor()
    voice.handle({"type": "intent", "intent": "walk", "slots": {}, "text": "walk"}, ex)
    voice.handle({"type": "stop", "text": "stop"}, ex)
    assert client.cmds[-1] == (0.0, 0.0, 0.0) and ex.active is None


# -- ElevenLabs -----------------------------------------------------------------------------
class Resp(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def test_elevenlabs_writes_wav(tmp_path):
    seen = {}

    def urlopen(req, timeout):
        seen["url"], seen["key"], seen["timeout"] = req.full_url, req.headers["Xi-api-key"], timeout
        return Resp(b"\x01\x00" * 2205)  # 0.1 s of 22.05 kHz PCM

    tts = el.ElevenLabsTTS("k123", urlopen=urlopen)
    out = tts.synthesize_to_wav("Hi, I'm Jero!", tmp_path / "a.wav")
    with wave.open(out) as wf:
        assert wf.getframerate() == 22050 and wf.getnframes() == 2205
    assert "output_format=pcm_22050" in seen["url"] and seen["key"] == "k123" and seen["timeout"] == 3.0
    assert not list(tmp_path.glob("*.part"))


def test_elevenlabs_falls_back_to_piper(tmp_path):
    class Piper:
        def synthesize_to_wav(self, text, path):
            Path(path).write_bytes(b"piper")
            return str(path)

    def offline(req, timeout):
        raise OSError("network is unreachable")

    tts = el.ElevenLabsTTS("k", fallback=Piper(), urlopen=offline)
    out = tts.synthesize_to_wav("Stopping.", tmp_path / "s.wav")
    assert Path(out).read_bytes() == b"piper" and tts.failures == 1


def test_no_key_means_keep_piper(tmp_path):
    assert el.ElevenLabsTTS.from_settings(settings=el.load_settings(tmp_path / "none.env", environ={})) is None


def test_settings_file_and_env_override(tmp_path):
    f = tmp_path / "elevenlabs.env"
    f.write_text('# key\nELEVENLABS_API_KEY="abc"\nELEVENLABS_VOICE_ID=v1\n')
    s = el.load_settings(f, environ={"ELEVENLABS_VOICE_ID": "v2"})
    assert s["ELEVENLABS_API_KEY"] == "abc" and s["ELEVENLABS_VOICE_ID"] == "v2"
    assert s["ELEVENLABS_OUTPUT_FORMAT"] == "pcm_22050"


# -- jero-speech import ------------------------------------------------------------------------
def test_load_speech_uses_jero_speech_brain_package(tmp_path, monkeypatch):
    speech = tmp_path / "jero-speech" / "brain" / "speech"
    speech.mkdir(parents=True)
    (speech.parent / "__init__.py").write_text("")
    (speech / "__init__.py").write_text("class SpeechService:\n    ORIGIN = 'jero-speech'\n")
    (speech / "prerender.py").write_text("def prerender(tts, lines, cache_dir):\n    return {}\n")
    monkeypatch.setattr(sys, "path", list(sys.path))
    saved = {k: v for k, v in sys.modules.items() if k == "brain" or k.startswith("brain.")}
    try:
        svc_cls, pre = voice.load_speech(tmp_path / "jero-speech")
        assert svc_cls.ORIGIN == "jero-speech" and pre(None, [], "") == {}
    finally:
        for k in [m for m in sys.modules if m == "brain" or m.startswith("brain.")]:
            del sys.modules[k]
        sys.modules.update(saved)
