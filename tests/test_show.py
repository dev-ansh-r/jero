"""Show controller (brain/show.py), walk telemetry (robot/telemetry.py) and the path card's tracker."""

import json
import math
import socket
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "brain"))
sys.path.insert(0, str(ROOT / "robot"))

import eyes
import show
import skills
import telemetry


class FakeDash:
    def __init__(self):
        self.said, self.modes = [], []

    def say(self, text):
        self.said.append(text)

    def mode(self, m):
        self.modes.append(m)


class FakeVoice:
    def __init__(self):
        self.spoken = []

    def speak(self, text):
        self.spoken.append(text)
        return 0.0


class FakeClient:
    def __init__(self):
        self.cmds, self.heads = [], []

    def drive(self, vx=0.0, vy=0.0, wz=0.0):
        self.cmds.append((vx, vy, wz))

    def head(self, **kw):
        self.heads.append(kw)

    def stop(self):
        self.drive()


def make_show():
    dash, voice, client = FakeDash(), FakeVoice(), FakeClient()
    s = show.Show(None, voice, dash, sleep=lambda dt: None)
    s.executor = skills.SkillExecutor(client, say=s.say)
    return s, dash, voice, client


def wait_for(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not cond():
        time.sleep(0.01)
    return cond()


def test_say_hi_speaks_shows_and_nods():
    s, dash, voice, client = make_show()
    s.handle({"type": "command", "name": "hi"})
    assert voice.spoken == [show.LINES["hi"]] == dash.said
    assert s.executor.active == "greet" and client.heads


def test_dance_asks_then_dances(monkeypatch):
    monkeypatch.setattr(show, "CROWD_PAUSE_S", 0.0)
    s, dash, voice, _client = make_show()
    s.handle({"type": "command", "name": "dance"})
    assert wait_for(lambda: s.executor.active == "dance")
    assert voice.spoken[:2] == [show.LINES["dance_ask"], show.LINES["lets_dance"]] and dash.modes[0] == "dancing"


def test_stop_cancels_a_pending_dance(monkeypatch):
    monkeypatch.setattr(show, "CROWD_PAUSE_S", 5.0)
    s, dash, voice, client = make_show()
    s.sleep = time.sleep
    s.handle({"type": "command", "name": "dance"})
    s.handle({"type": "command", "name": "stop"})
    s._job.join(2.0)
    assert s.executor.active is None and show.LINES["lets_dance"] not in voice.spoken
    assert dash.modes[-1] == "standby" and client.cmds[-1] == (0.0, 0.0, 0.0)


def test_photo_lines():
    s, _dash, voice, _ = make_show()
    s.handle({"type": "countdown", "seconds": 3})
    s.handle({"type": "photo", "name": "x.jpg"})
    assert voice.spoken == [show.LINES["cheese"], show.LINES["got_it"]]


def test_sse_parser():
    lines = ["retry: 1500", "", ": ping", "", 'data: {"type": "say", "text": "hi"}', "", "data: {bad", ""]
    assert list(show.sse_events(lines)) == [{"type": "say", "text": "hi"}]


def test_voice_caches_and_survives_no_tts(tmp_path):
    class TTS:
        voice_id, model_id, calls = "v", "m", 0

        def synthesize_to_wav(self, text, path):
            import wave

            TTS.calls += 1
            with wave.open(str(path), "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(22050)
                wf.writeframes(b"\0\0" * 22050)
            return str(path)

    v = show.Voice(TTS(), cache=tmp_path, player=[])
    assert v.prerender(["a", "b", "a"]) == 3 and TTS.calls == 2
    assert show.Voice(None, cache=tmp_path / "empty", player=[]).speak("nothing") == 0.0


# -- path card ----------------------------------------------------------------------------------
def test_path_straight_then_turn():
    p = eyes.PathTracker()
    t = 100.0
    for _ in range(20):  # 2 s forward at 0.1 m/s
        t += 0.1
        p.update({"t": t, "vx": 0.1, "vy": 0, "wz": 0, "gz": 0.0, "paused": False})
    x, y, _ = p.state()["pose"]
    assert x == pytest.approx(0.19, abs=0.02) and abs(y) < 1e-6
    for _ in range(10):  # 1 s turning left (gyro) while walking
        t += 0.1
        p.update({"t": t, "vx": 0.1, "vy": 0, "wz": 0, "gz": math.pi / 2, "paused": False})
    st = p.state()
    assert st["pose"][2] == pytest.approx(math.pi / 2, abs=0.01) and st["pose"][1] > 0.03
    assert st["distance"] == pytest.approx(0.29, abs=0.03) and len(st["points"]) > 5


def test_path_ignores_paused_and_resets():
    p = eyes.PathTracker()
    for k in range(10):
        p.update({"t": 1 + k * 0.1, "vx": 0.15, "paused": True})
    assert p.state()["distance"] == 0
    p.update({"t": 5.0, "vx": 0.1, "paused": False})
    p.update({"t": 5.1, "vx": 0.1, "paused": False})
    assert p.state()["distance"] > 0
    p.reset()
    assert p.state()["points"] == [(0.0, 0.0), (0.0, 0.0)] and p.state()["distance"] == 0


def test_telemetry_reaches_the_dashboard_listener():
    class Imu:
        def __init__(self):
            self.last_imu_data = {"gyro": [0.0, 0.0, 0.3], "accelero": [0, 0, 9.8]}

    class RL:
        def __init__(self):
            self.last_commands, self.paused, self.imu = [0.1, 0.0, 0.2, 0, 0, 0, 0], False, Imu()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    tracker = eyes.PathTracker()
    eyes.listen_telemetry(tracker, port)
    telemetry.start(RL(), ("127.0.0.1", port), rate_hz=50)
    assert wait_for(lambda: tracker.state()["distance"] > 0.005)
    snap = telemetry.snapshot(RL())
    assert snap["gz"] == 0.3 and snap["vx"] == 0.1 and json.dumps(snap)
