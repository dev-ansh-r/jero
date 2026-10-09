"""Standing expressions: head poses inside the link limits, sounds render, perform() timing."""

import sys
import wave
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "brain"))

import expressions as ex
from skills import NEUTRAL


class FakeClient:
    def __init__(self):
        self.heads, self.drives = [], []

    def drive(self, vx=0.0, vy=0.0, wz=0.0):
        self.drives.append((vx, vy, wz))

    def head(self, **kw):
        self.heads.append(kw)

    def stop(self):
        self.drive()


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


@pytest.mark.parametrize("name", sorted(ex.EXPRESSIONS))
def test_poses_stay_inside_link_limits(name):
    e = ex.EXPRESSIONS[name]
    assert e.keys[0] == (0.0, NEUTRAL) and e.keys[-1][1] == NEUTRAL
    assert all(b[0] > a[0] for a, b in zip(e.keys, e.keys[1:])), "keyframe times must increase"
    assert 8.0 <= e.duration <= 15.0
    for t in np.arange(0, e.duration + 0.5, 0.01):
        for joint, v in e.pose_at(t).items():
            lo, hi = ex.LIMITS[joint]
            assert lo + 0.02 <= v <= hi - 0.02, (name, t, joint, v)
    assert all(0 <= t <= e.duration and s in ex.SOUNDS for t, s in e.sounds)


def test_pose_is_smooth_between_keys():
    e = ex.EXPRESSIONS["curious"]
    steps = [e.pose_at(t)["head_yaw"] for t in np.arange(0, e.duration, 0.05)]
    assert max(abs(b - a) for a, b in zip(steps, steps[1:])) < 0.08  # < 1.6 rad/s at 20 Hz


@pytest.mark.parametrize("name", sorted(ex.SOUNDS))
def test_sounds_render(name, tmp_path):
    w = ex.render(name)
    assert 0.1 < len(w) / ex.RATE < 2.5 and np.max(np.abs(w)) == pytest.approx(0.7, abs=1e-3)
    path = ex.write_wav(w, tmp_path / f"{name}.wav")
    with wave.open(str(path)) as wf:
        assert wf.getframerate() == ex.RATE and wf.getnchannels() == 1 and wf.getnframes() == len(w)


def test_perform_plays_sounds_in_order_and_ends_neutral():
    client, clock, played = FakeClient(), Clock(), []
    e = ex.EXPRESSIONS["glitch"]
    assert ex.perform(e, client, played.append, clock=clock, sleep=clock.sleep)
    assert played == [s for _, s in sorted(e.sounds)]
    assert client.drives == [(0.0, 0.0, 0.0)] and client.heads[-1] == NEUTRAL
    assert len(client.heads) == pytest.approx(e.duration * 20, abs=3)


def test_perform_stops_early_and_still_ends_neutral():
    client, clock = FakeClient(), Clock()
    done = ex.perform(
        ex.EXPRESSIONS["curious"],
        client,
        lambda s: None,
        clock=clock,
        sleep=clock.sleep,
        should_stop=lambda: clock.t > 2.0,
    )
    assert not done and clock.t < 2.5 and client.heads[-1] == NEUTRAL


def test_speaker_caches_and_survives_no_player(tmp_path, monkeypatch):
    monkeypatch.setattr(ex.shutil, "which", lambda name: None)
    sp = ex.Speaker(cache=tmp_path)
    sp.play("happy")  # silent, no crash
    assert (tmp_path / "happy.wav").exists() and sp.cmd is None
