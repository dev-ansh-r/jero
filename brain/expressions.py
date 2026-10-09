#!/usr/bin/env python3
"""Standing expressions: Jero stands, moves its head and makes robot noises from the speaker.

Runs next to the walk (robot/jero.sh keeps the Wi-Fi link on), like brain/voice.py. The walk keeps
balancing at zero speed; this only sends head poses over the link and plays sounds. The walk must
be unpaused (Cross on the pad) for the head to move.

    python ~/Jero/brain/expressions.py curious          # look around, question chirps
    python ~/Jero/brain/expressions.py glitch           # malfunction, power down, reboot, happy
    python ~/Jero/brain/expressions.py --loop           # both, again and again (stand-by at a booth)
    python ~/Jero/brain/expressions.py curious --dry-run   # sounds + printed poses, robot untouched
    python ~/Jero/brain/expressions.py --export ~/sounds   # just write the sounds as WAVs

Sounds are synthesised (numpy, no files to ship), cached as WAVs in ~/.cache/jero/sounds/ and
played with aplay (ALSA; --audio-device to pick one), else paplay / pw-play.
"""

from __future__ import annotations

import argparse
import logging
import os
import random
import shutil
import subprocess
import sys
import time
import wave
from dataclasses import dataclass
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(1, str(HERE.parent / "jero_link" / "src"))

from skills import HEAD_UP, NEUTRAL

log = logging.getLogger("jero.expressions")
RATE = 22050
CACHE = Path.home() / ".cache" / "jero" / "sounds"
# jero_link's head limits (rad); poses stay inside them with a margin, the robot clamps again
LIMITS = {"neck_pitch": (-0.34, 1.1), "head_pitch": (-0.78, 0.3), "head_yaw": (-0.5, 0.5), "head_roll": (-0.5, 0.5)}


# -- sounds ---------------------------------------------------------------------------------
def _env(n: int, attack: float = 0.01, release: float = 0.04) -> np.ndarray:
    a, r = max(1, int(attack * RATE)), max(1, int(release * RATE))
    e = np.ones(n)
    e[: min(a, n)] = np.linspace(0, 1, min(a, n))
    e[-min(r, n) :] *= np.linspace(1, 0, min(r, n))
    return e


def tone(
    f0: float,
    f1: float,
    dur: float,
    shape: str = "sine",
    vib_hz: float = 0.0,
    vib_depth: float = 0.0,
    curve: float = 1.0,
) -> np.ndarray:
    """A pitch slide f0 -> f1 (Hz) with optional vibrato; curve > 1 bends the slide late."""
    n = max(1, int(dur * RATE))
    x = np.linspace(0, 1, n) ** curve
    f = f0 * (f1 / f0) ** x  # exponential slide sounds like a pitch glide
    if vib_hz:
        f = f * (1 + vib_depth * np.sin(2 * np.pi * vib_hz * np.arange(n) / RATE))
    phase = 2 * np.pi * np.cumsum(f) / RATE
    if shape == "square":
        w = np.sign(np.sin(phase)) * 0.5
    elif shape == "saw":
        w = (phase / np.pi % 2 - 1) * 0.6
    else:
        w = np.sin(phase)
    return w * _env(n)


def silence(dur: float) -> np.ndarray:
    return np.zeros(int(dur * RATE))


def babble(seed: int, count: int = 9) -> np.ndarray:
    """R2-style chatter: short random bleeps and slides."""
    rng = random.Random(seed)
    parts = []
    for _ in range(count):
        f0 = rng.uniform(700, 2600)
        f1 = f0 * rng.choice((0.6, 0.8, 1.0, 1.3, 1.7))
        parts.append(tone(f0, f1, rng.uniform(0.04, 0.13), rng.choice(("sine", "sine", "square"))))
        parts.append(silence(rng.uniform(0.01, 0.06)))
    return np.concatenate(parts)


def glitch(seed: int) -> np.ndarray:
    """Stuttering, bit-crushed square sweeps: a malfunction."""
    rng = random.Random(seed)
    parts = []
    for _ in range(7):
        chunk = tone(rng.uniform(200, 1800), rng.uniform(200, 3000), rng.uniform(0.05, 0.12), "square")
        parts += [chunk] * rng.choice((1, 2, 3))  # stutter
        parts.append(silence(rng.uniform(0.0, 0.04)))
    w = np.concatenate(parts)
    w = np.round(w * 4) / 4  # bit crush
    hold = 4  # sample-rate crush
    return np.repeat(w[::hold], hold)[: len(w)]


SOUNDS = {
    "question": lambda: np.concatenate(
        [tone(520, 1450, 0.42, vib_hz=9, vib_depth=0.03, curve=1.8), silence(0.05), tone(1800, 1900, 0.07)]
    ),
    "hmm": lambda: tone(330, 270, 0.7, "saw", vib_hz=6, vib_depth=0.04) * 0.8,
    "happy": lambda: np.concatenate(
        [
            tone(1047, 1047, 0.09),
            silence(0.02),
            tone(1319, 1319, 0.09),
            silence(0.02),
            tone(1568, 1568, 0.09),
            silence(0.02),
            tone(2093, 2093, 0.3, vib_hz=18, vib_depth=0.04),
        ]
    ),
    "surprise": lambda: tone(800, 2700, 0.22, curve=0.6),
    "babble": lambda: babble(7),
    "babble2": lambda: babble(23, count=6),
    "glitch": lambda: glitch(3),
    "powerdown": lambda: tone(1300, 70, 0.9, "saw", curve=0.7),
    "bootup": lambda: np.concatenate(
        [
            tone(140, 1600, 0.75, "square", curve=1.5) * 0.7,
            silence(0.06),
            tone(1600, 1600, 0.06),
            silence(0.04),
            tone(2100, 2100, 0.1),
        ]
    ),
}


def render(name: str) -> np.ndarray:
    w = SOUNDS[name]()
    peak = float(np.max(np.abs(w))) or 1.0
    return (w / peak * 0.7).astype(np.float32)


def write_wav(samples: np.ndarray, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".part")
    with wave.open(str(tmp), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(RATE)
        wf.writeframes((np.clip(samples, -1, 1) * 32767).astype("<i2").tobytes())
    os.replace(tmp, path)
    return path


class Speaker:
    """Plays named sounds without blocking (one at a time: a new one cuts the last)."""

    def __init__(self, cache: Path = CACHE, device: str | None = None, mute: bool = False):
        self.cache, self.device, self.mute = Path(cache), device, mute
        self.cmd = self._player()
        self._proc: subprocess.Popen | None = None
        if not mute and not self.cmd:
            log.warning("no audio player (aplay / paplay / pw-play): expressions run silent")

    def _player(self) -> list[str] | None:
        if shutil.which("aplay"):
            return ["aplay", "-q", *(["-D", self.device] if self.device else [])]
        for p in ("paplay", "pw-play"):
            if shutil.which(p):
                return [p]
        return None

    def path(self, name: str) -> Path:
        p = self.cache / f"{name}.wav"
        if not p.exists():
            write_wav(render(name), p)
        return p

    def play(self, name: str) -> None:
        log.info("sound: %s", name)
        path = self.path(name)
        if self.mute or not self.cmd:
            return
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
        try:
            self._proc = subprocess.Popen([*self.cmd, str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        except OSError as exc:
            log.warning("can't play %s: %s", name, exc)

    def close(self) -> None:
        if self._proc and self._proc.poll() is None:
            self._proc.wait(timeout=3)


# -- head timelines -------------------------------------------------------------------------
def pose(**kw) -> dict:
    return {**NEUTRAL, **kw}


UP, DOWN = 1.0 * HEAD_UP, -1.0 * HEAD_UP  # head_pitch sign for looking up / down


@dataclass(frozen=True)
class Expression:
    name: str
    keys: tuple  # ((t, pose), ...) increasing t, first at 0 and neutral
    sounds: tuple = ()  # ((t, sound name), ...)

    @property
    def duration(self) -> float:
        return self.keys[-1][0]

    def pose_at(self, t: float) -> dict:
        """Smoothstep between keyframes."""
        keys = self.keys
        if t <= keys[0][0]:
            return dict(keys[0][1])
        for (ta, pa), (tb, pb) in zip(keys, keys[1:]):
            if t <= tb:
                u = (t - ta) / (tb - ta) if tb > ta else 1.0
                u = u * u * (3 - 2 * u)
                return {k: pa[k] + (pb[k] - pa[k]) * u for k in NEUTRAL}
        return dict(keys[-1][1])


def _curious() -> Expression:
    keys = [
        (0.0, pose()),
        (1.2, pose(head_yaw=0.42, neck_pitch=0.15)),  # look left
        (2.0, pose(head_yaw=0.42, neck_pitch=0.15, head_roll=0.3)),  # tilt: "hm?"
        (2.6, pose(head_yaw=0.42, neck_pitch=0.15, head_roll=0.3)),
        (3.6, pose(head_yaw=-0.42, neck_pitch=0.15, head_roll=-0.25)),  # look right
        (4.6, pose(head_yaw=-0.42, neck_pitch=0.15, head_roll=-0.25)),
        (5.6, pose(neck_pitch=0.35, head_pitch=0.22 * DOWN)),  # peer down at the floor
        (6.6, pose(neck_pitch=0.35, head_pitch=0.22 * DOWN, head_yaw=0.1)),
        (6.9, pose(head_pitch=0.45 * UP)),  # startled: head up
        (7.8, pose(head_pitch=0.45 * UP)),
        (8.4, pose()),
        (8.75, pose(head_pitch=0.25 * DOWN)),  # double nod
        (9.1, pose()),
        (9.45, pose(head_pitch=0.25 * DOWN)),
        (9.8, pose()),
        (10.5, pose()),
    ]
    sounds = [(1.3, "question"), (3.7, "hmm"), (5.7, "babble"), (6.9, "surprise"), (8.6, "happy")]
    return Expression("curious", tuple(keys), tuple(sounds))


def _glitch() -> Expression:
    keys = [(0.0, pose())]
    t, s = 0.12, 1.0
    while t < 1.6:  # jitter
        keys.append((round(t, 2), pose(head_yaw=0.14 * s, head_roll=-0.09 * s, neck_pitch=0.05)))
        t, s = t + 0.12, -s
    keys += [
        (1.9, pose(head_yaw=0.22, head_roll=0.35, neck_pitch=0.1)),  # stuck, tilted
        (2.4, pose(head_yaw=0.22, head_roll=0.35, neck_pitch=0.1)),
        (3.4, pose(neck_pitch=0.55, head_pitch=0.25 * DOWN, head_roll=0.15)),  # power down: droop
        (4.4, pose(neck_pitch=0.55, head_pitch=0.25 * DOWN, head_roll=0.15)),
        (5.4, pose()),  # boot up
        (5.9, pose(head_pitch=0.4 * UP)),  # proud
        (6.4, pose(head_pitch=0.4 * UP)),
    ]
    t, s = 6.8, 1.0
    for _ in range(5):  # happy wiggle
        keys.append((round(t, 2), pose(head_roll=0.3 * s, head_yaw=0.18 * s)))
        t, s = t + 0.4, -s
    keys += [(t + 0.3, pose()), (t + 0.65, pose(head_pitch=0.25 * DOWN)), (t + 1.0, pose()), (t + 1.6, pose())]
    sounds = [(0.0, "glitch"), (2.4, "powerdown"), (4.5, "bootup"), (6.0, "babble2"), (round(t + 0.3, 2), "happy")]
    return Expression("glitch", tuple(keys), tuple(sounds))


EXPRESSIONS = {e.name: e for e in (_curious(), _glitch())}


def perform(
    expr: Expression,
    client,
    play,
    rate_hz: float = 20.0,
    clock=time.monotonic,
    sleep=time.sleep,
    should_stop=lambda: False,
) -> bool:
    """Stand still and play ``expr``. Returns False if stopped early. Always ends at the neutral pose."""
    period = 1.0 / rate_hz
    pending = sorted(expr.sounds)
    client.drive(0.0, 0.0, 0.0)
    t0 = clock()
    try:
        while True:
            t = clock() - t0
            while pending and pending[0][0] <= t:
                play(pending.pop(0)[1])
            if t >= expr.duration:
                return True
            if should_stop():
                return False
            client.head(**expr.pose_at(t))
            sleep(period)
    finally:
        client.head(**NEUTRAL)


class PrintClient:
    """--dry-run: log the head pose once a second instead of moving the robot."""

    def __init__(self):
        self.n = 0

    def drive(self, vx=0.0, vy=0.0, wz=0.0):
        log.info("drive vx=%.2f vy=%.2f wz=%.2f", vx, vy, wz)

    def head(self, **kw):
        if self.n % 20 == 0:
            log.info("head %s", " ".join(f"{k}={v:+.2f}" for k, v in kw.items()))
        self.n += 1

    def stop(self):
        self.drive()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("expression", nargs="?", choices=sorted(EXPRESSIONS), help="default with --loop: all of them")
    p.add_argument("--loop", action="store_true", help="repeat (all expressions in turn, or the one given)")
    p.add_argument("--pause", type=float, default=6.0, help="--loop: seconds standing still between expressions")
    p.add_argument("--host", default="127.0.0.1", help="robot running jero_walk.py with the link on")
    p.add_argument("--port", type=int, default=5005)
    p.add_argument("--key", default=os.path.expanduser("~/.config/jero/link.key"))
    p.add_argument("--audio-device", default=os.environ.get("JERO_AUDIO_DEVICE"), help="aplay -D device")
    p.add_argument("--mute", action="store_true")
    p.add_argument("--dry-run", action="store_true", help="sounds + printed poses, robot untouched")
    p.add_argument("--export", metavar="DIR", help="write every sound as DIR/<name>.wav and exit")
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    if args.export:
        for name in SOUNDS:
            print(write_wav(render(name), Path(args.export).expanduser() / f"{name}.wav"))
        return 0
    if not args.expression and not args.loop:
        p.error("name an expression, or --loop")

    speaker = Speaker(device=args.audio_device, mute=args.mute)
    for name in SOUNDS:
        speaker.path(name)  # render once up front: no delay mid-expression
    if args.dry_run:
        client = PrintClient()
    else:
        from jero_link import JeroClient, load_key

        client = JeroClient(args.host, port=args.port, key=load_key(args.key)).start()

    order = [args.expression] if args.expression else sorted(EXPRESSIONS)
    try:
        k = 0
        while True:
            expr = EXPRESSIONS[order[k % len(order)]]
            log.info("expression: %s (%.1f s)", expr.name, expr.duration)
            perform(expr, client, speaker.play)
            k += 1
            if not args.loop:
                break
            time.sleep(args.pause)
    except KeyboardInterrupt:
        pass
    finally:
        client.stop()
        client.head(**NEUTRAL)
        speaker.close()
        if hasattr(client, "close"):
            client.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
