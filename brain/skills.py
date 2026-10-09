"""Voice intents -> robot motion plans, and the executor that streams them over the Jero link.

Input: jero-speech intent events (brain/speech in github.com/nameissakthi25/jero-speech):
intents  walk turn stop dance look greet emote yes no cancel chit_chat none
slots    direction steps duration_s angle_deg speed style target emotion

Output: a Plan = timed segments of (vx, vy, wz, head pose) + lines to say, streamed by
SkillExecutor through a JeroClient at 20 Hz. Everything stays inside the jero_link limits
(the robot clamps again). A new command replaces the running one; stop/cancel zero the command
at once. The walk must be unpaused (Cross on the pad) for any of it to move the robot.

Head angle signs follow upstream's command layout; check them on the robot once ("look up"
should look up) and flip HEAD_UP if not.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field

log = logging.getLogger("jero.skills")

SPEED = {"slow": 0.07, "normal": 0.10, "fast": 0.15}  # m/s forward/back (limit 0.15)
SIDESTEP = 0.12  # m/s (limit 0.2)
TURN_RATE = 0.8  # rad/s (limit 1.0)
STEP_S = 0.6  # seconds per "step" when a number of steps is asked for
MAX_MOVE_S = 10.0
HEAD_UP = -1.0  # sign of head_pitch that tilts the head up (verify on the robot)
NEUTRAL = {"neck_pitch": 0.0, "head_pitch": 0.0, "head_yaw": 0.0, "head_roll": 0.0}


@dataclass(frozen=True)
class Segment:
    duration: float
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    head: dict = field(default_factory=lambda: dict(NEUTRAL))


@dataclass(frozen=True)
class Plan:
    name: str
    segments: tuple = ()
    say: str | None = None  # spoken when the plan starts
    say_after: str | None = None  # spoken when it finishes normally

    @property
    def duration(self) -> float:
        return sum(s.duration for s in self.segments)


def _head(**kw) -> dict:
    return {**NEUTRAL, **kw}


def _move_seconds(slots: dict, default: float = 3.0) -> float:
    if slots.get("duration_s"):
        t = float(slots["duration_s"])
    elif slots.get("steps"):
        t = float(slots["steps"]) * STEP_S
    else:
        t = default
    return max(0.5, min(MAX_MOVE_S, t))


def plan_for(intent: str, slots: dict | None = None) -> Plan | None:
    """The motion plan for one intent event, or None (no movement, nothing to say)."""
    slots = slots or {}
    direction = str(slots.get("direction") or "").lower()

    if intent in ("stop", "cancel"):
        return Plan("stop")

    if intent == "walk":
        t = _move_seconds(slots)
        v = SPEED.get(str(slots.get("speed") or "normal"), SPEED["normal"])
        if direction in ("back", "backward"):
            return Plan("walk back", (Segment(t, vx=-v),), say="Walking back.")
        if direction == "left":
            return Plan("step left", (Segment(t, vy=SIDESTEP),), say="Stepping left.")
        if direction == "right":
            return Plan("step right", (Segment(t, vy=-SIDESTEP),), say="Stepping right.")
        return Plan("walk forward", (Segment(t, vx=v),), say="Walking forward.")

    if intent == "turn":
        angle = math.radians(float(slots.get("angle_deg") or 90))
        t = max(0.5, min(MAX_MOVE_S, angle / TURN_RATE))
        if direction == "right":
            return Plan("turn right", (Segment(t, wz=-TURN_RATE),), say="Turning right.")
        return Plan("turn left", (Segment(t, wz=TURN_RATE),), say="Turning left.")

    if intent == "dance":
        segs = []
        for k in range(8):
            s = 1.0 if k % 2 == 0 else -1.0
            segs.append(Segment(1.0, vy=0.08 * s, head=_head(head_yaw=0.35 * s, neck_pitch=0.2)))
        return Plan("dance", tuple(segs), say="Let's dance!", say_after="Ta-da!")

    if intent == "look":
        target = str(slots.get("target") or "").lower()
        pose = {
            "up": _head(head_pitch=0.4 * HEAD_UP),
            "down": _head(head_pitch=-0.4 * HEAD_UP, neck_pitch=0.2),
            "left": _head(head_yaw=0.45),
            "right": _head(head_yaw=-0.45),
        }.get(target, _head())  # "me" / unknown: look straight ahead
        return Plan(f"look {target or 'ahead'}", (Segment(2.5, head=pose),))

    if intent in ("greet", "yes"):
        nod = []
        for _ in range(2):
            nod.append(Segment(0.35, head=_head(head_pitch=-0.25 * HEAD_UP)))
            nod.append(Segment(0.35, head=_head()))
        return Plan(intent, tuple(nod), say="Hi, I'm Jero!" if intent == "greet" else None)

    if intent == "no":
        shake = []
        for _ in range(2):
            shake.append(Segment(0.3, head=_head(head_yaw=0.4)))
            shake.append(Segment(0.3, head=_head(head_yaw=-0.4)))
        return Plan("no", (*shake, Segment(0.3)))

    if intent == "emote":
        emotion = str(slots.get("emotion") or "happy").lower()
        if emotion in ("sad", "tired"):
            return Plan("sad", (Segment(2.0, head=_head(head_pitch=-0.4 * HEAD_UP, neck_pitch=0.3)),), say="Oh no.")
        wiggle = []
        for k in range(4):
            wiggle.append(Segment(0.3, head=_head(head_roll=0.3 if k % 2 == 0 else -0.3)))
        return Plan("happy", tuple(wiggle), say="Yay!")

    if intent == "chit_chat":
        return Plan("chat", (), say="I'm Jero. Nice to meet you!")

    return None  # "none" / unknown: the speech service already handles unclear input


class SkillExecutor:
    """Streams the active Plan to a JeroClient at ``rate_hz``; stop() zeroes it immediately."""

    def __init__(self, client, say=None, rate_hz: float = 20.0, clock=time.monotonic):
        self.client = client
        self.say = say or (lambda text: None)
        self.period = 1.0 / rate_hz
        self.clock = clock
        self._lock = threading.Lock()
        self._plan: Plan | None = None
        self._t0 = 0.0
        self._thread: threading.Thread | None = None
        self._running = False

    @property
    def active(self) -> str | None:
        with self._lock:
            return self._plan.name if self._plan else None

    def run(self, plan: Plan) -> None:
        """Start ``plan`` now, replacing whatever is running."""
        if plan.name == "stop" or not plan.segments:
            if plan.name == "stop":
                self.stop()
            if plan.say:
                self.say(plan.say)
            return
        with self._lock:
            self._plan, self._t0 = plan, self.clock()
        log.info("skill: %s (%.1f s)", plan.name, plan.duration)
        if plan.say:
            self.say(plan.say)
        self.tick()

    def stop(self) -> None:
        with self._lock:
            was = self._plan
            self._plan = None
        self.client.stop()
        self.client.head(**NEUTRAL)
        if was:
            log.info("skill: stopped %s", was.name)

    def tick(self) -> None:
        """Send the command for 'now'. Called by the thread every period (tests call it directly)."""
        with self._lock:
            plan, t0 = self._plan, self._t0
        if plan is None:
            return
        elapsed = self.clock() - t0
        for seg in plan.segments:
            if elapsed < seg.duration:
                self.client.drive(vx=seg.vx, vy=seg.vy, wz=seg.wz)
                self.client.head(**seg.head)
                return
            elapsed -= seg.duration
        with self._lock:  # finished, unless something else started meanwhile
            if self._plan is plan:
                self._plan = None
            else:
                return
        self.client.stop()
        self.client.head(**NEUTRAL)
        log.info("skill: done %s", plan.name)
        if plan.say_after:
            self.say(plan.say_after)

    def start(self) -> SkillExecutor:
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="jero-skills", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        self._running = False
        self.stop()

    def _loop(self) -> None:
        while self._running:
            self.tick()
            time.sleep(self.period)
