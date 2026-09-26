# Jero link protocol (v1)

Commands go from the brain to the robot over one-way UDP, **port 5005**, one JSON command per datagram. The implementation is in
[`jero_link/src/jero_link/protocol.py`](../jero_link/src/jero_link/protocol.py).

## Frame

```
"JR1" | flags (1 B, bit0 = signed) | HMAC-SHA256(key, body) (32 B, zeros if unsigned) | body (JSON, UTF-8)
```

Frames are at most 1024 B; a typical one is about 200 B.

## Body

```json
{"v":1,"sid":"9f2c1a...","seq":42,"t":1790400000.123,
 "cmd":{"vx":0.1,"vy":0.0,"wz":0.3,"neck_pitch":0.0,"head_pitch":0.0,"head_yaw":0.0,"head_roll":0.0},
 "btn":["A"],"trig":[0.0,0.0],"estop":false,"ttl":500}
```

| Field | Meaning |
|---|---|
| `sid` | Random per client start. The newest session wins. |
| `seq` | Must increase within a session. Duplicates and out-of-order frames are dropped. |
| `cmd` | Same 7-vector and limits as the upstream Xbox mapping. Values outside the limits are clamped. |
| `btn` | Momentary presses (`A B X Y LB RB dpad_up dpad_down`), held for 100 ms on the robot. `A` toggles pause. |
| `trig` | Antenna positions, 0..1. |
| `estop` | Sets `RLWalk.paused = True`. Resume with `A`. |
| `ttl` | ms (50–2000). After it expires with no new frame, the robot falls back to a zero command. |

## Rules for senders

- Stream at 10–20 Hz even when nothing changes (`JeroClient` does this for you).
- Always sign at the venue (`~/.config/jero/link.key`, 32 random bytes as hex, identical on both ends).
- The pad always overrides the link. Nothing in the link can un-pause or out-prioritise the pad.

## Known limits

- There's no replay protection across sessions: someone who captured signed frames could replay an old session.
  That's acceptable on a controlled demo network. Use your own router/hotspot anyway (see demo-day.md).
- There's no telemetry back-channel yet. The robot's health is only visible in `journalctl -u jero-walk`.
