# Architecture

Diagrams, interface contracts and decisions D8 onwards are in [`architecture/README.md`](architecture/README.md).

## Decisions

| # | Decision | Why |
|---|---|---|
| D1 | Build Open Duck Mini v2 as-is, not Bimo or Microduck | Bimo's CAD and controller PCB aren't published, and its kit ships EU/US only. Microduck's hardware isn't open. ODM v2 has public STLs, an off-the-shelf BOM and a policy that already walks. |
| D2 | Keep the Pi Zero 2 W as the onboard computer | The pre-built image, runtime and `BEST_WALK_ONNX_2` are validated on this exact hardware. Any change to mass or centre of mass means retraining. |
| D3 | Jetson Orin Nano Super stays **off-board** for 10 Oct | It needs 9–20 V input (the pack is 2S, 7.4 V), measures ~100×79×21 mm (won't fit the head) and would shift the CoM. Onboard is a post-event v2 project. |
| D4 | Brain → robot over UDP at 20 Hz with a TTL | Stateless and loss-tolerant. If the brain dies, the robot drops to a zero command within `ttl_ms` (500 ms). |
| D5 | Xbox pad always has priority over the link | A human can always take over. `A` pauses the walk. |
| D6 | HMAC-signed frames | Event Wi-Fi is shared. Without the key, nobody else can drive Jero. |
| D7 | Upstream pinned as submodules; nothing is forked | Upstream fixes stay easy to pull in. Jero code wraps upstream and never patches it. |

## Data flow on the robot (50 Hz)

```
upstream RLWalk.run()                                   robot/jero_controller.py
  └─ self.xbox_controller.get_last_command()  ───────►  MuxController
        returns (cmd[7], Buttons, lt, rt)                  ├─ XBoxController (upstream, optional)
  └─ obs = [gyro, acc, cmd[7], q, dq, a(t-1..3), ...]      └─ UdpReceiver (jero_link) ◄── UDP :5005
  └─ action = onnx.infer(obs)
  └─ servo targets → rustypot → /dev/ttyACM0
```

`cmd[7]` = `[vx, vy, wz, neck_pitch, head_pitch, head_yaw, head_roll]`, with limits as in upstream:
vx ±0.15 m/s, vy ±0.2 m/s, wz ±1.0 rad/s.

`jero_walk.py` builds upstream `RLWalk` with `commands=False`, then attaches `MuxController` as
`xbox_controller`. The policy loop itself is untouched.

## Failure modes

| Failure | Behaviour |
|---|---|
| Brain crashes / Wi-Fi drops | Command goes stale after the TTL → zero velocity (steps in place). |
| Xbox pad disconnects | Link keeps working. With `--no-xbox`, the robot starts without a pad. |
| Robot is misbehaving | Xbox `A`, or `tools/estop.py jero.local` → `RLWalk.paused = True`. |
| Wrong or unsigned sender | Frame dropped (counted in `Receiver.dropped`). |
| Battery sag | Upstream has no low-voltage cutoff. The BMS cuts at cell level, so watch pack voltage (`scripts/check_voltage.py`). |

## Post-event: Jetson onboard (v2)

- Power: a 3S pack with a buck converter to ~7.4 V for the servos, or a boost converter for the Jetson off 2S.
- Mount: a new head or back module, designed in the Onshape fork.
- Sim: update the MJCF masses, then retrain with `training/train.sh` and validate with `training/eval_mujoco.sh`.
- Runtime: run the policy loop on the Jetson and retire the Pi. The observation and action contract stays the same.
