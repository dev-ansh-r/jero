# Build plan: 26 Sep → 10 Oct 2026

2 Oct (Gandhi Jayanti) is a courier holiday, so every part must be delivered by **1 Oct**.

## Timeline

| Date | Workstream | Task | Gate |
|---|---|---|---|
| 26 Sep | Procurement | Place every order in [bom/](bom/) | G0 |
| 26–30 Sep | Printing | Legs + trunk first, then head/body, TPU feet last (~50–70 printer-hours) | G1 |
| 27–29 Sep | Software | Flash the Duck image, run `robot/setup_pi.sh`, set up the brain + link on the bench | |
| 27–30 Sep | Training | `training/setup_server.sh` + smoke run (`STEPS=50000000`) to prove the pipeline | |
| 29 Sep–1 Oct | Receiving | Chase anything not shipped by 29 Sep | |
| 30 Sep–1 Oct | Servos | `tools/configure_servos.sh`: IDs, horns at zero, labels | G2 |
| 1 Oct | Power | Build the 2S pack; check 5 V at the Pi and ~8 V at the servo board | |
| 2–4 Oct | Assembly | [upstream assembly guide](https://github.com/apirrone/Open_Duck_Mini/blob/b23317a485b3cec7d8417f352478778b3475173c/docs/assembly_guide.md) + Onshape; Loctite on metal-to-metal screws | |
| 4–5 Oct | Bring-up | [bringup.md](bringup.md) steps 1–6 | G3 |
| 6–7 Oct | Walking | Tune offsets, IMU orientation, `pitch_bias`; test on the event floor type | G4 |
| 7–8 Oct | Brain | `brain/teleop.py`, then `brain/follow.py` with the Jetson camera | G5 |
| 8 Oct | Hardening | Re-torque screws, endurance test, charge the spare pack | |
| 9 Oct | Buffer | Rework slot + full demo rehearsal | G6 |
| 10 Oct | Event | [demo-day.md](demo-day.md) | |

## Owners

| Workstream | Owner | Backup |
|---|---|---|
| Procurement | | |
| Printing | | |
| Servos + mechanical assembly | | |
| Power + wiring | | |
| Pi software + walking | | |
| Training server | | |
| Jetson brain + demo behaviour | | |

## Cut lines (if we slip)

1. Drop the expression pack (camera, speaker, LEDs, antennas). Walking doesn't depend on it.
2. Drop `follow.py` and demo with keyboard teleop from the Jetson.
3. Drop the link entirely and demo on the Xbox pad (upstream path, zero Jero code).
