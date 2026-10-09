# Demo day: 10 Oct (Pi Zero 2 W)

## Tonight, once, on the new Pi (after it has walked well with the pad)

```bash
cd ~/Jero && git fetch && git checkout main && git pull     # the Pi branch: robot/demo.sh + freeze_demo.sh
~/Jero/robot/demo.sh --check          # nothing moves: Pico, 14 servos, IMU, Bluetooth, pad
~/Jero/robot/demo.sh                  # test walk: Cross starts/pauses, sticks walk/turn, Ctrl+C stops
~/Jero/robot/freeze_demo.sh --autostart
sudo reboot                           # then: PS on the pad, Cross -> it walks, no laptop needed
```

`freeze_demo.sh` records the commit, Python packages and checksums of the policy, `~/duck_config.json`
and `~/.config/jero/*`, backs them up to `~/jero-demo/backup/`, turns off automatic apt updates and
makes sure the pad reconnect service is enabled. `--autostart` starts Jero at every boot: it waits for
the pad as long as it takes, starts paused, and restarts if the walk ever exits.

## On the day

| Step | What |
|---|---|
| 1 | Servo battery on, Pi powered |
| 2 | ~30 s boot, then **PS** on the pad (light goes solid) |
| 3 | **Cross**: walks. Cross again: pauses (stands) |
| 4 | Problem? Cross to pause, then power off the servos |

With a laptop (optional): `ssh` in, `~/Jero/robot/demo.sh --check` (it reports the running service
and how to watch it: `journalctl -fu jero-demo`). To run by hand instead:
`sudo systemctl stop jero-demo && ~/Jero/robot/demo.sh`. Logs of every run: `~/jero-logs/`.

Don't, on the day: no calibration (`imu_check`, `imu_tilt`), no `git pull`, no `apt`. If
`demo.sh --check` warns that a file changed, the frozen copy is in `~/jero-demo/backup/`. Robot
rebuilt or IMU remounted since the freeze? Then `imu_tilt` again, and `freeze_demo.sh` again.

## Network

- Bring our own router or phone hotspot, and don't rely on the venue Wi-Fi. Put the Pi and one laptop on it.
  The walk itself doesn't need any network: the pad is Bluetooth.
- Confirm the Pi's default passwords and the `Duckspot`/`Openduck` fallback networks have been changed (see `robot/setup_pi.sh`).

## Before doors open

- [ ] Packs charged (3S: 12.6 V full), with the spare in the kit
- [ ] Screws re-checked on the hips and knees (Loctite 243)
- [ ] `~/Jero/robot/demo.sh --check` is clean (or `journalctl -u jero-demo -n 50`); the pad is paired
- [ ] `tools/estop.py <pi address>` tested once, with the laptop left open on that command
- [ ] Demo floor tested: a mat or low-pile carpet is fine, polished tile is risky
- [ ] Barrier/tape line so the audience can't step on Jero

## Run-of-show

1. Pad walk: forward, turn, head moves.
2. Take back on the pad (Cross = pause) at any sign of trouble.

## Spares kit

Spare servos (pre-configured spare IDs noted on a label), a spare Pico flashed with
`firmware/pico_bridge`, a spare pack plus a charger, a microSD with a cloned image, M3 screws/inserts,
Loctite, a printed spare `foot_bottom_tpu` + `knee_to_ankle` sheets, hex drivers, a soldering iron, a
multimeter, zip ties.

Swap a servo: power off → unplug the dead one → plug in the spare → `configure_motor.py --id <id>` with it alone on
the bus → re-fit the horn at zero → re-run `find_soft_offsets.py` for that joint.

## After the event

```bash
sudo systemctl disable --now jero-demo
sudo systemctl enable --now apt-daily.timer apt-daily-upgrade.timer
```
