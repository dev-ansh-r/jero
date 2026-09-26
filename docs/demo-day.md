# Demo day: 10 Oct

## Network

- Bring our own router or phone hotspot, and don't rely on the venue Wi-Fi. Put the Pi, the Jetson and one laptop on it.
- Confirm the Pi's default passwords and the `Duckspot`/`Openduck` fallback networks have been changed (see `robot/setup_pi.sh`).

## Before doors open

- [ ] Both packs charged (8.3–8.4 V), with the spare in the kit
- [ ] Screws re-checked on the hips and knees (Loctite 243)
- [ ] `journalctl -u jero-walk -n 50` is clean; the pad is paired
- [ ] `tools/estop.py jero.local` tested once, with the laptop left open on that command
- [ ] Demo floor tested: a mat or low-pile carpet is fine, polished tile is risky
- [ ] Barrier/tape line so the audience can't step on Jero

## Run-of-show

1. Pad walk: forward, turn, head moves, antennas.
2. Hand over to the Jetson: `brain/follow.py` (or teleop from the Jetson if follow is flaky).
3. Take back on the pad at any sign of trouble.

## Spares kit

2× STS3215-C001 (pre-configured spare IDs noted on a label), a Bus Servo Adapter, 2× 18650 plus a charger, a microSD
with a cloned image, M3 screws/inserts, Loctite, a printed spare `foot_bottom_tpu` + `knee_to_ankle` sheets,
hex drivers, a soldering iron, a multimeter, zip ties.

Swap a servo: power off → unplug the dead one → plug in the spare → `configure_motor.py --id <id>` with it alone on
the bus → re-fit the horn at zero → re-run `find_soft_offsets.py` for that joint.
