# Jero

Nunnari Labs' bipedal robot for the **10 Oct 2026** showcase. Jero is an
[Open Duck Mini v2](https://github.com/apirrone/Open_Duck_Mini) build sourced in India: 14× Feetech STS3215,
Raspberry Pi Zero 2 W onboard, and an RL walking policy trained in MuJoCo. This repo adds a
Wi-Fi command link, so a Jetson Orin Nano Super can act as the robot's off-board "brain".

```
 Xbox pad ──BT──┐                                   GPU server (optional)
                ▼                                   Open_Duck_Playground → .onnx
 Jetson ──Wi-Fi/UDP 20 Hz──►  Pi Zero 2 W  ──USB──► servo bus (14× STS3215)
 (vision, voice,              RL policy 50 Hz       BNO055 IMU, foot switches
  follow-me)                  robot/jero_walk.py
```

See [docs/architecture.md](docs/architecture.md) for the design decisions.

## Status

| Gate | Target date | Done when | State |
|---|---|---|---|
| G0 Parts ordered | 26 Sep | every BOM order confirmed | ☐ |
| G1 Parts printed | 30 Sep | every part in [printing.md](docs/printing.md) ticked | ☐ |
| G2 Servos configured | 1 Oct | 14 servos have IDs and labels, horns fitted at zero | ☐ |
| G3 Assembled + wired | 4 Oct | `check_motors.py` sees 14 servos; IMU frame correct | ☐ |
| G4 Walking | 7 Oct | walks for 2+ min on the Xbox pad, turns, recovers | ☐ |
| G5 Brain demo | 8 Oct | Jetson drives it through the link; e-stop tested | ☐ |
| G6 Demo ready | 9 Oct | 30-min loop stable, spares kit packed | ☐ |

The full plan and owners are in [docs/build-plan.md](docs/build-plan.md).

## Repo layout

```
docs/          build plan, BOM, printing, wiring, bring-up, training, link spec, demo day
robot/         runs on the Pi: jero_walk.py (upstream policy loop + controller mux), setup, systemd
jero_link/     pip package: UDP + HMAC command protocol, client and receiver (Pi and Jetson)
brain/         runs on the Jetson or a laptop: keyboard teleop, follow-a-person behaviour
training/      GPU server: set up, train, watch in MuJoCo, deploy a policy
tools/         configure servos, e-stop, fetch the policy, generate the link key
upstream/      pinned git submodules (read-only): Open_Duck_Mini, _Runtime, _Playground
tests/         hardware-free tests (mux, follow control)
```

## Quick start by machine

**Laptop (setting up servos and development)**
```bash
git clone --recurse-submodules --shallow-submodules https://github.com/dev-ansh-r/Jero && cd Jero
pip install -e jero_link[test] numpy && make test
```

**Pi Zero 2 W** (flash the pre-built Duck image first; see [docs/bringup.md](docs/bringup.md))
```bash
git clone https://github.com/dev-ansh-r/Jero ~/Jero && ~/Jero/robot/setup_pi.sh
workon open-duck-mini-runtime && python ~/Jero/robot/jero_walk.py
```

**Jetson Orin Nano Super / laptop brain**
```bash
git clone https://github.com/dev-ansh-r/Jero && cd Jero && pip install -e jero_link
scp bdxv2@jero.local:.config/jero/link.key ~/.config/jero/link.key
python brain/teleop.py jero.local          # or: python brain/follow.py jero.local --dry-run
```

**GPU server**
```bash
git clone --recurse-submodules=upstream/Open_Duck_Playground --shallow-submodules https://github.com/dev-ansh-r/Jero
Jero/training/setup_server.sh && Jero/training/train.sh
```

**E-stop from any laptop on the network:** `python tools/estop.py jero.local`

## Pinned upstream

| Repo | Branch | Commit |
|---|---|---|
| [apirrone/Open_Duck_Mini](https://github.com/apirrone/Open_Duck_Mini) | v2 | `b23317a` (STLs, docs, `BEST_WALK_ONNX_2.onnx`) |
| [apirrone/Open_Duck_Mini_Runtime](https://github.com/apirrone/Open_Duck_Mini_Runtime) | v2 | `376de65` |
| [apirrone/Open_Duck_Playground](https://github.com/apirrone/Open_Duck_Playground) | main | `b9be205` |

Bump a submodule deliberately (`git -C upstream/<repo> checkout <sha>`), then note why in the commit message.

## Credits

The mechanical design, runtime and training environment are by Antoine Pirrone and the Open Duck community
(Open_Duck_Mini is Apache-2.0). Jero only adds the integration layer in this repo.
