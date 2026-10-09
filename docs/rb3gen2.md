# Running Jero on the Qualcomm RB3 Gen 2 (instead of the Pi Zero 2 W)

Branch `rb3gen2`. The RB3 Gen 2 (QCS6490) takes the Pi Zero's place as the robot computer: it runs
the same 50 Hz walk (upstream policy loop, `robot/jero_walk.py`), talks to the servos through the
same Pico bridge, and pairs with the same PS4 pad. Only the Raspberry Pi specific parts change.

## What changes and what doesn't

| Part | Pi Zero 2 W | RB3 Gen 2 |
|---|---|---|
| Walk loop, policy, Wi-Fi link, PS4 mapping | `jero_walk.py` | **same** |
| Servo bus | Pico on the Pi's only USB data port | **Pico on a USB-A port**, same firmware, same `/dev/ttyACM0` (stable name `/dev/jero-servo`) |
| Servo IO | `robot/feetech_io.py` | **same** (rustypot not needed) |
| IMU | MPU9250 board on the Pi's I2C (3.3 V) | MPU9250 on an RB3 I2C bus **through a level shifter** (see wiring), or later the RB3's own ICM-42688 |
| Foot switches | upstream `feet_contacts` (Adafruit `board`, Pi only) | `robot/board_shims.py`: `none` (unwired) or `gpiod` |
| Eyes, antennas, projector | upstream, Pi GPIO | disabled (stand-ins raise if enabled) |
| OS | Raspberry Pi OS | **Ubuntu** (aarch64) |
| Setup | `robot/setup_pi.sh` | **`robot/setup_rb3.sh`** |
| Power | 5 V from a buck converter | **12 V straight from the 3S pack** (no buck) |

`jero_walk.py` detects the board (`/proc/device-tree/model`): on anything that isn't a Raspberry
Pi it installs the board shims, and requires `--imu mpu6050`. Force it with `--board other`.

## Wiring

> **Check the RB3 Gen 2 hardware docs before connecting anything to its expansion headers.**
> 96Boards-style low-speed connectors run their GPIO and I2C at **1.8 V**. The MPU9250 breakout
> has 3.3 V pull-ups on SDA/SCL: wired straight on, it would push 3.3 V into 1.8 V pins.

| Signal | Connect | Notes |
|---|---|---|
| Servo bus | Pico USB → RB3 **USB-A** | Pico powered by USB; its GND also to the servo supply GND (star point at the battery −) |
| IMU (MPU9250) | RB3 I2C SDA/SCL **via a bidirectional level shifter** (1.8 V ↔ 3.3 V, e.g. PCA9306/TXS0102 board), 3.3 V + GND for the IMU | find the bus with `i2cdetect -l`, check `i2cdetect -y <bus>` shows `68` |
| Foot switches (optional) | each switch between a GPIO and GND | safe at any logic voltage (the switch only pulls to GND); find chip/line with `gpioinfo` |
| Power | 3S pack → RB3 DC input (12 V) | **check the RB3's input voltage range**: a full 3S pack is 12.6 V |
| PS4 pad | Bluetooth (onboard) | same pairing as on the Pi |

## Setup (on the RB3, Ubuntu)

```bash
git clone -b rb3gen2 https://github.com/dev-ansh-r/jero ~/Jero
~/Jero/robot/setup_rb3.sh          # packages, Python env, runtime @376de65, policy, key, Bluetooth, udev
# log out and back in (new groups: dialout, i2c, input)
```

It installs the runtime with `--no-deps` (its dependency list pulls Pi-only Adafruit GPIO and
pypot) plus only what the walk uses: numpy, onnxruntime, scipy, pygame, smbus2, and optionally
rustypot and gpiod.

## Bring-up checklist

```bash
# 1. servos (power on, nothing moves)
python ~/Jero/tools/bus_test.py --port /dev/jero-servo
python ~/Jero/tools/pose.py zero --port /dev/jero-servo      # all joints to 0, check, Enter releases

# 2. IMU: new board = new mounting, so calibrate both again
i2cdetect -l && i2cdetect -y <bus>                           # 68
python ~/Jero/tools/imu_check.py --bus <bus>                 # mounting + gyro bias (sets the bus too)
python ~/Jero/tools/imu_tilt.py --port /dev/jero-servo       # standing tilt correction

# 3. policy + IMU, no servos
python ~/Jero/tools/policy_dryrun.py --seconds 10

# 4. PS4 pad: pair once (sudo btmgmt ssp on / bluetoothctl pair, trust, connect / sudo btmgmt ssp off)
python ~/Jero/tools/pad_test.py

# 5. walk (in the air first; X = cross starts/pauses)
python ~/Jero/robot/jero_walk.py --board other --imu mpu6050 --no-link --serial-port /dev/jero-servo
```

Foot switches wired? Put them in `~/.config/jero/board.json`:

```json
{"feet": {"backend": "gpiod", "chip": "/dev/gpiochip4", "lines": [22, 27]}}
```

(chip and line numbers from `gpioinfo`; the ones above are placeholders), or pass
`--feet gpiod --feet-chip ... --feet-lines L,R`. Without switches the walk runs with both feet read
as "not touching", as on an unwired Pi (in sim: walks, recovers from pushes less well).

## Before trusting it on the floor

- **Mass and balance.** The RB3 with heatsink is 210-300 g where the Pi Zero was ~10 g. In sim,
  extra mass at the trunk's centre of mass is tolerated, but mass **behind or above** it costs push
  recovery the most (300 g 4 cm back or 10 cm up: 1/5 vs 4/5). Mount it low and central.
  Check in sim: `training/jero_sim.py --payload-kg <kg> --payload-at <dx> <dy> <dz> gate -o ...`.
  Retraining with the real mass is the proper fix after the event.
- **Re-run `imu_check` and `imu_tilt`** after mounting: the IMU's position and the standing pose
  can both change.
- **Not verified on hardware yet:** this branch is tested against upstream's real modules on a
  laptop (tests/test_board_shims.py: the walk script imports and builds with no Pi GPIO library),
  but not yet run on an RB3. Expect to adjust the I2C bus number, GPIO lines and group permissions.

## Later: the RB3's own IMU

The RB3 Gen 2 has a TDK ICM-42688 on the board, read through Qualcomm's sensing hub (QSH client
API, test with `ssc_drva_test`), not plain I2C. Using it means a small bridge into an `Imu` class like
`robot/imu_mpu6050.py`, and the board itself becomes the IMU mount (rigid, no foam).
