# Bring-up runbook

Work through the steps in order, and don't start a step until the previous one passes.
Put Jero on a stand with its feet off the ground until step 6.

## 0. Flash + first boot (bench, no servos)

1. Download the pre-built Duck image (v0.2.3) from the
   [runtime releases](https://github.com/apirrone/Open_Duck_Mini_Runtime/releases). Flash the `.zip` directly with Raspberry Pi Imager
   onto a 32 GB card, and pre-set your Wi-Fi in the Imager.
2. `ssh bdxv2@bdxv2.local` (default password `ilovemyduck`), then:
   ```bash
   passwd
   sudo hostnamectl set-hostname jero && sudo reboot      # afterwards: ssh bdxv2@jero.local
   sudo raspi-config nonint do_expand_rootfs               # if the card shows ~7 GB
   git clone https://github.com/dev-ansh-r/Jero ~/Jero && ~/Jero/robot/setup_pi.sh
   ```
3. Copy `~/.config/jero/link.key` to the brain machine(s).

**Pass:** `setup_pi.sh` prints `Done.`, and `python -c "import jero_link"` works inside `workon open-duck-mini-runtime`.

## 1. Servo IDs (before assembly)

```bash
workon open-duck-mini-runtime
~/Jero/tools/configure_servos.sh          # one servo on the bus at a time
```

**Pass:** 14 labelled servos, each with its horn fitted at the zero position.

## 2. IMU

```bash
cd ~/Open_Duck_Mini_Runtime && python3 mini_bdx_runtime/mini_bdx_runtime/raw_imu.py
python3 scripts/imu_server.py        # laptop: python3 scripts/imu_client.py --ip jero.local
```

**Pass:** tilting the body forward, back and sideways moves the frame the same way. If it's mirrored,
set `imu_upside_down` in `~/duck_config.json`. Then run `scripts/calibrate_imu.py`.

### 2b. IMU: MPU6050 (GY-521) instead of BNO055

The policy only reads raw gyro + accelerometer, so an MPU6050 works. Mount it **rigidly** where the
BNO055 goes (screws or a printed clip, no foam tape), wired to the same pins (VCC to 3V3, AD0 to GND).

```bash
i2cdetect -y 1                                   # expect 68
~/open-duck-mini-runtime/bin/python tools/imu_check.py   # guided, ~1 min, robot in your hands
```

**Pass:** `ALL PASS`. That saves the measured mounting + gyro bias to `~/.config/jero/imu.json`.
From then on run the walk with `--imu mpu6050` (or `JERO_IMU=mpu6050` in `/etc/default/jero-walk`).
Re-run the check whenever the IMU is unscrewed. At every start, hold the robot still for 2 s:
the gyro bias is re-measured (falls back to the saved one if it moved).

## 3. Motors

```bash
python3 scripts/check_voltage.py && python3 scripts/check_motors.py
```

**Pass:** all 14 IDs respond and the bus voltage reads 7.4–8.4 V.

## 4. Joint offsets

```bash
cd scripts && python find_soft_offsets.py       # write the results into ~/duck_config.json
```

**Pass:** in the init pose (`turn_on.py`), the legs look symmetric and the feet sit parallel to the floor.

## 5. Foot switches

```bash
python3 scripts/fc_test.py
```

**Pass:** pressing each foot flips the matching value.

## 6. Walk (Xbox pad, upstream path first)

Pair the pad (see the upstream `docs/INSTALL.md` §4), then:

```bash
cd ~/Open_Duck_Mini_Runtime/scripts && python v2_rl_walk_mujoco.py --onnx_model_path ~/BEST_WALK_ONNX_2.onnx
```

**Pass:** it stands, steps in place, walks and turns on the sticks, and `A` pauses. If it leans, use `--pitch_bias`
(degrees), or D-pad up/down to trim the gait frequency.

## 7. Walk through Jero (pad + link)

```bash
python ~/Jero/robot/jero_walk.py                  # pad still has priority
# brain:  python brain/teleop.py jero.local
# any laptop:  python tools/estop.py jero.local
```

**Pass:** teleop drives it, touching the pad sticks takes over immediately, killing teleop brings it to a
stop within 0.5 s, and `estop.py` pauses it.

## 8. Autostart (only once 7 passes)

```bash
echo 'JERO_WALK_ARGS="--pitch_bias 0"' | sudo tee /etc/default/jero-walk
~/Jero/robot/setup_pi.sh --replace-autostart && sudo reboot
journalctl -u jero-walk -f
```
