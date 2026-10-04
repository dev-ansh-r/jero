# Running the walking policy (`BEST_WALK_ONNX_2.onnx`)

For the Pi Zero 2 W build: 14 Feetech servos, MPU6050-family IMU (ours: MPU9250 board, MPU6500 chip) or BNO055.
`BEST_WALK_ONNX_2.onnx` is the upstream policy pinned in this repo (Open_Duck_Mini `b23317a`), trained for
the STS3215 (7.4 V, 2S). With 12 V STS3235 servos on 3S it still walks in sim but tolerates pushes less;
retraining for the STS3235 is the fix.

Order: **watch it in sim → robot on a stand → robot on the floor → drive it through Jero.**
Don't skip ahead. Each step catches problems that are cheap to fix at that stage and expensive later.

---

## 1. Watch it walk in simulation (laptop, 5 min)

From the repo on your laptop (the `.venv` already has MuJoCo and the policy runtime):

```powershell
cd D:\Projects\personal\Jero
.venv\Scripts\python.exe training\jero_sim.py --servo sts3215 view -o upstream\Open_Duck_Mini\BEST_WALK_ONNX_2.onnx
```

Use `--servo sts3235` instead if the robot has the 12 V STS3235 servos.

Click the window, then: **↑/↓** walk, **←/→** sidestep, **A/E** turn, **H** switch arrows to head control,
**P/M** faster/slower stepping. Double-click a body and Ctrl+right-drag to shove it.

This is what "working" looks like: compare the real robot against it.

Optional headless check (prints PASS/FAIL):

```powershell
.venv\Scripts\python.exe training\jero_sim.py --servo sts3215 gate -o upstream\Open_Duck_Mini\BEST_WALK_ONNX_2.onnx
```

---

## 2. Before the robot runs the policy

All of these from [bringup.md](bringup.md) must already pass:

| Step | Check | Pass |
|---|---|---|
| 0 | Pi set up ([bringup](bringup.md) step 0), `setup_pi.sh` done, `tools/policy_dryrun.py` | `Done.`, dry run `PASS` |
| 1 | Servo IDs 10–14, 20–24, 30–33, horns at zero | 14 labelled servos |
| 2 | IMU direction (BNO055: `raw_imu.py`; MPU6050: `tools/imu_check.py`) | tilting matches; `ALL PASS` for MPU6050 |
| 3 | `check_voltage.py` + `check_motors.py` | 14 IDs respond, 7.4–8.4 V |
| 4 | `find_soft_offsets.py` → `~/duck_config.json` | legs symmetric, feet flat in init pose |
| 5 | `fc_test.py` | each foot switch flips |

**Battery:** fully charged: 2S 8.2–8.4 V (STS3215) or 3S 12.4–12.6 V (STS3235). The servos get weak and the gait
gets sloppy as it sags (below ~7.4 V on 2S, ~11.1 V on 3S).

---

## 3. Get the policy onto the Pi

`setup_pi.sh` already downloads it to `~/BEST_WALK_ONNX_2.onnx`. Check:

```bash
ssh <user>@jero.local
ls -l ~/BEST_WALK_ONNX_2.onnx          # should exist, a few hundred KB
```

If it's missing: `~/Jero/tools/fetch_policy.sh` (downloads the pinned file to `~`).

---

## 4. First run: on a stand, feet off the ground

Hang or prop the robot so **the feet can't touch anything**. You're checking that every joint moves the
right way and nothing jerks, collides or overheats.

Pair the Xbox pad first (upstream `docs/INSTALL.md` §4). Then run the **upstream** script, which is the
reference behaviour with no Jero code involved:

```bash
cd ~/Open_Duck_Mini_Runtime/scripts      # venv is active from ~/.bashrc (bringup step 0)
python v2_rl_walk_mujoco.py --onnx_model_path ~/BEST_WALK_ONNX_2.onnx
```

> The upstream scripts open the servo bus at `/dev/ttyACM0` (Waveshare adapter). With a USB-TTL
> adapter (`/dev/ttyUSB0`) run `sudo ln -sf /dev/ttyUSB0 /dev/ttyACM0` first (lasts until reboot), or use
> `jero_walk.py --serial-port /dev/ttyUSB0`.

> **MPU6050 instead of BNO055?** The upstream script only knows the BNO055. Use the Jero launcher
> for every run instead: `python ~/Jero/robot/jero_walk.py --imu mpu6050 --no-link` (same policy loop,
> Xbox pad still drives it). Hold the robot still for 2 s at start while it measures the gyro bias.

**Look for:**
- It moves into the init pose smoothly, then the legs start stepping.
- Left and right legs mirror each other. A leg moving the opposite way means a servo ID or offset is wrong:
  stop and recheck bring-up steps 1 and 4.
- No grinding, no part hitting another, no servo getting hot within a minute.

**Stop:** press **A** on the pad (pauses), or Ctrl+C in the terminal.

---

## 5. On the floor

Use a flat, grippy surface (the event floor type if you can), with space all around. Have one person ready to catch it.

Same command as step 4. Hold the robot upright on the floor, start it, let go once it's stepping in place.

| Xbox pad | Action |
|---|---|
| Left stick | walk forward/back, sideways |
| Right stick | turn |
| **A** | pause / resume (your first reaction to anything odd) |
| D-pad up / down | gait frequency trim |

**Pass ([G4](../README.md#status)):** stands, steps in place, walks and turns on the sticks, recovers from
a light nudge, and keeps going for **2+ minutes**.

### Tuning

| Symptom | Fix |
|---|---|
| Leans or drifts forward/back | `--pitch_bias <deg>`, small steps (±1–2°), e.g. `--pitch_bias 2` |
| Steps too fast/slow, shuffles | D-pad up/down while it runs |
| Falls immediately, flails | IMU direction wrong: redo bring-up step 2 (BNO055: `imu_upside_down` in `~/duck_config.json`) |
| One leg drags or the feet aren't flat | joint offsets: redo `find_soft_offsets.py` |
| Gets weaker over a few minutes | battery sagging: charge; check with `scripts/check_voltage.py` |
| Jerky, stuttering motion | USB latency rule missing: re-run `setup_pi.sh` (adds the udev rule), reboot |

---

## 6. Through Jero (pad + Wi-Fi link)

Once step 5 passes, run the Jero launcher. It's the same policy loop, with the Wi-Fi command link added
and the Xbox pad still taking priority:

```bash
python ~/Jero/robot/jero_walk.py --pitch_bias <your value>      # add --imu mpu6050 if you use it
```

From a laptop on the same network (with `pip install -e jero_link` and the link key copied from the Pi:
`scp <user>@jero.local:.config/jero/link.key ~/.config/jero/link.key`):

```bash
python brain/teleop.py jero.local        # drive from the keyboard
python tools/estop.py jero.local         # emergency stop from any laptop
```

**Pass ([bringup](bringup.md) step 7):** teleop drives it; touching the pad sticks takes over immediately;
killing teleop brings it to a stop within 0.5 s; `estop.py` pauses it.

---

## 7. Start on boot (only after step 6 passes)

```bash
echo 'JERO_WALK_ARGS="--pitch_bias <your value>"' | sudo tee /etc/default/jero-walk
~/Jero/robot/setup_pi.sh --replace-autostart && sudo reboot
journalctl -u jero-walk -f               # live log
```

---

## Safety checklist (every session)

- [ ] Battery charged, cell voltages matched
- [ ] A person within reach the first minute of every floor run
- [ ] Pad in hand: **A** pauses
- [ ] `tools/estop.py jero.local` ready on a laptop when using the link
- [ ] Touch the servos after a few minutes: hot means stop and let them cool
