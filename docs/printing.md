# Printing

The STLs are in [`upstream/Open_Duck_Mini/print/`](https://github.com/apirrone/Open_Duck_Mini/tree/b23317a485b3cec7d8417f352478778b3475173c/print). Settings come from the upstream
[print guide](https://github.com/apirrone/Open_Duck_Mini/blob/b23317a485b3cec7d8417f352478778b3475173c/docs/print_guide.md): **PLA, 15% infill**, except `foot_bottom_tpu`
(**TPU 95A, 40% infill**). Community variants in `print/mods/` are not used for Jero v1.

Priority order: **A** = legs/hips/trunk (needed first for assembly), **B** = neck/head/body,
**C** = expression pack only (skip if we're behind).

| ✓ | Part | Qty | Material | Pri |
|---|---|---|---|---|
| ☐ | foot_top.stl | 2 | PLA | A |
| ☐ | foot_side.stl | 2 | PLA | A |
| ☐ | foot_bottom_pla.stl | 2 | PLA | A |
| ☐ | foot_bottom_tpu.stl | 2 | **TPU** | A |
| ☐ | knee_to_ankle_left_sheet.stl | 4 | PLA | A |
| ☐ | knee_to_ankle_right_sheet.stl | 4 | PLA | A |
| ☐ | leg_spacer.stl | 4 | PLA | A |
| ☐ | left_roll_to_pitch.stl | 1 | PLA | A |
| ☐ | right_roll_to_pitch.stl | 1 | PLA | A |
| ☐ | roll_motor_bottom.stl | 2 | PLA | A |
| ☐ | roll_motor_top.stl | 2 | PLA | A |
| ☐ | trunk_bottom.stl | 1 | PLA | A |
| ☐ | trunk_top.stl | 1 | PLA | A |
| ☐ | neck_left_sheet.stl | 1 | PLA | B |
| ☐ | neck_right_sheet.stl | 1 | PLA | B |
| ☐ | head_pitch_to_yaw.stl | 1 | PLA | B |
| ☐ | head_yaw_to_roll.stl | 1 | PLA | B |
| ☐ | head_roll_mount.stl | 1 | PLA | B |
| ☐ | head.stl | 1 | PLA | B |
| ☐ | head_bot_sheet.stl | 1 | PLA | B |
| ☐ | left_cache.stl | 1 | PLA | B |
| ☐ | right_cache.stl | 1 | PLA | B |
| ☐ | body_front.stl | 1 | PLA | B |
| ☐ | body_middle_bottom.stl | 1 | PLA | B |
| ☐ | body_middle_top.stl | 1 | PLA | B |
| ☐ | body_back.stl | 1 | PLA | B |
| ☐ | battery_pack_lid.stl | 1 | PLA | B |
| ☐ | left_antenna_holder.stl | 1 | PLA | C |
| ☐ | right_antenna_holder.stl | 1 | PLA | C |
| ☐ | left_eye.stl | 1 | PLA | C |
| ☐ | right_eye.stl | 1 | PLA | C |
| ☐ | bulb.stl | 1 | PLA | C |
| ☐ | flash_light_module.stl | 1 | PLA | C |
| ☐ | flash_reflector_interface.stl | 1 | PLA | C |
| ☐ | speaker_interface.stl | 1 | PLA | C |
| ☐ | speaker_stand.stl | 1 | PLA | C |

Totals: 36 STL types, 51 pieces (2 of them TPU).

## Notes

- The sim uses slicer-estimated masses (see upstream `docs/sim2real.md`). Stick to 15% infill and
  default walls. Heavier parts change the dynamics the policy was trained on.
- Print one `knee_to_ankle_*_sheet` first and test-fit a servo and an M3 insert before batching the rest.
- TPU: print slowly (~20–25 mm/s) on a direct-drive extruder, with retraction mostly off.
- Weigh each finished batch and log it here, so there's data if we retrain with measured masses.
