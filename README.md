# Robot Data Collection Tool

Isaac Sim + Franka teleop tool for collecting manipulation demos (HDF5) and
running eval / inference trials with summary stats.

All commands assume Windows + Isaac Sim launcher `./run.bat` in the repo root.

---

## 1. Data collection

Full command with every applicable flag:

```powershell
./run.bat --mode ik --scene home_cabinet --template cabinet_open --container drawer --repeat 5 --debug
```

(Only `./run.bat` is required — everything else falls back to the values in
`configs/`. The line above just shows all available flags at once.)

### Flags

| Flag           | Example            | What it does |
|----------------|--------------------|--------------|
| `--mode`       | `--mode ik`        | Controller type: `ik` (default) or `rmpflow`. |
| `--scene`      | `--scene home_cabinet` | Which USD scene to load (file under `usdfiles/scenes/`). Defaults to the value in `configs/usd_config.py`. |
| `--template`   | `--template cabinet_open` | Overrides `instruction_config.ACTIVE_TEMPLATE` for this run. Must be a key in `TEMPLATES`. Decides the language instruction and (for `stove_turn`) the success rule. |
| `--container`  | `--container drawer` | Overrides `container_config.ACTIVE_CONTAINER`. Must be a key in `CONTAINERS`. Decides which container is spawned + its success box. |
| `--repeat`     | `--repeat 5`       | Reuse the same object layout for N **successful** demos before re-randomizing. Manual R resets don't consume the count. Omit = randomize every reset. |
| `--debug`      | `--debug`          | Show the red IK/RMP target ball in the scene. |

Output: appends a new `demo_<N>` group to `datasets/dataset.hdf5`. The first
frame of each demo is also saved to `preview_recording.png` for a quick check.

---

## 2. Eval / inference

Full command with every applicable flag:

```powershell
./run.bat --eval --mode ik --scene home_cabinet --template stove_turn --container stove --max-steps 600 --trials 20 --debug
```

(`--eval` alone = no recording, no stats. Add `--max-steps` to enable the
timeout + per-trial stats. Add `--trials` to auto-exit after N trials.)

### Flags

| Flag           | Example            | What it does |
|----------------|--------------------|--------------|
| `--eval`       | `--eval`           | Eval mode: **no data is recorded** (replaces the old `--inference`). |
| `--mode`       | `--mode ik`        | Controller type: `ik` (default) or `rmpflow`. |
| `--scene`      | `--scene home_cabinet` | Which USD scene to load. |
| `--template`   | `--template stove_turn` | Overrides the active template (language instruction + success rule). |
| `--container`  | `--container stove` | Overrides the active container. |
| `--max-steps`  | `--max-steps 600`  | Per-trial step timeout. Exceeding it = `timeout` (a counted failure). Enables trial stats. |
| `--trials`     | `--trials 20`      | Total counted trials before auto-exit + summary. Omit = run until window close. |
| `--save_video` | `--save_video`     | Save each trial as an MP4 under `eval_videos/` (named `eval_trial_<N>_<outcome>.mp4`). RGB cameras side by side (no depth). Also turns on trial tracking on its own. |
| `--debug`      | `--debug`          | Show the red IK/RMP target ball. |

> Trial tracking (stats + per-trial segmentation) turns on when **any** of
> `--max-steps`, `--trials`, or `--save_video` is set. `--eval` alone stays pure
> no-record mode.

### What counts as a "trial"

| Event                          | Outcome       | Counted? |
|--------------------------------|---------------|----------|
| Task success (termination_mgr) | `success`     | yes |
| Step counter ≥ `--max-steps`   | `timeout`     | yes |
| User presses **N**             | `manual_fail` | yes |
| User presses **R**             | `redo`        | **no** (do-over) |
| Window closed mid-trial        | discarded     | no |

Summary prints on auto-exit (`--trials` reached) and on window close:

```
========== Eval Summary ==========
Trials:        5 / 5
  Success:      3  (60.0%)
  Timeout:      1  (20.0%)
  Manual fail:  1  (20.0%)
Success steps: avg=312.3  min=247  max=412

Per-trial details:
  #1   success      287 steps  | "pick up the mug from the table center and place it on the stove"
  #2   timeout      600 steps  | "pick up the screw and place it on the drawer"
  #3   success      247 steps  | "turn the knob on the stove"
  #4   manual_fail  142 steps  | "pick up the gear and place it on the stove"
  #5   success      412 steps  | "pick up the bottle from the table center and place it on the basket"
==================================
```

---

## 3. File inspection

```powershell
# Print HDF5 structure + success/fail counts
./run.bat --readfile dataset

# Render one frame of one demo to PNG (cameras + depth, side by side)
./run.bat --checkfile dataset --demo 3 --index 50

# Render a whole demo as MP4
./run.bat --checkvideo dataset --demo 3
```

| Flag           | What it does |
|----------------|--------------|
| `--readfile`   | Dataset name (no `.hdf5`). Dumps group structure + per-demo success/fail summary. |
| `--checkfile`  | Dataset name. Renders one frame to PNG. Pair with `--demo` and `--index`. |
| `--checkvideo` | Dataset name. Renders a whole demo to MP4. Pair with `--demo`. |
| `--demo`       | Demo index (default 0) for `--checkfile` / `--checkvideo`. |
| `--index`      | Frame index (default 0) for `--checkfile`. |

---

## Keyboard during operation

| Key            | Action |
|----------------|--------|
| `W A S D Q E`  | Move EE (x ± / y ± / z ±) |
| `Z X T G C V`  | Rotate EE (roll / pitch / yaw, each pair = ±) |
| `K`            | Toggle gripper open / closed |
| `R`            | Reset scene. **Collection**: no save. **Eval**: redo, not counted. |
| `N`            | **Collection**: save current demo as failure. **Eval**: count as manual fail. |

`F` is avoided on purpose — Omniverse binds it to "Frame Selected" which moves
the perspective camera.

---

## Notes for future ROS integration

During `--eval`, all 5 RGBD cameras are initialized in
`setup_scene_with_occlusion()` (same as collection mode) and stay live for the
whole session. `camera_mgr.get_all_camera_data()` returns the current RGB
(uint8 256×256×3) + depth (uint16 mm 256×256) per camera on every call — use it
from a ROS publisher. Robot actions go through
`controller.apply_control(delta_pos, gripper_cmd, delta_rot)`, which a ROS
subscriber can drive in place of `input_mgr.get_command()`.
