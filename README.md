# Robot Data Collection Tool

Isaac Sim + Franka teleop tool for collecting manipulation demos (HDF5) and
running eval / inference trials with summary stats.

All commands assume Windows + Isaac Sim launcher `./run.bat` in the repo root.

---

## 1. Data collection

Full command with every applicable flag:

```powershell
./run.bat --task warehouse_in_drawer --repeat 5 --debug
```

Or legacy way (without `--task`):

```powershell
./run.bat --mode ik --scene home_cabinet --template cabinet_open --container drawer --repeat 5 --debug
```

(Only `./run.bat` is required — everything else falls back to the values in
`configs/`. The line above just shows all available flags at once.)

### Flags

| Flag           | Example            | What it does |
|----------------|--------------------|--------------|
| `--task`       | `--task warehouse_in_drawer` | **Recommended**: load a pre-configured task profile (scene + template + container + support_fixture + object filtering all bundled). Must be a key in `task_config.TASKS`. Overrides `--scene`, `--template`, `--container`. |
| `--mode`       | `--mode ik`        | Controller type: `ik` (default) or `rmpflow`. |
| `--scene`      | `--scene home_cabinet` | Which USD scene to load (file under `usdfiles/scenes/`). Defaults to the value in `configs/usd_config.py`. Only used if `--task` is not set. |
| `--template`   | `--template cabinet_open` | Overrides `instruction_config.ACTIVE_TEMPLATE` for this run. Must be a key in `TEMPLATES`. Decides the language instruction and (for `stove_turn`) the success rule. Only used if `--task` is not set. |
| `--container`  | `--container drawer` | Overrides `container_config.ACTIVE_CONTAINER`. Must be a key in `CONTAINERS`. Decides which container is spawned + its success box. Only used if `--task` is not set. |
| `--repeat`     | `--repeat 5`       | Reuse the same object layout for N **successful** demos before re-randomizing. Manual R resets don't consume the count. Omit = randomize every reset. |
| `--debug`      | `--debug`          | Show the red IK/RMP target ball in the scene. |

Output: appends a new `demo_<N>` group to `datasets/dataset.hdf5`. The first
frame of each demo is also saved to `preview_recording.png` for a quick check.

---

## 2. Eval / inference

Full command with every applicable flag:

```powershell
./run.bat --eval --task warehouse_in_drawer --max-steps 600 --trials 20 --debug
```

(`--eval` alone = no recording, no stats. Add `--max-steps` to enable the
timeout + per-trial stats. Add `--trials` to auto-exit after N trials.)

### Flags

| Flag           | Example            | What it does |
|----------------|--------------------|--------------|
| `--eval`       | `--eval`           | Eval mode: **no data is recorded** (replaces the old `--inference`). |
| `--task`       | `--task warehouse_in_drawer` | **Recommended**: load a pre-configured task profile. Overrides `--scene`, `--template`, `--container`. |
| `--mode`       | `--mode ik`        | Controller type: `ik` (default) or `rmpflow`. |
| `--scene`      | `--scene home_cabinet` | Which USD scene to load. Only used if `--task` is not set. |
| `--template`   | `--template stove_turn` | Overrides the active template. Only used if `--task` is not set. |
| `--container`  | `--container stove` | Overrides the active container. Only used if `--task` is not set. |
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

## 3. Creating a new task

A **task profile** bundles scene + template + container + support fixture + object filtering into a single named entry. Instead of juggling multiple `--scene`, `--template`, `--container` flags, just use `--task <name>`.

### Task profile anatomy

Open `configs/task_config.py` and add an entry to the `TASKS` dict:

```python
"warehouse_in_drawer": {
    # === Required ===
    "scene": "Warehouse",           # USD scene filename (no .usd)
    "container": "plate",           # Container key from container_config.CONTAINERS
    "template": "in_drawer",        # Instruction template key from instruction_config.TEMPLATES
    
    # === Optional ===
    "support_fixture": {            # A dynamic USD spawned during the task
        "usd": "usdfiles/cabinet/white_cabinet_open.usd",
        "anchor_point": "cabinet_pos",  # Spawn point under /World/target_points/
        "name": "white_cabinet",    # Name for {fixture} slot in instructions
        "scale": 1.5,               # Uniform scale (XYZ same). Omit to use USD default.
        "hide_init": True,          # Hide during Phase 1 (baseline), show in Phase 2
        "target_objects_dir": "usdfiles/objects/small",  # Restrict target to this dir
    },
    
    "spawn_overrides": {            # Position/orientation deltas for spawn points
        "target_point": {"z_offset": 0.15},  # 15cm above target_point
        "point_left": {"z_offset": 0.25},    # 25cm above to avoid clipping
    },
    
    "bg_skip": ["point_front"],     # Background spawn points to leave empty
    
    "target_offset_above_fixture": 0.30,    # (Alternative) target 30cm above fixture
    "target_inside_fixture": {              # (Alternative) target inside fixture sub-prim
        "subpath": "/drawer",
        "offset": [0.0, 0.0, 0.1],  # 10cm up from drawer origin in local frame
    },
}
```

### Quick reference: support_fixture fields

| Field | Type | Required? | Example | What it does |
|-------|------|-----------|---------|--------------|
| `usd` | str | yes | `"usdfiles/cabinet/white_cabinet_open.usd"` | Path to USD file to spawn |
| `anchor_point` | str | yes | `"cabinet_pos"` | Spawn point under `/World/target_points/` to position the fixture |
| `name` | str | yes | `"white_cabinet"` | Name for `{fixture}` slot in instructions; auto-hidden in occlusion baseline |
| `scale` | float | no | `1.5` | Uniform scale (same for X, Y, Z). Omit = use USD default. |
| `hide_init` | bool | no | `True` | Hide during Phase 1 baseline, show in Phase 2. Default: `False`. |
| `target_objects_dir` | str | no | `"usdfiles/objects/small"` | Restrict target object selection to this directory. Default: all objects. |

### Quick reference: spawn_overrides

Apply position/orientation deltas to individual spawn points at runtime:

```python
"spawn_overrides": {
    "target_point": {
        "x_offset": 0.1,      # Shift +10cm in X
        "y_offset": -0.05,    # Shift -5cm in Y
        "z_offset": 0.2,      # Shift +20cm in Z
    },
    "point_left": {"z_offset": 0.15},
}
```

### Minimal example: add a new task in 3 steps

**Step 1:** Create the task entry in `configs/task_config.py`:
```python
"warehouse_on_stool": {
    "scene": "Warehouse",
    "container": "plate",
    "template": "on_fixture",
    "support_fixture": {
        "usd": "usdfiles/fixtures/stool.usd",
        "anchor_point": "target_point",
        "name": "stool",
    },
    "target_offset_above_fixture": 0.30,
}
```

**Step 2:** Ensure the template exists in `configs/instruction_config.py`:
```python
"on_fixture": "pick up the {obj} on the {fixture} and place it on the {container}",
```

**Step 3:** Run it:
```powershell
./run.bat --task warehouse_on_stool
```

No edits to `main.py`, `object_spawner.py`, or `container_manager.py` needed!

---

## 4. File inspection

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

## 5. Keyboard during operation

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

## 6. Notes for future ROS integration

During `--eval`, all 5 RGBD cameras are initialized in
`setup_scene_with_occlusion()` (same as collection mode) and stay live for the
whole session. `camera_mgr.get_all_camera_data()` returns the current RGB
(uint8 256×256×3) + depth (uint16 mm 256×256) per camera on every call — use it
from a ROS publisher. Robot actions go through
`controller.apply_control(delta_pos, gripper_cmd, delta_rot)`, which a ROS
subscriber can drive in place of `input_mgr.get_command()`.
