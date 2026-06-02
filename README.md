# Robot Data Collection Tool

Isaac Sim + Franka teleop tool for collecting manipulation demos (HDF5) and
running eval / inference trials with summary stats.

All commands assume Windows + Isaac Sim launcher `./run.bat` in the repo root.

---

## 1. Data collection

Full command with every applicable flag:

```powershell
./run.bat --task in_drawer --scene warehouse --container plate --mode ik --repeat 5 --debug
```

A task supplies the **instruction + special behavior**; scene and container are
chosen separately, so the same task works across scenes:

```powershell
./run.bat --task behind_named --scene warehouse --container plate
./run.bat --task behind_named --scene home_cabinet --container drawer
```

(Only `./run.bat` is required — everything else falls back to the values in
`configs/`. The line above just shows all available flags at once.)

### Flags

| Flag           | Example            | What it does |
|----------------|--------------------|--------------|
| `--task`       | `--task in_drawer` | **Required for collection**: selects the instruction + special settings. Must be a key in `task_config.TASKS`. See section 3. |
| `--scene`      | `--scene warehouse` | Which USD scene to load (file under `usdfiles/scenes/`). Defaults to the value in `configs/usd_config.py`. |
| `--container`  | `--container plate` | Which container to spawn (key in `container_config.CONTAINERS`). Decides the success box + fills the `{container}` instruction slot. |
| `--mode`       | `--mode ik`        | Controller type: `ik` (default) or `rmpflow`. |
| `--repeat`     | `--repeat 5`       | Reuse the same object layout for N **successful** demos before re-randomizing. Manual R resets don't consume the count. Omit = randomize every reset. |
| `--debug`      | `--debug`          | Show the red IK/RMP target ball in the scene. |
| `--template`   | `--template basic` | **[Deprecated]** Alias for `--task` (kept for backward compatibility). |

Output: appends a new `demo_<N>` group to `datasets/dataset.hdf5`. The first
frame of each demo is also saved to `preview_recording.png` for a quick check.

---

## 2. Eval / inference

Full command with every applicable flag:

```powershell
./run.bat --eval --task stove_turn --scene warehouse --container stove --max-steps 600 --trials 20 --debug
```

(`--eval` alone = no recording, no stats. Add `--max-steps` to enable the
timeout + per-trial stats. Add `--trials` to auto-exit after N trials.)

### Flags

| Flag           | Example            | What it does |
|----------------|--------------------|--------------|
| `--eval`       | `--eval`           | Eval mode: **no data is recorded** (replaces the old `--inference`). |
| `--task`       | `--task stove_turn` | Selects the instruction + special settings (see section 3). |
| `--scene`      | `--scene warehouse` | Which USD scene to load. |
| `--container`  | `--container stove` | Which container to spawn. |
| `--mode`       | `--mode ik`        | Controller type: `ik` (default) or `rmpflow`. |
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

A **task** lives in `configs/task_config.py` and only needs an `instruction`
sentence. Scene and container are **not** part of the task — they're picked at
run time via `--scene` / `--container`. So one task (e.g. `behind_named`) works
across every scene/container combo without duplication:

```powershell
./run.bat --task behind_named --scene warehouse --container plate
./run.bat --task behind_named --scene home_cabinet --container drawer
```

### Minimal task (instruction only)

For ordinary pick-and-place tasks, that's the whole entry:

```python
"behind_named": {
    "instruction": "pick up the {obj} behind the {front} and place it on the {container}",
},
```

Run it: `./run.bat --task behind_named --scene warehouse --container plate`. Done — no edits to `main.py` or any core file.

### Instruction slots

The `instruction` string is filled at runtime. Available `{...}` slots:

| Slot | Filled with | Notes |
|------|-------------|-------|
| `{obj}` | Target object class name | The object being picked up |
| `{container}` | Active container name (from `--container`) | Where the object goes |
| `{left}` | Background object at `point_left` | Errors if that point is empty |
| `{right}` | Background object at `point_right` | Errors if that point is empty |
| `{front}` | Background object at `point_front` | Errors if that point is empty |
| `{fixture}` | `support_fixture.name` | Only valid if the task has a `support_fixture` |

### Full task field reference

All fields except `instruction` are optional — add them only when the task needs special behavior:

| Field | Type | What it does | Example use |
|-------|------|--------------|-------------|
| `instruction` | str | **(Required)** The instruction sentence with `{...}` slots. | Every task |
| `bg_skip` | list | Background spawn points to leave **empty** (no object spawned there). | `["point_front"]` to clear the area in front of a stove |
| `spawn_overrides` | dict | Per-point position deltas (`x/y/z_offset`, meters) applied at spawn. | Raise target 15cm so it drops into a drawer |
| `support_fixture` | dict | A dynamic USD spawned during the task (a cabinet, stool, pan...). See sub-table below. | Spawn a cabinet for the "in drawer" task |
| `target_offset_above_fixture` | float | Spawn the target N meters **above** the fixture (world frame). | `0.30` = target sits 30cm on top of a stool |
| `target_inside_fixture` | dict | Spawn the target inside a fixture sub-prim (`subpath` + local `offset`). | Place target inside a `/drawer` sub-prim |
| `success_criterion` | str | Override the success rule. Default `"place_in_container"`. Use `"turn_knob"` for knob tasks. | Stove knob-turning task |
| `skip_occlusion` | bool | Skip the two-phase occlusion-rate calc (for tasks with no target to occlude). Default `False`. | Knob tasks |

#### `support_fixture` sub-fields

| Field | Type | Required? | What it does |
|-------|------|-----------|--------------|
| `usd` | str | yes | Path to the USD file to spawn (relative to project root) |
| `anchor_point` | str | yes | Spawn point under `/World/target_points/` that positions the fixture |
| `name` | str | yes | Fills the `{fixture}` slot; auto-hidden in the occlusion baseline |
| `scale` | float | no | Uniform scale (same X/Y/Z). Omit = use the USD's native scale |
| `hide_init` | bool | no | Hide during Phase 1 baseline, reveal in Phase 2. Default `False` |
| `target_objects_dir` | str | no | Restrict target selection to this dir (e.g. only small objects fit in a drawer). Default: all objects |

### Example: special task with a fixture

```python
"in_drawer": {
    "instruction": "pick up the {obj} in the top drawer of the {fixture} and place it on the {container}",
    "support_fixture": {
        "usd": "usdfiles/cabinet/white_cabinet_open.usd",
        "anchor_point": "cabinet_pos",
        "name": "white_cabinet",
        "scale": 1.4,
        "hide_init": True,
        "target_objects_dir": "usdfiles/objects/small",
    },
    "spawn_overrides": {
        "target_point": {"z_offset": 0.15},  # raise target so it drops into the drawer
        "point_left":   {"z_offset": 0.35},  # raise BG object so it clears the cabinet body
    },
    "bg_skip": ["point_front"],
},
```

Run: `./run.bat --task in_drawer --scene warehouse --container plate`

### Example: knob task (no placement, no occlusion)

```python
"stove_turn": {
    "instruction": "turn the knob on the {container}",
    "bg_skip": ["point_front"],
    "success_criterion": "turn_knob",  # success = knob rotated past threshold
    "skip_occlusion": True,            # no target object to occlude
},
```

Run: `./run.bat --task stove_turn --scene warehouse --container stove`
(The stove container's knob spec lives in `configs/container_config.py`.)

> **Object pools:** objects are scanned recursively from `usdfiles/objects/`
> (including subdirectories). Background objects can be any of them; the target
> defaults to any object too, unless `target_objects_dir` restricts it.

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
