# Robot Data Collection Tool

Isaac Sim + Franka teleop tool for collecting manipulation demos (HDF5) and
running eval / inference trials with summary stats.

All commands assume Windows + Isaac Sim launcher `./run.bat` in the repo root.

---

## 1. Data collection

Full command with every applicable flag:

```powershell
./run.bat --task in_drawer --scene Warehouse --mode ik --out my_run --debug
```

A task supplies the **instruction + special behavior**. The **object layout and
container are deterministic** — you set them in `configs/scene_config.py` (see
section 2), not on the command line. The same task works across scenes:

```powershell
./run.bat --task behind_named --scene Warehouse
./run.bat --task behind_named --scene home_cabinet
```

(Only `./run.bat` is required — everything else falls back to the values in
`configs/`. The line above just shows all available flags at once.)

### Flags

| Flag           | Example            | What it does |
|----------------|--------------------|--------------|
| `--task`       | `--task in_drawer` | **Required for collection**: selects the instruction + special settings. Must be a key in `task_config.TASKS`. See section 4. |
| `--scene`      | `--scene Warehouse` | Which USD scene to load (file under `usdfiles/scenes/`). Defaults to the value in `configs/usd_config.py`. |
| `--out`        | `--out my_run`     | Output dataset name → `datasets/<name>.hdf5` (default `dataset`). If that file already holds demos from a **different** config, an auto-suffixed file (`<name>_2.hdf5`, …) is used so one file never mixes layouts. |
| `--mode`       | `--mode ik`        | Controller type: `ik` (default) or `rmpflow`. |
| `--show-zone`  | `--show-zone`      | Draw the container's interior success/drop zone as a translucent box, to tune `container_config` `interior`. **The box shows up in camera RGB — tuning only, never while recording.** |
| `--debug`      | `--debug`          | Show the red IK/RMP target ball in the scene. |
| `--template`   | `--template basic` | **[Deprecated]** Alias for `--task` (kept for backward compatibility). |

Output: appends a new `demo_<N>` group to `datasets/<--out>.hdf5` (default
`dataset.hdf5`). The first frame of each demo is also saved to
`preview_recording.png` for a quick check. See section 5 for the file layout.

---

## 2. Scene layout (objects + container)

Object placement is **deterministic**, defined in `configs/scene_config.py`. You
assign exactly which object USD spawns at each spawn point, which container to
use, and how much each is randomized. This makes a run reproducible — the
recorded `data/initial_scene` (section 5) matches this file, so eval can replay
the exact same scene.

```python
# configs/scene_config.py
DEFAULT_RADIUS    = 0.05   # m   fallback XY spawn radius
DEFAULT_YAW_RANGE = 45.0   # deg fallback yaw half-range

LAYOUT = {
    "target_point": {"file": "spoon.usd",       "radius": 0.02, "yaw_range": 45.0},
    "point_right":  {"file": "mustard.usd",      "radius": 0.02, "yaw_range": 45.0},
    "point_left":   {"file": "banana.usd",       "radius": 0.02, "yaw_range": 45.0},
    "point_front":  {"file": "power_drill.usd",  "radius": 0.02, "yaw_range": 45.0},
    "container_pos": {"container": "plate",       "radius": 0.02, "yaw_range": 30.0},
}
```

### Per-point fields

| Field | Applies to | What it does |
|-------|------------|--------------|
| `file` | object points | USD filename (e.g. `spoon.usd`), resolved recursively under `usdfiles/objects/` so a bare name in a subdir (`bolt.usd` in `objects/small/`) is found. `None` → leave the point empty. |
| `container` | `container_pos` | Container to spawn, by key in `container_config.CONTAINERS` (keeps that file's interior / knob specs). `None` → no container. |
| `radius` | all | XY random-spawn radius (m) around the preset point. `0.0` → exact point. Omit → `DEFAULT_RADIUS`. |
| `yaw_range` | all | Z-axis (vertical) random yaw **half**-range in degrees, sampled from `[-yaw_range, +yaw_range]`. `0.0` → keep preset orientation. Omit → `DEFAULT_YAW_RANGE`. |
| `euler_deg` | all (optional) | `[rx, ry, rz]` base orientation (deg) that **replaces** the spawn point's preset orientation. Use it to fix an asset that spawns upside-down/sideways (e.g. `[180, 0, 0]` flips it upright). Random yaw is applied on top. |

> Position/angle re-randomize within these ranges **every reset** (data
> augmentation); only the object identity at each point is fixed. Spawn-point
> names must match the scene's prims under `/World/target_points/`.

> **Tuning the container zone:** run with `--show-zone` to see the success box
> drawn as a translucent cube, then adjust `interior` in
> `configs/container_config.py`. Remove the flag before recording.

---

## 3. Eval / inference

Eval is **keyboard-controlled** (same input path as collection; the ROS bridge
is currently removed). Full command with every applicable flag:

```powershell
./run.bat --eval --task basic --scene Warehouse --dataset my_run --headless --max-steps 600 --trials 20 --save_video
```

(`--eval` alone = no recording, no stats. Add `--max-steps` to enable the
timeout + per-trial stats. Add `--trials` to auto-exit after N trials.)

### Replaying a recorded scene (`--dataset`)

`--dataset <name>` loads `datasets/<name>.hdf5` and reproduces its
`data/initial_scene` — the **same objects + container** the dataset was
collected with. The **first** reset places everything at the exact recorded
positions/angles; **subsequent** resets keep the same objects but re-randomize
position/angle (per the `scene_config` ranges). The container choice comes from
the dataset, overriding `scene_config`.

### Flags

| Flag           | Example            | What it does |
|----------------|--------------------|--------------|
| `--eval`       | `--eval`           | Eval mode: **no data is recorded** (replaces the old `--inference`). |
| `--dataset`    | `--dataset my_run` | Reproduce the `initial_scene` recorded in `datasets/<name>.hdf5` (see above). |
| `--task`       | `--task basic`     | Selects the instruction + special settings (see section 4). Still needed: success rules + instruction live in the task. |
| `--scene`      | `--scene Warehouse` | Which USD scene to load. |
| `--headless`   | `--headless`       | Run Isaac Sim with no GUI viewport (recommended for eval; avoids RTX viewport init crashes). |
| `--mode`       | `--mode ik`        | Controller type: `ik` (default) or `rmpflow`. |
| `--max-steps`  | `--max-steps 600`  | Per-trial step timeout. Exceeding it = `timeout` (a counted failure). Enables trial stats. |
| `--trials`     | `--trials 20`      | Total counted trials before auto-exit + summary. Omit = run until window close. |
| `--save_video` | `--save_video`     | Save each trial as an MP4 under `eval_videos/` (named `eval_trial_<N>_<outcome>.mp4`). RGB cameras side by side (no depth). Also turns on trial tracking on its own. |
| `--debug`      | `--debug`          | Show the red IK/RMP target ball. |

> Container and object identities come from `configs/scene_config.py` (or from
> `--dataset` when replaying). Trial tracking (stats + per-trial segmentation)
> turns on when **any** of `--max-steps`, `--trials`, or `--save_video` is set.
> `--eval` alone stays pure no-record mode.

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

## 4. Creating a new task

A **task** lives in `configs/task_config.py` and only needs an `instruction`
sentence. Scene is picked at run time via `--scene`; container and objects come
from `configs/scene_config.py` (section 2). So one task (e.g. `behind_named`)
works across every scene/layout combo without duplication:

```powershell
./run.bat --task behind_named --scene Warehouse
./run.bat --task behind_named --scene home_cabinet
```

### Minimal task (instruction only)

For ordinary pick-and-place tasks, that's the whole entry:

```python
"behind_named": {
    "instruction": "pick up the {obj} behind the {front} and place it on the {container}",
},
```

Run it: `./run.bat --task behind_named --scene Warehouse`. Done — no edits to `main.py` or any core file.

### Instruction slots

The `instruction` string is filled at runtime. Available `{...}` slots:

| Slot | Filled with | Notes |
|------|-------------|-------|
| `{obj}` | Target object class name | The object being picked up |
| `{container}` | Container name (from `scene_config` `container_pos`) | Where the object goes |
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
    },
    "spawn_overrides": {
        "target_point": {"z_offset": 0.15},  # raise target so it drops into the drawer
        "point_left":   {"z_offset": 0.35},  # raise BG object so it clears the cabinet body
    },
    "bg_skip": ["point_front"],
},
```

Run: `./run.bat --task in_drawer --scene Warehouse` (set objects + the `drawer`
container in `configs/scene_config.py`)

### Example: knob task (no placement, no occlusion)

```python
"stove_turn": {
    "instruction": "turn the knob on the {container}",
    "bg_skip": ["point_front"],
    "success_criterion": "turn_knob",  # success = knob rotated past threshold
    "skip_occlusion": True,            # no target object to occlude
},
```

Run: `./run.bat --task stove_turn --scene Warehouse` (set the `stove` container
in `configs/scene_config.py`; its knob spec lives in
`configs/container_config.py`).

> **Objects:** which object spawns at each point is fixed in
> `configs/scene_config.py` (section 2), resolved recursively from
> `usdfiles/objects/`. Set a point's `file` to `None` to leave it empty.

---

## 5. File inspection

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

### HDF5 layout

```
data/                                       (group)
  ├─ env_args            { "env_name": "warehouse" }
  ├─ language_instruction "pick up the spoon and place it on the plate"  ← once, dataset-wide
  ├─ config_signature    JSON of env/container/task/objects (guards mixing)
  ├─ total               number of demos
  ├─ initial_scene/                          ← recorded once, from the first demo
  │    ├─ objects        { "point_right": "mustard", …, "target": "spoon", "container": "plate" }
  │    ├─ target/root_pose        (7,)  [x,y,z, qw,qx,qy,qz]  ← spawn placement pose
  │    ├─ point_right/root_pose   (7,)
  │    ├─ … point_left / point_front …
  │    └─ container/root_pose     (7,)
  ├─ demo_0/
  │    ├─ num_samples, success, occlusion_rate(s)
  │    ├─ initial_state/   (robot + target_object first-frame pose/vel)
  │    └─ obs/             actions, joint_pos/vel, eef_pos/quat, cameras (rgb+depth), states/…
  └─ demo_1/ …
```

`language_instruction` and `initial_scene` are written **once** (from the first
demo) and are dataset-wide. `--dataset` (section 3) replays `initial_scene`;
`config_signature` is what `--out` compares to avoid mixing layouts in one file.
`--readfile` prints `initial_scene` first (right under the `data/` attributes),
then the demos.

---

## 6. Keyboard during operation

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

## 7. Notes for future ROS integration

The ROS2 bridge is **currently removed** — eval runs on keyboard control, the
same input path as collection. The hooks for re-adding ROS later are still in
place:

During `--eval`, all 5 RGBD cameras are initialized in
`setup_scene_with_occlusion()` (same as collection mode) and stay live for the
whole session. `camera_mgr.get_all_camera_data()` returns the current RGB
(uint8 256×256×3) + depth (uint16 mm 256×256) per camera on every call — use it
from a ROS publisher. Robot actions go through
`controller.apply_control(delta_pos, gripper_cmd, delta_rot)`, which a ROS
subscriber can drive in place of `input_mgr.get_command()`. To re-enable the
bridge, restore the `enable_extension("isaacsim.ros2.bridge")` call near the top
of `main()` (before `open_stage`).
