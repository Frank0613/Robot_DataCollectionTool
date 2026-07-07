"""Offline occlusion-rate computation.

Data collection no longer measures occlusion live (that rebuilt the camera
render products every reset — leaking them so recording slowed over time — and
rendered a semantic-segmentation pass every frame). Each demo instead records
its full initial layout under `demo_<i>/initial_layout`, and occlusion is
measured afterwards by reproducing that layout and running the SAME two-phase
semantic-segmentation measurement the live path used to do inline:

  Phase 1 (baseline) : target only, static occluders + fixture hidden ->
                       count the target's unoccluded pixels per camera.
  Phase 2 (occluded) : add the background objects, restore fixture + static
                       occluders -> count the still-visible target pixels.
  occlusion_rate = 1 - visible / baseline    (per camera, clamped to [0, 1])

This module is shared by two callers:
  * main.py    — runs the pass in-process right after a collection session
                 ends (reusing the live managers, no Isaac Sim relaunch).
  * tools/compute_occlusion.py — standalone entry point that launches its own
                 headless Isaac Sim and builds the managers.
"""
import json
import os
from datetime import datetime

import h5py
import numpy as np


def set_render_visibility(stage, path_or_name, visible):
    """Toggle render visibility of a prim subtree (visibility only — colliders
    stay active so a hidden fixture still supports the target). Accepts an exact
    prim path or a bare prim name searched across the stage. Returns True if a
    matching prim was found."""
    from pxr import UsdGeom
    prim = stage.GetPrimAtPath(path_or_name)
    if not prim.IsValid():
        prim = None
        for p in stage.Traverse():
            if p.GetName() == path_or_name:
                prim = p
                break
    if prim is None or not prim.IsValid():
        return False
    imageable = UsdGeom.Imageable(prim)
    if not imageable:
        return False
    if visible:
        imageable.MakeVisible()
    else:
        imageable.MakeInvisible()
    return True


def read_layout(demo_group):
    """Return (roles, poses) for a demo, or None when it has no recorded layout.

    roles: {role: class_name}, e.g. {"target": "red_cube", "point_front": "apple",
            "container": "bowl", "fixture": "stool"}
    poses: {role: np.ndarray(7,)}  spawn pose [x,y,z, qw,qx,qy,qz]. Objects use
           the reference-root spawn pose (re-referenced + re-settled with
           physics here); container/fixture are static so it's their exact pose.
    """
    if 'initial_layout' not in demo_group:
        return None
    lg = demo_group['initial_layout']
    if 'objects' not in lg.attrs:
        return None
    roles = json.loads(lg.attrs['objects'])
    poses = {}
    for role in roles:
        ds = f"{role}/root_pose"
        if ds in lg:
            poses[role] = np.asarray(lg[ds], dtype=np.float64)
    return roles, poses


def _pose_pair(pose):
    """Split a 7-vec root_pose into (position[3], quaternion[4] w-first)."""
    return pose[:3], pose[3:7]


def compute_demo_occlusion(roles, poses, world, controller, camera_mgr, spawner,
                           container_mgr, fixture_mgr):
    """Reproduce one demo's initial layout and measure per-camera occlusion.
    Returns {cam_name: rate} or None when the demo can't be measured (no target).
    """
    from configs import usd_config
    from core.occlusion_calculator import OcclusionCalculator

    if "target" not in roles or "target" not in poses:
        print("[occlusion]   no target in layout, skipping")
        return None

    target_usd = spawner._resolve_file(f"{roles['target']}.usd")
    if not target_usd:
        print(f"[occlusion]   could not resolve target USD '{roles['target']}', skipping")
        return None

    world.reset()
    controller.initialize_handles()
    # Rebuild cameras so the semantic-segmentation annotator picks up THIS
    # demo's object labels (same reason the live path used force=True).
    camera_mgr.initialize_cameras(force=True)

    # Container + fixture: present in both phases (fixture hidden for baseline so
    # it supports the target via colliders without occluding it).
    if "container" in poses:
        container_mgr.respawn(override_pose=_pose_pair(poses["container"]))
    has_fixture = "fixture" in poses and fixture_mgr.has_fixture()
    if has_fixture:
        fixture_mgr.respawn(override_pose=_pose_pair(poses["fixture"]))

    # --- Phase 1: target only ---
    spawner._clean_all()
    t_pos, t_quat = _pose_pair(poses["target"])
    spawner._add_object(target_usd, "spawned_obj_target", t_pos, t_quat)
    spawner.target_object = "spawned_obj_target"
    spawner.target_class_name = roles["target"]

    # Hide static occluders + the fixture for the baseline (they must not count
    # against the target's unoccluded pixel baseline).
    hidden = []
    for name in getattr(usd_config, "OCCLUSION_HIDE_PRIMS", []):
        if set_render_visibility(world.stage, name, visible=False):
            hidden.append(name)
    if has_fixture:
        fixture_mgr.make_invisible()

    for _ in range(8):
        world.step(render=True)

    occ = OcclusionCalculator(camera_mgr, roles["target"])
    camera_mgr.enable_semantic_segmentation()
    for _ in range(5):
        world.step(render=True)
    occ.capture_baseline(camera_mgr, render_fn=lambda: world.step(render=True), num_samples=5)

    # Restore occluders for phase 2.
    for name in hidden:
        set_render_visibility(world.stage, name, visible=True)
    if has_fixture:
        fixture_mgr.make_visible()

    # --- Phase 2: add background objects ---
    for role, cls in roles.items():
        if role in ("target", "container", "fixture"):
            continue
        if role not in poses:
            continue
        usd = spawner._resolve_file(f"{cls}.usd")
        if not usd:
            print(f"[occlusion]   could not resolve bg USD '{cls}' for {role}, skipping it")
            continue
        b_pos, b_quat = _pose_pair(poses[role])
        spawner._add_object(usd, f"spawned_obj_{role}", b_pos, b_quat)
        spawner.spawned_objects.append(f"spawned_obj_{role}")
        spawner.bg_class_by_point[role] = cls

    for _ in range(13):
        world.step(render=True)
    occ.capture_occluded(camera_mgr, render_fn=lambda: world.step(render=True), num_samples=5)

    return occ.get_occlusion_rates()


def _default_log_path(dataset_path):
    os.makedirs("occlusion_logs", exist_ok=True)
    stem = os.path.splitext(os.path.basename(dataset_path))[0]
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join("occlusion_logs", f"occlusion_{stem}_{stamp}.log")


def run_occlusion_pass(dataset_path, world, controller, camera_mgr, spawner,
                       container_mgr, fixture_mgr, log_path=None,
                       overwrite=False, demo_indices=None):
    """Compute occlusion for every demo in `dataset_path` (or just
    `demo_indices` if given), writing `occlusion_rates` / `occlusion_rate_avg`
    back onto each demo. Reuses the caller's already-built managers.

    Logs one line per demo to stdout and appends the same to `log_path`
    (auto-named under occlusion_logs/ when None). Returns
    (processed, skipped, failed).
    """
    if log_path is None:
        log_path = _default_log_path(dataset_path)

    def log(msg):
        print(msg)
        try:
            with open(log_path, "a", encoding="utf-8") as f:
                f.write(msg + "\n")
        except Exception as e:
            print(f"[occlusion] (log write failed: {e})")

    processed, skipped, failed = 0, 0, 0
    log(f"[occlusion] === pass start {datetime.now().isoformat(timespec='seconds')} ===")
    log(f"[occlusion] dataset: {dataset_path}")

    with h5py.File(dataset_path, 'a') as f:
        if 'data' not in f:
            log("[occlusion] ERROR: no 'data' group in file")
            return 0, 0, 0
        root = f['data']
        total = int(root.attrs.get('total', 0))
        indices = demo_indices if demo_indices is not None else list(range(total))
        log(f"[occlusion] {len(indices)} demo(s) to consider (total in file: {total})")

        for i in indices:
            demo_name = f"demo_{i}"
            if demo_name not in root:
                log(f"[occlusion] {demo_name}: not found, skipping")
                skipped += 1
                continue
            demo_group = root[demo_name]

            if 'occlusion_rate_avg' in demo_group.attrs and not overwrite:
                log(f"[occlusion] {demo_name}: already has occlusion, skipping "
                    f"(use overwrite to recompute)")
                skipped += 1
                continue

            layout = read_layout(demo_group)
            if layout is None:
                log(f"[occlusion] {demo_name}: no initial_layout recorded, skipping")
                skipped += 1
                continue

            roles, poses = layout
            log(f"[occlusion] {demo_name}: reproducing layout {roles}")
            try:
                rates = compute_demo_occlusion(
                    roles, poses, world, controller, camera_mgr, spawner,
                    container_mgr, fixture_mgr)
            except Exception as e:
                log(f"[occlusion] {demo_name}: FAILED ({e})")
                failed += 1
                continue

            if rates is None:
                skipped += 1
                continue

            valid = [r for r in rates.values() if r >= 0]
            avg = sum(valid) / len(valid) if valid else 0.0
            rounded = {k: round(v, 3) for k, v in rates.items()}
            demo_group.attrs['occlusion_rates'] = json.dumps(rounded)
            demo_group.attrs['occlusion_rate_avg'] = round(avg, 3)
            log(f"[occlusion] {demo_name}: avg={round(avg, 3)} per_cam={rounded}")
            processed += 1

    log(f"[occlusion] === done: processed={processed} skipped={skipped} failed={failed} ===")
    log(f"[occlusion] log written to {log_path}")
    return processed, skipped, failed
