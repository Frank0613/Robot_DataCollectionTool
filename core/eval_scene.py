"""Read a recorded dataset's data/initial_scene so evaluation can reproduce the
exact starting layout (same objects + container at the same positions/angles).

The layout is written by DataCollector._write_initial_scene during collection:
  data/initial_scene
    attrs['objects'] = {"point_right": "mustard", ..., "target": "gear",
                        "container": "plate"}
    <role>/root_pose = [x, y, z, qw, qx, qy, qz]   (one per role)
"""
import os
import json
import h5py
import numpy as np


def load_initial_scene(filepath):
    """Return {role: {"class": str, "pose": np.ndarray(7,) or None}} for every
    role recorded in data/initial_scene, where role is one of target / container
    / point_right / point_left / point_front. Returns None if the file is missing
    or has no initial_scene group.
    """
    if not os.path.exists(filepath):
        print(f"[EvalScene] File not found: {filepath}")
        return None

    with h5py.File(filepath, 'r') as f:
        if 'data' not in f or 'initial_scene' not in f['data']:
            print(f"[EvalScene] No data/initial_scene in {filepath}")
            return None

        grp = f['data']['initial_scene']
        try:
            roles = json.loads(grp.attrs.get('objects', '{}'))
        except Exception:
            roles = {}

        scene = {}
        for role, cls in roles.items():
            pose = None
            if role in grp and 'root_pose' in grp[role]:
                pose = np.asarray(grp[role]['root_pose'][...], dtype=np.float64)
            scene[role] = {"class": cls, "pose": pose}

    print(f"[EvalScene] Loaded initial_scene from {os.path.basename(filepath)}: "
          f"{ {r: scene[r]['class'] for r in scene} }")
    return scene
