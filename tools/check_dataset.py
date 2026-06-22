"""Sanity-check a recorded dataset for the "target never captured" bug.

Verifies that the target object's pose was actually recorded (not stuck at the
origin), which is what eval replay needs to spawn the target. Background objects
are checked too for completeness.

Usage:
    python tools/check_dataset.py datasets/dataset.hdf5
"""
import sys
import json
import h5py
import numpy as np


def check(path):
    print(f"==== {path} ====")
    ok = True
    with h5py.File(path, "r") as f:
        data = f["data"]

        # --- initial_scene ---
        if "initial_scene" not in data:
            print("  [FAIL] no data/initial_scene group")
            return False
        g = data["initial_scene"]
        roles = json.loads(g.attrs.get("objects", "{}"))
        print(f"  initial_scene objects: {roles}")

        # Every role listed in `objects` should have a matching root_pose dataset.
        for role in roles:
            has_pose = role in g and "root_pose" in g[role]
            if not has_pose:
                print(f"  [FAIL] role '{role}' ({roles[role]}) has NO {role}/root_pose")
                ok = False
        if "target" not in roles:
            print("  [WARN] no 'target' role recorded at all")

        # --- per-demo target trajectory ---
        demos = sorted(k for k in data.keys() if k.startswith("demo"))
        print(f"  demos: {len(demos)}")
        bad = []
        for dn in demos:
            d = data[dn]
            try:
                to = d["obs"]["states"]["target_object"]
            except KeyError:
                print(f"  [FAIL] {dn}: no obs/states/target_object")
                ok = False
                continue
            for name in to.keys():
                rp = to[name]["root_pose"][...]
                if np.allclose(rp[:, :3], 0.0):
                    bad.append(f"{dn}/{name}")
        if bad:
            ok = False
            print(f"  [FAIL] target stuck at origin in {len(bad)} demo(s): {bad[:5]}"
                  + (" ..." if len(bad) > 5 else ""))
        else:
            print("  [OK] all demos have a non-zero target trajectory")

    print("  ==>", "PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    paths = sys.argv[1:] or ["datasets/dataset.hdf5"]
    all_ok = all(check(p) for p in paths)
    sys.exit(0 if all_ok else 1)
