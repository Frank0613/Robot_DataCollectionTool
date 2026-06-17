"""Deterministic scene layout.

Instead of randomly drawing objects from usdfiles/objects, you assign exactly
which object USD spawns at each spawn point here. This makes a run reproducible
(important for evaluation: the recorded data/initial_scene matches this file).

Per-point entry fields:
  file       : USD filename, e.g. "cracker_box.usd". Resolved recursively under
               usd_config.OBJECTS_DIR, so a bare filename in a subdirectory
               (e.g. "bolt.usd" inside objects/small) is found automatically.
               You may also give a path relative to OBJECTS_DIR ("small/bolt.usd").
               None -> leave this point empty (nothing spawns there).
  container  : (container_pos only) which container to spawn, by key in
               container_config.CONTAINERS (keeps that file's interior / knob
               specs). None -> spawn no container.
  radius     : XY random-spawn radius in meters around the preset point.
               0.0 -> spawn exactly on the point. Omit -> DEFAULT_RADIUS.
  yaw_range  : Z-axis (vertical) random yaw HALF-range in degrees; the yaw is
               sampled uniformly from [-yaw_range, +yaw_range]. 180 -> full
               circle, 0.0 -> keep the preset orientation. Omit -> DEFAULT_YAW_RANGE.
  euler_deg  : (optional) [rx, ry, rz] base resting orientation in degrees that
               REPLACES the spawn point's preset orientation. Use this to fix an
               asset that spawns upside-down / sideways (e.g. [180, 0, 0] flips
               it upright). The random yaw is then applied on top. Omit -> use
               the spawn point's orientation as before.

Position/angle still re-randomize within these ranges every reset (data
augmentation); only the object identity at each point is fixed by this file.

Spawn-point names (must match the scene's prims under target_points/):
  target_point : the task target object
  point_right / point_left / point_front : background distractors
  container_pos : the container (success/drop zone) — pick it via "container"
"""

# Fallback randomization for any point/container that omits radius / yaw_range.
DEFAULT_RADIUS = 0.05       # meters
DEFAULT_YAW_RANGE = 45.0    # degrees (half-range)

# Per-point assignment. Set an object point's "file" to None (or container_pos's
# "container" to None) to leave it empty.
LAYOUT = {
    "target_point": {"file": "spoon.usd",   "radius": 0.02, "yaw_range": 45.0},
    "point_right":  {"file": "mustard.usd",      "radius": 0.02, "yaw_range": 45.0},
    "point_left":   {"file": "banana.usd",       "radius": 0.02, "yaw_range": 45.0},
    "point_front":  {"file": "power_drill.usd",  "radius": 0.02, "yaw_range": 45.0},
    "container_pos": {"container": "plate",       "radius": 0.02, "yaw_range": 30.0},
}


def _entry(point_name):
    return LAYOUT.get(point_name) or {}


def get_file(point_name):
    """USD filename assigned to `point_name`, or None if empty/unset."""
    return _entry(point_name).get("file")


def get_radius(point_name):
    """XY random-spawn radius (meters) for `point_name`, defaulting to DEFAULT_RADIUS."""
    return _entry(point_name).get("radius", DEFAULT_RADIUS)


def get_yaw_range(point_name):
    """Z-axis random yaw half-range (degrees) for `point_name`, defaulting to DEFAULT_YAW_RANGE."""
    return _entry(point_name).get("yaw_range", DEFAULT_YAW_RANGE)


def get_euler_deg(point_name):
    """Optional [rx, ry, rz] base orientation (degrees) for `point_name`, or None."""
    return _entry(point_name).get("euler_deg")


def get_container_key():
    """Container key (in container_config.CONTAINERS) for container_pos, or None."""
    return _entry("container_pos").get("container")


def get_container_radius():
    return get_radius("container_pos")


def get_container_yaw_range():
    return get_yaw_range("container_pos")


def get_container_euler_deg():
    return get_euler_deg("container_pos")
