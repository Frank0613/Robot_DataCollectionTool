import os
import math
import random

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# --- Spawn randomization fallbacks --------------------------------------
# randomize_xy() / randomize_yaw() below are pure helpers: each spawnable
# object is shifted within a circle (XY only, Z preserved) and rotated about
# the world Z axis. The ACTUAL per-point radius/yaw values now live in
# configs/scene_config.py and are passed in explicitly by the spawn managers.
# The module-level constants here are only used as a last-resort default if a
# caller invokes the helpers without a radius / yaw_range argument.
SPAWN_RANDOM_RADIUS = 0.05      # meters
SPAWN_RANDOM_RADII = {}
SPAWN_RANDOM_YAW_RANGE = 45.0   # degrees (half-range)
SPAWN_RANDOM_YAW_RANGES = {}


def quat_mul(q1, q2):
    """Hamilton product of two (w, x, y, z) quaternions; returns a (w,x,y,z) tuple."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def euler_deg_to_quat(rx, ry, rz):
    """Convert XYZ Euler angles in DEGREES to a (w, x, y, z) quaternion.

    Rotation order is Rz @ Ry @ Rx (apply X, then Y, then Z about the world
    axes). For the common 'flip an upside-down asset upright' case a single
    180 on one axis is enough, e.g. (180, 0, 0).
    """
    hx, hy, hz = (math.radians(rx) / 2, math.radians(ry) / 2, math.radians(rz) / 2)
    qx = (math.cos(hx), math.sin(hx), 0.0, 0.0)
    qy = (math.cos(hy), 0.0, math.sin(hy), 0.0)
    qz = (math.cos(hz), 0.0, 0.0, math.sin(hz))
    return quat_mul(qz, quat_mul(qy, qx))


def randomize_yaw(orientation, point_name=None, yaw_range=None):
    """Return `orientation` rotated by a random angle about the world Z axis.

    `orientation` is a quaternion in (w, x, y, z) order (Isaac Sim convention).
    The added yaw is sampled uniformly from [-yaw_range, +yaw_range] degrees.

    range resolution: explicit `yaw_range` arg > SPAWN_RANDOM_YAW_RANGES[point_name]
    > SPAWN_RANDOM_YAW_RANGE. A range <= 0 (or None orientation) returns the
    orientation unchanged.

    Accepts a numpy array, list, or tuple and returns the same kind.
    """
    if orientation is None:
        return orientation
    if yaw_range is None:
        yaw_range = SPAWN_RANDOM_YAW_RANGES.get(point_name, SPAWN_RANDOM_YAW_RANGE)
    if yaw_range is None or yaw_range <= 0:
        return orientation

    angle = math.radians(random.uniform(-yaw_range, yaw_range))
    c = math.cos(angle / 2.0)
    s = math.sin(angle / 2.0)

    # World-frame yaw applied on top of the preset orientation:
    # q_new = q_z (w,x,y,z)=(c,0,0,s)  composed with  q_orig (pre-multiply).
    w0, x0, y0, z0 = (float(orientation[0]), float(orientation[1]),
                      float(orientation[2]), float(orientation[3]))
    w = c * w0 - s * z0
    x = c * x0 - s * y0
    y = c * y0 + s * x0
    z = c * z0 + s * w0

    # Preserve the input container type (numpy array / list support item set).
    if hasattr(orientation, "copy") and hasattr(orientation, "__setitem__"):
        out = orientation.copy()
        out[0], out[1], out[2], out[3] = w, x, y, z
        return out
    return (w, x, y, z)


def randomize_xy(position, point_name=None, radius=None):
    """Return `position` shifted by a random offset uniformly sampled within a
    circle in the XY plane; Z is left unchanged.

    radius resolution: explicit `radius` arg > SPAWN_RANDOM_RADII[point_name]
    > SPAWN_RANDOM_RADIUS. A radius <= 0 returns the position unchanged.

    Accepts a numpy array, list, or tuple and returns the same kind.
    """
    if radius is None:
        radius = SPAWN_RANDOM_RADII.get(point_name, SPAWN_RANDOM_RADIUS)
    if radius is None or radius <= 0:
        return position

    # sqrt() on the random radius gives a uniform distribution over the disk
    # area (otherwise points cluster near the center).
    r = radius * math.sqrt(random.random())
    theta = random.uniform(0.0, 2.0 * math.pi)
    new_x = float(position[0]) + r * math.cos(theta)
    new_y = float(position[1]) + r * math.sin(theta)

    # Preserve the input container type (numpy array / list support item set).
    if hasattr(position, "copy") and hasattr(position, "__setitem__"):
        out = position.copy()
        out[0] = new_x
        out[1] = new_y
        return out
    return (new_x, new_y, float(position[2]))

# Scene
SCENE_NAME = "Warehouse"
USD_PATH = os.path.join(
    BASE_DIR,
    "usdfiles",
    "scenes",
    f"{SCENE_NAME}.usd"
)
# Target Objects
OBJECTS_DIR = os.path.join(BASE_DIR, "usdfiles", "objects")
OBJ_SPAWN_POINT_ROOT = "/World/target_points"
# Background spawn points (named by position relative to target).
# Rename scene prims to match. Missing points are skipped.
BG_SPAWN_POINTS = ["point_right", "point_left", "point_front"]

# Container
CONTAINER_POINT_PATH = "/World/target_points/container_pos"
CONTAINER_PRIM_PATH = "/World/Container"

# Occlusion baseline: static scene prims whose VISUAL mesh is hidden during the
# Phase-1 baseline capture (so the target's unobstructed pixel count is the
# reference), then restored for the Phase-2 occluded capture so they DO count
# toward measured occlusion. Render visibility only — physics colliders stay
# active, so the target keeps resting on the now-invisible surface.
# Accepts exact prim paths or bare prim names (searched across the stage).
# Names not present in the current scene are silently skipped.
#
# Per-task fixtures (e.g. a stool the target sits on) are auto-appended at
# runtime by FixtureManager.get_occlusion_hide_names() — list ONLY scene-baked
# prims here (e.g. a cabinet already inside home_cabinet.usd).
OCCLUSION_HIDE_PRIMS = []