import os
import math
import random

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# --- Spawn position randomization ---------------------------------------
# Each spawnable object (target, background, container) is placed at a random
# point inside a circle centered on its preset spawn point, so every reset
# produces a slightly different layout. The radius is in meters and is applied
# in the XY plane only (Z, the resting height, is preserved).
#
# Set SPAWN_RANDOM_RADIUS = 0.0 to disable and spawn exactly at the preset
# point (the previous fixed behavior). Use SPAWN_RANDOM_RADII to override the
# radius for specific points, keyed by point name (e.g. "container_pos",
# "target_point", "point_right").
SPAWN_RANDOM_RADIUS = 0.05
SPAWN_RANDOM_RADII = {}


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