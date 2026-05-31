import os

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

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
OCCLUSION_HIDE_PRIMS = ["white_cabinet"]