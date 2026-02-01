import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

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

# Container 
CONTAINER_CONFIG_PATH = os.path.join(BASE_DIR, "container_config.json")
CONTAINER_POINT_PATH = "/World/target_points/container_pos" 
CONTAINER_PRIM_PATH = "/World/Container"