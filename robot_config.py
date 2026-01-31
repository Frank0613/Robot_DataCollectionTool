import os
import numpy as np

#"ik", "rmpflow"
CONTROLLER_MODE = "ik" 

# Usd Scene
SCENE_NAME = "Warehouse"
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
USD_PATH = os.path.join(
    BASE_DIR,
    "usdfiles",
    "scenes",
    f"{SCENE_NAME}.usd"
)

# Target robot root & End effector name
ROBOT_PRIM_PATH = "/World/Franka_with_cam"
TARGET_ROBOT_NAME = "Franka"
EE_FRAME_NAME = "panda_hand"

# Robot movement speed
MOVE_SPEED = 0.0025 

# Gripper close speed
GRIPPER_SPEED = 0.002

# --- PD Controller ---
# 7 axles (Arm)
KPS_ARM = 100000.0
KDS_ARM = 150.0

# 2 axles (Gripper)
KPS_GRIPPER = 100000.0
KDS_GRIPPER = 100.0

# Target Objects
OBJECTS_DIR = os.path.join(BASE_DIR, "usdfiles", "objects")
OBJ_SPAWN_POINT_ROOT = "/World/target_points"

# Container 
CONTAINER_CONFIG_PATH = os.path.join(BASE_DIR, "container_config.json")
CONTAINER_POINT_PATH = "/World/target_points/container_pos" 
CONTAINER_PRIM_PATH = "/World/Container"