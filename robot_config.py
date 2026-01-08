import os
import numpy as np

#"ik", "rmpflow"
CONTROLLER_MODE = "ik" 

# Usd Scene
SCENE_NAME = "Scene02"
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
MOVE_SPEED = 0.005 

# Gripper close speed
GRIPPER_SPEED = 0.002

# --- PD Controller ---
# 7 axles (Arm)
KPS_ARM = 10000.0
KDS_ARM = 150.0

# 2 axles (Gripper)
KPS_GRIPPER = 100000.0
KDS_GRIPPER = 100.0

# --- Target Cube ---
CUBE_POSITION = np.array([0.50, 1.5, 1.3])
CUBE_SCALE = np.array([0.05, 0.05, 0.05])
CUBE_COLOR = np.array([0.0, 0.0, 1.0])