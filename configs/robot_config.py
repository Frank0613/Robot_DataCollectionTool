import os
import numpy as np

#"ik", "rmpflow"
CONTROLLER_MODE = "ik" 

# Target robot root & End effector name
ROBOT_PRIM_PATH = "/World/Franka_with_cam"
TARGET_ROBOT_NAME = "Franka"
EE_FRAME_NAME = "panda_hand"

# Robot movement speed
MOVE_SPEED = 0.005

# Robot rotation speed (radians per step)
ROTATE_SPEED = 0.02

# Gripper close speed
GRIPPER_SPEED = 0.002

# --- PD Controller ---
# 7 axles (Arm)
KPS_ARM = 1000.0
KDS_ARM = 100.0

# 2 axles (Gripper)
KPS_GRIPPER = 1000.0
KDS_GRIPPER = 100.0
