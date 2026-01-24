import os
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

from omni.isaac.core.utils.prims import delete_prim, is_prim_path_valid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import GeometryPrim

import robot_config as robot_config

def load_targets():
    """
    Read JSON file
    """
    if os.path.exists(robot_config.OBJECTS_JSON_PATH):
        with open(robot_config.OBJECTS_JSON_PATH, 'r') as f:
            return json.load(f)
    else:
        print(f"[Warning] JSON not found at {robot_config.OBJECTS_JSON_PATH}")

        # If JSON is empty, return default cube
        return [{
            "type": "cube", 
            "position": robot_config.CUBE_POSITION.tolist(), 
            "scale": robot_config.CUBE_SCALE.tolist()
        }]

def spawn_target(world, obj_data):
    """
    Clear previous obj and spawn new obj
    """
    prim_path = "/World/target_obj"
    name = f"target_obj_{obj_data.get('id', 0)}"
    
    # Clear obj
    if world.scene.get_object(name):
        world.scene.remove_object(name)

    if is_prim_path_valid(prim_path):
        delete_prim(prim_path)

    position = np.array(obj_data.get("position", [0.4, -0.0, 0.0]))
    scale = np.array(obj_data.get("scale", [0.05, 0.05, 0.05]))
    
    # Read rotation_euler ,default [0, 0, 0]
    euler_angles = obj_data.get("rotation_euler", [0, 0, 0])
    
    # Transfer to [x, y, z, w]
    r = R.from_euler('xyz', euler_angles, degrees=True)
    q = r.as_quat() 
    
    # Transfer back to [w, x, y, z]
    orientation = np.array([q[3], q[0], q[1], q[2]])
    
    # spawn usd obj
    if obj_data.get("type") == "usd" and "usd_path" in obj_data:
        add_reference_to_stage(usd_path=obj_data["usd_path"], prim_path=prim_path)
        target_prim = GeometryPrim(
            prim_path=prim_path, 
            name=name, 
            position=position, 
            orientation=orientation, 
            scale=scale
        )
    else:
        color = np.array(obj_data.get("color", [1.0, 0.0, 0.0]))
        target_prim = world.scene.add(
            DynamicCuboid(
                prim_path=prim_path,
                name=name,
                position=position,
                orientation=orientation,
                scale=scale,
                color=color,
                mass=0.1
            )
        )
    local_offset = np.array(obj_data.get("grasp_offset", [0.0, 0.0, 0.0]))
    world_offset = r.apply(local_offset)
    final_grasp_pos = position + world_offset
    grasp_rot_euler = obj_data.get("grasp_rotation", None)

    return target_prim, final_grasp_pos, grasp_rot_euler

def compute_grasp_orientation(target_prim,grasp_rot_euler=None):
    """
    Compute EE pose (Quaternion)
    Logic：Get obj pose -> make EE look down
    """
    # Get Obj pose
    _, quat_wxyz = target_prim.get_world_pose()

    # Transfer to Scipy (x, y, z, w)
    r_obj = R.from_quat([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])

    # Turning (look down)
    if grasp_rot_euler is not None:
        # 使用 JSON 定義的旋轉 (例如側抓)
        # 注意：這裡的 rotation 是相對於物件座標系的
        r_offset = R.from_euler('xyz', grasp_rot_euler, degrees=True)
    else:
        # 預設行為 (舊的邏輯)：轉 180 度朝下
        r_offset = R.from_euler('y', 180, degrees=True)
        
    r_target = r_obj * r_offset
    
    # Transfer back to Isaac (w, x, y, z)
    t_q = r_target.as_quat() 
    return np.array([t_q[3], t_q[0], t_q[1], t_q[2]])