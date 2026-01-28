# core/obj_loader.py
import os
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

from omni.isaac.core.utils.prims import delete_prim, is_prim_path_valid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import GeometryPrim

import robot_config as robot_config
from core.grasp_loader import GraspLoader
from core.grasp_visualizer import GraspVisualizer

# Global visualizer instance
_grasp_visualizer = None

def get_grasp_visualizer():
    global _grasp_visualizer
    if _grasp_visualizer is None:
        _grasp_visualizer = GraspVisualizer()
    return _grasp_visualizer

def load_targets():
    """Read JSON file"""
    if os.path.exists(robot_config.OBJECTS_JSON_PATH):
        with open(robot_config.OBJECTS_JSON_PATH, 'r') as f:
            return json.load(f)
    else:
        print(f"[Warning] JSON not found at {robot_config.OBJECTS_JSON_PATH}")
        return [{
            "type": "cube", 
            "position": robot_config.CUBE_POSITION.tolist(), 
            "scale": robot_config.CUBE_SCALE.tolist()
        }]

def spawn_target(world, obj_data, visualize_grasps: bool = True, show_top_k: int = 10):
    """
    Clear previous obj and spawn new obj.
    Now supports automatic grasp loading from H5 files with proper scaling.
    """
    prim_path = "/World/target_obj"
    name = f"target_obj_{obj_data.get('id', 0)}"
    
    # Clear obj
    if world.scene.get_object(name):
        world.scene.remove_object(name)

    if is_prim_path_valid(prim_path):
        delete_prim(prim_path)

    position = np.array(obj_data.get("position", [0.4, -0.0, 0.0]))
    scale = np.array(obj_data.get("scale", [1.0, 1.0, 1.0]))
    
    # Read rotation_euler, default [0, 0, 0]
    euler_angles = obj_data.get("rotation_euler", [0, 0, 0])
    r = R.from_euler('xyz', euler_angles, degrees=True)
    q = r.as_quat() 
    orientation = np.array([q[3], q[0], q[1], q[2]])  # [w, x, y, z]
    
    # Spawn USD obj
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
    
    # ========== H5 Grasp Loading ==========
    grasp_data = None
    
    if "h5_path" in obj_data and os.path.exists(obj_data["h5_path"]):
        print(f"[obj_loader] Loading grasps from H5: {obj_data['h5_path']}")
        
        try:
            if "grasp_scale_factor" in obj_data:
                scale_factor = obj_data["grasp_scale_factor"]
            else:
                scale_factor = 1.0
            
            grasp_loader = GraspLoader(obj_data["h5_path"], scale_factor=scale_factor)
            successful_grasps = grasp_loader.get_successful_grasps()
            
            if len(successful_grasps) == 0:
                print("[obj_loader] No successful grasps found!")
                successful_grasps = grasp_loader.get_top_k_grasps(k=50)
            
            num_candidates = min(100, len(successful_grasps))
            random_index = np.random.randint(0, num_candidates)
            
            best_grasp_local = successful_grasps[random_index]
            
            grasp_data = grasp_loader.transform_grasp_to_world(
                best_grasp_local, 
                object_pos=position,
                object_quat_wxyz=orientation
            )
            
            final_grasp_pos = grasp_data['position']
            grasp_rot_euler = grasp_data['orientation_euler']                 
            if visualize_grasps:
                visualizer = get_grasp_visualizer()
                top_100_successful = successful_grasps[:100]
                
                print(f"[obj_loader] Visualizing top {len(top_100_successful)} successful grasps")
                
            
                visualizer.visualize_grasps(
                    grasps=top_100_successful,
                    object_pos=position,
                    object_quat_wxyz=orientation,
                    highlight_best=True,
                    selected_index=random_index  
                )
            
            return target_prim, final_grasp_pos, grasp_rot_euler, grasp_data
            
        except Exception as e:
            print(f"[obj_loader] Error: {e}")
            import traceback
            traceback.print_exc()
    
    # Fallback
    local_offset = np.array(obj_data.get("grasp_offset", [0.0, 0.0, 0.0]))
    world_offset = r.apply(local_offset)
    final_grasp_pos = position + world_offset
    grasp_rot_euler = obj_data.get("grasp_rotation", None)

    return target_prim, final_grasp_pos, grasp_rot_euler, grasp_data


def compute_grasp_orientation(target_prim, grasp_rot_euler=None):
    """
    Compute EE pose (Quaternion)
    Logic: Get obj pose -> apply grasp rotation
    """
    _, quat_wxyz = target_prim.get_world_pose()
    r_obj = R.from_quat([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])

    if grasp_rot_euler is not None:
        r_offset = R.from_euler('xyz', grasp_rot_euler, degrees=True)
    else:
        r_offset = R.from_euler('y', 180, degrees=True)
        
    r_target = r_obj * r_offset
    t_q = r_target.as_quat() 
    return np.array([t_q[3], t_q[0], t_q[1], t_q[2]])