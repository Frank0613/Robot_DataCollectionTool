# core/grasp_visualizer.py
import numpy as np
from scipy.spatial.transform import Rotation as R

from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.prims import create_prim, delete_prim, is_prim_path_valid
from omni.isaac.core.utils.stage import get_current_stage
from pxr import UsdGeom, Gf

class GraspVisualizer:
    """
    Visualize grasp poses as simple spheres.
    """
    
    def __init__(self, base_prim_path: str = "/World/grasp_visualizations"):
        self.base_path = base_prim_path
        self.visualized_grasps = []
        
    def clear_all(self):
        """Remove all visualizations."""
        if is_prim_path_valid(self.base_path):
            delete_prim(self.base_path)
        self.visualized_grasps = []
    
    def visualize_grasps(self, grasps: list, object_pos: np.ndarray, 
                     object_quat_wxyz: np.ndarray,
                     highlight_best: bool = True,
                     selected_index: int = 0): 
        """
        Visualize all grasp poses as spheres.
        """
        self.clear_all()
        
        if len(grasps) == 0:
            print("[GraspVisualizer] No grasps to visualize")
            return
        
        # Create base prim
        create_prim(self.base_path, "Xform")
        
        # Object rotation
        q = object_quat_wxyz
        r_obj = R.from_quat([q[1], q[2], q[3], q[0]])
        
        # Sort by score
        sorted_grasps = sorted(grasps, key=lambda g: g['score'], reverse=True)
        
        stage = get_current_stage()
        
        for i, grasp in enumerate(sorted_grasps):
            # Transform to world frame
            local_pos = grasp['position']
            world_pos = object_pos + r_obj.apply(local_pos)
            
            # Create sphere
            sphere_path = f"{self.base_path}/grasp_{i}"
            sphere = UsdGeom.Sphere.Define(stage, sphere_path)

            is_selected = (i == selected_index)
            
            if is_selected:
                radius = 0.005   
                color = [1.0, 0.0, 1.0]  
            else:
                radius = 0.0015  
                color = [1.0, 1.0, 0.0]  # Yellow
            
            sphere.GetRadiusAttr().Set(radius)
            
            # Position
            xformable = UsdGeom.Xformable(sphere)
            xformable.AddTranslateOp().Set(Gf.Vec3d(*world_pos))
            
            sphere.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
            
            self.visualized_grasps.append({
                'index': i,
                'world_pos': world_pos,
                'score': grasp['score']
            })
        
        print(f"[GraspVisualizer] Visualized {len(sorted_grasps)} grasps")
        if selected_index < len(sorted_grasps):
            print(f"[GraspVisualizer] Selected grasp #{selected_index} highlighted in MAGENTA")

    
    def get_best_visualized_grasp(self) -> dict:
        if self.visualized_grasps:
            return self.visualized_grasps[0]
        return None