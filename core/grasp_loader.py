# core/grasp_loader.py
import h5py
import numpy as np
from scipy.spatial.transform import Rotation as R

class GraspLoader:
    """
    Load grasp poses from H5 file and select the best grasp.
    """
    
    def __init__(self, h5_path: str, scale_factor: float = 1.0):
        """
        Args:
            h5_path: Path to H5 file
            scale_factor: Object scale ratio (scene_scale / original_scale)
                          e.g., if original=0.001, scene=0.0003, then factor=0.3
        """
        self.h5_path = h5_path
        self.scale_factor = scale_factor
        self.transforms = None      # (N, 4, 4) grasp poses
        self.scores = None          # (N,) quality scores
        self.best_idx = None
        
        self._load_h5()
    
    def _load_h5(self):
        """Load and parse H5 file."""
        with h5py.File(self.h5_path, 'r') as f:
            # Load grasp transforms
            self.transforms = np.array(f['grasps/transforms'])  # (2000, 4, 4)
            
            # Load quality scores
            qualities = f['grasps/qualities/flex']
            object_in_gripper = np.array(qualities['object_in_gripper'])
            motion_linear = np.array(qualities['object_motion_during_shaking_linear'])
            motion_angular = np.array(qualities['object_motion_during_shaking_angular'])
            
            self.scores = (
                object_in_gripper * 1000
                - motion_linear * 10
                - motion_angular * 1
            )
            
            self.best_idx = np.argmax(self.scores)
            
            print(f"[GraspLoader] Loaded {len(self.transforms)} grasps from {self.h5_path}")
            print(f"[GraspLoader] Scale factor: {self.scale_factor}")
            print(f"[GraspLoader] Best grasp index: {self.best_idx}, score: {self.scores[self.best_idx]:.2f}")
    
    def get_best_grasp(self) -> dict:
        """Get the highest-scoring grasp pose."""
        return self.get_grasp_by_index(self.best_idx)
    
    def get_grasp_by_index(self, idx: int) -> dict:
        """Get grasp pose by index, with scale factor applied to position."""
        T = self.transforms[idx]  # 4x4 matrix
        
        # Extract position and apply scale factor
        position = T[:3, 3] * self.scale_factor  # <-- ��𣈯枤嚗帋�滨蔭蝮格𦆮
        
        # Rotation stays the same
        rotation_matrix = T[:3, :3]
        r = R.from_matrix(rotation_matrix)
        q_xyzw = r.as_quat()
        q_wxyz = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
        
        return {
            'position': position,
            'orientation_quat': q_wxyz,
            'orientation_euler': r.as_euler('xyz', degrees=True),
            'score': self.scores[idx],
            'index': idx
        }
    
    def get_top_k_grasps(self, k: int = 10) -> list:
        """Get top-k highest scoring grasps."""
        top_indices = np.argsort(self.scores)[::-1][:k]
        return [self.get_grasp_by_index(i) for i in top_indices]
    
    def get_successful_grasps(self) -> list:
        """Get all grasps where object_in_gripper == 1."""
        with h5py.File(self.h5_path, 'r') as f:
            success_mask = np.array(f['grasps/qualities/flex/object_in_gripper']) == 1
        
        successful_indices = np.where(success_mask)[0]
        print(f"[GraspLoader] Found {len(successful_indices)} successful grasps")
        return [self.get_grasp_by_index(i) for i in successful_indices]
    
    def transform_grasp_to_world(self, grasp: dict, object_pos: np.ndarray, 
                                  object_quat_wxyz: np.ndarray) -> dict:
        """
        Transform grasp from object-local frame to world frame.
        Note: scale_factor is already applied in get_grasp_by_index()
        """
        # Object rotation matrix
        q = object_quat_wxyz
        r_obj = R.from_quat([q[1], q[2], q[3], q[0]])
        
        # Grasp local position (already scaled) and rotation
        local_pos = grasp['position']
        q_grasp = grasp['orientation_quat']
        r_grasp = R.from_quat([q_grasp[1], q_grasp[2], q_grasp[3], q_grasp[0]])
        
        # Transform to world frame
        world_pos = object_pos + r_obj.apply(local_pos)
        world_rot = r_obj * r_grasp
        
        # Convert back to [w,x,y,z]
        q_world = world_rot.as_quat()
        q_world_wxyz = np.array([q_world[3], q_world[0], q_world[1], q_world[2]])
        
        return {
            'position': world_pos,
            'orientation_quat': q_world_wxyz,
            'orientation_euler': world_rot.as_euler('xyz', degrees=True),
            'score': grasp['score'],
            'index': grasp['index']
        }