import numpy as np
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.sensor import Camera
import omni.usd

class CameraManager:
    def __init__(self):
        # Camera paths based on the USD structure
        base_prefix = "/World/Franka_with_cam/Franka_with_cam/Franka"
        
        self.full_camera_list = [
            f"{base_prefix}/Cameras/Camera_back/RSD455",
            f"{base_prefix}/Cameras/Camera_front/RSD455",
            f"{base_prefix}/Cameras/Camera_left/RSD455",
            f"{base_prefix}/Cameras/Camera_right/RSD455",
            f"{base_prefix}/Cameras/Camera_top/RSD455",
            f"{base_prefix}/panda_hand/Camera_wrist/RSD455"
        ]
        self.camera_groups = {
            #"camera_back":  f"{base_prefix}/Cameras/Camera_back/RSD455",
            "camera_front": f"{base_prefix}/Cameras/Camera_front/RSD455",
            "camera_left":  f"{base_prefix}/Cameras/Camera_left/RSD455",
            "camera_right": f"{base_prefix}/Cameras/Camera_right/RSD455",
            "camera_top":   f"{base_prefix}/Cameras/Camera_top/RSD455",
            "camera_wrist": f"{base_prefix}/panda_hand/Camera_wrist/RSD455"
        }
        self.cameras = {}
        

    def initialize_cameras(self):
        stage = omni.usd.get_context().get_stage()
        active_paths = set(self.camera_groups.values())

        # Find RGB cameras
        for root_path in self.full_camera_list:
            child_rgb = f"{root_path}/Camera_OmniVision_OV9782_Color"
            child_depth = f"{root_path}/Camera_Pseudo_Depth"
            
            is_active = root_path in active_paths
            
            # disable all cameras first, then only enable the ones in camera_groups
            for path in [child_rgb, child_depth]:
                prim = stage.GetPrimAtPath(path)
                if prim.IsValid():
                    prim.SetActive(is_active)
                    if not is_active:
                        print(f"[CameraManager] Disabled sensor: {path}")

        for name, root_path in self.camera_groups.items():
            rgb_path = f"{root_path}/Camera_OmniVision_OV9782_Color"
            depth_path = f"{root_path}/Camera_Pseudo_Depth"
            
            self.cameras[name] = {}
            
            # Init RGB camera
            if prim_utils.is_prim_path_valid(rgb_path):
                try:
                    rgb_cam = Camera(prim_path=rgb_path, resolution=(256, 256))
                    rgb_cam.initialize()
                    self.cameras[name]["rgb"] = rgb_cam
                except Exception as e:
                    print(f"[CameraManager] Failed to init RGB for {name}: {e}")

            # Init Depth camera
            if prim_utils.is_prim_path_valid(depth_path):
                try:
                    depth_cam = Camera(prim_path=depth_path, resolution=(256, 256))
                    depth_cam.initialize()
                    depth_cam.add_distance_to_image_plane_to_frame()
                    
                    self.cameras[name]["depth"] = depth_cam
                except Exception as e:
                    print(f"[CameraManager] Failed to init Depth for {name}: {e}")

    def get_all_camera_data(self):
        data = {}
        for name, sensors in self.cameras.items():
            if "rgb" in sensors:
                rgba = sensors["rgb"].get_rgba()
                if rgba is not None:
                    data[f"{name}/rgb"] = rgba[:, :, :3].astype(np.uint8)
            
            if "depth" in sensors:
                depth_raw = sensors["depth"].get_depth()
                
                if depth_raw is not None:
                    depth_processed = np.nan_to_num(depth_raw, nan=0.0, posinf=10.0, neginf=0.0)
                    depth_clipped = np.clip(depth_processed, 0, 10.0)
                    data[f"{name}/depth"] = (depth_clipped * 1000).astype(np.uint16)
                else:
                    data[f"{name}/depth"] = np.zeros((256, 256), dtype=np.uint16)
                    
        return data

    # Occlusion Rate Calculation
    def get_all_cameras(self) -> dict:
        """
        Return a dict {name: Camera} of all RGB cameras.
        Used for semantic segmentation in occlusion rate calculation.
        """
        rgb_cameras = {}
        for name, sensors in self.cameras.items():
            if "rgb" in sensors:
                rgb_cameras[name] = sensors["rgb"]
        return rgb_cameras

    def enable_semantic_segmentation(self):
        """
        Enable semantic segmentation annotator for all RGB cameras.
        After calling, run several frames of world.step(render=True) to initialize the annotator.
        """
        for name, sensors in self.cameras.items():
            if "rgb" in sensors:
                try:
                    sensors["rgb"].add_semantic_segmentation_to_frame()
                except Exception as e:
                    print(f"[CameraManager] Failed to enable semantic seg for {name}: {e}")
