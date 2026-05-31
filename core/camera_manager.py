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
        self._initialized = False
        # Cache of last good RGB per camera so an occasional render glitch
        # never writes a black frame into the dataset.
        self._last_rgb = {}


    def initialize_cameras(self, force=False):
        # By default this is idempotent: cameras are built once and kept alive
        # across resets. Re-creating the Camera render products every reset is
        # what causes the intermittent "empty scene + skybox" flicker, so we
        # only rebuild when explicitly asked (force=True) — which collection
        # mode does so the semantic-segmentation annotator picks up the new
        # scene's id_to_labels for correct occlusion rates. Eval mode keeps
        # force=False -> stable cameras, no flicker (it doesn't need occlusion).
        if self._initialized and not force:
            return
        self.cameras = {}
        self._last_rgb = {}
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

        self._initialized = True

    def get_all_camera_data(self):
        """Read one frame from every camera. RGB has a last-good fallback so
        an occasional glitch never writes a black frame into the dataset
        (which would also misalign the per-frame array lengths).
        """
        data = {}
        for name, sensors in self.cameras.items():
            if "rgb" in sensors:
                rgba = sensors["rgb"].get_rgba()
                rgb_ok = rgba is not None and rgba.size > 0 and rgba[:, :, :3].any()
                if rgb_ok:
                    rgb = rgba[:, :, :3].astype(np.uint8)
                    self._last_rgb[name] = rgb
                    data[f"{name}/rgb"] = rgb
                elif name in self._last_rgb:
                    data[f"{name}/rgb"] = self._last_rgb[name]
                    print(f"[CameraManager] {name} RGB glitch this frame, reusing last good")
                else:
                    data[f"{name}/rgb"] = np.zeros((256, 256, 3), dtype=np.uint8)
                    print(f"[CameraManager] {name} RGB invalid and no cached frame")

            if "depth" in sensors:
                depth_raw = sensors["depth"].get_depth()
                if depth_raw is not None:
                    depth_processed = np.nan_to_num(depth_raw, nan=0.0, posinf=10.0, neginf=0.0)
                    depth_clipped = np.clip(depth_processed, 0, 10.0)
                    data[f"{name}/depth"] = (depth_clipped * 1000).astype(np.uint16)
                else:
                    data[f"{name}/depth"] = np.zeros((256, 256), dtype=np.uint16)

        return data

    def save_current_frame_preview(self, save_path):
        """Capture one frame from every camera and save a side-by-side preview
        PNG (same layout as tools/hdf5_checker.py). Use this at recording start
        so the operator can sanity-check camera views before continuing.
        """
        import matplotlib
        matplotlib.use("Agg")  # headless backend, never opens a window
        import matplotlib.pyplot as plt

        data = self.get_all_camera_data()
        rgb_keys = sorted([k for k in data if k.endswith("/rgb")])
        depth_keys = sorted([k for k in data if k.endswith("/depth")])
        num = len(rgb_keys)
        if num == 0:
            print("[CameraManager] No RGB cameras to preview")
            return None

        rows = 2 if depth_keys else 1
        fig, axes = plt.subplots(rows, num, figsize=(4 * num, 4 * rows))
        if num == 1:
            axes = np.atleast_2d(axes).T
        elif rows == 1:
            axes = np.atleast_2d(axes)

        for i, k in enumerate(rgb_keys):
            ax = axes[0, i]
            ax.imshow(data[k])
            ax.set_title(f"RGB: {k.split('/')[0]}")
            ax.axis("off")

        for i, k in enumerate(depth_keys):
            ax = axes[1, i]
            d = data[k].astype(np.float32)
            d_viz = (d - d.min()) / (d.max() - d.min() + 1e-5)
            ax.imshow(d_viz, cmap="magma")
            ax.set_title(f"Depth: {k.split('/')[0]}")
            ax.axis("off")

        plt.tight_layout()
        plt.savefig(save_path)
        plt.close(fig)
        return save_path

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

        NOTE: do NOT try to detach the annotator between setups — calling
        `remove_semantic_segmentation_from_frame` crashes omni.syntheticdata
        during reset. `add_semantic_segmentation_to_frame` is idempotent on
        the Isaac Sim side so calling it repeatedly is safe.
        """
        for name, sensors in self.cameras.items():
            if "rgb" in sensors:
                try:
                    sensors["rgb"].add_semantic_segmentation_to_frame()
                except Exception as e:
                    print(f"[CameraManager] Failed to enable semantic seg for {name}: {e}")
