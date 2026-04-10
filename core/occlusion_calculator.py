import numpy as np


class OcclusionCalculator:
    """
    Calculate the occlusion rate of the target object from each camera viewpoint.

    Pipeline:
    1. With only target_obj in the scene, capture semantic segmentation -> record target pixel count (baseline)
    2. After all objects are loaded, capture again -> record visible target pixel count
    3. occlusion_rate = 1 - (visible_pixels / baseline_pixels)
    """

    def __init__(self, camera_manager, target_class_name: str):
        """
        Args:
            camera_manager: CameraManager instance that manages all cameras
            target_class_name: class name set in the Semantic Schema Editor
                               (i.e. the file name, e.g. "mug", "bottle", etc.)
        """
        self.camera_manager = camera_manager
        self.target_class_name = target_class_name
        self.baseline_pixels = {}   # {cam_name: pixel_count}
        self.occluded_pixels = {}   # {cam_name: pixel_count}
        self.occlusion_rates = {}   # {cam_name: float}
        self._computed = False

    def _count_target_pixels(self, semantic_data: dict, target_class: str) -> int:
        """
        Count the number of pixels belonging to the target class from semantic segmentation data.

        Args:
            semantic_data: semantic_segmentation data from Camera.get_current_frame()
                           contains "data" (2D array of semantic IDs) and
                           "info" (dict mapping ID -> class label)
            target_class: the class name to search for

        Returns:
            number of pixels occupied by the class in the frame
        """
        seg_image = semantic_data["data"]
        id_to_labels = semantic_data["info"]["idToLabels"]

        target_pixel_count = 0
        for semantic_id, label_info in id_to_labels.items():
            # label_info is typically in the format {"class": "your_class_name"}
            class_name = label_info.get("class", "")
            if class_name == target_class:
                target_id = int(semantic_id)
                target_pixel_count += np.sum(seg_image == target_id)

        return int(target_pixel_count)

    def capture_baseline(self, camera_mgr):
        """
        [Phase 1 - Sync] Call when only target_obj is in the scene.
        Read semantic segmentation from each camera and record the baseline pixel count.

        Prerequisites:
           1. camera.add_semantic_segmentation_to_frame() has been called
           2. Several frames of world.step(render=True) have been run to produce annotator data
        """
        for cam_name, camera in camera_mgr.get_all_cameras().items():
            frame = camera.get_current_frame()
            if "semantic_segmentation" not in frame:
                print(f"[OcclusionCalc] Warning: {cam_name} has no semantic_segmentation data")
                self.baseline_pixels[cam_name] = 0
                continue

            pixel_count = self._count_target_pixels(
                frame["semantic_segmentation"],
                self.target_class_name
            )
            self.baseline_pixels[cam_name] = pixel_count

    def capture_occluded(self, camera_mgr):
        """
        [Phase 2 - Sync] Call after all objects have been loaded.
        Read each camera again to count the visible pixels of the target after occlusion.

        Prerequisite: several frames of world.step(render=True) have been run
        """
        for cam_name, camera in camera_mgr.get_all_cameras().items():
            frame = camera.get_current_frame()
            if "semantic_segmentation" not in frame:
                print(f"[OcclusionCalc] Warning: {cam_name} has no semantic_segmentation data")
                self.occluded_pixels[cam_name] = 0
                continue

            pixel_count = self._count_target_pixels(
                frame["semantic_segmentation"],
                self.target_class_name
            )
            self.occluded_pixels[cam_name] = pixel_count

        # Compute occlusion rates
        self._compute_occlusion_rates()

    def _compute_occlusion_rates(self):
        """Compute the occlusion rate for each camera"""
        for cam_name in self.baseline_pixels:
            baseline = self.baseline_pixels.get(cam_name, 0)
            visible = self.occluded_pixels.get(cam_name, 0)

            if baseline == 0:
                self.occlusion_rates[cam_name] = -1.0  # mark as invalid (target not in view)
                print(f"[OcclusionCalc] {cam_name}: target not in view (baseline=0)")
            else:
                rate = 1.0 - (visible / baseline)
                rate = max(0.0, min(1.0, rate))  # clamp to [0, 1]
                self.occlusion_rates[cam_name] = rate
                print(f"[OcclusionCalc] {cam_name}: occlusion_rate = {rate:.3f} "
                      f"({baseline} → {visible} pixels)")

        self._computed = True

    def get_occlusion_rates(self) -> dict:
        """Return {cam_name: occlusion_rate} dict"""
        if not self._computed:
            print("[OcclusionCalc] Warning: occlusion rates have not been computed yet!")
        return self.occlusion_rates.copy()

    def get_avg_occlusion_rate(self) -> float:
        """Return the average occlusion rate across all valid cameras"""
        valid_rates = [r for r in self.occlusion_rates.values() if r >= 0]
        if not valid_rates:
            return 0.0
        return sum(valid_rates) / len(valid_rates)
