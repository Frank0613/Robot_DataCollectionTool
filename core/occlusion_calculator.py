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

    def _count_target_pixels(self, semantic_data: dict, target_class: str,
                              cam_name: str = "?", phase: str = "?") -> int:
        """
        Count the number of pixels belonging to the target class from semantic segmentation data.
        When the target class is missing from id_to_labels (e.g. annotator's
        cached labels are stale, or the prim has no semantic schema), print
        a diagnostic so we can see what was actually labelled in the frame.
        """
        seg_image = semantic_data["data"]
        id_to_labels = semantic_data["info"]["idToLabels"]

        target_pixel_count = 0
        target_found = False
        for semantic_id, label_info in id_to_labels.items():
            class_name = label_info.get("class", "")
            if class_name == target_class:
                target_found = True
                target_id = int(semantic_id)
                target_pixel_count += int(np.sum(seg_image == target_id))

        if not target_found:
            labels = sorted({li.get("class", "") for li in id_to_labels.values()})
            print(f"[OcclusionCalc][DEBUG] {cam_name} {phase}: target_class "
                  f"'{target_class}' NOT in id_to_labels. Labels seen: {labels}")

        return int(target_pixel_count)

    def _capture_max_pixels(self, camera_mgr, phase: str,
                             render_fn=None, num_samples: int = 5,
                             renders_between: int = 3):
        """
        Read each camera N times and keep the MAX target-pixel count across
        samples. Max instead of mean/median because Isaac Sim's semantic
        annotator occasionally returns zero for small targets (camera_top in
        particular); those "drop-outs" are pure noise, not real occlusion.
        A static scene's true target-pixel count is well-defined, so the
        highest reading across N samples is the best estimate.
        """
        result = {cam_name: 0 for cam_name in camera_mgr.get_all_cameras()}
        warned_no_seg = set()

        for s in range(num_samples):
            for cam_name, camera in camera_mgr.get_all_cameras().items():
                frame = camera.get_current_frame()
                if "semantic_segmentation" not in frame:
                    if cam_name not in warned_no_seg:
                        print(f"[OcclusionCalc] Warning: {cam_name} has no semantic_segmentation data")
                        warned_no_seg.add(cam_name)
                    continue
                count = self._count_target_pixels(
                    frame["semantic_segmentation"],
                    self.target_class_name,
                    cam_name=cam_name,
                    phase=phase,
                )
                if count > result[cam_name]:
                    result[cam_name] = count

            if s < num_samples - 1 and render_fn is not None:
                for _ in range(renders_between):
                    render_fn()

        return result

    def capture_baseline(self, camera_mgr, render_fn=None, num_samples: int = 5):
        """
        [Phase 1] Call when only target_obj is in the scene. Reads each camera
        `num_samples` times (with renders between if `render_fn` provided) and
        keeps the max target-pixel count per camera as the baseline.
        """
        self.baseline_pixels = self._capture_max_pixels(
            camera_mgr, phase="baseline",
            render_fn=render_fn, num_samples=num_samples,
        )

    def capture_occluded(self, camera_mgr, render_fn=None, num_samples: int = 5):
        """
        [Phase 2] Call after all objects have been loaded. Reads each camera
        `num_samples` times to find the max visible target-pixel count, then
        computes the occlusion rate.
        """
        self.occluded_pixels = self._capture_max_pixels(
            camera_mgr, phase="occluded",
            render_fn=render_fn, num_samples=num_samples,
        )
        self._compute_occlusion_rates()

    def _compute_occlusion_rates(self):
        """Compute the occlusion rate for each camera. Rates are always in
        [0, 1]: 0 = fully visible, 1 = fully occluded / not visible at all.
        Baseline=0 means the target was already hidden during phase 1 (e.g.
        small object resting in a drawer whose side wall blocks this camera);
        we treat that as fully occluded rather than a -1 sentinel so downstream
        averaging / filtering still produces meaningful numbers.
        """
        for cam_name in self.baseline_pixels:
            baseline = self.baseline_pixels.get(cam_name, 0)
            visible = self.occluded_pixels.get(cam_name, 0)

            if baseline == 0:
                self.occlusion_rates[cam_name] = 1.0
                print(f"[OcclusionCalc] {cam_name}: target not visible during baseline "
                      f"(baseline=0) -> rate=1.0 (treated as fully occluded)")
            else:
                rate = 1.0 - (visible / baseline)
                rate = max(0.0, min(1.0, rate))  # clamp to [0, 1]
                self.occlusion_rates[cam_name] = rate
                print(f"[OcclusionCalc] {cam_name}: occlusion_rate = {rate:.3f} "
                      f"({baseline} -> {visible} pixels)")

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
