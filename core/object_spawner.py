import os
import glob
import random
from omni.isaac.core.utils.prims import delete_prim, is_prim_path_valid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from pxr import UsdGeom
from configs import usd_config


class ObjectSpawner:
    def __init__(self, world, repeat_count=None):
        self.world = world
        self.spawned_objects = []
        self.target_object = None
        self.target_class_name = None  # semantic class name of the target
        self.bg_class_by_point = {}    # {"point_right": "ramekin", ...}

        # Pre-selected files (shared across two-phase spawn)
        self._selected_files = []

        # Repeat control: if set, reuse the same selection for `repeat_count`
        # successful demos before re-randomizing. None -> randomize every reset.
        # Only successful task completions advance the cycle; manual R resets
        # keep the current selection without consuming the counter.
        self.repeat_count = repeat_count
        self._success_count = 0

        search_path = os.path.join(usd_config.OBJECTS_DIR, "*.usd")
        self.available_usds = glob.glob(search_path)
        print(f"[ObjectSpawner] Found {len(self.available_usds)} USD files in {usd_config.OBJECTS_DIR}")

    def get_target_class_name(self) -> str:
        """Return the semantic class name of the target object (USD filename without extension)"""
        return self.target_class_name

    def _clean_all(self):
        """Remove all spawned objects"""
        all_managed = self.spawned_objects + ([self.target_object] if self.target_object else [])
        for name in all_managed:
            if self.world.scene.object_exists(name):
                self.world.scene.remove_object(name)
            prim_path = f"/World/{name}"
            if is_prim_path_valid(prim_path):
                delete_prim(prim_path)
        self.spawned_objects = []
        self.target_object = None
        self.target_class_name = None
        self.bg_class_by_point = {}

    def notify_task_success(self):
        """Advance the repeat cycle by one. Call after a demo is successfully saved."""
        if self.repeat_count:
            self._success_count += 1

    def _select_files(self):
        """Randomly select 4 USD files (3 background + 1 target) and store in _selected_files.

        When repeat_count is set, the same selection is reused until the user has
        collected `repeat_count` successful demos; then it re-randomizes.
        Manual R resets reuse the current selection without consuming the counter.
        """
        should_reuse = (
            self.repeat_count is not None
            and self._selected_files
            and self._success_count < self.repeat_count
        )
        if should_reuse:
            iteration = self._success_count + 1
            print(f"[ObjectSpawner] Reusing selection (iteration {iteration}/{self.repeat_count})")
            return

        if not self.available_usds:
            print("[Warning] No USD files found to spawn!")
            self._selected_files = []
            return

        total_needed = len(usd_config.BG_SPAWN_POINTS) + 1
        if len(self.available_usds) >= total_needed:
            self._selected_files = random.sample(self.available_usds, k=total_needed)
        else:
            self._selected_files = random.choices(self.available_usds, k=total_needed)

        self._success_count = 0
        if self.repeat_count:
            print(f"[ObjectSpawner] New random selection (iteration 1/{self.repeat_count})")

    def spawn_target_only(self):
        """
        [Phase 1] Spawn only the target object to target_point.
        Used for baseline capture in occlusion rate calculation.
        """
        self._clean_all()
        self._select_files()

        if not self._selected_files:
            return

        # Target is the last file; preceding files go to background points
        target_usd = self._selected_files[-1]
        self.target_class_name = os.path.splitext(os.path.basename(target_usd))[0]

        target_point_path = f"{usd_config.OBJ_SPAWN_POINT_ROOT}/target_point"
        if is_prim_path_valid(target_point_path):
            position, orientation = XFormPrim(target_point_path).get_world_pose()
            obj_name = "spawned_obj_target"
            container_path = f"/World/{obj_name}"
            if is_prim_path_valid(container_path):
                delete_prim(container_path)

            add_reference_to_stage(usd_path=target_usd, prim_path=container_path)
            XFormPrim(prim_path=container_path, position=position, orientation=orientation)

            stage = self.world.stage
            parent_prim = stage.GetPrimAtPath(container_path)
            target_path = container_path
            for child in parent_prim.GetChildren():
                if child.IsA(UsdGeom.Imageable):
                    target_path = child.GetPath().pathString
                    break

            real_object_prim = XFormPrim(prim_path=target_path, name=obj_name)
            self.world.scene.add(real_object_prim)
            self.target_object = obj_name
            print(f"[ObjectSpawner] Target spawned: {self.target_class_name} (class for semantic)")
        else:
            print(f"[Warning] target_point not found at {target_point_path}")

    def spawn_remaining_objects(self):
        """
        [Phase 2] Spawn background objects to named points (see usd_config.BG_SPAWN_POINTS).
        Call after baseline capture is complete.
        """
        if not self._selected_files:
            print("[Warning] No selected files. Did you call spawn_target_only() first?")
            return

        for i, point_name in enumerate(usd_config.BG_SPAWN_POINTS):
            point_path = f"{usd_config.OBJ_SPAWN_POINT_ROOT}/{point_name}"
            if not is_prim_path_valid(point_path):
                print(f"[Warning] Spawn point not found: {point_path}, skipping")
                continue
            position, orientation = XFormPrim(point_path).get_world_pose()

            obj_name = f"spawned_obj_{point_name}"
            usd_file = self._selected_files[i]
            self.bg_class_by_point[point_name] = os.path.splitext(os.path.basename(usd_file))[0]
            container_path = f"/World/{obj_name}"
            if is_prim_path_valid(container_path):
                delete_prim(container_path)

            add_reference_to_stage(usd_path=self._selected_files[i], prim_path=container_path)
            XFormPrim(prim_path=container_path, position=position, orientation=orientation)

            stage = self.world.stage
            parent_prim = stage.GetPrimAtPath(container_path)
            target_path = container_path
            for child in parent_prim.GetChildren():
                if child.IsA(UsdGeom.Imageable):
                    target_path = child.GetPath().pathString
                    break

            real_object_prim = XFormPrim(prim_path=target_path, name=obj_name)
            self.world.scene.add(real_object_prim)
            self.spawned_objects.append(obj_name)

        print(f"[ObjectSpawner] Spawned {len(self.spawned_objects)} background objects")

    def respawn(self):
        """
        Full respawn (backward compatible).
        Runs sequentially: spawn_target_only -> spawn_remaining_objects.
        Use this directly if occlusion rate calculation is not needed.
        """
        self.spawn_target_only()
        self.spawn_remaining_objects()
        print(f"[ObjectSpawner] Spawned {len(self.spawned_objects)} background objects + target: {self.target_object}")
