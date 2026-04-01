import os
import glob
import random
from omni.isaac.core.utils.prims import delete_prim, is_prim_path_valid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from pxr import UsdGeom 
from configs import usd_config

class ObjectSpawner:
    def __init__(self, world):
        self.world = world
        self.spawned_objects = []
        self.target_object = None

        search_path = os.path.join(usd_config.OBJECTS_DIR, "*.usd")
        self.available_usds = glob.glob(search_path)
        print(f"[ObjectSpawner] Found {len(self.available_usds)} USD files in {usd_config.OBJECTS_DIR}")

    def respawn(self):
        # Clean existing spawned objects
        all_managed = self.spawned_objects + ([self.target_object] if self.target_object else [])
        for name in all_managed:
            if self.world.scene.object_exists(name):
                self.world.scene.remove_object(name)
            prim_path = f"/World/{name}"
            if is_prim_path_valid(prim_path):
                delete_prim(prim_path)
        self.spawned_objects = []
        self.target_object = None

        if not self.available_usds:
            print("[Warning] No USD files found to spawn!")
            return

        # Need 4 objects total: 3 background + 1 target
        total_needed = 4
        if len(self.available_usds) >= total_needed:
            selected_files = random.sample(self.available_usds, k=total_needed)
        else:
            selected_files = random.choices(self.available_usds, k=total_needed)

        # Spawn background objects at point_01 ~ point_03
        for i in range(3):
            point_path = f"{usd_config.OBJ_SPAWN_POINT_ROOT}/point_0{i+1}"
            if not is_prim_path_valid(point_path):
                continue
            position, orientation = XFormPrim(point_path).get_world_pose()

            obj_name = f"spawned_obj_{i}"
            container_path = f"/World/{obj_name}"
            if is_prim_path_valid(container_path):
                delete_prim(container_path)

            add_reference_to_stage(usd_path=selected_files[i], prim_path=container_path)
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

        # Spawn target object at target_point
        target_point_path = f"{usd_config.OBJ_SPAWN_POINT_ROOT}/target_point"
        if is_prim_path_valid(target_point_path):
            position, orientation = XFormPrim(target_point_path).get_world_pose()
            obj_name = "spawned_obj_target"
            container_path = f"/World/{obj_name}"
            if is_prim_path_valid(container_path):
                delete_prim(container_path)

            add_reference_to_stage(usd_path=selected_files[3], prim_path=container_path)
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
        else:
            print(f"[Warning] target_point not found at {target_point_path}")

        print(f"[ObjectSpawner] Spawned {len(self.spawned_objects)} background objects + target: {self.target_object}")