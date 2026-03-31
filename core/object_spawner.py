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
        
        search_path = os.path.join(usd_config.OBJECTS_DIR, "*.usd")
        self.available_usds = glob.glob(search_path)
        print(f"[ObjectSpawner] Found {len(self.available_usds)} USD files in {usd_config.OBJECTS_DIR}")

    def respawn(self):
        # Clean exiting spawned objects
        for name in self.spawned_objects:
            if self.world.scene.object_exists(name):
                self.world.scene.remove_object(name) 

            prim_path = f"/World/{name}"
            if is_prim_path_valid(prim_path):
                delete_prim(prim_path) 
        self.spawned_objects = []

        if not self.available_usds:
            print("[Warning] No USD files found to spawn!")
            return

        count = min(4, len(self.available_usds))
        if len(self.available_usds) >= 5:
            selected_files = random.sample(self.available_usds, k=count)
        else:
            selected_files = random.choices(self.available_usds, k=5)

        for i, usd_path in enumerate(selected_files):
            point_path = f"{usd_config.OBJ_SPAWN_POINT_ROOT}/point_0{i+1}"
            if is_prim_path_valid(point_path):
                position, orientation = XFormPrim(point_path).get_world_pose()
            else:
                continue

            obj_name = f"spawned_obj_{i}"          
            container_path = f"/World/{obj_name}"

            if is_prim_path_valid(container_path):
                delete_prim(container_path)

            add_reference_to_stage(usd_path=usd_path, prim_path=container_path)

            container_prim = XFormPrim(
                prim_path=container_path,
                position=position,
                orientation=orientation
            )

            
            stage = self.world.stage
            parent_prim = stage.GetPrimAtPath(container_path)
            target_path = container_path 

            children = parent_prim.GetChildren()
            for child in children:
                if child.IsA(UsdGeom.Imageable):
                    target_path = child.GetPath().pathString
                    break
            
            real_object_prim = XFormPrim(
                prim_path=target_path,
                name=obj_name 
            )
            self.world.scene.add(real_object_prim)
            self.spawned_objects.append(obj_name)
        
        print(f"[ObjectSpawner] Spawned {len(self.spawned_objects)} objects.")