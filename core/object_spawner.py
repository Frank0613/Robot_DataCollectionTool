import os
import glob
import random
from omni.isaac.core.utils.prims import delete_prim, is_prim_path_valid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
import robot_config

class ObjectSpawner:
    def __init__(self, world):
        self.world = world
        self.spawned_objects = [] 
        
        search_path = os.path.join(robot_config.OBJECTS_DIR, "*.usd")
        self.available_usds = glob.glob(search_path)
        print(f"[ObjectSpawner] Found {len(self.available_usds)} USD files in {robot_config.OBJECTS_DIR}")

    def respawn(self):
        # Clean exiting spawned objects
        for name in self.spawned_objects:
            prim_path = f"/World/{name}"
            if self.world.scene.object_exists(name):
                self.world.scene.remove_object(name) 
            if is_prim_path_valid(prim_path):
                delete_prim(prim_path) 
        self.spawned_objects = []

        if not self.available_usds:
            print("[Warning] No USD files found to spawn!")
            return

        # Choose 5 random USD files
        count = min(5, len(self.available_usds))
        if len(self.available_usds) >= 5:
            selected_files = random.sample(self.available_usds, k=count)
        else:
            # If less than 5 files, allow duplicates
            selected_files = random.choices(self.available_usds, k=5)

        # Spawn objects at 5 target points
        for i, usd_path in enumerate(selected_files):
            point_path = f"{robot_config.OBJ_SPAWN_POINT_ROOT}/point_0{i+1}"
            
            if is_prim_path_valid(point_path):
                position, orientation = XFormPrim(point_path).get_world_pose()
            else:
                print(f"[Warning] Spawn point not found: {point_path}")
                continue

            obj_name = f"spawned_obj_{i}"
            obj_prim_path = f"/World/{obj_name}"
            
            if is_prim_path_valid(obj_prim_path):
                delete_prim(obj_prim_path)

            add_reference_to_stage(usd_path=usd_path, prim_path=obj_prim_path)

            spawned_prim = XFormPrim(
                prim_path=obj_prim_path,
                name=obj_name,
                position=position,
                orientation=orientation
            )
            
            self.world.scene.add(spawned_prim)
            self.spawned_objects.append(obj_name)
        
        print(f"[ObjectSpawner] Spawned {len(self.spawned_objects)} objects.")