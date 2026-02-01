import os
import json
import numpy as np
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from pxr import UsdGeom, Usd, Gf
import usd_config

class ContainerManager:
    def __init__(self, world):
        self.world = world
        self.config_path = usd_config.CONTAINER_CONFIG_PATH
        self.container_prim = None
        
        self._load_config()

    def _load_config(self):
        if not os.path.exists(self.config_path):
            return None
        with open(self.config_path, 'r', encoding='utf-8') as f:
            return json.load(f)

    def respawn(self):
        config_data = self._load_config()
        if not config_data: return

        if self.container_prim and self.world.scene.object_exists(self.container_prim.name):
            self.world.scene.remove_object(self.container_prim.name)
        if prim_utils.is_prim_path_valid(usd_config.CONTAINER_PRIM_PATH):
            prim_utils.delete_prim(usd_config.CONTAINER_PRIM_PATH)

        # Read JSON
        selected_key = config_data.get("selected", "")
        container_info = config_data.get("containers", {}).get(selected_key, {})
        
        name = container_info.get("name", "container")
        rel_path = container_info.get("path", "")

        # Spawn Position (If point doesn't exists, use it)
        spawn_pos = np.array([0.5, 0.0, 0.0])
        spawn_rot = np.array([1.0, 0.0, 0.0, 0.0])
        if prim_utils.is_prim_path_valid(usd_config.CONTAINER_POINT_PATH):
            point_prim = XFormPrim(usd_config.CONTAINER_POINT_PATH)
            spawn_pos, spawn_rot = point_prim.get_world_pose()

        # Spawn Container
        usd_path = os.path.join(usd_config.BASE_DIR, rel_path)
        add_reference_to_stage(usd_path=usd_path, prim_path=usd_config.CONTAINER_PRIM_PATH)
        
        self.container_prim = XFormPrim(
            prim_path=usd_config.CONTAINER_PRIM_PATH,
            name=name,
            position=spawn_pos,
            orientation=spawn_rot,
            scale=np.array([1.0, 1.0, 1.0])
        )
        self.world.scene.add(self.container_prim)
        print(f"[ContainerManager] Spawned '{name}' (Auto Bounding Box Mode)")

    def is_inside(self, object_pos):
        """
        Use USD API count Container's World Bounding Box, 
        and check if object_pos is inside the box.
        """
        if self.container_prim is None:
            return False

        stage = self.world.stage
        prim = stage.GetPrimAtPath(self.container_prim.prim_path)
        if not prim.IsValid():
            return False


        imageable = UsdGeom.Imageable(prim)
        bound = imageable.ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_)
        
        # Get AABB (Axis Aligned Bounding Box)
        box_range = bound.ComputeAlignedBox()
        min_p = box_range.GetMin() # Gf.Vec3d
        max_p = box_range.GetMax() # Gf.Vec3d

        # In range check
        x, y, z = object_pos
        
        in_x = min_p[0] <= x <= max_p[0]
        in_y = min_p[1] <= y <= max_p[1]
        in_z = min_p[2] <= z <= max_p[2] + 0.5

        return in_x and in_y and in_z