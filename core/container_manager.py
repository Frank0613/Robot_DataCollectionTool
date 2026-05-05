import os
import numpy as np
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from pxr import UsdGeom, Usd, Gf
from configs import usd_config
from configs.container_config import get_container_info

class ContainerManager:
    def __init__(self, world):
        self.world = world
        self.container_prim = None
        self._interior_spec = None

    def get_container_info(self, key=None):
        return get_container_info(key)

    def respawn(self):
        info = self.get_container_info()
        if not info: return

        if self.container_prim and self.world.scene.object_exists(self.container_prim.name):
            self.world.scene.remove_object(self.container_prim.name)
        if prim_utils.is_prim_path_valid(usd_config.CONTAINER_PRIM_PATH):
            prim_utils.delete_prim(usd_config.CONTAINER_PRIM_PATH)

        name = info.get("name", "container")
        rel_path = info.get("path", "")
        self._interior_spec = info.get("interior", "auto")

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
        mode = self._interior_spec if isinstance(self._interior_spec, str) else "local_box"
        print(f"[ContainerManager] Spawned '{name}' (interior={mode})")

    def is_inside(self, object_pos):
        """
        Check if object_pos lies inside the container's valid drop zone.
        Strategy depends on the active container's `interior` spec
        (see configs/container_config.py).
        """
        if self.container_prim is None:
            return False

        prim = self.world.stage.GetPrimAtPath(self.container_prim.prim_path)
        if not prim.IsValid():
            return False

        spec = self._interior_spec
        if spec == "auto" or spec is None:
            return self._is_inside_aabb(prim, object_pos)
        if isinstance(spec, dict):
            return self._is_inside_local_box(object_pos, spec)
        return False

    def _is_inside_aabb(self, prim, object_pos):
        imageable = UsdGeom.Imageable(prim)
        bound = imageable.ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_)
        box_range = bound.ComputeAlignedBox()
        min_p = box_range.GetMin()
        max_p = box_range.GetMax()

        x, y, z = object_pos
        in_x = min_p[0] <= x <= max_p[0]
        in_y = min_p[1] <= y <= max_p[1]
        in_z = min_p[2] <= z <= max_p[2] + 0.5
        return in_x and in_y and in_z

    def _is_inside_local_box(self, object_pos, spec):
        anchor_subpath = spec.get("anchor_subpath")
        if anchor_subpath:
            anchor_path = self.container_prim.prim_path.rstrip("/") + "/" + anchor_subpath.lstrip("/")
            anchor_prim = self.world.stage.GetPrimAtPath(anchor_path)
            if not anchor_prim.IsValid():
                print(f"[ContainerManager] Warning: anchor sub-prim '{anchor_path}' not found, falling back to container root")
                anchor_prim = self.world.stage.GetPrimAtPath(self.container_prim.prim_path)
        else:
            anchor_prim = self.world.stage.GetPrimAtPath(self.container_prim.prim_path)

        # Transform world point into anchor's local frame so the box moves with the anchor
        # (e.g. drawer cavity stays correct as the drawer slides out).
        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        local_to_world = xform_cache.GetLocalToWorldTransform(anchor_prim)
        world_to_local = local_to_world.GetInverse()
        local_pt = world_to_local.Transform(
            Gf.Vec3d(float(object_pos[0]), float(object_pos[1]), float(object_pos[2]))
        )

        ox, oy, oz = spec.get("offset", (0.0, 0.0, 0.0))
        hx, hy, hz = spec.get("half_size", (0.1, 0.1, 0.1))
        z_above = float(spec.get("z_tolerance_above", 0.0))

        in_x = (ox - hx) <= local_pt[0] <= (ox + hx)
        in_y = (oy - hy) <= local_pt[1] <= (oy + hy)
        in_z = (oz - hz) <= local_pt[2] <= (oz + hz + z_above)
        return in_x and in_y and in_z
