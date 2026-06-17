import os
import glob
import numpy as np
from omni.isaac.core.utils.prims import delete_prim, is_prim_path_valid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from pxr import UsdGeom
from configs import usd_config, scene_config


class ObjectSpawner:
    def __init__(self, world, task_profile=None, fixture_mgr=None):
        self.world = world
        self.spawned_objects = []
        self.target_object = None
        self.target_class_name = None  # semantic class name of the target
        self.bg_class_by_point = {}    # {"point_right": "ramekin", ...}

        # point name -> resolved USD path (or None to leave empty), filled by
        # _select_files() from configs/scene_config.py LAYOUT.
        self._point_to_usd = {}

        # Eval replay: when set (via apply_scene_objects), the randomized spawn
        # path uses these per-point objects instead of scene_config.LAYOUT, so
        # resets after the first exact reproduction keep the same objects.
        self._object_override = None

        # Optional task profile (configs/task_config.py): drives bg_skip and
        # delegates target-pose resolution to FixtureManager when set.
        self.task_profile = task_profile or {}
        self.fixture_mgr = fixture_mgr

        # Scan all USD files recursively so a bare filename in scene_config can be
        # resolved to its full path regardless of which subdirectory it lives in.
        search_path = os.path.join(usd_config.OBJECTS_DIR, "**/*.usd")
        self.available_usds = glob.glob(search_path, recursive=True)
        print(f"[ObjectSpawner] Found {len(self.available_usds)} USD files in {usd_config.OBJECTS_DIR} (recursive)")
        print(f"[ObjectSpawner] Using deterministic layout from configs/scene_config.py")

    def get_target_class_name(self) -> str:
        """Return the semantic class name of the target object (USD filename without extension)"""
        return self.target_class_name

    def _resolve_file(self, filename):
        """Resolve a scene_config `file` entry to a full USD path.

        Accepts a bare filename ("bolt.usd") searched by basename across
        available_usds, or a path relative to OBJECTS_DIR ("small/bolt.usd").
        Returns None for an unset/empty entry or when no match is found.
        """
        if not filename:
            return None

        # Path relative to OBJECTS_DIR (allows disambiguating duplicate basenames).
        rel_candidate = os.path.normpath(os.path.join(usd_config.OBJECTS_DIR, filename))
        if os.path.isfile(rel_candidate):
            return rel_candidate

        # Otherwise match by basename across all discovered USDs.
        base = os.path.basename(filename)
        matches = [u for u in self.available_usds if os.path.basename(u) == base]
        if matches:
            if len(matches) > 1:
                print(f"[ObjectSpawner] Warning: '{filename}' matches {len(matches)} files; "
                      f"using {matches[0]}")
            return matches[0]

        print(f"[ObjectSpawner] Warning: object file '{filename}' not found under "
              f"{usd_config.OBJECTS_DIR}; leaving point empty")
        return None

    def _base_orientation(self, point_name, point_orientation):
        """Resolve the object's base resting orientation before random yaw.

        If scene_config gives an euler_deg override for `point_name`, build the
        orientation from it (replacing the spawn point's preset orientation) so a
        flipped/sideways asset can be corrected. Otherwise keep the spawn point's
        orientation.
        """
        euler = scene_config.get_euler_deg(point_name)
        if euler is not None:
            return np.array(usd_config.euler_deg_to_quat(*euler), dtype=np.float64)
        return point_orientation

    def _apply_spawn_override(self, point_name, position):
        """Apply per-task position deltas (x/y/z_offset) from task_profile's
        spawn_overrides for `point_name`. Returns a (possibly shifted) tuple."""
        override = self.task_profile.get("spawn_overrides", {}).get(point_name)
        if not override:
            return position
        dx = float(override.get("x_offset", 0.0))
        dy = float(override.get("y_offset", 0.0))
        dz = float(override.get("z_offset", 0.0))
        shifted = (
            float(position[0]) + dx,
            float(position[1]) + dy,
            float(position[2]) + dz,
        )
        print(f"[ObjectSpawner] spawn_override on {point_name}: "
              f"dx={dx:+.3f} dy={dy:+.3f} dz={dz:+.3f}")
        return shifted

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

    def _select_files(self):
        """Resolve the deterministic per-point object assignment from scene_config.

        Builds self._point_to_usd = {point_name: full_usd_path_or_None}. Unlike
        the old random selection, this is fully determined by configs/scene_config.py.
        """
        # Eval replay locks objects to a recorded scene; otherwise use scene_config.
        if self._object_override is not None:
            self._point_to_usd = dict(self._object_override)
            return

        self._point_to_usd = {}
        for point_name, entry in scene_config.LAYOUT.items():
            filename = entry.get("file") if entry else None
            self._point_to_usd[point_name] = self._resolve_file(filename)

    def apply_scene_objects(self, scene):
        """Lock per-point object identities to a recorded scene (eval replay).

        After this, the randomized spawn path (spawn_target_only /
        spawn_remaining_objects) uses these objects instead of scene_config.LAYOUT,
        so resets AFTER the first exact reproduction keep the same objects but
        re-randomize position/angle. The container role is handled separately.
        """
        if not scene:
            self._object_override = None
            return
        override = {}
        for role, entry in scene.items():
            if role == "container":
                continue
            # "target" maps to the target_point slot; point_* roles map to themselves.
            point = "target_point" if role == "target" else role
            override[point] = self._resolve_file(f"{entry['class']}.usd")
        self._object_override = override

    def _add_object(self, usd_path, obj_name, position, orientation):
        """Reference `usd_path` into the stage at /World/<obj_name>, register the
        first imageable child as a scene XFormPrim, and return its scene name."""
        container_path = f"/World/{obj_name}"
        if is_prim_path_valid(container_path):
            delete_prim(container_path)

        add_reference_to_stage(usd_path=usd_path, prim_path=container_path)
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
        return obj_name

    def spawn_target_only(self):
        """
        [Phase 1] Spawn only the target object to target_point.
        Used for baseline capture in occlusion rate calculation.
        """
        self._clean_all()
        self._select_files()

        target_usd = self._point_to_usd.get("target_point")
        if not target_usd:
            print("[ObjectSpawner] No target object assigned (scene_config target_point); "
                  "skipping target spawn")
            return

        self.target_class_name = os.path.splitext(os.path.basename(target_usd))[0]

        target_point_path = f"{usd_config.OBJ_SPAWN_POINT_ROOT}/target_point"

        # Task profile may delegate target placement to FixtureManager (e.g.
        # spawn on top of a stool, or inside a drawer sub-prim). When no
        # fixture rule applies it falls back to reading target_point directly.
        if self.fixture_mgr is not None:
            position, orientation = self.fixture_mgr.get_target_spawn_pose(target_point_path)
        elif is_prim_path_valid(target_point_path):
            position, orientation = XFormPrim(target_point_path).get_world_pose()
            position = self._apply_spawn_override("target_point", position)
            position = usd_config.randomize_xy(position, radius=scene_config.get_radius("target_point"))
            orientation = self._base_orientation("target_point", orientation)
            orientation = usd_config.randomize_yaw(orientation, yaw_range=scene_config.get_yaw_range("target_point"))
        else:
            position, orientation = None, None

        if position is not None:
            obj_name = "spawned_obj_target"
            self._add_object(target_usd, obj_name, position, orientation)
            self.target_object = obj_name
            print(f"[ObjectSpawner] Target spawned: {self.target_class_name} (class for semantic)")
        else:
            print(f"[Warning] target_point not found at {target_point_path}")

    def spawn_remaining_objects(self):
        """
        [Phase 2] Spawn background objects to named points (see usd_config.BG_SPAWN_POINTS).
        Call after baseline capture is complete.
        """
        # bg_skip is declared by the active task profile (configs/task_config.py).
        # Stove tasks, for example, place the container in front of the robot
        # and need point_front skipped to avoid clashing with stove geometry.
        bg_skip = set(self.task_profile.get("bg_skip", []))

        for point_name in usd_config.BG_SPAWN_POINTS:
            if point_name in bg_skip:
                print(f"[ObjectSpawner] Skipping {point_name} (task bg_skip)")
                continue
            usd_file = self._point_to_usd.get(point_name)
            if not usd_file:
                # Empty point in scene_config -> nothing spawns here.
                continue
            point_path = f"{usd_config.OBJ_SPAWN_POINT_ROOT}/{point_name}"
            if not is_prim_path_valid(point_path):
                print(f"[Warning] Spawn point not found: {point_path}, skipping")
                continue
            position, orientation = XFormPrim(point_path).get_world_pose()
            position = self._apply_spawn_override(point_name, position)
            position = usd_config.randomize_xy(position, radius=scene_config.get_radius(point_name))
            orientation = self._base_orientation(point_name, orientation)
            orientation = usd_config.randomize_yaw(orientation, yaw_range=scene_config.get_yaw_range(point_name))

            obj_name = f"spawned_obj_{point_name}"
            self.bg_class_by_point[point_name] = os.path.splitext(os.path.basename(usd_file))[0]
            self._add_object(usd_file, obj_name, position, orientation)
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

    def respawn_from_scene(self, scene):
        """Reproduce a recorded initial_scene EXACTLY (eval): spawn the same
        objects at the same world poses with NO randomization.

        `scene` is the dict from core.eval_scene.load_initial_scene:
        {role: {"class": str, "pose": np.ndarray(7,)}}. The container role is
        handled separately by ContainerManager; this method spawns the target
        and the background distractors.
        """
        self._clean_all()
        if not scene:
            print("[ObjectSpawner] respawn_from_scene: empty scene, nothing to spawn")
            return

        def _spawn(role, obj_name):
            entry = scene.get(role)
            if not entry or entry.get("pose") is None:
                return None
            usd = self._resolve_file(f"{entry['class']}.usd")
            if not usd:
                print(f"[ObjectSpawner] [replay] could not resolve '{entry['class']}' for {role}")
                return None
            pose = entry["pose"]
            self._add_object(usd, obj_name, pose[:3], pose[3:7])
            return entry["class"]

        # Target
        target_cls = _spawn("target", "spawned_obj_target")
        if target_cls is not None:
            self.target_object = "spawned_obj_target"
            self.target_class_name = target_cls
            print(f"[ObjectSpawner] [replay] target: {target_cls}")

        # Background distractors
        for point_name in usd_config.BG_SPAWN_POINTS:
            cls = _spawn(point_name, f"spawned_obj_{point_name}")
            if cls is not None:
                self.bg_class_by_point[point_name] = cls
                self.spawned_objects.append(f"spawned_obj_{point_name}")

        print(f"[ObjectSpawner] [replay] spawned {len(self.spawned_objects)} "
              f"background objects + target: {self.target_object}")
