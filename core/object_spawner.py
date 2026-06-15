import os
import glob
import random
from omni.isaac.core.utils.prims import delete_prim, is_prim_path_valid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from pxr import UsdGeom
from configs import usd_config


class ObjectSpawner:
    def __init__(self, world, repeat_count=None, task_profile=None, fixture_mgr=None):
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

        # Optional task profile (configs/task_config.py): drives bg_skip and
        # delegates target-pose resolution to FixtureManager when set.
        self.task_profile = task_profile or {}
        self.fixture_mgr = fixture_mgr

        # Scan all USD files recursively (including subdirectories)
        search_path = os.path.join(usd_config.OBJECTS_DIR, "**/*.usd")
        self.available_usds = glob.glob(search_path, recursive=True)
        print(f"[ObjectSpawner] Found {len(self.available_usds)} USD files in {usd_config.OBJECTS_DIR} (recursive)")

        # Determine target object list based on task profile
        self.available_target_usds = self._build_target_list()
        target_source = "task-filtered" if self.available_target_usds != self.available_usds else "all objects"
        print(f"[ObjectSpawner] Target can be selected from {target_source} ({len(self.available_target_usds)} options)")

    def _build_target_list(self):
        """Build list of available target USDs based on task profile.

        If task_profile has target_objects_dir (via support_fixture),
        only return USDs from that directory. Otherwise return all available USDs.
        """
        target_dir = None
        if self.task_profile.get("support_fixture"):
            target_dir = self.task_profile["support_fixture"].get("target_objects_dir")

        if target_dir is None:
            # No restriction: target can be any object
            return self.available_usds

        # Filter to only objects in the specified directory
        abs_target_dir = os.path.normpath(os.path.join(usd_config.BASE_DIR, target_dir))
        filtered = [usd for usd in self.available_usds
                   if os.path.normpath(usd).startswith(abs_target_dir)]

        if filtered:
            print(f"[ObjectSpawner] Target objects filtered to {target_dir}: {len(filtered)} found")
        else:
            print(f"[ObjectSpawner] Warning: No objects found in {target_dir}, using all objects")
            return self.available_usds

        return filtered

    def get_target_class_name(self) -> str:
        """Return the semantic class name of the target object (USD filename without extension)"""
        return self.target_class_name

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

    def notify_task_success(self):
        """Advance the repeat cycle by one. Call after a demo is successfully saved."""
        if self.repeat_count:
            self._success_count += 1

    def _select_files(self):
        """Randomly select USD files (3 background + 1 target) and store in _selected_files.

        Background objects are selected from all available USDs.
        Target object is selected from available_target_usds (which may be filtered by task profile).

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

        if not self.available_target_usds:
            print("[Warning] No target USD files found to spawn!")
            self._selected_files = []
            return

        bg_count = len(usd_config.BG_SPAWN_POINTS)

        # Select target object first (from task-filtered list)
        target_usd = random.choice(self.available_target_usds)

        # Background objects come from all available USDs, excluding the target
        # so no object is spawned twice. random.sample guarantees the
        # backgrounds are also distinct from each other.
        bg_pool = [u for u in self.available_usds if u != target_usd]
        if len(bg_pool) >= bg_count:
            bg_selected = random.sample(bg_pool, k=bg_count)
        else:
            # Not enough unique objects to fill every point without repeats;
            # take all uniques and pad with random extras (duplicates possible).
            print(f"[ObjectSpawner] Warning: only {len(bg_pool)} unique background "
                  f"objects for {bg_count} points; some may repeat.")
            bg_selected = bg_pool[:]
            if bg_pool:
                bg_selected += random.choices(bg_pool, k=bg_count - len(bg_pool))

        self._selected_files = bg_selected + [target_usd]

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

        # Task profile may delegate target placement to FixtureManager (e.g.
        # spawn on top of a stool, or inside a drawer sub-prim). When no
        # fixture rule applies it falls back to reading target_point directly.
        if self.fixture_mgr is not None:
            position, orientation = self.fixture_mgr.get_target_spawn_pose(target_point_path)
        elif is_prim_path_valid(target_point_path):
            position, orientation = XFormPrim(target_point_path).get_world_pose()
            position = self._apply_spawn_override("target_point", position)
            position = usd_config.randomize_xy(position, point_name="target_point")
        else:
            position, orientation = None, None

        if position is not None:
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

        # bg_skip is declared by the active task profile (configs/task_config.py).
        # Stove tasks, for example, place the container in front of the robot
        # and need point_front skipped to avoid clashing with stove geometry.
        bg_skip = set(self.task_profile.get("bg_skip", []))

        for i, point_name in enumerate(usd_config.BG_SPAWN_POINTS):
            if point_name in bg_skip:
                print(f"[ObjectSpawner] Skipping {point_name} (task bg_skip)")
                continue
            point_path = f"{usd_config.OBJ_SPAWN_POINT_ROOT}/{point_name}"
            if not is_prim_path_valid(point_path):
                print(f"[Warning] Spawn point not found: {point_path}, skipping")
                continue
            position, orientation = XFormPrim(point_path).get_world_pose()
            position = self._apply_spawn_override(point_name, position)
            position = usd_config.randomize_xy(position, point_name=point_name)

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
