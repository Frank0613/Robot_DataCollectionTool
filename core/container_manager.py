import os
import numpy as np
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from pxr import UsdGeom, Usd, Gf
from configs import usd_config, scene_config
from configs.container_config import get_container_info

class ContainerManager:
    def __init__(self, world):
        self.world = world
        self.container_prim = None
        self._interior_spec = None

        # Knob-task state (populated on respawn if the active container has a knob spec)
        self._knob_subpath = None
        self._knob_threshold_rad = None
        self._knob_baseline_quat = None

        # Debug: when True, respawn() also draws the interior success zone box.
        self.debug_zone = False
        self._zone_path = None

    def get_container_info(self, key=None):
        return get_container_info(key)

    def respawn(self, override_pose=None):
        """Spawn the active container. When `override_pose` (a (position, quat)
        pair) is given, place it there EXACTLY with no randomization — used by
        eval to reproduce a recorded initial_scene. Otherwise read the preset
        point and apply the scene_config randomization."""
        info = self.get_container_info()
        if not info: return

        if self.container_prim and self.world.scene.object_exists(self.container_prim.name):
            self.world.scene.remove_object(self.container_prim.name)
        if prim_utils.is_prim_path_valid(usd_config.CONTAINER_PRIM_PATH):
            prim_utils.delete_prim(usd_config.CONTAINER_PRIM_PATH)

        name = info.get("name", "container")
        rel_path = info.get("path", "")
        self._interior_spec = info.get("interior", "auto")

        if override_pose is not None:
            # Eval replay: place exactly at the recorded pose, no randomization.
            spawn_pos = np.asarray(override_pose[0], dtype=np.float64)
            spawn_rot = np.asarray(override_pose[1], dtype=np.float64)
        else:
            # Spawn Position (If point doesn't exists, use a default)
            spawn_pos = np.array([0.5, 0.0, 0.0])
            spawn_rot = np.array([1.0, 0.0, 0.0, 0.0])
            if prim_utils.is_prim_path_valid(usd_config.CONTAINER_POINT_PATH):
                point_prim = XFormPrim(usd_config.CONTAINER_POINT_PATH)
                spawn_pos, spawn_rot = point_prim.get_world_pose()

            # Optional euler_deg override (scene_config) replaces the preset
            # orientation so a flipped container can be corrected upright.
            euler = scene_config.get_container_euler_deg()
            if euler is not None:
                spawn_rot = np.array(usd_config.euler_deg_to_quat(*euler), dtype=np.float64)

            # Randomize the container within a circle around its preset point,
            # plus a random yaw about the world Z axis (ranges from scene_config).
            spawn_pos = usd_config.randomize_xy(spawn_pos, radius=scene_config.get_container_radius())
            spawn_rot = usd_config.randomize_yaw(spawn_rot, yaw_range=scene_config.get_container_yaw_range())

        # Spawn Container
        usd_path = os.path.join(usd_config.BASE_DIR, rel_path)
        add_reference_to_stage(usd_path=usd_path, prim_path=usd_config.CONTAINER_PRIM_PATH)

        xform_kwargs = dict(
            prim_path=usd_config.CONTAINER_PRIM_PATH,
            name=name,
            position=spawn_pos,
            orientation=spawn_rot,
        )
        info_scale = info.get("scale")
        if info_scale is not None:
            s = float(info_scale)
            xform_kwargs["scale"] = np.array([s, s, s])
        self.container_prim = XFormPrim(**xform_kwargs)
        self.world.scene.add(self.container_prim)
        mode = self._interior_spec if isinstance(self._interior_spec, str) else "local_box"
        scale_str = f", scale={info_scale}" if info_scale is not None else ""
        print(f"[ContainerManager] Spawned '{name}' (interior={mode}){scale_str}")

        # Reset and (re)capture knob baseline if this container has a knob spec
        self._knob_subpath = None
        self._knob_threshold_rad = None
        self._knob_baseline_quat = None
        knob_spec = info.get("knob")
        if knob_spec:
            self._knob_subpath = knob_spec.get("subpath", "")
            self._knob_threshold_rad = float(np.deg2rad(knob_spec.get("threshold_deg", 30.0)))
            _, baseline_quat = self._get_knob_world_pose()
            if baseline_quat is None:
                print(f"[ContainerManager] Warning: knob sub-prim '{self._knob_subpath}' not found")
            else:
                self._knob_baseline_quat = baseline_quat
                print(f"[ContainerManager] Knob baseline saved "
                      f"(subpath='{self._knob_subpath}', threshold={knob_spec.get('threshold_deg', 30.0)}°)")

        if self.debug_zone:
            self._draw_interior_zone()

    def _draw_interior_zone(self):
        """Draw the success/drop zone (the exact volume is_inside() tests) as a
        semi-transparent green cube, for visually tuning container_config's
        'interior'. The cube is parented under the anchor so it inherits the
        container's world transform (scale/rotation) and therefore matches
        is_inside() exactly — including a drawer cavity that slides with the tray.

        NOTE: this is a visible mesh — it WILL show up in camera RGB. Use it for
        manual tuning in the viewport, NOT while recording demos.
        """
        if self.container_prim is None:
            return

        # Clean up a previous zone box.
        if self._zone_path and prim_utils.is_prim_path_valid(self._zone_path):
            prim_utils.delete_prim(self._zone_path)
            self._zone_path = None

        stage = self.world.stage
        spec = self._interior_spec

        if isinstance(spec, dict):
            ox, oy, oz = spec.get("offset", (0.0, 0.0, 0.0))
            hx, hy, hz = spec.get("half_size", (0.1, 0.1, 0.1))
            za = float(spec.get("z_tolerance_above", 0.0))
            # Box spans local z in [oz-hz, oz+hz+za]; center/size reflect that.
            center = (float(ox), float(oy), float(oz) + za / 2.0)
            size = (2.0 * float(hx), 2.0 * float(hy), 2.0 * float(hz) + za)

            anchor_sub = spec.get("anchor_subpath")
            if anchor_sub:
                parent = self.container_prim.prim_path.rstrip("/") + "/" + anchor_sub.lstrip("/")
            else:
                parent = self.container_prim.prim_path
            cube_path = parent.rstrip("/") + "/_interior_zone_debug"
        else:
            # "auto": world AABB of the container + 0.5 m z headroom (matches
            # _is_inside_aabb). Axis-aligned in world, so draw at top level.
            prim = stage.GetPrimAtPath(self.container_prim.prim_path)
            bound = UsdGeom.Imageable(prim).ComputeWorldBound(
                Usd.TimeCode.Default(), UsdGeom.Tokens.default_)
            box = bound.ComputeAlignedBox()
            mn, mx = box.GetMin(), box.GetMax()
            center = ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0,
                      (mn[2] + mx[2]) / 2.0 + 0.25)  # +0.5 headroom / 2
            size = (mx[0] - mn[0], mx[1] - mn[1], (mx[2] - mn[2]) + 0.5)
            cube_path = "/World/_interior_zone_debug"

        if prim_utils.is_prim_path_valid(cube_path):
            prim_utils.delete_prim(cube_path)

        cube = UsdGeom.Cube.Define(stage, cube_path)
        cube.CreateSizeAttr(1.0)  # unit cube; scale op sets the real extents
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*[float(c) for c in size]))
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.0, 1.0, 0.0)])
        cube.CreateDisplayOpacityAttr([0.25])
        self._zone_path = cube_path
        print(f"[ContainerManager] Interior zone box drawn at {cube_path} "
              f"(size={tuple(round(float(c), 3) for c in size)}); visible in cameras")

    def get_state(self):
        """Return the active container's name and world root_pose, or None.

        root_pose is [x, y, z, qw, qx, qy, qz] (position + quaternion = position
        & angle), used to snapshot the initial scene layout for evaluation.
        """
        if self.container_prim is None:
            return None
        pos, quat = self.container_prim.get_world_pose()
        return {
            "name": self.container_prim.name,
            "root_pose": np.concatenate(
                [np.asarray(pos, dtype=np.float64),
                 np.asarray(quat, dtype=np.float64)]
            ),
        }

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

    def _get_knob_world_pose(self):
        """Return (pos, quat) for the knob in world space, both np.arrays.
        Reads live state via XFormPrim so physics-driven rotation is captured.
        Returns (None, None) if the knob is not configured or not found.
        """
        if not self._knob_subpath or self.container_prim is None:
            return None, None
        knob_path = self.container_prim.prim_path.rstrip("/") + "/" + self._knob_subpath.lstrip("/")
        if not prim_utils.is_prim_path_valid(knob_path):
            return None, None
        pos, quat = XFormPrim(prim_path=knob_path).get_world_pose()
        return np.asarray(pos, dtype=np.float64), np.asarray(quat, dtype=np.float64)

    def _knob_z_angle_delta(self):
        """Relative rotation around world-z (radians) between current knob pose
        and the baseline captured at spawn. None if no baseline available.
        """
        if self._knob_baseline_quat is None:
            return None
        _, cur = self._get_knob_world_pose()
        if cur is None:
            return None
        # q_rel = q_baseline^-1 * q_current  (Hamilton product, baseline conjugated)
        wb, xb, yb, zb = self._knob_baseline_quat
        wc, xc, yc, zc = cur
        wbi, xbi, ybi, zbi = wb, -xb, -yb, -zb
        w_rel = wbi*wc - xbi*xc - ybi*yc - zbi*zc
        z_rel = wbi*zc + xbi*yc - ybi*xc + zbi*wc
        # Twist around world-z: angle = 2 * atan2(z, w)
        return 2.0 * float(np.arctan2(z_rel, w_rel))

    def is_knob_turned(self):
        """True iff |z-axis rotation delta| >= threshold_deg (configured per container)."""
        if self._knob_threshold_rad is None:
            return False
        delta = self._knob_z_angle_delta()
        if delta is None:
            return False
        return abs(delta) >= self._knob_threshold_rad

    def get_knob_state(self):
        """Live knob state for HDF5 logging. Returns dict or None if no knob configured.
        Keys: world_pos (3,), world_quat (4, w-first), angle_rad (z-axis delta from baseline).
        """
        if self._knob_baseline_quat is None:
            return None
        pos, quat = self._get_knob_world_pose()
        if pos is None:
            return None
        delta = self._knob_z_angle_delta()
        return {
            "world_pos":  pos,
            "world_quat": quat,
            "angle_rad":  float(delta) if delta is not None else 0.0,
        }

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
