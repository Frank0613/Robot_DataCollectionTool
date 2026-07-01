"""Spawns the optional support fixture (a static USD that the target object
either sits on top of or spawns inside, e.g. a stool, a cabinet, a tabletop).

Distinct from ContainerManager: containers are the SUCCESS zone (where the
arm places the object); fixtures are the SOURCE site (where the target lives
before pickup) and don't participate in termination checks.

No-op when the active task profile has no `support_fixture` field, so it's
safe to instantiate unconditionally from main.py.
"""
import os
import numpy as np
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from pxr import UsdGeom, Usd, Gf
from configs import usd_config, scene_config


FIXTURE_PRIM_PATH = "/World/SupportFixture"


class FixtureManager:
    def __init__(self, world, task_profile=None):
        self.world = world
        self.task_profile = task_profile or {}
        self.fixture_prim = None
        self._spec = self.task_profile.get("support_fixture")  # None when not used

    def has_fixture(self):
        return self._spec is not None

    def get_fixture_name(self):
        if not self._spec:
            return None
        return self._spec.get("name")

    def get_state(self):
        """Current fixture pose for initial_scene recording, or None when there
        is no fixture / it hasn't spawned yet. Mirrors ContainerManager.get_state.
        Returns {"name": str, "root_pose": np.ndarray(7,) [x,y,z,qw,qx,qy,qz]}."""
        if not self._spec or self.fixture_prim is None:
            return None
        pos, quat = self.fixture_prim.get_world_pose()
        return {
            "name": self._spec.get("name", "fixture"),
            "root_pose": np.concatenate(
                [np.asarray(pos, dtype=np.float64), np.asarray(quat, dtype=np.float64)]
            ),
        }

    def make_invisible(self):
        """Hide the fixture from rendering."""
        if self.fixture_prim is not None:
            prim = self.world.stage.GetPrimAtPath(self.fixture_prim.prim_path)
            if prim.IsValid():
                imageable = UsdGeom.Imageable(prim)
                imageable.MakeInvisible()

    def make_visible(self):
        """Show the fixture in rendering."""
        if self.fixture_prim is not None:
            prim = self.world.stage.GetPrimAtPath(self.fixture_prim.prim_path)
            if prim.IsValid():
                imageable = UsdGeom.Imageable(prim)
                imageable.MakeVisible()

    def get_occlusion_hide_names(self):
        """Names the occlusion baseline should hide (a fixture sitting under
        the target counts as static occlusion). Returns [] when no fixture."""
        if not self._spec:
            return []
        name = self._spec.get("name")
        return [name] if name else []

    def respawn(self, override_pose=None):
        """Re-spawn the fixture at its anchor_point. Safe to call every reset.
        No-op when the task profile has no support_fixture.

        override_pose: optional (pos[3], quat[4 w-first]) placing the fixture
        EXACTLY (eval replay reproducing a recorded scene). Skips euler_deg,
        z_offset and per-reset randomization so the box lands back under the
        recorded target."""
        if not self._spec:
            return

        # Tear down previous spawn
        if prim_utils.is_prim_path_valid(FIXTURE_PRIM_PATH):
            prim_utils.delete_prim(FIXTURE_PRIM_PATH)

        rel_usd = self._spec.get("usd", "")
        anchor_point = self._spec.get("anchor_point", "target_point")
        name = self._spec.get("name", "fixture")

        if override_pose is not None:
            # Eval replay: exact recorded pose, no randomization.
            spawn_pos = np.asarray(override_pose[0], dtype=np.float64)
            spawn_rot = np.asarray(override_pose[1], dtype=np.float64)
        else:
            anchor_path = f"{usd_config.OBJ_SPAWN_POINT_ROOT}/{anchor_point}"
            if not prim_utils.is_prim_path_valid(anchor_path):
                print(f"[FixtureManager] Warning: anchor point '{anchor_path}' not "
                      f"found in scene; falling back to (0.5, 0, 0)")
                spawn_pos = np.array([0.5, 0.0, 0.0])
                spawn_rot = np.array([1.0, 0.0, 0.0, 0.0])
            else:
                spawn_pos, spawn_rot = XFormPrim(anchor_path).get_world_pose()

            # Optional euler_deg override replaces the anchor's orientation so an
            # asset can be laid down / tipped over (e.g. stand a box on its face).
            euler = self._spec.get("euler_deg")
            if euler is not None:
                spawn_rot = np.array(usd_config.euler_deg_to_quat(*euler), dtype=np.float64)

            # Optional z lift (meters) so a laid-down fixture rests on the table
            # surface instead of sinking through it.
            z_offset = self._spec.get("z_offset")
            if z_offset is not None:
                spawn_pos = np.array([spawn_pos[0], spawn_pos[1], spawn_pos[2] + float(z_offset)])

            # Opt-in per-reset randomization (data augmentation): jitter the
            # fixture's XY + yaw using the anchor point's ranges from
            # scene_config. The target reads the fixture's world pose, so the
            # object on top follows the box. Off by default so other fixture
            # tasks (pan, cabinet) keep their fixed placement.
            if self._spec.get("randomize", False):
                spawn_pos = usd_config.randomize_xy(
                    np.asarray(spawn_pos, dtype=np.float64),
                    radius=scene_config.get_radius(anchor_point))
                spawn_rot = usd_config.randomize_yaw(
                    np.asarray(spawn_rot, dtype=np.float64),
                    yaw_range=scene_config.get_yaw_range(anchor_point))

        usd_path = os.path.join(usd_config.BASE_DIR, rel_usd)
        add_reference_to_stage(usd_path=usd_path, prim_path=FIXTURE_PRIM_PATH)

        xform_kwargs = dict(
            prim_path=FIXTURE_PRIM_PATH,
            name=name,
            position=spawn_pos,
            orientation=spawn_rot,
        )
        spec_scale = self._spec.get("scale")
        if spec_scale is not None:
            s = float(spec_scale)
            xform_kwargs["scale"] = np.array([s, s, s])
        self.fixture_prim = XFormPrim(**xform_kwargs)
        scale_str = f", scale={spec_scale}" if spec_scale is not None else ""
        print(f"[FixtureManager] Spawned fixture '{name}' at {anchor_point}{scale_str}")

    def get_target_spawn_pose(self, fallback_point_path):
        """Where should ObjectSpawner place the target?

        Resolution order:
          1. target_inside_fixture -> local frame of a fixture sub-prim
          2. target_offset_above_fixture -> fixture world position + z
          3. fallback (no fixture rules) -> read fallback_point_path directly

        Returns (np.ndarray pos[3], np.ndarray quat[4 w-first]).
        """
        inside = self.task_profile.get("target_inside_fixture")
        above = self.task_profile.get("target_offset_above_fixture")

        if inside and self.fixture_prim is not None:
            subpath = inside.get("subpath", "")
            offset = inside.get("offset", [0.0, 0.0, 0.0])
            sub_path = FIXTURE_PRIM_PATH.rstrip("/") + "/" + subpath.lstrip("/")
            anchor_prim = self.world.stage.GetPrimAtPath(sub_path)
            if not anchor_prim.IsValid():
                print(f"[FixtureManager] Warning: target_inside_fixture sub-prim "
                      f"'{sub_path}' not found; falling back to fixture root")
                anchor_prim = self.world.stage.GetPrimAtPath(FIXTURE_PRIM_PATH)
            xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
            l2w = xform_cache.GetLocalToWorldTransform(anchor_prim)
            world_pt = l2w.Transform(
                Gf.Vec3d(float(offset[0]), float(offset[1]), float(offset[2]))
            )
            pos = np.array([world_pt[0], world_pt[1], world_pt[2]])
            # Reuse fixture orientation so the target faces the right way
            _, quat = self.fixture_prim.get_world_pose()
            return pos, np.asarray(quat, dtype=np.float64)

        if above is not None and self.fixture_prim is not None:
            fpos, _ = self.fixture_prim.get_world_pose()
            pos = np.array([fpos[0], fpos[1], fpos[2] + float(above)])
            # The target's orientation must be INDEPENDENT of the fixture's:
            # when the fixture is tipped via euler_deg, the object resting on
            # top should still stand upright. Take orientation from the anchor
            # point (optionally overridden by scene_config euler_deg for
            # target_point), never from the (possibly tipped) fixture quat.
            euler = scene_config.get_euler_deg("target_point")
            if euler is not None:
                quat = np.array(usd_config.euler_deg_to_quat(*euler), dtype=np.float64)
            elif prim_utils.is_prim_path_valid(fallback_point_path):
                _, quat = XFormPrim(fallback_point_path).get_world_pose()
            else:
                quat = np.array([1.0, 0.0, 0.0, 0.0])
            return pos, np.asarray(quat, dtype=np.float64)

        # No fixture-driven rule: defer to the scene's target_point
        if prim_utils.is_prim_path_valid(fallback_point_path):
            pos, quat = XFormPrim(fallback_point_path).get_world_pose()
            # Apply spawn_override if present in task_profile
            override = self.task_profile.get("spawn_overrides", {}).get("target_point")
            if override:
                dx = float(override.get("x_offset", 0.0))
                dy = float(override.get("y_offset", 0.0))
                dz = float(override.get("z_offset", 0.0))
                pos = np.array([pos[0] + dx, pos[1] + dy, pos[2] + dz])
            # Randomize the target within a circle around its preset point,
            # plus a random yaw about the world Z axis (ranges from scene_config).
            pos = usd_config.randomize_xy(np.asarray(pos, dtype=np.float64),
                                          radius=scene_config.get_radius("target_point"))
            quat = usd_config.randomize_yaw(np.asarray(quat, dtype=np.float64),
                                            yaw_range=scene_config.get_yaw_range("target_point"))
            return np.asarray(pos, dtype=np.float64), np.asarray(quat, dtype=np.float64)
        return None, None
