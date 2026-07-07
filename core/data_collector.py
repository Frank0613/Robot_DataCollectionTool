import os
import h5py
import numpy as np
import json
from omni.isaac.core.prims import RigidPrim, XFormPrim
from omni.isaac.core.utils.prims import is_prim_path_valid
from pxr import Usd, UsdPhysics
from configs import task_config

class DataCollector:
    def __init__(self, save_dir="datasets", filename="dataset.hdf5", env_name="Default_Scene",
                 enabled=True, config_signature=None):
        self.enabled = enabled
        self.recording = False
        self.current_demo_data = []
        self.initial_state_snapshot = None
        self.camera_manager = None
        self.step_counter = 0
        self.record_interval = 1

        if not self.enabled:
            self.save_dir = None
            self.filepath = None
            print("[DataCollector] Disabled (inference mode) — no data will be recorded")
            return

        # A run's config signature ties a dataset file to one object/container/
        # scene/task combination. When appending to an existing file whose
        # signature differs, we auto-route to a fresh file (dataset_2.hdf5, ...)
        # so a single file never mixes layouts (which would desync the one-shot
        # initial_scene / language_instruction used for eval reproduction).
        self._signature = (
            json.dumps(config_signature, sort_keys=True)
            if config_signature is not None else None
        )

        self.save_dir = save_dir
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        self.filepath = self._resolve_filepath(os.path.join(self.save_dir, filename))

        # Init file if not exists
        with h5py.File(self.filepath, 'a') as f:
            if 'data' not in f:
                data_group = f.create_group('data')
                env_args = {"env_name": env_name}
                data_group.attrs['env_args'] = json.dumps(env_args)
                data_group.attrs['total'] = 0
                if self._signature is not None:
                    data_group.attrs['config_signature'] = self._signature

        print(f"[DataCollector] Recording to {self.filepath}")

    def _file_is_compatible(self, path):
        """True if `path` can receive demos for the current run: an empty/new
        file, or one whose stored config_signature matches this run's. A file
        with data but a different (or missing) signature is incompatible."""
        try:
            with h5py.File(path, 'r') as f:
                if 'data' not in f or 'total' not in f['data'].attrs:
                    return True
                if int(f['data'].attrs['total']) == 0:
                    return True
                stored = f['data'].attrs.get('config_signature')
                return stored is not None and stored == self._signature
        except Exception:
            return False

    def _resolve_filepath(self, base_path):
        """Pick the dataset file for this run. Same config -> reuse the matching
        file; different config -> the next free/compatible suffixed name."""
        if self._signature is None:
            return base_path
        if not os.path.exists(base_path) or self._file_is_compatible(base_path):
            return base_path

        root, ext = os.path.splitext(base_path)
        idx = 2
        while True:
            candidate = f"{root}_{idx}{ext}"
            if not os.path.exists(candidate) or self._file_is_compatible(candidate):
                print(f"[DataCollector] Config differs from {os.path.basename(base_path)}; "
                      f"using {os.path.basename(candidate)} instead")
                return candidate
            idx += 1

    def _get_real_name(self, robot_controller, spawn_obj_name):
        """get the actual object name inside the USD (RigidBodyAPI name)"""
        scene_obj = robot_controller.world.scene.get_object(spawn_obj_name)
        if not scene_obj:
            return spawn_obj_name
        
        stage = robot_controller.world.stage
        base_prim = stage.GetPrimAtPath(scene_obj.prim_path)
        
        for prim in Usd.PrimRange(base_prim):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                return prim.GetName()
        
        return spawn_obj_name 
    
    def capture_initial_state(self, robot_controller, spawner, container_manager=None,
                              fixture_manager=None):
        """capture initial state of robot and spawned objects"""
        snapshot = {
            "robot": {
                "root_pose": None,
                "root_vel": None
            },
            "objects": {},
            "container": None,
            "fixture": None
        }

        if container_manager is not None:
            snapshot["container"] = container_manager.get_state()

        if fixture_manager is not None:
            snapshot["fixture"] = fixture_manager.get_state()
        
        pos, quat = robot_controller.franka.get_world_pose()
        snapshot["robot"]["root_pose"] = np.concatenate([pos, quat])
        snapshot["robot"]["root_vel"] = np.concatenate([
            robot_controller.franka.get_linear_velocity(),
            robot_controller.franka.get_angular_velocity()
        ])
        stage = robot_controller.world.stage

        # Capture every background object plus the target, so the very first
        # demo can record a full snapshot of the initial scene layout.
        obj_names = list(spawner.spawned_objects)
        if spawner.target_object and spawner.target_object not in obj_names:
            obj_names.append(spawner.target_object)

        for obj_name in obj_names:
            real_name = self._get_real_name(robot_controller, obj_name)

            stage = robot_controller.world.stage
            # Search from the loaded root prim (/World/<obj_name>) — where
            # add_reference_to_stage mounts the whole USD — instead of the
            # registered imageable child, so the rigid body is found regardless
            # of each object's internal prim hierarchy.
            base_prim = stage.GetPrimAtPath(f"/World/{obj_name}")

            target_rb_prim = None
            for prim in Usd.PrimRange(base_prim):
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    target_rb_prim = prim
                    break
            
            if target_rb_prim:
                rb_path = str(target_rb_prim.GetPath())
                temp_rb = RigidPrim(prim_path=rb_path, name="temp_fetch")

                # Pose of the reference ROOT prim we actually placed at spawn
                # (/World/<obj_name>). initial_scene must record THIS (not the
                # inner rigid body) so eval replay — which sets the root — round
                # trips exactly. Falls back to the rigid body pose if absent.
                spawn_root_pose = None
                root_path = f"/World/{obj_name}"
                if is_prim_path_valid(root_path):
                    rpos, rquat = XFormPrim(prim_path=root_path).get_world_pose()
                    spawn_root_pose = np.concatenate([rpos, rquat])

                snapshot["objects"][obj_name] = {
                    "child_name": real_name,
                    "root_pose": np.concatenate(temp_rb.get_world_pose()),
                    "root_velocity": np.concatenate([temp_rb.get_linear_velocity(), temp_rb.get_angular_velocity()]),
                    "spawn_root_pose": spawn_root_pose,
                }
        
        self.initial_state_snapshot = snapshot

    def reset_collector(self):
        """clear current demo data"""
        self.current_demo_data = []
        self.step_counter = 0
        self.recording = False

    def get_demo_count(self):
        """Number of demos already saved to the hdf5 file (0 if disabled / unreadable)."""
        if not self.enabled or self.filepath is None:
            return 0
        try:
            with h5py.File(self.filepath, 'r') as f:
                return int(f['data'].attrs.get('total', 0))
        except Exception:
            return 0

    def collect_frame(self, robot_controller, delta_pos, delta_rot, gripper_cmd, spawner, camera_manager, container_manager=None, fixture_manager=None):
            if not self.enabled:
                return
            if robot_controller.franka is None:
                return

            self.step_counter += 1

            if not self.current_demo_data:
                self.capture_initial_state(robot_controller, spawner, container_manager, fixture_manager)

            lin_vel = robot_controller.franka.get_linear_velocity()
            ang_vel = robot_controller.franka.get_angular_velocity()
            
            if lin_vel is None or ang_vel is None:
                return 

            joint_pos = robot_controller.franka.get_joint_positions()
            joint_vel = robot_controller.franka.get_joint_velocities()
            robot_pos, robot_quat = robot_controller.franka.get_world_pose()
            robot_root_pose = np.concatenate([robot_pos, robot_quat])
            robot_root_vel = np.concatenate([lin_vel, ang_vel])

            ee_pos, ee_quat = robot_controller.ee_prim.get_world_pose()
            cur_gripper_width = robot_controller.current_gripper_width
            
            obj_pos = np.zeros(3)
            obj_quat = np.array([1.0, 0.0, 0.0, 0.0])
            obj_root_vel = np.zeros(6)
            display_name = "none"
            
            if spawner.target_object:
                obj_name_id = spawner.target_object
                display_name = self._get_real_name(robot_controller, obj_name_id)
                scene_obj = robot_controller.world.scene.get_object(obj_name_id)
                
                if scene_obj:
                    stage = robot_controller.world.stage
                    # Search from the loaded root prim (see capture_initial_state)
                    # so the rigid body is found whatever the object's hierarchy.
                    base_prim = stage.GetPrimAtPath(f"/World/{obj_name_id}")
                    target_rb_prim = None
                    for prim in Usd.PrimRange(base_prim):
                        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                            target_rb_prim = prim
                            break
                    
                    if target_rb_prim:
                        rb_path = str(target_rb_prim.GetPath())
                        temp_rb = RigidPrim(prim_path=rb_path, name="temp_collect")
                        o_pos, o_quat = temp_rb.get_world_pose()
                        o_lin_vel = temp_rb.get_linear_velocity()
                        o_ang_vel = temp_rb.get_angular_velocity()
                        
                        if o_lin_vel is not None:
                            obj_pos, obj_quat = o_pos, o_quat
                            obj_root_vel = np.concatenate([o_lin_vel, o_ang_vel])

            actions = np.zeros(7, dtype=np.float32)
            actions[:3] = delta_pos
            actions[3] = gripper_cmd
            actions[4:7] = delta_rot

            object_vec = np.concatenate([
                obj_pos, obj_quat, ee_pos, [cur_gripper_width], [0.0, 0.0]
            ]).astype(np.float32)

            frame_data = {
                "actions": actions,
                "joint_pos": joint_pos.astype(np.float32),
                "joint_vel": joint_vel.astype(np.float32),
                "eef_pos": ee_pos.astype(np.float32),
                "eef_quat": ee_quat.astype(np.float32),
                "states/articulation/robot/joint_position": joint_pos.astype(np.float32),
                "states/articulation/robot/joint_velocity": joint_vel.astype(np.float32),
                "states/articulation/robot/root_pose": robot_root_pose.astype(np.float32),
                "states/articulation/robot/root_velocity": robot_root_vel.astype(np.float32),
                f"states/target_object/{display_name}/root_pose": np.concatenate([obj_pos, obj_quat]).astype(np.float32),
                f"states/target_object/{display_name}/root_velocity": obj_root_vel.astype(np.float32)
            }

            # --- Camera capture every frame ---
            if camera_manager:
                from core.profiler import profiler
                _t = profiler.start()
                cam_data = camera_manager.get_all_camera_data()
                profiler.stop("  get_all_camera_data (all cams)", _t)
                frame_data.update(cam_data)

            # --- Knob state (only if active container has a knob spec) ---
            if container_manager is not None:
                knob_state = container_manager.get_knob_state()
                if knob_state is not None:
                    frame_data["states/knob/world_pose"] = np.concatenate(
                        [knob_state["world_pos"], knob_state["world_quat"]]
                    ).astype(np.float32)
                    frame_data["states/knob/angle_deg"] = np.array(
                        [np.rad2deg(knob_state["angle_rad"])], dtype=np.float32
                    )

            self.current_demo_data.append(frame_data)

    def _get_spawner_bg_map(self, spawner):
        return getattr(spawner, "bg_class_by_point", {}) or {}

    def _write_initial_scene(self, root, spawner):
        """Record the FIRST demo's scene layout under data/initial_scene.

        For each background point and the target, store the object class name
        (USD filename stem) and its initial root_pose (position + quaternion =
        position & angle). Written only once per file so evaluation can later
        reproduce the exact starting configuration. Subsequent demos skip this.
        """
        if self.initial_state_snapshot is None:
            return

        scene_group = root.create_group('initial_scene')
        snapshot_objs = self.initial_state_snapshot.get("objects", {})

        # role -> class name map, e.g. {"point_front": "apple", "target": "red_cube"}
        roles = {}

        def _spawn_pose(obj_name):
            """Pose of the reference root prim we placed at spawn (what eval
            replay re-applies). Falls back to the rigid body pose if unavailable."""
            info = snapshot_objs[obj_name]
            pose = info.get("spawn_root_pose")
            if pose is None:
                pose = info["root_pose"]
            return np.asarray(pose, dtype=np.float32)

        # Background objects, keyed by their spawn point (point_front/left/right).
        bg_map = self._get_spawner_bg_map(spawner)
        for point_name, class_name in bg_map.items():
            obj_name = f"spawned_obj_{point_name}"
            roles[point_name] = class_name
            if obj_name in snapshot_objs:
                scene_group.create_dataset(f"{point_name}/root_pose", data=_spawn_pose(obj_name))

        # Target object.
        target_class = spawner.get_target_class_name()
        if target_class is not None:
            roles["target"] = target_class
        target_obj_name = spawner.target_object
        if target_obj_name in snapshot_objs:
            scene_group.create_dataset("target/root_pose", data=_spawn_pose(target_obj_name))

        # Container (position & angle), so evaluation can restore it too.
        container_state = self.initial_state_snapshot.get("container")
        if container_state is not None:
            roles["container"] = container_state["name"]
            pose = np.asarray(container_state["root_pose"], dtype=np.float32)
            scene_group.create_dataset("container/root_pose", data=pose)

        # Support fixture (e.g. the box the target sits on). Recording its pose
        # lets eval replay put it back under the recorded target, even when the
        # fixture is randomized each reset.
        fixture_state = self.initial_state_snapshot.get("fixture")
        if fixture_state is not None:
            roles["fixture"] = fixture_state["name"]
            pose = np.asarray(fixture_state["root_pose"], dtype=np.float32)
            scene_group.create_dataset("fixture/root_pose", data=pose)

        scene_group.attrs['objects'] = json.dumps(roles)
        print(f"[DataCollector] Initial scene layout recorded: {roles}")

    def _write_demo_layout(self, demo_group, spawner):
        """Record THIS demo's full initial layout so occlusion can be computed
        OFFLINE later (see tools/compute_occlusion.py) from the exact starting
        arrangement — the hot collection loop no longer computes it live.

        For every background object, the target, the container and the fixture
        we store the class name (USD stem) plus its spawn pose. Objects use the
        reference-ROOT spawn pose (`spawn_root_pose`, same as the file-level
        `initial_scene`): the offline pass re-references each USD at that pose
        and re-settles with physics — the reproduction path add_reference_to_
        stage sets the root, not the inner rigid body, so its post-settle
        rigid-body pose would not round-trip. Container/fixture are static, so
        their get_state root_pose is used directly. Unlike `initial_scene`
        (first demo only, for eval replay) this is written for EVERY demo
        because each demo randomizes its layout.
        """
        if self.initial_state_snapshot is None:
            return

        layout = demo_group.create_group('initial_layout')
        snapshot_objs = self.initial_state_snapshot.get("objects", {})
        roles = {}  # role -> class name, e.g. {"target": "red_cube", "point_front": "apple"}

        def _pose(obj_name):
            # Reference-root spawn pose so re-referencing + re-settling round
            # trips; fall back to the rigid-body pose only if it's unavailable.
            info = snapshot_objs[obj_name]
            pose = info.get("spawn_root_pose")
            if pose is None:
                pose = info["root_pose"]
            return np.asarray(pose, dtype=np.float32)

        # Background objects, keyed by their spawn point.
        bg_map = self._get_spawner_bg_map(spawner)
        for point_name, class_name in bg_map.items():
            obj_name = f"spawned_obj_{point_name}"
            if obj_name in snapshot_objs:
                roles[point_name] = class_name
                layout.create_dataset(f"{point_name}/root_pose", data=_pose(obj_name))

        # Target object.
        target_class = spawner.get_target_class_name()
        target_obj_name = spawner.target_object
        if target_class is not None and target_obj_name in snapshot_objs:
            roles["target"] = target_class
            layout.create_dataset("target/root_pose", data=_pose(target_obj_name))

        # Container (post-settle pose; usually static).
        container_state = self.initial_state_snapshot.get("container")
        if container_state is not None:
            roles["container"] = container_state["name"]
            layout.create_dataset(
                "container/root_pose",
                data=np.asarray(container_state["root_pose"], dtype=np.float32),
            )

        # Support fixture (e.g. the box/stool the target rests on).
        fixture_state = self.initial_state_snapshot.get("fixture")
        if fixture_state is not None:
            roles["fixture"] = fixture_state["name"]
            layout.create_dataset(
                "fixture/root_pose",
                data=np.asarray(fixture_state["root_pose"], dtype=np.float32),
            )

        layout.attrs['objects'] = json.dumps(roles)

    def save_demo(self, controller, spawner, success_obj_name,
                  container_name="container", success=True,
                  occlusion_rates=None, fixture_name=None, task_profile=None):
        """
        save current demo data to hdf5 file

        Args:
            task_profile: task dict from task_config.TASKS (for instruction building)
            occlusion_rates: dict from OcclusionCalculator.get_occlusion_rates()
                             format: {"cam_0": 0.15, "cam_1": 0.32, ...}
                             pass None to skip recording occlusion rates
        """
        if not self.enabled:
            self.reset_collector()
            return
        if len(self.current_demo_data) < 20:
            self.reset_collector()
            return
        
        display_name = self._get_real_name(controller, success_obj_name)

        with h5py.File(self.filepath, 'a') as f:
            root = f['data']
            demo_id = int(root.attrs['total'])
            demo_name = f"demo_{demo_id}"
            demo_group = root.create_group(demo_name)
            
            # === Save demo Attribute ===
            demo_group.attrs['num_samples'] = len(self.current_demo_data)
            demo_group.attrs['success'] = success

            # Build instruction from task profile. Every demo in a dataset shares
            # the same instruction, so it lives on the top-level data group and
            # is written only once (from the first saved demo).
            if 'language_instruction' not in root.attrs:
                instr_text = ""
                if task_profile is not None:
                    try:
                        instr_text = task_config.build_instruction_from_task(
                            task_profile,
                            container_name=container_name,
                            target_obj_name=display_name,
                            fixture_name=fixture_name,
                            bg_class_by_point=self._get_spawner_bg_map(spawner),
                        )
                    except Exception as e:
                        instr_text = f"<instruction build failed: {e}>"
                root.attrs['language_instruction'] = instr_text

            # Record the initial scene layout once, from the first saved demo,
            # so evaluation can reproduce the exact starting configuration.
            if 'initial_scene' not in root:
                self._write_initial_scene(root, spawner)

            # Record THIS demo's full initial layout (all objects + container +
            # fixture) so occlusion can be computed offline afterwards.
            self._write_demo_layout(demo_group, spawner)

            # --- Occlusion Rate Attribute ---
            if occlusion_rates is not None:
                valid_rates = [r for r in occlusion_rates.values() if r >= 0]
                avg_rate = sum(valid_rates) / len(valid_rates) if valid_rates else 0.0

                rounded_rates = {k: round(v, 3) for k, v in occlusion_rates.items()}
                demo_group.attrs['occlusion_rates'] = json.dumps(rounded_rates)
                demo_group.attrs['occlusion_rate_avg'] = round(avg_rate, 3)
                print(f"[DataCollector] Occlusion recorded: avg={round(avg_rate, 3)}, "
                      f"per_cam={rounded_rates}")

            # --- Group initial_state ---
            init_group = demo_group.create_group('initial_state')

            robot_grp = init_group.create_group('articulation/robot')
            first_frame = self.current_demo_data[0]
            robot_grp.create_dataset("joint_position", data=first_frame['joint_pos'].astype(np.float32))
            robot_grp.create_dataset("joint_velocity", data=first_frame['joint_vel'].astype(np.float32))
            robot_grp.create_dataset("root_pose", data=self.initial_state_snapshot["robot"]["root_pose"].astype(np.float32))
            robot_grp.create_dataset("root_velocity", data=self.initial_state_snapshot["robot"]["root_vel"].astype(np.float32))
            
            if success_obj_name in self.initial_state_snapshot["objects"]:
                obj_info = self.initial_state_snapshot["objects"][success_obj_name]
                child_name = obj_info["child_name"]
                obj_grp = init_group.create_group(f'target_object/{child_name}')
                obj_grp.create_dataset("root_pose", data=obj_info["root_pose"].astype(np.float32))
                obj_grp.create_dataset("root_velocity", data=obj_info["root_velocity"].astype(np.float32))
            
            all_keys = set()
            for frame in self.current_demo_data:
                all_keys.update(frame.keys())

            obs_group = demo_group.create_group('obs')
            
            for key in all_keys:
                data_list = [frame[key] for frame in self.current_demo_data if key in frame]
                if not data_list:
                    continue
                
                data = np.array(data_list)
                
                if "/rgb" in key or "/depth" in key:
                    obs_group.create_dataset(
                        key, 
                        data=data, 
                        compression="gzip", 
                        compression_opts=4
                    )
                else:
                    obs_group.create_dataset(key, data=data.astype(np.float32))

            root.attrs['total'] = demo_id + 1
        
        print(f"[DataCollector] {demo_name} saved to {self.filepath} ({len(self.current_demo_data)} frames)")
        self.reset_collector()