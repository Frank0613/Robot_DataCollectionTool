import os
import h5py
import numpy as np
import json
from omni.isaac.core.prims import RigidPrim
from pxr import Usd, UsdPhysics
from configs import instruction_config

class DataCollector:
    def __init__(self, save_dir="datasets", filename="dataset.hdf5", env_name="Default_Scene"):
        self.save_dir = save_dir
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        self.filepath = os.path.join(self.save_dir, filename)
        self.recording = False
        self.current_demo_data = []
        self.initial_state_snapshot = None
        self.camera_manager = None
        self.step_counter = 0
        self.record_interval = 1
        
        # Init file if not exists
        with h5py.File(self.filepath, 'a') as f:
            if 'data' not in f:
                data_group = f.create_group('data')
                env_args = {"env_name": env_name} 
                data_group.attrs['env_args'] = json.dumps(env_args)
                data_group.attrs['total'] = 0

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
    
    def capture_initial_state(self, robot_controller, spawner):
        """capture initial state of robot and spawned objects"""
        snapshot = {
            "robot": {
                "root_pose": None,
                "root_vel": None
            },
            "objects": {}
        }
        
        pos, quat = robot_controller.franka.get_world_pose()
        snapshot["robot"]["root_pose"] = np.concatenate([pos, quat])
        snapshot["robot"]["root_vel"] = np.concatenate([
            robot_controller.franka.get_linear_velocity(),
            robot_controller.franka.get_angular_velocity()
        ])
        stage = robot_controller.world.stage

        for obj_name in spawner.spawned_objects:
            real_name = self._get_real_name(robot_controller, obj_name)
    
            scene_obj = robot_controller.world.scene.get_object(obj_name)
            stage = robot_controller.world.stage
            base_prim = stage.GetPrimAtPath(scene_obj.prim_path)
            
            target_rb_prim = None
            for prim in Usd.PrimRange(base_prim):
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    target_rb_prim = prim
                    break
            
            if target_rb_prim:
                rb_path = str(target_rb_prim.GetPath())
                temp_rb = RigidPrim(prim_path=rb_path, name="temp_fetch")
                
                snapshot["objects"][obj_name] = {
                    "child_name": real_name,
                    "root_pose": np.concatenate(temp_rb.get_world_pose()),
                    "root_velocity": np.concatenate([temp_rb.get_linear_velocity(), temp_rb.get_angular_velocity()])
                }
        
        self.initial_state_snapshot = snapshot

    def reset_collector(self):
        """clear current demo data"""
        self.current_demo_data = []
        self.step_counter = 0
        self.recording = False

    def collect_frame(self, robot_controller, delta_pos, delta_rot, gripper_cmd, spawner, camera_manager):
            if robot_controller.franka is None:
                return

            self.step_counter += 1
            
            if not self.current_demo_data:
                self.capture_initial_state(robot_controller, spawner)

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
                    base_prim = stage.GetPrimAtPath(scene_obj.prim_path)
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

            # --- Camera Sample rate (30 step) ---
            if camera_manager and (self.step_counter % 30 == 0):
                cam_data = camera_manager.get_all_camera_data()
                frame_data.update(cam_data)

            self.current_demo_data.append(frame_data)

    def _get_spawner_bg_map(self, spawner):
        return getattr(spawner, "bg_class_by_point", {}) or {}

    def save_demo(self, controller, spawner, success_obj_name,
                  container_name="container", success=True,
                  occlusion_rates=None):
        """
        save current demo data to hdf5 file

        Args:
            occlusion_rates: dict from OcclusionCalculator.get_occlusion_rates()
                             format: {"cam_0": 0.15, "cam_1": 0.32, ...}
                             pass None to skip recording occlusion rates
        """
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
            instr_text, tmpl_key = instruction_config.build_instruction(
                obj=display_name,
                container=container_name,
                bg_class_by_point=self._get_spawner_bg_map(spawner),
            )
            demo_group.attrs['language_instruction'] = instr_text
            demo_group.attrs['instruction_template'] = tmpl_key

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