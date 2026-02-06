import os
import h5py
import numpy as np
import json
from omni.isaac.core.prims import RigidPrim
from pxr import Usd, UsdPhysics

class DataCollector:
    def __init__(self, save_dir="datasets", filename="dataset.hdf5"):
        self.save_dir = save_dir
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        self.filepath = os.path.join(self.save_dir, filename)
        self.recording = False
        self.current_demo_data = []
        self.initial_state_snapshot = None
        
        # Init file if not exists
        with h5py.File(self.filepath, 'a') as f:
            if 'data' not in f:
                data_group = f.create_group('data')
                # Global info (Attribute)
                env_args = {"env_name": "Franka_Tabletop_Scene"} 
                data_group.attrs['env_args'] = json.dumps(env_args)
                data_group.attrs['total'] = 0

    def _get_real_name(self, robot_controller, spawn_obj_name):
        """
        get the actual object name inside the USD (RigidBodyAPI name)
        """
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
        """ capture initial state of robot and spawned objects """
        snapshot = {
            "robot": {
                "root_pose": None,
                "root_vel": None
            },
            "objects": {}
        }
        
        # Init robot state
        pos, quat = robot_controller.franka.get_world_pose()
        snapshot["robot"]["root_pose"] = np.concatenate([pos, quat])
        snapshot["robot"]["root_vel"] = np.concatenate([
            robot_controller.franka.get_linear_velocity(),
            robot_controller.franka.get_angular_velocity()
        ])
        stage = robot_controller.world.stage

        # Init objects state
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
        self.recording = False

    def collect_frame(self, robot_controller, delta_pos, spawner):
        """record current frame data"""
        if not self.current_demo_data:
            self.capture_initial_state(robot_controller, spawner)

        joint_pos = robot_controller.franka.get_joint_positions()
        joint_vel = robot_controller.franka.get_joint_velocities()
        ee_pos, ee_quat = robot_controller.ee_prim.get_world_pose()
        
        frame_data = {
            "action": delta_pos,
            "joint_pos": joint_pos,
            "joint_vel": joint_vel,
            "eef_pos": ee_pos,
            "eef_quat": ee_quat,
            "gripper_pos": np.array([robot_controller.current_gripper_width] * 2)
        }
        self.current_demo_data.append(frame_data)

    def save_demo(self, controller, spawner, success_obj_name,container_name="container", success=True):
        """save current demo data to hdf5 file"""
        if len(self.current_demo_data) < 20:
            self.reset_collector()
            return
        
        display_name = self._get_real_name(controller, success_obj_name)

        with h5py.File(self.filepath, 'a') as f:
            root = f['data']
            # get new demo id
            demo_id = int(root.attrs['total'])
            demo_name = f"demo_{demo_id}"
            demo_group = root.create_group(demo_name)
            
            # Save demo Attribute
            demo_group.attrs['num_samples'] = len(self.current_demo_data)
            demo_group.attrs['success'] = success
            demo_group.attrs['language_instruction'] = f"pick up the {display_name} and place it in the {container_name}"

            # --- Group initial_state ---
            init_group = demo_group.create_group('initial_state')

            # Robot initial state
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
            
            # obs group
            obs_group = demo_group.create_group('obs')
            
            # list to Dataset
            for key in self.current_demo_data[0].keys():
                data = np.array([frame[key] for frame in self.current_demo_data])
                obs_group.create_dataset(key, data=data.astype(np.float32))

            # update total demos
            root.attrs['total'] = demo_id + 1
        
        print(f"[DataCollector] {demo_name} saved to {self.filepath} ({len(self.current_demo_data)} frames)")
        self.reset_collector()