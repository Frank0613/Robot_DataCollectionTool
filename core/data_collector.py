import os
import h5py
import numpy as np
import json
from datetime import datetime

class DataCollector:
    def __init__(self, save_dir="datasets", filename="dataset.hdf5"):
        self.save_dir = save_dir
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        self.filepath = os.path.join(self.save_dir, filename)
        self.recording = False
        self.current_demo_data = []
        
        # Init file if not exists
        with h5py.File(self.filepath, 'a') as f:
            if 'data' not in f:
                data_group = f.create_group('data')
                # Global info (Attribute)
                env_args = {"env_name": "Franka_Tabletop_Scene"} 
                data_group.attrs['env_args'] = json.dumps(env_args)
                data_group.attrs['total'] = 0

    def reset_collector(self):
        """clear current demo data"""
        self.current_demo_data = []
        self.recording = False

    def collect_frame(self, robot_controller, delta_pos):
        """record current frame data"""
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

    def save_demo(self, success=True):
        """save current demo data to hdf5 file"""
        if len(self.current_demo_data) < 20:
            self.reset_collector()
            return

        with h5py.File(self.filepath, 'a') as f:
            root = f['data']
            # get new demo id
            demo_id = int(root.attrs['total'])
            demo_name = f"demo_{demo_id}"
            demo_group = root.create_group(demo_name)
            
            # Save demo Attribute
            demo_group.attrs['num_samples'] = len(self.current_demo_data)
            demo_group.attrs['success'] = success
            demo_group.attrs['language_instruction'] = "pick up the object and place it in the container"

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