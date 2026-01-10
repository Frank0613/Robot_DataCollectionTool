import numpy as np
import os
import omni
from pxr import Usd
from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.core.prims import RigidPrim
from omni.isaac.core.objects import VisualSphere, FixedCuboid
from omni.isaac.motion_generation import (
    RmpFlow, ArticulationMotionPolicy, 
    LulaKinematicsSolver, interface_config_loader
)
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.prims import is_prim_path_valid
from scipy.spatial.transform import Rotation as R
import robot_config as robot_config

# Looking for robot, Init gripper
class _BaseFrankaController:
    def __init__(self, world: World):
        self.world = world
        
        # Looking for robot
        search_root = getattr(robot_config, "ROBOT_PRIM_PATH", "/World")
        target_name = getattr(robot_config, "TARGET_ROBOT_NAME", "Franka")
        self.prim_path = self.Find_Robot(search_root, target_name)
        if self.prim_path is None:
            raise ValueError("Can't find robot")
        
        # Init Franka
        self.franka = Franka(prim_path=self.prim_path, name="franka")
        self.world.scene.add(self.franka)
        
        # Init End Effector
        ee_path = f"{self.prim_path}/panda_hand"
        if not is_prim_path_valid(ee_path):
            print("Init EE failed")
        self.ee_prim = RigidPrim(prim_path=ee_path, name="end_effector")

        # Common State
        self.target_pos = None
        self.target_rot = None
        self.gripper_state = 1.0 
        self.current_gripper_width = 0.04

    def Find_Robot(self, root_path, target_name):
        stage = omni.usd.get_context().get_stage()
        root_prim = stage.GetPrimAtPath(root_path)
        
        if not root_prim.IsValid():
            root_prim = stage.GetPrimAtPath("/World")

        if root_prim.GetName() == target_name:
            return str(root_prim.GetPath())
        
        for prim in Usd.PrimRange(root_prim):
            if prim.GetName() == target_name:
                return str(prim.GetPath())
        return None

    def initialize_handles(self):
        """ Common reset logic """
        print(f"Resetting Handles ({robot_config.CONTROLLER_MODE})...")
        self.ee_prim.initialize()
        
        self.franka.get_articulation_controller().set_gains(
            kps=np.array([robot_config.KPS_ARM] * 7 + [robot_config.KPS_GRIPPER] * 2), 
            kds=np.array([robot_config.KDS_ARM] * 7 + [robot_config.KDS_GRIPPER] * 2)
        )
        
    def _update_gripper(self, gripper_cmd):
        if gripper_cmd != 0:
            self.gripper_state = float(gripper_cmd)
            
        goal_width = 0.04 if self.gripper_state > 0 else 0.0
        step = robot_config.GRIPPER_SPEED
        
        if self.current_gripper_width < goal_width:
            self.current_gripper_width = min(self.current_gripper_width + step, goal_width)
        elif self.current_gripper_width > goal_width:
            self.current_gripper_width = max(self.current_gripper_width - step, goal_width)
        return self.current_gripper_width


# IK Controller Implementation
class _FrankaControllerIK(_BaseFrankaController):
    def __init__(self, world: World):
        super().__init__(world)
        self._setup_ik()
        self.robot_base_pos = None
        self.robot_base_rot = None

    def _setup_ik(self):
        temp_cfg = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        default_yaml = temp_cfg.get("robot_description_yaml_path") or \
                       temp_cfg.get("robot_description_path") or \
                       temp_cfg.get("rmpflow_config_path")
        if default_yaml:
            base = os.path.dirname(os.path.dirname(os.path.dirname(default_yaml)))
            desc = os.path.join(base, "franka/rmpflow/robot_descriptor.yaml")
            urdf = os.path.join(base, "franka/lula_franka_gen.urdf")
            self.kinematics_solver = LulaKinematicsSolver(robot_description_path=desc, urdf_path=urdf)

    def initialize_handles(self):
        super().initialize_handles()
        self.update_base_pose()
        curr_pos, curr_rot = self.ee_prim.get_world_pose()
        self.target_pos, self.target_rot = self.world_to_local(curr_pos, curr_rot)

    def update_base_pose(self):
        self.robot_base_pos, self.robot_base_rot = self.franka.get_world_pose()

    def world_to_local(self, world_pos, world_quat):
        if self.robot_base_pos is None: self.update_base_pose()
        rel_pos = world_pos - self.robot_base_pos
        r_base = R.from_quat([self.robot_base_rot[1], self.robot_base_rot[2], self.robot_base_rot[3], self.robot_base_rot[0]])
        r_base_inv = r_base.inv()
        local_pos = r_base_inv.apply(rel_pos)
        
        r_target = R.from_quat([world_quat[1], world_quat[2], world_quat[3], world_quat[0]])
        local_rot_r = r_base_inv * r_target
        local_rot_quat = local_rot_r.as_quat() 
        return local_pos, np.array([local_rot_quat[3], local_rot_quat[0], local_rot_quat[1], local_rot_quat[2]])

    def apply_control(self, delta_pos, gripper_cmd,target_quat=None):
        current_joints = self.franka.get_joint_positions()
        if current_joints is None: return

        # Logic specific to IK: Update internal target state
        if np.linalg.norm(delta_pos) > 0:
            self.target_pos += delta_pos

        if target_quat is not None:
            self.target_rot = target_quat
        
        # Calculate IK
        self.update_base_pose()
        ik_results, success = self.kinematics_solver.compute_inverse_kinematics(
            frame_name=robot_config.EE_FRAME_NAME,
            target_position=self.target_pos,
            target_orientation=self.target_rot,
            warm_start=current_joints[:7]
        )
        
        if success:
            action = np.zeros(9)
            action[:7] = ik_results
            gripper_width = self._update_gripper(gripper_cmd)
            action[7] = gripper_width
            action[8] = gripper_width
            self.franka.apply_action(ArticulationAction(joint_positions=action))
        else:
            print(f"IK failed | Target: {self.target_pos}")


# RMPFlow Controller Implementation
class _FrankaControllerRMP(_BaseFrankaController):
    def __init__(self, world: World):
        super().__init__(world)
        
        # Obstacle (Only RMP needs this)
        # self.obstacle = FixedCuboid(
        #     prim_path="/World/Obstacle_Pillar",
        #     name="obstacle_pillar",
        #     position=np.array([0.5, -1.5, 1.4]), 
        #     scale=np.array([0.05, 0.5, 0.8]),    
        #     color=np.array([0.8, 0.8, 0.8])     
        # )
        # self.world.scene.add(self.obstacle)

        # Visualizer
        self.target_visualizer = VisualSphere(
            prim_path="/World/target_visualizer",
            name="target_visualizer",
            position=np.array([0, 0, 0]), 
            scale=np.array([0.03, 0.03, 0.03]),
            color=np.array([1.0, 0.0, 0.0]),
            visible=False 
        )
        self.world.scene.add(self.target_visualizer)

        self._setup_rmpflow()
        #self._rmpflow.add_obstacle(self.obstacle)
        
        self._first_frame = True
        self.physics_dt = 1.0 / 60.0

    def _setup_rmpflow(self):
        rmp_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        self._rmpflow = RmpFlow(**rmp_config)
        self._articulation_rmpflow = ArticulationMotionPolicy(self.franka, self._rmpflow)

    def initialize_handles(self):
        super().initialize_handles()
        
        # RMP specific reset
        safe_joint_pos = np.array([0.0, -0.78, 0.0, -2.35, 0.0, 1.57, 0.0, 0.04, 0.04])
        self.franka.set_joint_positions(safe_joint_pos)
        
        base_pos, base_rot = self.franka.get_world_pose()
        self._rmpflow.set_robot_base_pose(base_pos, base_rot)
        self._rmpflow.reset()
        #self._rmpflow.add_obstacle(self.obstacle)
        
        self._first_frame = True
        self.target_pos = None
        self.target_rot = np.array([0.0, 1.0, 0.0, 0.0])
        
        if hasattr(self, "target_visualizer"):
            self.target_visualizer.set_visibility(False)

    def apply_control(self, delta_pos, gripper_cmd,target_quat=None):
        base_pos, base_rot = self.franka.get_world_pose()
        self._rmpflow.set_robot_base_pose(base_pos, base_rot)

        # Handle First Frame Latch
        if self._first_frame:
            current_pos, _ = self.ee_prim.get_world_pose()
            if np.linalg.norm(current_pos) < 0.1: return
            self.target_pos = current_pos
            self._rmpflow.set_end_effector_target(self.target_pos, self.target_rot)
            self.target_visualizer.set_world_pose(position=self.target_pos)
            self.target_visualizer.set_visibility(True)
            self._first_frame = False
            return 

        # Update Target
        if self.target_pos is not None:
            if np.linalg.norm(delta_pos) > 0:
                self.target_pos += delta_pos
                self.target_visualizer.set_world_pose(position=self.target_pos)

            self._rmpflow.set_end_effector_target(
                target_position=self.target_pos,
                target_orientation=target_quat 
            )
            self._rmpflow.update_world()
            
            rmp_action = self._articulation_rmpflow.get_next_articulation_action(self.physics_dt)
            
            # Combine Arm + Gripper
            gripper_width = self._update_gripper(gripper_cmd)
            
            if rmp_action.joint_positions is not None:
                full_joint_pos = np.concatenate([
                    rmp_action.joint_positions, 
                    [gripper_width, gripper_width]
                ])
                
                if rmp_action.joint_velocities is not None:
                    full_joint_vel = np.concatenate([
                        rmp_action.joint_velocities, 
                        [0.0, 0.0]
                    ])
                    action = ArticulationAction(joint_positions=full_joint_pos, joint_velocities=full_joint_vel)
                else:
                    action = ArticulationAction(joint_positions=full_joint_pos)

                self.franka.apply_action(action)

# Main Factory Class
class FrankaController:
    """
    Factory class that returns either an IK controller or an RMPflow controller
    based on robot_config.CONTROLLER_MODE.
    """
    def __new__(cls, world: World):
        mode = getattr(robot_config, "CONTROLLER_MODE", "rmpflow")
        
        if mode == "ik":
            print(">>> Initializing IK Controller")
            return _FrankaControllerIK(world)
        else:
            print(">>> Initializing RMPFlow Controller")
            return _FrankaControllerRMP(world)