import numpy as np
import robot_config as robot_config
import auto_pilot_config as ap_config

class AutoPilot:
    def __init__(self, target_pos, start_pos, target_rot=None):
        self.start_pos = np.array(start_pos)
        self.virtual_target = np.array(start_pos)
        self.target_rot = target_rot 
        
        # Base Target Point
        base_target_pos = np.array(target_pos)
        
        # Load auto_pilot_config.py
        self.step = []
        if hasattr(ap_config, 'GRASP_SEQUENCE'):
            for step_cfg in ap_config.GRASP_SEQUENCE:
                # Base pos + Offset
                offset = np.array(step_cfg["offset"])
                abs_pos = base_target_pos + offset
                
                step = {
                    "name": step_cfg["name"],
                    "pos": abs_pos,
                    "tol": step_cfg["tol"],
                    "gripper": step_cfg["gripper"],
                    "wait_frames": step_cfg.get("wait_frames", 0)
                }
                self.step.append(step)
        else:
            print("[Error] GRASP_SEQUENCE not found in config!")
        
        self.current_step_idx = 0
        self.step_timer = 0
        self._done = False

    def get_command(self, current_ee_pos, current_gripper_width):
        if self._done:
            last_stage = self.step[-1]
            return np.zeros(3), self.target_rot, last_stage["gripper"]

        # Get Stage setting
        curr_stage = self.step[self.current_step_idx]
        
        target_goal = curr_stage["pos"]
        tolerance = curr_stage["tol"]
        gripper_cmd = curr_stage["gripper"]
        required_wait = curr_stage.get("wait_frames", 0)

        # Virtual Target movement
        error_vec = target_goal - self.virtual_target
        dist_virtual = np.linalg.norm(error_vec)
        
        delta = np.zeros(3)
        if dist_virtual > 0.001: 
            # Use robot_config.MOVE_SPEED
            step = min(dist_virtual, robot_config.MOVE_SPEED)
            direction = error_vec / dist_virtual
            delta = direction * step
            self.virtual_target += delta
        
        # Change step
        dist_real = np.linalg.norm(target_goal - current_ee_pos)
        
        # print(f"Stage {self.current_stage_idx} ({curr_stage['name']}) | Dist: {dist_real:.4f} | Timer: {self.stage_timer}/{required_wait}")

        if dist_real < tolerance:
            self.step_timer += 1
            
            if self.step_timer >= required_wait:
                print(f">>> [{self.current_step_idx}] {curr_stage['name']} Completed! (Waited {required_wait} frames)")
                
                # Go to next step
                self.current_step_idx += 1
                self.step_timer = 0
                
                # All step finished
                if self.current_step_idx >= len(self.step):
                    print(">>> All Stages Completed!")
                    self._done = True
                    return np.zeros(3), self.target_rot, gripper_cmd
        else:
            pass
        
        return delta, self.target_rot, gripper_cmd

    def is_done(self):
        return self._done