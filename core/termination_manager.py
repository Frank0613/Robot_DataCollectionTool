import numpy as np

class TerminationManager:
    def __init__(self, world, container_manager, object_spawner, robot_controller):
        self.world = world
        self.container_manager = container_manager
        self.object_spawner = object_spawner
        self.robot_controller = robot_controller
        self.success_counter = 0
    def reset(self):
        """reset success counter"""
        self.success_counter = 0
    def check_task_success(self):
        """
            Check if any spawned object is inside the container.
        """
        # Check gripper state
        is_gripper_open = self.robot_controller.current_gripper_width > 0.035

        is_obj_inside = False
        inside_obj_name = None

        # Check each spawned object
        for obj_name in self.object_spawner.spawned_objects:
            obj = self.world.scene.get_object(obj_name)
            if obj:
                pos, _ = obj.get_world_pose()
                if self.container_manager.is_inside(pos):
                    is_obj_inside = True
                    inside_obj_name = obj_name
                    break 
        
        if is_gripper_open and is_obj_inside:
            self.success_counter += 1
        else:
            if self.success_counter > 0:
                self.success_counter = 0
        
        # Success if stable for 60 frames
        if self.success_counter >= 60:
            print(f"==========================================")
            print(f"[Task Success] Object '{inside_obj_name}' stable in container for 60 frames!")
            print(f"==========================================")
            return True, inside_obj_name
            
        return False, None