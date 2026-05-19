from configs import instruction_config


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
        Dispatch to the success rule selected by the active instruction template.
        See instruction_config.TEMPLATE_TASKS.
        """
        task = instruction_config.get_task_type()
        if task == "turn_knob":
            return self._check_knob_task()
        return self._check_place_task()

    def _check_place_task(self):
        """
        Check if any spawned object is inside the container while the gripper
        has released it.
        """
        is_gripper_open = self.robot_controller.current_gripper_width > 0.035

        is_obj_inside = False
        inside_obj_name = None

        for obj_name in ([self.object_spawner.target_object] if self.object_spawner.target_object else []):
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

        if self.success_counter >= 30:
            print("==========================================")
            print(f"[Task Success] Object '{inside_obj_name}' stable in container for 30 frames!")
            print("==========================================")
            return True, inside_obj_name

        return False, None

    def _check_knob_task(self):
        """
        Success when the container's knob has stayed past its rotation threshold
        for 60 frames.
        """
        if self.container_manager.is_knob_turned():
            self.success_counter += 1
        else:
            if self.success_counter > 0:
                self.success_counter = 0

        if self.success_counter >= 30:
            print("==========================================")
            print("[Task Success] Knob turned past threshold for 30 frames!")
            print("==========================================")
            # save_demo expects an object name; fall back to the spawned target
            # (or None if knob task was run without a target object).
            return True, self.object_spawner.target_object

        return False, None
