# Target below this world-Z (meters) is considered fallen to the floor.
FALL_Z_THRESHOLD = 0.7


class TerminationManager:
    def __init__(self, world, container_manager, object_spawner, robot_controller,
                 task_profile=None):
        self.world = world
        self.container_manager = container_manager
        self.object_spawner = object_spawner
        self.robot_controller = robot_controller
        self.success_counter = 0
        # Success rule comes from the active task profile (configs/task_config.py).
        # Defaults to place_in_container when no profile / no override.
        self.task_profile = task_profile or {}
        self.success_criterion = self.task_profile.get(
            "success_criterion", "place_in_container"
        )

    def reset(self):
        """reset success counter"""
        self.success_counter = 0

    def check_task_success(self):
        """
        Dispatch to the success rule selected by the active task profile's
        success_criterion (configs/task_config.py).
        """
        if self.success_criterion == "turn_knob":
            return self._check_knob_task()
        return self._check_place_task()

    def check_task_failure(self):
        """
        Detect automatic failure conditions (used by eval mode so the operator
        does not have to press N manually). Returns (is_fail, reason).

        Add future fail conditions here, each returning a short reason string.
        """
        # Condition 1: target object fell to the floor.
        target = self.object_spawner.target_object
        if target:
            obj = self.world.scene.get_object(target)
            if obj:
                pos, _ = obj.get_world_pose()
                if pos[2] < FALL_Z_THRESHOLD:
                    return True, f"object fell (z={float(pos[2]):.2f} < {FALL_Z_THRESHOLD})"

        return False, None

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
