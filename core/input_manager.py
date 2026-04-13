import carb
import omni.appwindow
import numpy as np
from configs import robot_config

class InputManager:
    def __init__(self):
        # Input Interface
        self._input = carb.input.acquire_input_interface()
        self._app_window = omni.appwindow.get_default_app_window()
        self._keyboard = self._app_window.get_keyboard()
        self.gripper_is_open = True
        self.prev_k_state = False
        self.prev_r_state = False

    def get_command(self):
        """
        Return delta pos, delta rot, gripper state, reset cmd, is_any_action
        """
        delta = np.zeros(3)
        delta_rot = np.zeros(3)
        speed = robot_config.MOVE_SPEED
        rot_speed = robot_config.ROTATE_SPEED

        # Robot movement (WASDQE)
        move_keys = [
            carb.input.KeyboardInput.W, carb.input.KeyboardInput.S,
            carb.input.KeyboardInput.A, carb.input.KeyboardInput.D,
            carb.input.KeyboardInput.Q, carb.input.KeyboardInput.E
        ]

        any_move_pressed = False
        for key in move_keys:
            if self._input.get_keyboard_value(self._keyboard, key):
                any_move_pressed = True
                if key == carb.input.KeyboardInput.W: delta[0] += speed
                elif key == carb.input.KeyboardInput.S: delta[0] -= speed
                elif key == carb.input.KeyboardInput.A: delta[1] += speed
                elif key == carb.input.KeyboardInput.D: delta[1] -= speed
                elif key == carb.input.KeyboardInput.Q: delta[2] += speed
                elif key == carb.input.KeyboardInput.E: delta[2] -= speed

        # Robot rotation (Z/X: x-axis, T/G: y-axis, C/V: z-axis)
        rot_keys = [
            carb.input.KeyboardInput.Z, carb.input.KeyboardInput.X,
            carb.input.KeyboardInput.T, carb.input.KeyboardInput.G,
            carb.input.KeyboardInput.C, carb.input.KeyboardInput.V
        ]

        any_rot_pressed = False
        for key in rot_keys:
            if self._input.get_keyboard_value(self._keyboard, key):
                any_rot_pressed = True
                if key == carb.input.KeyboardInput.Z: delta_rot[0] += rot_speed
                elif key == carb.input.KeyboardInput.X: delta_rot[0] -= rot_speed
                elif key == carb.input.KeyboardInput.T: delta_rot[1] += rot_speed
                elif key == carb.input.KeyboardInput.G: delta_rot[1] -= rot_speed
                elif key == carb.input.KeyboardInput.C: delta_rot[2] += rot_speed
                elif key == carb.input.KeyboardInput.V: delta_rot[2] -= rot_speed

        # Gripper Control (K key)
        k_pressed = self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.K)
        if k_pressed and not self.prev_k_state:
            self.gripper_is_open = not self.gripper_is_open

        self.prev_k_state = k_pressed
        gripper_cmd = 0 if self.gripper_is_open else 1

        is_any_action = any_move_pressed or any_rot_pressed or k_pressed

        # Reset Command
        reset_cmd = False
        r_pressed = self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.R)
        if r_pressed and not self.prev_r_state:
            reset_cmd = True
        self.prev_r_state = r_pressed

        return delta, delta_rot, gripper_cmd, reset_cmd, is_any_action
