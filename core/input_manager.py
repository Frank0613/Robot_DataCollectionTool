import carb
import omni.appwindow
import numpy as np
import robot_config

class InputManager:
    def __init__(self):
        # Input Interface
        self._input = carb.input.acquire_input_interface()
        self._app_window = omni.appwindow.get_default_app_window()
        self._keyboard = self._app_window.get_keyboard()
        self.gripper_is_open = True
        self.prev_c_state = False
        self.prev_r_state = False
        
    def get_command(self):
        """
        Return delta pos & Gripper state
        """
        delta = np.zeros(3)
        speed = robot_config.MOVE_SPEED
        move_keys = [
            carb.input.KeyboardInput.W, carb.input.KeyboardInput.S,
            carb.input.KeyboardInput.A, carb.input.KeyboardInput.D,
            carb.input.KeyboardInput.Q, carb.input.KeyboardInput.E
        ]

        # Robot movement
        any_move_pressed = False
        for i, key in enumerate(move_keys):
            if self._input.get_keyboard_value(self._keyboard, key):
                any_move_pressed = True
                if key == carb.input.KeyboardInput.W: delta[0] += speed
                elif key == carb.input.KeyboardInput.S: delta[0] -= speed
                elif key == carb.input.KeyboardInput.A: delta[1] += speed
                elif key == carb.input.KeyboardInput.D: delta[1] -= speed
                elif key == carb.input.KeyboardInput.Q: delta[2] += speed
                elif key == carb.input.KeyboardInput.E: delta[2] -= speed
        
        # Gripper Control
        c_pressed = self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.C)
        if c_pressed and not self.prev_c_state:
            self.gripper_is_open = not self.gripper_is_open
            
        self.prev_c_state = c_pressed
        gripper_cmd = 0 if self.gripper_is_open else 1

        is_any_action = any_move_pressed or c_pressed

        # Reset Command
        reset_cmd = False
        r_pressed = self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.R)
        if r_pressed and not self.prev_r_state:
            reset_cmd = True
        self.prev_r_state = r_pressed

        return delta, gripper_cmd, reset_cmd,is_any_action