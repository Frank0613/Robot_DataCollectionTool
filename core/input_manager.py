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
        
    def get_command(self):
        """
        Return delta pos & Gripper state
        """
        delta = np.zeros(3)
        speed = robot_config.MOVE_SPEED
        
        # Robot movement
        if self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.W): delta[0] += speed
        if self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.S): delta[0] -= speed
        if self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.A): delta[1] += speed
        if self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.D): delta[1] -= speed
        if self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.Q): delta[2] += speed
        if self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.E): delta[2] -= speed
        
        # Gripper Control
        gripper_cmd = 0
        if self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.C): 
            gripper_cmd = -1 # Close
        if self._input.get_keyboard_value(self._keyboard, carb.input.KeyboardInput.V): 
            gripper_cmd = 1  # Open
            
        return delta, gripper_cmd