import argparse
import os
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import open_stage, is_stage_loading

import robot_config as robot_config
from core.input_manager import InputManager
from core.robot_controller import FrankaController

def main():

    # Argument Parser
    parser = argparse.ArgumentParser(description="Robot Control Tool")
    parser.add_argument("--mode", type=str, choices=["ik", "rmpflow"], help="Override Controller Mode")
    parser.add_argument("--scene", type=str, help="Scene selection")
    args, unknown = parser.parse_known_args()

    param_mapping = {
        "mode": "CONTROLLER_MODE",
    }
    for cli_arg, config_var in param_mapping.items():
        val = getattr(args, cli_arg, None)
        if val is not None:
            setattr(robot_config, config_var, val)
    if args.scene:
        robot_config.SCENE_NAME = args.scene
        robot_config.USD_PATH = os.path.join(
            robot_config.BASE_DIR,
            "usdfiles",
            "scenes",
            f"{robot_config.SCENE_NAME}.usd"
        )


    # Load scene
    print(f"Loading Scene: {robot_config.USD_PATH}")
    open_stage(robot_config.USD_PATH)
    while is_stage_loading():
        simulation_app.update()

    # Init world
    world = World(stage_units_in_meters=1.0)
    
    # Add target cube
    print("Add target cube...")
    world.scene.add(
        DynamicCuboid(
            prim_path="/World/cube",
            name="cube",
            position=robot_config.CUBE_POSITION,
            scale=robot_config.CUBE_SCALE,
            color=robot_config.CUBE_COLOR,
            mass=0.1
        )
    )

    # Init controller & Input
    controller = FrankaController(world)
    input_mgr = InputManager()
    world.reset()
    controller.initialize_handles()
    for _ in range(20):
        if world.is_playing(): world.step(render=False)

    print("==========================================")
    print(f"Successful")
    print("Move : WASDQE | Gripper : C/V")
    print("==========================================")

    needs_reset = False
    
    while simulation_app.is_running():
        
        if world.is_playing():
            # Stop simulation -> Reset
            if needs_reset:
                world.reset()
                controller.initialize_handles()
                needs_reset = False
            
            # Get input
            delta_pos, gripper_cmd = input_mgr.get_command()
            
            # Apply control
            controller.apply_control(delta_pos, gripper_cmd)
            
            # Update physics
            world.step(render=True)
            
        else:
            needs_reset = True
            simulation_app.update()

    simulation_app.close()

if __name__ == "__main__":
    main()