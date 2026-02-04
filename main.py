import argparse
import os 
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage, is_stage_loading
from omni.isaac.core.prims import XFormPrim

import robot_config as robot_config
import usd_config as usd_config
from core.input_manager import InputManager
from core.robot_controller import FrankaController
from core.object_spawner import ObjectSpawner
from core.container_manager import ContainerManager
from core.termination_manager import TerminationManager
from core.data_collector import DataCollector

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
        usd_config.SCENE_NAME = args.scene
        usd_config.USD_PATH = os.path.join(
            usd_config.BASE_DIR,
            "usdfiles",
            "scenes",
            f"{usd_config.SCENE_NAME}.usd"
        )


    # Load scene
    print(f"Loading Scene: {usd_config.USD_PATH}")
    open_stage(usd_config.USD_PATH)
    while is_stage_loading():
        simulation_app.update()

    # Init world
    world = World(stage_units_in_meters=1.0)

    # Init controller & Input
    controller = FrankaController(world)
    input_mgr = InputManager()
    spawner = ObjectSpawner(world)
    container_mgr = ContainerManager(world)
    termination_mgr = TerminationManager(world, container_mgr, spawner,controller)
    data_collector = DataCollector()

    world.reset()
    controller.initialize_handles()
    spawner.respawn()
    container_mgr.respawn()
    for _ in range(20):
        if world.is_playing(): world.step(render=False)

    print("==========================================")
    print(f"Successful")
    print("Move : WASDQE | Gripper : C")
    print("==========================================")

    needs_reset = False
    
    while simulation_app.is_running():
        
        if world.is_playing():
            # Stop simulation -> Reset
            if needs_reset:
                world.reset()
                controller.initialize_handles()
                spawner.respawn()
                container_mgr.respawn()
                termination_mgr.reset()
                data_collector.reset_collector()
                needs_reset = False
            
            # Get input
            delta_pos, gripper_cmd, reset_cmd,is_any_action = input_mgr.get_command()
            needs_reset = reset_cmd

            # Start recording if detect any keyboard action
            if not data_collector.recording and is_any_action:
                print(" Start Recording...")
                data_collector.recording = True

            # Apply control
            controller.apply_control(delta_pos, gripper_cmd)
            # Update physics
            world.step(render=True)

            if data_collector.recording:
                data_collector.collect_frame(controller, delta_pos)

            # Check termination condition
            if not needs_reset and termination_mgr.check_task_success():
                data_collector.save_demo(success=True)
                needs_reset = True
        else:
            needs_reset = True
            simulation_app.update()

    simulation_app.close()

if __name__ == "__main__":
    main()