import argparse
import os
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage, is_stage_loading

import robot_config as robot_config
from core.input_manager import InputManager
from core.robot_controller import FrankaController
from core.auto_pilot import AutoPilot
import core.obj_loader as obj_loader

def main():
    # Arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", type=str, choices=["ik", "rmpflow"])
    parser.add_argument("--scene", type=str)
    parser.add_argument("--auto", action="store_true")
    args, unknown = parser.parse_known_args()

    if args.mode: robot_config.CONTROLLER_MODE = args.mode
    if args.scene:
        robot_config.SCENE_NAME = args.scene
        robot_config.USD_PATH = os.path.join(robot_config.BASE_DIR, "usdfiles", "scenes", f"{robot_config.SCENE_NAME}.usd")

    print(f"Loading Scene: {robot_config.USD_PATH}")
    open_stage(robot_config.USD_PATH)
    while is_stage_loading(): simulation_app.update()

    # Init Scene & Robot
    world = World(stage_units_in_meters=1.0)
    controller = FrankaController(world)

    # Load target obj from JSON
    task_list = obj_loader.load_targets()
    current_task_idx = 0
    target_prim, target_pos_goal = obj_loader.spawn_target(world, task_list[current_task_idx])
    
    world.reset()
    controller.initialize_handles()
    
    # Compute EE pose from target obj
    world.step(render=True)
    target_quat = obj_loader.compute_grasp_orientation(target_prim)

    # Mode control
    start_pos, _ = controller.ee_prim.get_world_pose()
    if args.auto:
        agent = AutoPilot(target_pos=target_pos_goal, start_pos=start_pos, target_rot=target_quat)
    else:
        agent = InputManager()

    # Warmup
    for _ in range(20):
        if world.is_playing(): world.step(render=False)

    print("==========================================")
    print("System Ready. Loop Started.")
    print("==========================================")

    needs_reset = False
    
    while simulation_app.is_running():
        if world.is_playing():
            
            # --- Reset & Next Task Logic ---
            if needs_reset:
                print(">>> Task Done. Preparing next object...")

                world.stop()
                # change to next obj
                current_task_idx = (current_task_idx + 1) % len(task_list)
                if current_task_idx == 0: print("All tasks finished! Looping start.")

                print(f"Spawning Task {current_task_idx+1}/{len(task_list)}")
                target_prim, target_pos_goal = obj_loader.spawn_target(world, task_list[current_task_idx])

                simulation_app.update()
                world.reset()
                controller.initialize_handles()
                
                for _ in range(5): world.step(render=False)
                target_quat = obj_loader.compute_grasp_orientation(target_prim)
                
                # Reset AutoPilot
                curr_start_pos, _ = controller.ee_prim.get_world_pose()
                if args.auto:
                    agent = AutoPilot(target_pos=target_pos_goal, start_pos=curr_start_pos, target_rot=target_quat)
                
                needs_reset = False
                continue

            # Control Loop
            curr_pos, _ = controller.ee_prim.get_world_pose()
            curr_width = controller.current_gripper_width

            # Get command
            if args.auto:
                delta_pos, target_rot_cmd, gripper_cmd = agent.get_command(curr_pos, curr_width)
            else:
                d, g = agent.get_command(curr_pos, curr_width)
                delta_pos, target_rot_cmd, gripper_cmd = d, None, g

            if agent.is_done():
                needs_reset = True
                continue
            
            # Apply control
            controller.apply_control(delta_pos, gripper_cmd, target_quat=target_rot_cmd)
            world.step(render=True)
            
        else:
            needs_reset = True
            simulation_app.update()

    simulation_app.close()

if __name__ == "__main__":
    main()