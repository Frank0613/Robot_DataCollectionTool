import argparse
import os
import sys

from tools.hdf5_reader import print_structure_by_path
from tools.hdf5_checker import visualize_hdf5_cameras_by_path
from tools.hdf5_video import visualize_hdf5_demo_as_video
from core.occlusion_calculator import OcclusionCalculator


def main():
    parser = argparse.ArgumentParser(description="Robot Control Tool")
    parser.add_argument("--mode", type=str, choices=["ik", "rmpflow"])
    parser.add_argument("--scene", type=str)
    parser.add_argument("--readfile", type=str)
    parser.add_argument("--checkfile", type=str)
    parser.add_argument("--checkvideo", type=str,
                        help="Render a whole demo as an MP4 (use with --demo).")
    parser.add_argument("--demo", type=int, default=0,
                        help="Demo index for --checkfile and --checkvideo.")
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--repeat", type=int, default=None,
                        help="Repeat the task N times with the same object layout before re-randomizing. "
                             "Omit to randomize every reset.")
    parser.add_argument("--debug", action="store_true",
                        help="Visualize the IK/RMP target ball in the scene.")
    parser.add_argument("--inference", action="store_true",
                        help="Run without recording any data (for model inference or testing).")
    args, unknown = parser.parse_known_args()

    # file checking tools
    if args.readfile:
        target = os.path.join("datasets", f"{args.readfile}.hdf5")
        print_structure_by_path(target)
        return 

    if args.checkfile:
        target = os.path.join("datasets", f"{args.checkfile}.hdf5")
        visualize_hdf5_cameras_by_path(target, demo_idx=args.demo, frame_idx=args.index)
        return

    if args.checkvideo:
        target = os.path.join("datasets", f"{args.checkvideo}.hdf5")
        visualize_hdf5_demo_as_video(target, demo_idx=args.demo)
        return
    
    print("Starting Isaac Sim Environment...")
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": False})

    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import open_stage, is_stage_loading
    from configs import robot_config, usd_config
    from configs.instruction_config import ACTIVE_TEMPLATE
    from configs.container_config import ACTIVE_CONTAINER
    from core.input_manager import InputManager
    from core.robot_controller import FrankaController
    from core.object_spawner import ObjectSpawner
    from core.container_manager import ContainerManager
    from core.termination_manager import TerminationManager
    from core.data_collector import DataCollector
    from core.camera_manager import CameraManager
    import omni.kit.app
    
    param_mapping = {
        "mode": "CONTROLLER_MODE",
    }
    for cli_arg, config_var in param_mapping.items():
        val = getattr(args, cli_arg, None)
        if val is not None:
            setattr(robot_config, config_var, val)
    if args.debug:
        robot_config.DEBUG_VISUALIZE_TARGET = True
    if args.scene:
        usd_config.SCENE_NAME = args.scene
        usd_config.USD_PATH = os.path.join(
            usd_config.BASE_DIR,
            "usdfiles",
            "scenes",
            f"{usd_config.SCENE_NAME}.usd"
        )

    if ACTIVE_TEMPLATE.startswith("cabinet") and not usd_config.SCENE_NAME.endswith("cabinet"):
        print(
            f"\033[91m[ERROR] ACTIVE_TEMPLATE='{ACTIVE_TEMPLATE}' expects a cabinet scene, "
            f"but --scene='{usd_config.SCENE_NAME}' does not end with 'cabinet' "
            f"(e.g. 'home_cabinet'). Aborting.\033[0m"
        )
        simulation_app.close()
        sys.exit(1)

    if ACTIVE_TEMPLATE.startswith("stove") and ACTIVE_CONTAINER != "stove":
        print(
            f"\033[91m[ERROR] ACTIVE_TEMPLATE='{ACTIVE_TEMPLATE}' requires "
            f"ACTIVE_CONTAINER='stove' (in configs/container_config.py), "
            f"but got '{ACTIVE_CONTAINER}'. Aborting.\033[0m"
        )
        simulation_app.close()
        sys.exit(1)

    # Load scene
    print(f"Loading Scene: {usd_config.USD_PATH}")
    open_stage(usd_config.USD_PATH)
    while is_stage_loading():
        simulation_app.update()
    env_name = os.path.splitext(os.path.basename(usd_config.USD_PATH))[0]

    # Init world
    world = World(stage_units_in_meters=1.0)

    # Init controller & Input
    controller = FrankaController(world)
    input_mgr = InputManager()
    spawner = ObjectSpawner(world, repeat_count=args.repeat)
    container_mgr = ContainerManager(world)
    termination_mgr = TerminationManager(world, container_mgr, spawner, controller)
    data_collector = DataCollector(env_name=env_name, enabled=not args.inference)
    camera_mgr = CameraManager()

    # ============================================
    # Occlusion rate calculation (run on scene init & every reset)
    # ============================================
    current_occlusion_rates = None  # stores occlusion rate results for the current scene

    def setup_scene_with_occlusion():
        nonlocal current_occlusion_rates

        world.reset()
        controller.initialize_handles()
        camera_mgr.initialize_cameras()

        # === Phase 1: spawn target only ===
        spawner.spawn_target_only()
        container_mgr.respawn()

        for _ in range(8):
            world.step(render=True)

        target_class = spawner.get_target_class_name()
        if target_class is None:
            print("[Main] Warning: target_class_name is None, skipping occlusion calc")
            spawner.spawn_remaining_objects()
            for _ in range(8):
                world.step(render=True)
            current_occlusion_rates = None
            return

        occlusion_calc = OcclusionCalculator(camera_mgr, target_class)

        # Enable semantic segmentation via CameraManager
        camera_mgr.enable_semantic_segmentation()
        for _ in range(5):
            world.step(render=True)

        # Capture baseline (multi-sample with renders between for noise robustness)
        occlusion_calc.capture_baseline(
            camera_mgr,
            render_fn=lambda: world.step(render=True),
            num_samples=5,
        )

        # === Phase 2: spawn remaining objects ===
        spawner.spawn_remaining_objects()

        # Match Phase-1 settle time (8 + 5 = 13) for annotator symmetry.
        for _ in range(13):
            world.step(render=True)

        # Capture after occlusion (multi-sample with renders between)
        occlusion_calc.capture_occluded(
            camera_mgr,
            render_fn=lambda: world.step(render=True),
            num_samples=5,
        )

        current_occlusion_rates = occlusion_calc.get_occlusion_rates()
        avg_rate = occlusion_calc.get_avg_occlusion_rate()
        rounded_rates = {k: round(v, 3) for k, v in current_occlusion_rates.items()}
        print(f"[Main] Occlusion calculation done! avg = {avg_rate:.3f}, per_cam = {rounded_rates}")

    def log_demo_status():
        """Print the current demo count. Called after scene setup so it lands
        below all the Isaac Sim warnings (same spot as the occlusion log)."""
        if not data_collector.enabled:
            return
        count = data_collector.get_demo_count()
        print(f"[Main] >>> Demos saved so far: {count}  |  now recording demo #{count + 1} <<<")

    # === First initialization ===
    setup_scene_with_occlusion()

    for _ in range(20):
        if world.is_playing():
            world.step(render=False)

    print("==========================================")
    mode_str = "INFERENCE (no recording)" if args.inference else "DATA COLLECTION"
    print(f"Mode: {mode_str}")
    print("Move : WASDQE | Rotate : Z/X T/G C/V")
    print("Gripper : K   | Reset  : R   | Save Fail : N")
    print("==========================================")
    log_demo_status()

    needs_reset = False
    
    while simulation_app.is_running():
        
        if world.is_playing():
            # Stop simulation -> Reset
            if needs_reset:
                setup_scene_with_occlusion()
                termination_mgr.reset()
                data_collector.reset_collector()
                needs_reset = False
                log_demo_status()
            
            # Get input
            delta_pos, delta_rot, gripper_cmd, reset_cmd, fail_cmd, is_any_action = input_mgr.get_command()
            needs_reset = reset_cmd

            # Start recording if detect any keyboard action (only when collecting)
            if data_collector.enabled and not data_collector.recording and is_any_action:
                print(" Start Recording...")
                data_collector.recording = True

            # Apply control
            controller.apply_control(delta_pos, gripper_cmd, delta_rot)
            # Update physics
            world.step(render=True)

            if data_collector.recording:
                data_collector.collect_frame(controller, delta_pos, delta_rot, gripper_cmd, spawner, camera_mgr, container_mgr)
                # First frame just landed -> save a preview PNG so the user can
                # eyeball the camera views and press N/R immediately if broken.
                if len(data_collector.current_demo_data) == 1:
                    preview_path = os.path.abspath("preview_recording.png")
                    saved = camera_mgr.save_current_frame_preview(preview_path)
                    if saved:
                        print(f"[Main] First-frame preview saved -> {saved}")

            # Manual fail save (F key): save current demo as failure and reset.
            # Layout selection is NOT advanced (notify_task_success is skipped) so
            # the same scene re-rolls until the user gets a successful demo.
            if fail_cmd and data_collector.recording:
                container_info = container_mgr.get_container_info()
                c_name = container_info.get("name", "container")
                print("[Main] Saving demo as FAILURE (manual)")
                data_collector.save_demo(
                    controller,
                    spawner,
                    success_obj_name=spawner.target_object,
                    container_name=c_name,
                    success=False,
                    occlusion_rates=current_occlusion_rates,
                )
                needs_reset = True

            # Check termination condition
            is_success, success_obj_name = termination_mgr.check_task_success()
            if not needs_reset and is_success:
                container_info = container_mgr.get_container_info()
                c_name = container_info.get("name", "container")
                data_collector.save_demo(
                    controller,
                    spawner,
                    success_obj_name,
                    container_name=c_name,
                    success=True,
                    occlusion_rates=current_occlusion_rates  # pass occlusion rates
                )
                spawner.notify_task_success()
                needs_reset = True
        else:
            needs_reset = True
            simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    main()
