import argparse
import os
import sys

from tools.hdf5_reader import print_structure_by_path
from tools.hdf5_checker import visualize_hdf5_cameras_by_path
from tools.hdf5_video import visualize_hdf5_demo_as_video
from core.occlusion_calculator import OcclusionCalculator
from core.inference_stats import InferenceStats


def _set_render_visibility(stage, path_or_name, visible):
    """Toggle render visibility of a prim subtree (visibility only — physics
    colliders are unaffected). Accepts an exact prim path or a bare prim name
    (searched across the stage). Returns True if a matching prim was found."""
    from pxr import UsdGeom
    prim = stage.GetPrimAtPath(path_or_name)
    if not prim.IsValid():
        prim = None
        for p in stage.Traverse():
            if p.GetName() == path_or_name:
                prim = p
                break
    if prim is None or not prim.IsValid():
        return False
    imageable = UsdGeom.Imageable(prim)
    if not imageable:
        return False
    if visible:
        imageable.MakeVisible()
    else:
        imageable.MakeInvisible()
    return True


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
    parser.add_argument("--eval", dest="eval_mode", action="store_true",
                        help="Run without recording any data (for model inference or manual eval).")
    parser.add_argument("--max-steps", dest="max_steps", type=int, default=None,
                        help="Per-trial step timeout (eval mode only). Required to enable stats.")
    parser.add_argument("--trials", type=int, default=None,
                        help="Total trials before auto-exit (eval mode only). Omit for unlimited.")
    parser.add_argument("--save_video", action="store_true",
                        help="Eval mode: save each trial as an MP4 under eval_videos/.")
    parser.add_argument("--template", type=str, default=None,
                        help="[DEPRECATED] Use --task instead. Kept as an alias for task selection.")
    parser.add_argument("--container", type=str, default=None,
                        help="Override container_config.ACTIVE_CONTAINER for this run.")
    parser.add_argument("--task", type=str, default=None,
                        help="Select a task profile (see configs/task_config.py TASKS). "
                             "Sets scene/template/container in one shot; "
                             "individual --scene/--template/--container still override.")
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
    from configs import robot_config, usd_config, container_config, task_config
    from core.input_manager import InputManager
    from core.robot_controller import FrankaController
    from core.object_spawner import ObjectSpawner
    from core.container_manager import ContainerManager
    from core.fixture_manager import FixtureManager
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

    # Task resolution order: --task wins, then --template (deprecated alias),
    # then task_config.ACTIVE_TASK default. Scene/container come from CLI flags.
    active_task = None
    if args.task is not None:
        task_name = args.task
    elif args.template is not None:
        print(f"[Main] Note: --template is deprecated. Use --task {args.template} instead.")
        task_name = args.template
    else:
        task_name = task_config.ACTIVE_TASK
    if task_name is not None:
        active_task = task_config.apply_task(task_name)
        print(f"[Main] Applied task profile: '{task_name}'")

    if args.scene:
        usd_config.SCENE_NAME = args.scene
        usd_config.USD_PATH = os.path.join(
            usd_config.BASE_DIR,
            "usdfiles",
            "scenes",
            f"{usd_config.SCENE_NAME}.usd"
        )

    if args.container is not None:
        if args.container not in container_config.CONTAINERS:
            print(
                f"\033[91m[ERROR] --container '{args.container}' not in container_config.CONTAINERS. "
                f"Available: {sorted(container_config.CONTAINERS.keys())}\033[0m"
            )
            simulation_app.close()
            sys.exit(1)
        container_config.ACTIVE_CONTAINER = args.container

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
    container_mgr = ContainerManager(world)
    fixture_mgr = FixtureManager(world, task_profile=active_task)
    spawner = ObjectSpawner(
        world,
        repeat_count=args.repeat,
        task_profile=active_task,
        fixture_mgr=fixture_mgr,
    )
    termination_mgr = TerminationManager(world, container_mgr, spawner, controller,
                                         task_profile=active_task)
    data_collector = DataCollector(env_name=env_name, enabled=not args.eval_mode)
    camera_mgr = CameraManager()

    # Eval/inference trial tracking — enabled when --eval AND any of
    # {--max-steps, --trials, --save_video} is set. --eval alone keeps the
    # legacy "no recording, no stats" behavior.
    if args.save_video and not args.eval_mode:
        print("[Eval] --save_video ignored: only applies with --eval")
    trial_tracking = args.eval_mode and (
        args.max_steps is not None or args.trials is not None or args.save_video
    )
    stats = InferenceStats(args.max_steps, args.trials) if trial_tracking else None

    recorder = None
    if args.eval_mode and args.save_video:
        from core.trial_video import TrialVideoRecorder
        recorder = TrialVideoRecorder(out_dir="eval_videos", fps=30)

    # ============================================
    # Occlusion rate calculation (run on scene init & every reset)
    # ============================================
    current_occlusion_rates = None  # stores occlusion rate results for the current scene

    def setup_scene_with_occlusion():
        nonlocal current_occlusion_rates

        world.reset()
        controller.initialize_handles()

        # Eval mode doesn't record data, so it doesn't need occlusion rates.
        # Skipping the occlusion calc lets us keep the cameras stable (init
        # once, never rebuilt) which removes the "empty scene + skybox" flicker
        # caused by re-creating render products every reset.
        if args.eval_mode:
            camera_mgr.initialize_cameras()          # idempotent: build once, stay stable
            container_mgr.respawn()
            fixture_mgr.respawn()
            spawner.spawn_target_only()
            spawner.spawn_remaining_objects()
            for _ in range(15):
                world.step(render=True)
            current_occlusion_rates = None
            return

        # Tasks that opt out of occlusion (e.g. knob tasks have no target to
        # occlude). Spawn everything in one shot and skip the two-phase capture.
        if active_task and active_task.get("skip_occlusion", False):
            camera_mgr.initialize_cameras(force=True)
            container_mgr.respawn()
            fixture_mgr.respawn()
            spawner.spawn_target_only()
            spawner.spawn_remaining_objects()
            for _ in range(13):
                world.step(render=True)
            current_occlusion_rates = None
            print("[Main] Occlusion calc skipped (task opted out via skip_occlusion)")
            return

        # Collection mode: rebuild cameras each reset so the semantic
        # segmentation annotator sees the new scene's labels for occlusion.
        camera_mgr.initialize_cameras(force=True)

        # === Phase 1: spawn target only ===
        container_mgr.respawn()
        fixture_mgr.respawn()
        spawner.spawn_target_only()

        # Hide static occluders (scene-baked names from usd_config + per-task
        # fixture names) IMMEDIATELY — before any settle/render — so they are
        # never shown during setup (the target just looks like it floats).
        # Visibility only -> colliders stay active, so the target still settles
        # onto the now-invisible surface.
        hide_names = list(getattr(usd_config, "OCCLUSION_HIDE_PRIMS", []))

        # Check if fixture should be hidden during Phase 1
        fixture_hide_init = (
            active_task and
            active_task.get("support_fixture", {}).get("hide_init", False)
        )
        fixture_names = fixture_mgr.get_occlusion_hide_names()
        if fixture_hide_init:
            # Don't use occlusion hide for fixture; we'll control it manually
            # via fixture_mgr.make_invisible/visible
            fixture_mgr.make_invisible()
        else:
            hide_names.extend(fixture_names)

        hidden = []
        for name in hide_names:
            if _set_render_visibility(world.stage, name, visible=False):
                hidden.append(name)
        if hidden:
            print(f"[Occlusion] Hidden for baseline: {hidden}")

        for _ in range(8):
            world.step(render=True)

        target_class = spawner.get_target_class_name()
        if target_class is None:
            print("[Main] Warning: target_class_name is None, skipping occlusion calc")
            for name in hidden:
                _set_render_visibility(world.stage, name, visible=True)
            if fixture_hide_init:
                fixture_mgr.make_visible()
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

        # Restore the hidden occluders so they DO count toward occlusion in Phase 2
        for name in hidden:
            _set_render_visibility(world.stage, name, visible=True)

        # Show fixture if it was hidden during Phase 1
        if fixture_hide_init:
            fixture_mgr.make_visible()

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
    if args.eval_mode:
        mode_str = "EVAL (no recording)"
        if args.max_steps is not None:
            mode_str += f" | max_steps={args.max_steps}"
        if args.trials is not None:
            mode_str += f" | trials={args.trials}"
        if args.save_video:
            mode_str += " | save_video"
    else:
        mode_str = "DATA COLLECTION"
    print(f"Mode: {mode_str}")
    print("Move : WASDQE | Rotate : Z/X T/G C/V")
    print("Gripper : K   | Reset  : R   | Save Fail / Eval Fail : N")
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
                if stats is not None:
                    # Build the language instruction for THIS trial's layout
                    # (target class + container + BG slots), store with stats so
                    # the end-of-run summary can list per-trial details.
                    tc = spawner.get_target_class_name() or "unknown"
                    cinfo = container_mgr.get_container_info()
                    cname = cinfo.get("name", "container")
                    bg_map = getattr(spawner, "bg_class_by_point", {}) or {}
                    try:
                        instr_text = task_config.build_instruction_from_task(
                            active_task,
                            container_name=cname,
                            target_obj_name=tc,
                            fixture_name=fixture_mgr.get_fixture_name(),
                            bg_class_by_point=bg_map,
                        )
                    except Exception as e:
                        instr_text = f"<instruction build failed: {e}>"
                    stats.start_trial(instruction=instr_text)
                if recorder is not None:
                    recorder.start_trial()

            # Get input
            delta_pos, delta_rot, gripper_cmd, reset_cmd, fail_cmd, is_any_action = input_mgr.get_command()

            # Eval mode: advance per-trial counter (starts on first action)
            timed_out = stats.tick(is_any_action) if stats is not None else False

            # R during an active eval trial = redo, don't count
            if stats is not None and reset_cmd:
                stats.record("redo")
                if recorder is not None:
                    recorder.discard()
            needs_reset = reset_cmd

            # Start recording if detect any keyboard action (only when collecting)
            if data_collector.enabled and not data_collector.recording and is_any_action:
                print(" Start Recording...")
                data_collector.recording = True

            # Apply control
            controller.apply_control(delta_pos, gripper_cmd, delta_rot)
            # Update physics
            world.step(render=True)

            # Eval video: capture this frame while a trial is active
            if recorder is not None and stats is not None and stats.trial_active:
                recorder.capture(camera_mgr.get_all_camera_data())

            if data_collector.recording:
                data_collector.collect_frame(controller, delta_pos, delta_rot, gripper_cmd, spawner, camera_mgr, container_mgr)
                # First frame just landed -> save a preview PNG so the user can
                # eyeball the camera views and press N/R immediately if broken.
                if len(data_collector.current_demo_data) == 1:
                    preview_path = os.path.abspath("preview_recording.png")
                    saved = camera_mgr.save_current_frame_preview(preview_path)
                    if saved:
                        print(f"[Main] First-frame preview saved -> {saved}")

            # Manual fail save (N key): save current demo as failure and reset.
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
                    fixture_name=fixture_mgr.get_fixture_name(),
                    task_profile=active_task,
                )
                needs_reset = True

            # Eval mode: N key = count this trial as manual fail
            if stats is not None and fail_cmd:
                stats.record("manual_fail", reason="manual (N key)")
                if recorder is not None:
                    recorder.save("manual_fail")
                needs_reset = True

            # Eval mode: automatic failure detection (no need to press N).
            # Only while a trial is active so the object's spawn settle doesn't
            # trip it. More conditions live in termination_mgr.check_task_failure.
            if stats is not None and stats.trial_active and not needs_reset:
                is_fail, fail_reason = termination_mgr.check_task_failure()
                if is_fail:
                    stats.record("manual_fail", reason=fail_reason)
                    if recorder is not None:
                        recorder.save("manual_fail")
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
                    occlusion_rates=current_occlusion_rates,  # pass occlusion rates
                    fixture_name=fixture_mgr.get_fixture_name(),
                    task_profile=active_task,
                )
                spawner.notify_task_success()
                if stats is not None:
                    stats.record("success")
                if recorder is not None:
                    recorder.save("success")
                needs_reset = True

            # Eval mode: timeout (placed after success so success wins same-frame ties)
            if stats is not None and timed_out and not needs_reset:
                stats.record("timeout")
                if recorder is not None:
                    recorder.save("timeout")
                needs_reset = True

            # Eval mode: auto-exit when trial budget exhausted
            if stats is not None and stats.is_done():
                print(stats.summary())
                break
        else:
            needs_reset = True
            if stats is not None:
                stats.record("redo")
            if recorder is not None:
                recorder.discard()
            simulation_app.update()

    if stats is not None:
        print(stats.summary())
    simulation_app.close()


if __name__ == "__main__":
    main()
