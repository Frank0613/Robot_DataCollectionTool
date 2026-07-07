import argparse
import os
import signal
import sys

from tools.hdf5_reader import print_structure_by_path
from tools.hdf5_checker import visualize_hdf5_cameras_by_path
from tools.hdf5_video import visualize_hdf5_demo_as_video, export_hdf5_demo_frames
from core.inference_stats import InferenceStats


def main():
    parser = argparse.ArgumentParser(description="Robot Control Tool")
    parser.add_argument("--mode", type=str, choices=["ik", "rmpflow"])
    parser.add_argument("--scene", type=str)
    parser.add_argument("--readfile", type=str)
    parser.add_argument("--checkfile", type=str)
    parser.add_argument("--checkvideo", type=str,
                        help="Render a whole demo as an MP4 (use with --demo).")
    parser.add_argument("--everyframe", type=str,
                        help="Export every RGB frame of a demo as PNGs, one folder per camera (use with --demo).")
    parser.add_argument("--demo", type=int, default=0,
                        help="Demo index for --checkfile and --checkvideo.")
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--debug", action="store_true",
                        help="Visualize the IK/RMP target ball in the scene.")
    parser.add_argument("--show-zone", dest="show_zone", action="store_true",
                        help="Draw the container's interior success/drop zone as a translucent box "
                             "for tuning container_config 'interior'. The box appears in camera RGB, "
                             "so use it only for manual tuning, not while recording.")
    parser.add_argument("--eval", dest="eval_mode", action="store_true",
                        help="Run without recording any data (for model inference or manual eval).")
    parser.add_argument("--dataset", type=str, default=None,
                        help="Eval mode: reproduce the initial_scene recorded in datasets/<name>.hdf5 "
                             "(same objects + container at the exact same positions/angles).")
    parser.add_argument("--out", type=str, default="dataset",
                        help="Collection output name -> datasets/<name>.hdf5. If the file already "
                             "holds demos from a different config, an auto-suffixed file is used.")
    parser.add_argument("--headless", action="store_true",
                        help="Run Isaac Sim without the GUI viewport (recommended for eval; "
                             "avoids RTX viewport init crashes).")
    parser.add_argument("--profile", action="store_true",
                        help="Print rolling per-phase timings (input/control/render/collect/"
                             "camera/termination) every 60 frames to diagnose FPS.")
    parser.add_argument("--max-steps", dest="max_steps", type=int, default=None,
                        help="Per-trial step timeout (eval mode only). Required to enable stats.")
    parser.add_argument("--trials", type=int, default=None,
                        help="Total trials before auto-exit (eval mode only). Omit for unlimited.")
    parser.add_argument("--save_video", action="store_true",
                        help="Eval mode: save each trial as an MP4 under eval_videos/.")
    parser.add_argument("--template", type=str, default=None,
                        help="[DEPRECATED] Use --task instead. Kept as an alias for task selection.")
    parser.add_argument("--task", type=str, default=None,
                        help="Select a task profile (see configs/task_config.py TASKS). "
                             "Sets scene/template; --scene/--template still override. "
                             "Container and object layout come from configs/scene_config.py.")
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

    if args.everyframe:
        target = os.path.join("datasets", f"{args.everyframe}.hdf5")
        export_hdf5_demo_frames(target, demo_idx=args.demo)
        return
    
    print("Starting Isaac Sim Environment...")
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": args.headless})

    # Eval uses keyboard control (same input path as data collection); no ROS
    # bridge is enabled. The Action Graph baked into the robot USD simply fails
    # to load its ROS2 nodes, which is harmless.

    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import open_stage, is_stage_loading
    from configs import robot_config, usd_config, container_config, task_config, scene_config
    from core.input_manager import InputManager
    from core.robot_controller import FrankaController
    from core.object_spawner import ObjectSpawner
    from core.container_manager import ContainerManager
    from core.fixture_manager import FixtureManager
    from core.termination_manager import TerminationManager
    from core.data_collector import DataCollector
    from core.camera_manager import CameraManager
    from core.eval_scene import load_initial_scene
    from core.profiler import profiler
    import omni.kit.app

    profiler.enabled = args.profile
    if args.profile:
        print(f"[Main] Profiling enabled — per-phase timings every 60 frames "
              f"-> log: {profiler.log_path}")
    
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

    # Container is chosen in configs/scene_config.py LAYOUT["container_pos"]
    # ("container"), referencing a key in container_config.CONTAINERS.
    # None -> keep container_config's default.
    container_key = scene_config.get_container_key()
    if container_key is not None:
        if container_key not in container_config.CONTAINERS:
            print(
                f"\033[91m[ERROR] scene_config container '{container_key}' not in "
                f"container_config.CONTAINERS. Available: {sorted(container_config.CONTAINERS.keys())}\033[0m"
            )
            simulation_app.close()
            sys.exit(1)
        container_config.ACTIVE_CONTAINER = container_key

    # Eval replay: load a recorded dataset's initial_scene so we can reproduce
    # the exact starting layout (same objects + container at the same poses).
    # The dataset's container overrides scene_config's choice.
    eval_scene = None
    eval_container_pose = None
    eval_fixture_pose = None
    if args.eval_mode and args.dataset is not None:
        dataset_path = os.path.join("datasets", f"{args.dataset}.hdf5")
        eval_scene = load_initial_scene(dataset_path)
        if eval_scene is None:
            print(f"\033[91m[ERROR] Could not load initial_scene from {dataset_path}\033[0m")
            simulation_app.close()
            sys.exit(1)
        container_entry = eval_scene.get("container")
        if container_entry is not None:
            ckey = container_entry["class"]
            if ckey in container_config.CONTAINERS:
                container_config.ACTIVE_CONTAINER = ckey
            else:
                print(f"[Main] Warning: recorded container '{ckey}' not in "
                      f"container_config.CONTAINERS; keeping '{container_config.ACTIVE_CONTAINER}'")
            if container_entry.get("pose") is not None:
                cp = container_entry["pose"]
                eval_container_pose = (cp[:3], cp[3:7])
        fixture_entry = eval_scene.get("fixture")
        if fixture_entry is not None and fixture_entry.get("pose") is not None:
            fp = fixture_entry["pose"]
            eval_fixture_pose = (fp[:3], fp[3:7])
        print(f"[Main] Eval replay enabled from dataset '{args.dataset}'")

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
    container_mgr.debug_zone = args.show_zone
    fixture_mgr = FixtureManager(world, task_profile=active_task)
    spawner = ObjectSpawner(
        world,
        task_profile=active_task,
        fixture_mgr=fixture_mgr,
    )
    # Eval replay: lock object identities to the recorded scene so resets after
    # the first exact reproduction keep the same objects (randomized poses).
    if eval_scene is not None:
        spawner.apply_scene_objects(eval_scene)
    termination_mgr = TerminationManager(world, container_mgr, spawner, controller,
                                         task_profile=active_task)
    # Config signature: ties an output dataset file to one object/container/
    # scene/task combo so a single file never mixes layouts (see DataCollector).
    config_signature = {
        "env": env_name,
        "container": container_config.ACTIVE_CONTAINER,
        "task": task_name,
        "objects": {p: scene_config.get_file(p)
                    for p in (["target_point"] + list(usd_config.BG_SPAWN_POINTS))},
    }
    data_collector = DataCollector(
        env_name=env_name,
        enabled=not args.eval_mode,
        filename=f"{args.out}.hdf5",
        config_signature=config_signature,
    )
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
    eval_replay_done = False         # eval replay: first reset is exact, rest randomize

    def setup_scene_with_occlusion():
        nonlocal current_occlusion_rates, eval_replay_done

        world.reset()
        controller.initialize_handles()

        # Eval mode doesn't record data, so it doesn't need occlusion rates.
        # Skipping the occlusion calc lets us keep the cameras stable (init
        # once, never rebuilt) which removes the "empty scene + skybox" flicker
        # caused by re-creating render products every reset.
        if args.eval_mode:
            camera_mgr.initialize_cameras()          # idempotent: build once, stay stable
            if eval_scene is not None and not eval_replay_done:
                # First reset: reproduce the recorded layout EXACTLY (same
                # objects + container at the same positions/angles), no randomization.
                container_mgr.respawn(override_pose=eval_container_pose)
                fixture_mgr.respawn(override_pose=eval_fixture_pose)
                spawner.respawn_from_scene(eval_scene)
                eval_replay_done = True
            else:
                # Subsequent eval resets (and non-replay eval): same objects
                # (locked via apply_scene_objects when replaying) at randomized
                # positions/angles.
                container_mgr.respawn()
                fixture_mgr.respawn()
                spawner.spawn_target_only()
                spawner.spawn_remaining_objects()
            for _ in range(15):
                world.step(render=True)
            current_occlusion_rates = None
            return

        # Collection mode: occlusion rates are computed OFFLINE afterwards
        # (tools/compute_occlusion.py) from each demo's recorded initial layout,
        # so the hot loop no longer rebuilds cameras or renders a semantic-
        # segmentation pass every frame — that per-reset rebuild leaked render
        # products and made recording get slower over time. Cameras are now
        # built ONCE and kept stable (force=False) — the same fast, flicker-free
        # path eval uses — and every object spawns in a single shot (no
        # two-phase baseline capture, no occluder hiding).
        camera_mgr.initialize_cameras()          # idempotent: build once, stay stable
        container_mgr.respawn()
        fixture_mgr.respawn()
        spawner.spawn_target_only()
        spawner.spawn_remaining_objects()
        for _ in range(13):
            world.step(render=True)
        # Live occlusion is no longer computed here; save_demo records each
        # demo's initial layout so it can be filled in offline.
        current_occlusion_rates = None

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
    if not args.eval_mode:
        print("Finish + compute occlusion : P  or  Ctrl+C")
    print("==========================================")
    log_demo_status()

    needs_reset = False
    finish_requested = False  # set by the P key OR Ctrl+C: end collection, then
                              # run the offline occlusion pass while the app is alive.

    # Intercept Ctrl+C so the operator's usual "collect then Ctrl+C" habit ends
    # the session gracefully instead of hard-killing the process: the handler
    # just raises a flag the loop checks, so we still reach the occlusion pass.
    # Installed AFTER SimulationApp init so it overrides omni's own SIGINT
    # handler. A SECOND Ctrl+C (during the occlusion pass) restores the default
    # handler and aborts, so a wedged pass is still killable.
    interrupt = {"stop": False}

    def _on_sigint(signum, frame):
        if not interrupt["stop"]:
            interrupt["stop"] = True
            print("\n[Main] Ctrl+C received — finishing session, will compute "
                  "occlusion before exit (press Ctrl+C again to abort).")
    signal.signal(signal.SIGINT, _on_sigint)

    while simulation_app.is_running():

        # Ctrl+C (or the P key below) ends collection cleanly.
        if interrupt["stop"]:
            if not args.eval_mode:
                finish_requested = True
            break
        
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

            profiler.tick_frame()

            # Get input
            _t = profiler.start()
            delta_pos, delta_rot, gripper_cmd, reset_cmd, fail_cmd, is_any_action, finish_cmd = input_mgr.get_command()
            profiler.stop("input.get_command", _t)

            # P key (collection only): end the session cleanly so the offline
            # occlusion pass can run below while the sim is still alive.
            if finish_cmd and not args.eval_mode:
                print("[Main] Finish requested (P) — ending collection.")
                finish_requested = True
                break

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
            _t = profiler.start()
            controller.apply_control(delta_pos, gripper_cmd, delta_rot)
            profiler.stop("controller.apply_control", _t)
            # Update physics
            _t = profiler.start()
            world.step(render=True)
            profiler.stop("world.step(render)", _t)

            # Eval video: capture this frame while a trial is active
            if recorder is not None and stats is not None and stats.trial_active:
                recorder.capture(camera_mgr.get_all_camera_data())

            if data_collector.recording:
                _t = profiler.start()
                data_collector.collect_frame(controller, delta_pos, delta_rot, gripper_cmd, spawner, camera_mgr, container_mgr, fixture_mgr)
                profiler.stop("collect_frame(total)", _t)
                # First frame just landed -> save a preview PNG so the user can
                # eyeball the camera views and press N/R immediately if broken.
                if len(data_collector.current_demo_data) == 1:
                    preview_path = os.path.abspath("preview_recording.png")
                    saved = camera_mgr.save_current_frame_preview(preview_path)
                    if saved:
                        print(f"[Main] First-frame preview saved -> {saved}")

            # Manual fail save (N key): save current demo as failure and reset.
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
            _t = profiler.start()
            is_success, success_obj_name = termination_mgr.check_task_success()
            profiler.stop("termination.check_success", _t)
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

    # Collection finished via the P key or Ctrl+C: compute occlusion rates
    # offline now, reusing the live managers (no Isaac Sim relaunch). Skipped if
    # nothing was collected. Failures here never block the clean shutdown below.
    if finish_requested and data_collector.enabled:
        # Restore the default handler so a second Ctrl+C aborts a wedged pass.
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        demo_count = data_collector.get_demo_count()
        if demo_count > 0:
            print(f"[Main] Computing occlusion rates offline for {demo_count} demo(s)...")
            try:
                from core.occlusion_offline import run_occlusion_pass
                run_occlusion_pass(
                    data_collector.filepath, world, controller, camera_mgr,
                    spawner, container_mgr, fixture_mgr,
                )
            except Exception as e:
                print(f"[Main] Occlusion pass failed: {e}")
        else:
            print("[Main] No demos collected — skipping occlusion pass.")

    simulation_app.close()


if __name__ == "__main__":
    main()
