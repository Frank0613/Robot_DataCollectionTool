"""Standalone offline occlusion-rate pass over a dataset.

Data collection records each demo's initial layout but no longer measures
occlusion live. This tool launches its own headless Isaac Sim, reproduces every
demo's starting arrangement and writes `occlusion_rates` / `occlusion_rate_avg`
back onto each demo. The heavy lifting lives in core.occlusion_offline, which
main.py also calls in-process when a collection session ends.

Use this tool to (re)compute occlusion for a dataset out of band — e.g. demos
collected before the end-of-session pass existed, or to recompute with
--overwrite.

Usage:
    python tools/compute_occlusion.py --dataset mydata
    python tools/compute_occlusion.py --dataset mydata --overwrite
    python tools/compute_occlusion.py --dataset mydata --demo 3
    python tools/compute_occlusion.py --dataset mydata --scene my_scene --task my_task
"""
import argparse
import json
import os
import sys

import h5py


def main():
    parser = argparse.ArgumentParser(description="Offline occlusion-rate pass over a dataset")
    parser.add_argument("--dataset", type=str, required=True,
                        help="Dataset name -> datasets/<name>.hdf5")
    parser.add_argument("--scene", type=str, default=None,
                        help="Scene name override (defaults to the dataset's recorded env)")
    parser.add_argument("--task", type=str, default=None,
                        help="Task profile override (defaults to the dataset's recorded task)")
    parser.add_argument("--container", type=str, default=None,
                        help="Container key override (defaults to the dataset's recorded container)")
    parser.add_argument("--demo", type=int, default=None,
                        help="Only process this single demo index")
    parser.add_argument("--overwrite", action="store_true",
                        help="Recompute even for demos that already have occlusion_rate_avg")
    args = parser.parse_args()

    dataset_path = os.path.join("datasets", f"{args.dataset}.hdf5")
    if not os.path.exists(dataset_path):
        print(f"[compute_occlusion] ERROR: {dataset_path} not found")
        sys.exit(1)

    # Read the recorded config so we reproduce the same scene/task/container.
    with h5py.File(dataset_path, 'r') as f:
        if 'data' not in f:
            print("[compute_occlusion] ERROR: no 'data' group in file")
            sys.exit(1)
        sig_raw = f['data'].attrs.get('config_signature')
        signature = json.loads(sig_raw) if sig_raw else {}
        total = int(f['data'].attrs.get('total', 0))

    task_name = args.task if args.task is not None else signature.get("task")
    container_key = args.container if args.container is not None else signature.get("container")
    scene_name = args.scene if args.scene is not None else signature.get("env")

    if total == 0:
        print("[compute_occlusion] Dataset has no demos, nothing to do")
        return

    print(f"[compute_occlusion] {dataset_path}: {total} demos | "
          f"scene={scene_name} task={task_name} container={container_key}")

    # ---- Launch Isaac Sim (headless) ----
    print("[compute_occlusion] Starting Isaac Sim (headless)...")
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": True})

    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import open_stage, is_stage_loading
    from configs import usd_config, container_config, task_config
    from core.robot_controller import FrankaController
    from core.object_spawner import ObjectSpawner
    from core.container_manager import ContainerManager
    from core.fixture_manager import FixtureManager
    from core.camera_manager import CameraManager
    from core.occlusion_offline import run_occlusion_pass

    active_task = None
    if task_name is not None:
        active_task = task_config.apply_task(task_name)
        print(f"[compute_occlusion] Applied task profile: '{task_name}'")
    if container_key is not None:
        if container_key not in container_config.CONTAINERS:
            print(f"[compute_occlusion] WARNING: container '{container_key}' not in "
                  f"CONTAINERS; keeping default '{container_config.ACTIVE_CONTAINER}'")
        else:
            container_config.ACTIVE_CONTAINER = container_key
    if scene_name is not None:
        usd_config.SCENE_NAME = scene_name
        usd_config.USD_PATH = os.path.join(
            usd_config.BASE_DIR, "usdfiles", "scenes", f"{scene_name}.usd")

    print(f"[compute_occlusion] Loading scene: {usd_config.USD_PATH}")
    open_stage(usd_config.USD_PATH)
    while is_stage_loading():
        simulation_app.update()

    world = World(stage_units_in_meters=1.0)
    controller = FrankaController(world)
    container_mgr = ContainerManager(world)
    fixture_mgr = FixtureManager(world, task_profile=active_task)
    spawner = ObjectSpawner(world, task_profile=active_task, fixture_mgr=fixture_mgr)
    camera_mgr = CameraManager()

    demo_indices = [args.demo] if args.demo is not None else None

    run_occlusion_pass(
        dataset_path, world, controller, camera_mgr, spawner,
        container_mgr, fixture_mgr,
        overwrite=args.overwrite, demo_indices=demo_indices)

    simulation_app.close()


if __name__ == "__main__":
    main()
