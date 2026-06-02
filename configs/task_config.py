"""Task profiles: bundle scene + container + template + spawn overrides + support
fixture into a single named entry. Pick one with --task on the CLI.

Adding a new instruction = add one entry to TASKS below. No edits to main.py,
object_spawner.py, container_manager.py, or fixture_manager.py required.
"""
import os
import sys


# Set this to a TASKS key (or leave None) to default-select a task when --task
# is omitted. Useful for committing a "current working task" without touching
# the CLI each run. None -> fall back to legacy --scene/--template/--container.
ACTIVE_TASK = None


# =============================================================================
# Schema (all optional except scene/container/template):
#   scene             : str  - usd filename under usdfiles/scenes/ (no .usd)
#   container         : str  - key from container_config.CONTAINERS
#   template          : str  - key from instruction_config.TEMPLATES
#   spawn_overrides   : dict - {spawn_point_name: {"position": [x,y,z],
#                                                  "orientation": [w,x,y,z]}}
#                              applied to /World/target_points/<name> at reset
#   bg_skip           : list - background spawn point names to leave empty
#                              (replaces the implicit "stove skips point_front")
#   support_fixture   : dict - a static USD spawned by FixtureManager. Schema:
#     usd             : str  - path relative to project root
#     anchor_point    : str  - spawn point name under /World/target_points/ that
#                              positions the fixture (e.g. "target_point",
#                              "cabinet_pos")
#     name            : str  - logical name; fills {fixture} slot in templates
#                              and is auto-added to occlusion_hide
#     scale           : float - optional. Uniform scale (XYZ same). Omit to use
#                              the USD's default scale (recommended).
#     hide_init       : bool  - optional. If True, hide the fixture during Phase 1
#                              (baseline), then show it during Phase 2. Default False.
#     target_objects_dir : str - optional. Restrict target object selection to this
#                              directory (e.g., "usdfiles/objects/small"). If omitted,
#                              target can be any object. Only applies when support_fixture
#                              is present.
#   target_offset_above_fixture : float - z meters above fixture for target
#                                         spawn (e.g. 0.30 = on stool top)
#   target_inside_fixture       : dict - target spawns inside a fixture sub-prim
#     subpath         : str  - sub-prim path relative to fixture root
#                              (e.g. "/drawer")
#     offset          : [x,y,z] in sub-prim local frame
# =============================================================================
TASKS = {
    # --- Warehouse: minimal, no fixture ---
    "warehouse_basic": {
        "scene": "Warehouse",
        "container": "plate",
        "template": "basic",
    },
    "warehouse_center": {
        "scene": "Warehouse",
        "container": "plastic_basket",
        "template": "center",
    },
    "warehouse_next_to_right": {
        "scene": "Warehouse",
        "container": "plate",
        "template": "next_to_right",
    },
    "warehouse_next_to_left": {
        "scene": "Warehouse",
        "container": "plate",
        "template": "next_to_left",
    },
    "warehouse_behind_named": {
        "scene": "Warehouse",
        "container": "plate",
        "template": "behind_named",
    },
    "warehouse_between_named": {
        "scene": "Warehouse",
        "container": "plate",
        "template": "between_named",
    },

    # --- Warehouse + stove container ---
    "warehouse_stove_turn": {
        "scene": "Warehouse",
        "container": "stove",
        "template": "stove_turn",
        "bg_skip": ["point_front"],
    },
    "warehouse_stove_place": {
        "scene": "Warehouse",
        "container": "stove",
        "template": "stove_place",
        "bg_skip": ["point_front"],
    },

    # --- home_cabinet scene (cabinet baked into the scene USD) ---
    "home_cabinet_place": {
        "scene": "home_cabinet",
        "container": "plate",
        "template": "cabinet_place",
    },
    "home_cabinet_open": {
        "scene": "home_cabinet",
        "container": "plate",
        "template": "cabinet_open",
    },

    # =========================================================================
    # Stubs below need extra assets / scene work before they can run.
    # Uncomment after the listed prerequisites are in place.
    # =========================================================================
    #
    "warehouse_on_fixture": {
        "scene": "Warehouse",
        "container": "plate",
        "template": "on_fixture",
        "support_fixture": {
            "usd": "usdfiles/fixtures/pan.usd",   
            "anchor_point": "target_point",
            "name": "pan",
        },
        "target_offset_above_fixture": 0.10,
    },
    #
    "warehouse_in_drawer": {
        "scene": "Warehouse",
        "container": "plate",
        "template": "in_drawer",
        "support_fixture": {
            "usd": "usdfiles/cabinet/white_cabinet_open.usd",
            "anchor_point": "cabinet_pos",
            "name": "white_cabinet",
            "scale": 1.4,
            "hide_init": True,
            "target_objects_dir": "usdfiles/objects/small",  # Only small objects for target
        },
        # Target and background spawn point offsets (in world coordinates)
        "spawn_overrides": {
            "target_point": {"z_offset": 0.15},  # 15cm above target_point so it drops into drawer
            "point_left": {"z_offset": 0.35},    # 35cm above to avoid clipping cabinet
        },
        "bg_skip": ["point_front"],
    },
}


def get_task(name):
    """Return the task dict for `name`, or None if not found."""
    return TASKS.get(name)


def list_tasks():
    return sorted(TASKS.keys())


def apply_task(name, usd_config, instruction_config, container_config):
    """Populate the three config modules from the named task profile.

    Returns the task dict so callers can access spawn_overrides / support_fixture
    / bg_skip / target_offset_above_fixture / target_inside_fixture later.
    Exits the process with a clear error if the task is unknown or references
    an unknown container / template.
    """
    task = get_task(name)
    if task is None:
        print(
            f"\033[91m[ERROR] --task '{name}' not found. "
            f"Available: {list_tasks()}\033[0m"
        )
        sys.exit(1)

    scene = task["scene"]
    container = task["container"]
    template = task["template"]

    if template not in instruction_config.TEMPLATES:
        print(
            f"\033[91m[ERROR] task '{name}' references unknown template '{template}'. "
            f"Available: {sorted(instruction_config.TEMPLATES.keys())}\033[0m"
        )
        sys.exit(1)
    if container not in container_config.CONTAINERS:
        print(
            f"\033[91m[ERROR] task '{name}' references unknown container '{container}'. "
            f"Available: {sorted(container_config.CONTAINERS.keys())}\033[0m"
        )
        sys.exit(1)

    usd_config.SCENE_NAME = scene
    usd_config.USD_PATH = os.path.join(
        usd_config.BASE_DIR, "usdfiles", "scenes", f"{scene}.usd"
    )
    instruction_config.ACTIVE_TEMPLATE = template
    container_config.ACTIVE_CONTAINER = container
    return task
