"""Unified task configuration: each task defines its instruction + optional special settings
(support_fixture, spawn_overrides, etc). Scene and container are set via CLI flags.

Usage:
  ./run.bat --task behind_named --scene warehouse --container plate
  ./run.bat --task in_drawer --scene warehouse --container plate
"""
import sys


ACTIVE_TASK = None


# =============================================================================
# Task Schema (all fields optional except for special tasks):
#   instruction      : str  - the full instruction sentence with {obj}, {container}, etc.
#   support_fixture  : dict - (optional) dynamic fixture to spawn. Contains:
#     usd, anchor_point, name, scale, hide_init
#     (object selection is deterministic via configs/scene_config.py)
#   spawn_overrides  : dict - (optional) per-point position/orientation deltas
#   bg_skip          : list - (optional) background spawn points to leave empty
#   target_offset_above_fixture : float - (optional) target height above fixture
#   target_inside_fixture : dict - (optional) target inside fixture sub-prim
#   success_criterion : str - (optional) override the success rule (default: place_in_container)
#   skip_occlusion   : bool - (optional) if True, skip occlusion-rate calculation
#                             (e.g. knob tasks have no target to occlude). Default False.
# =============================================================================
TASKS = {
    # === Basic instructions (instruction only) ===
    "basic": {
        "instruction": "pick up the {obj} and place it on the {container}",
    },
    "center": {
        "instruction": "pick up the {obj} from the table center and place it on the {container}",
    },
    "next_to_right": {
        "instruction": "pick up the {obj} next to the {right} and place it on the {container}",
    },
    "next_to_left": {
        "instruction": "pick up the {obj} next to the {left} and place it on the {container}",
    },
    "behind_named": {
        "instruction": "pick up the {obj} behind the {front} and place it on the {container}",
    },
    "between_named": {
        "instruction": "pick up the {obj} between the {left} and the {right} and place it on the {container}",
    },
    "cabinet_place": {
        "instruction": "pick up the {obj} from the cabinet and place it on the {container}",
    },
    "cabinet_open": {
        "instruction": "open the cabinet and pick up the {obj} and place it on the {container}",
    },
    "stove_place": {
        "instruction": "pick up the {obj} and place it on the {container}",
    },

    # === Special instructions (with support_fixture / spawn_overrides) ===
    "on_fixture": {
        "instruction": "pick up the {obj} on the {fixture} and place it on the {container}",
        "support_fixture": {
            "usd": "usdfiles/fixtures/pan.usd",
            "anchor_point": "target_point",
            "name": "pan",
        },
        "target_offset_above_fixture": 0.10,
    },

    "in_drawer": {
        "instruction": "pick up the {obj} in the top drawer of the {fixture} and place it on the {container}",
        "support_fixture": {
            "usd": "usdfiles/cabinet/white_cabinet_open.usd",
            "anchor_point": "cabinet_pos",
            "name": "white_cabinet",
            "scale": 1.4,
            "hide_init": True,
        },
        "spawn_overrides": {
            "target_point": {"z_offset": 0.15},
            "point_left": {"z_offset": 0.35},
        },
        "bg_skip": ["point_front"],
    },

    # === Knob task (success = turn knob, not place object) ===
    "stove_turn": {
        "instruction": "turn the knob on the {container}",
        "bg_skip": ["point_front"],
        "success_criterion": "turn_knob",
        "skip_occlusion": True,  # no target object to occlude
    },
}


# Which point name each slot maps to
POINT_SLOT_MAP = {
    "right": "point_right",
    "left": "point_left",
    "front": "point_front",
}


def get_task(name):
    """Return the task dict for `name`, or None if not found."""
    return TASKS.get(name)


def list_tasks():
    return sorted(TASKS.keys())


def get_success_criterion(task_name):
    """Get the success criterion for a task (defaults to 'place_in_container')."""
    task = get_task(task_name)
    if task is None:
        return "place_in_container"
    return task.get("success_criterion", "place_in_container")


def apply_task(name):
    """Validate and return a task from TASKS.

    Args:
        name: task name from TASKS

    Returns:
        Task dict, or exits if task unknown or invalid.
    """
    task = get_task(name)
    if task is None:
        print(
            f"\033[91m[ERROR] --task '{name}' not found. "
            f"Available: {list_tasks()}\033[0m"
        )
        sys.exit(1)

    # Validate that task's instruction exists
    if "instruction" not in task:
        print(
            f"\033[91m[ERROR] task '{name}' has no 'instruction' field.\033[0m"
        )
        sys.exit(1)

    return task


def build_instruction_from_task(
    task,
    container_name,
    target_obj_name,
    fixture_name=None,
    bg_class_by_point=None,
):
    """Build a formatted instruction from a task.

    Args:
        task: task dict from TASKS
        container_name: e.g., "stove", "plate" -> filled into {container}
        target_obj_name: filled into {obj}
        fixture_name: (optional) filled into {fixture}
        bg_class_by_point: dict like {"point_left": "bolt", "point_right": "gear"}

    Returns:
        Formatted instruction string.
    """
    instruction = task.get("instruction", "")
    if not instruction:
        return ""

    slots = {
        "obj": target_obj_name.replace("_", " "),
        "container": container_name.replace("_", " "),
    }

    # Handle {fixture} slot
    if "{fixture}" in instruction:
        if fixture_name is None:
            raise ValueError(
                f"Task instruction needs '{{fixture}}' slot but no fixture provided"
            )
        slots["fixture"] = fixture_name.replace("_", " ")

    # Handle {left}, {right}, {front} slots (point-based)
    bg_class_by_point = bg_class_by_point or {}
    for slot_key, point_name in POINT_SLOT_MAP.items():
        if f"{{{slot_key}}}" in instruction:
            cls = bg_class_by_point.get(point_name)
            if cls is None:
                raise ValueError(
                    f"Task instruction needs '{{{{ {slot_key} }}}}' slot "
                    f"but point '{point_name}' has no spawned object"
                )
            slots[slot_key] = cls.replace("_", " ")

    return instruction.format(**slots)
