# Switch template here before a collection run
ACTIVE_TEMPLATE = "center"


TEMPLATES = {
    "center":        "pick up the {obj} from the table center and place it on the {container}",
    "basic":         "pick up the {obj} and place it on the {container}",

    "next_to_right": "pick up the {obj} next to the {right} and place it on the {container}",
    "next_to_left":  "pick up the {obj} next to the {left} and place it on the {container}",
    "behind_named":  "pick up the {obj} behind the {front} and place it on the {container}",
    "between_named": "pick up the {obj} between the {left} and the {right} and place it on the {container}",

    "cabinet_place":       "pick up the {obj} from the cabinet and place it on the {container}",
    "cabinet_open":       "open the cabinet and pick up the {obj} and place it on the {container}",

    # Knob task: success criterion is the knob rotating past its threshold,
    # not placing an object. {obj} is unused so it can be omitted from the text.
    "stove_turn":          "turn the knob on the {container}",
    "stove_place":          "pick up the {obj} and place it on the {container}",
}

# Which point name (from usd_config.BG_SPAWN_POINTS) each slot maps to
POINT_SLOT_MAP = {
    "right": "point_right",
    "left":  "point_left",
    "front": "point_front",
}

# Maps template name -> success criterion used by TerminationManager.
# Templates not listed here default to "place_in_container".
TEMPLATE_TASKS = {
    "stove_turn": "turn_knob",
}


def get_task_type(template_name: str | None = None) -> str:
    name = template_name if template_name is not None else ACTIVE_TEMPLATE
    return TEMPLATE_TASKS.get(name, "place_in_container")


def build_instruction(obj: str, container: str, bg_class_by_point: dict | None = None) -> tuple[str, str]:
    tmpl = TEMPLATES[ACTIVE_TEMPLATE]
    slots = {"obj": obj, "container": container}

    bg_class_by_point = bg_class_by_point or {}
    for slot_key, point_name in POINT_SLOT_MAP.items():
        if f"{{{slot_key}}}" in tmpl:
            cls = bg_class_by_point.get(point_name)
            if cls is None:
                raise ValueError(
                    f"Template '{ACTIVE_TEMPLATE}' needs slot '{slot_key}' "
                    f"but point '{point_name}' has no spawned object"
                )
            slots[slot_key] = cls.replace("_", " ")

    return tmpl.format(**slots), ACTIVE_TEMPLATE
