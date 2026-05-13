# =============================================================================
# Switch active container here. The key must match an entry in CONTAINERS below.
# =============================================================================
ACTIVE_CONTAINER = "plate"


CONTAINERS = {
    "plastic_basket": {
        "name": "plastic_basket",
        "path": "usdfiles/containers/plastic_basket.usd",
        "interior": "auto",
    },
    "drawer": {
        "name": "drawer",
        "path": "usdfiles/cabinet/white_cabinet.usd",
        "interior": {
            "anchor_subpath": "/drawer",
            # TODO: tune these to match the actual drawer cavity
            "offset":    (0.0, 0.0, 0.05),
            "half_size": (0.15, 0.10, 0.05),
            "z_tolerance_above": 0.0,
        },
    },
    
    "plate": {
        "name": "plate",
        "path": "usdfiles/containers/plate.usd",
        "interior": {
            "anchor_subpath": None,
            "offset":    (0.0, 0.0, 0.01),
            "half_size": (0.12, 0.12, 0.005),
            "z_tolerance_above": 0.1,
        },
    },
    "stove": {
        "name": "stove",
        "path": "usdfiles/containers/stove.usd",
        "interior": {
            "anchor_subpath": None,
            # Box is centered at offset (in stove-local frame), with full size
            # (0.5, 0.7, 0.05) -> half_size (0.25, 0.35, 0.025).
            "offset":    (0.025, 0.0, 0.005),
            "half_size": (0.25, 0.35, 0.025),
            "z_tolerance_above": 0.1,
        },
        # Optional: enables the "turn_knob" instruction template. Success when
        # the knob sub-prim has rotated past `threshold_deg` from its baseline
        # (captured at spawn). See "knob spec" notes at the bottom of this file.
        "knob": {
            "subpath": "/knob_center",
            "threshold_deg": 30.0,
        },
    },
}


def get_container_info(key: str | None = None) -> dict:
    target = key if key is not None else ACTIVE_CONTAINER
    info = CONTAINERS.get(target)
    if info is None:
        print(f"[container_config] Warning: Key '{target}' not found.")
        return {}
    return info


# =============================================================================
# Mental model: success detection = an invisible box drawn somewhere on/around
# the container. If the object's position falls inside the box, the task is
# considered successful. The four fields below describe where the box sits,
# how big it is, and how much extra room to allow above it.
# =============================================================================
#
# ---------- interior modes ----------
#   "auto"   : use the prim's full world AABB (axis-aligned bounding box) as
#              the valid zone. Good for open-top containers whose overall shape
#              roughly equals the interior (e.g. basket).
#              An automatic +0.5m z headroom is added above the AABB.
#
#   dict {…} : an explicitly-defined box in a local frame, with the four fields
#              described below. Required for drawers, plates, and any container
#              whose full AABB does not match the actual valid drop zone.
#
# ---------- dict fields ----------
#
# (1) anchor_subpath : which prim the box is "pinned" to. When that prim moves,
#                      the box moves with it.
#       None      → pinned to the container root (use this for static shapes
#                   like plates).
#       "/drawer" → pinned to a sub-prim named "drawer" (the moving tray).
#                   When the drawer slides out, the box slides with it.
#       Path is relative to CONTAINER_PRIM_PATH (i.e. /World/Container) and
#       must start with "/".
#
# (2) offset : (x, y, z) where the CENTER of the box sits in the anchor's local
#              frame, in meters.
#       x, y → usually 0 to center on the anchor; non-zero shifts the box.
#       z    → commonly set to "half the container's thickness" so the box
#              sits flush with the surface (plate top, drawer cavity floor,
#              etc.) when the anchor's origin is at the bottom.
#
# (3) half_size : (x, y, z) HALF-extents of the box (NOT full edge length).
#       Example: half_size=(0.12, 0.12, 0.005) → an actual 24cm × 24cm × 1cm box.
#       Tuning hints:
#         x, y → set to the container's interior half-width; objects past the
#                edge will be excluded.
#         z    → for "cavity" shapes (drawer) use half the cavity depth.
#                For "surface" shapes (plate) use a tiny value like 0.005.
#
# (4) z_tolerance_above : extra height (meters) added only to the +z face of
#                         the box. The bottom is unaffected.
#       Why: when an object rests on a surface, its center sits ABOVE the
#       surface (the object itself has height).
#       Suggested values:
#         drawer → 0.0    (avoid false positives from objects on the cabinet
#                          top surface above the drawer)
#         plate  → 0.15 ~ 0.25 (so taller items like cans/bottles still count)
#         basket → not applicable ("auto" already includes a 0.5 headroom)
#
# ---------- tuning SOP ----------
#   1. In Isaac Sim, select the container prim and read its bounding box to
#      measure the real interior dimensions.
#   2. Fill half_size with the interior half-extents.
#   3. Adjust offset (mainly z) to push the box up to the surface / cavity.
#   4. Drawer → z_tolerance_above = 0.  Plate → start at 0.15, raise if needed.
#   5. To verify, temporarily print(local_pt) inside ContainerManager.is_inside()
#      and drop the object on the container — the printed coords show where the
#      box should land.
# =============================================================================
# Optional knob spec (for the "turn_knob" instruction template)
# =============================================================================
#   subpath        : sub-prim path to the rotating knob, relative to the
#                    container root (e.g. "/knob_center"). Must start with "/".
#   threshold_deg  : success when the knob has rotated this many degrees away
#                    from its baseline pose (captured at container spawn).
#                    The angle is the total quaternion delta, so it works
#                    regardless of which axis the knob actually rotates around.
# =============================================================================
