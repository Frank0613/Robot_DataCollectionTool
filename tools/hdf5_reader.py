import h5py
import os
import re


def print_attrs_of(obj, level):
    indent = "    " * level
    for key, val in obj.attrs.items():
        print(f"{indent}[Attribute] {key}: {val}")


def _child_sort_key(name):
    """Display order for a group's children: initial_scene first (so it shows
    right under the data-group attributes), then demo_N in numeric order, then
    anything else alphabetically. HDF5's own link order is alphabetical, which
    is why initial_scene would otherwise sort after every demo_*."""
    if name == "initial_scene":
        return (0, 0, "")
    m = re.match(r"demo_(\d+)$", name)
    if m:
        return (1, int(m.group(1)), "")
    return (2, 0, name)


def print_group(group, level):
    """Recursively print a group's children with custom ordering."""
    indent = "    " * level
    for name in sorted(group.keys(), key=_child_sort_key):
        obj = group[name]
        if isinstance(obj, h5py.Group):
            print(f"{indent}[Group] {name}/")
            print_attrs_of(obj, level + 1)
            print_group(obj, level + 1)
        elif isinstance(obj, h5py.Dataset):
            print(f"{indent}[Dataset] {name} | Shape: {obj.shape} | Type: {obj.dtype}")
            print_attrs_of(obj, level + 1)


def print_structure_by_path(file_path):
    """Function for main.py to call"""
    if not os.path.exists(file_path):
        print(f"[Error] File not found: {file_path}")
        return

    print(f"Analyzing File: {file_path}\n" + "="*60)
    try:
        with h5py.File(file_path, 'r') as f:
            print("[Global Attributes]")
            for key, val in f.attrs.items():
                print(f"  {key}: {val}")
            print("-" * 60)
            print_group(f, 0)
            print_success_summary(f)
    except Exception as e:
        print(f"Error: {e}")


def print_success_summary(f):
    """Count success / fail demos and print a summary at the end."""
    if 'data' not in f:
        return

    success_count = 0
    fail_count = 0
    unknown_count = 0
    for demo_name in f['data']:
        if not demo_name.startswith("demo_"):
            continue
        demo = f['data'][demo_name]
        if 'success' in demo.attrs:
            if bool(demo.attrs['success']):
                success_count += 1
            else:
                fail_count += 1
        else:
            unknown_count += 1

    total = success_count + fail_count + unknown_count
    print("=" * 60)
    line = f"[Summary] Total demos: {total}  |  Success: {success_count}  |  Fail: {fail_count}"
    if unknown_count:
        line += f"  |  No success attr: {unknown_count}"
    print(line)
    print("=" * 60)

if __name__ == "__main__":
    FILE_PATH = 'datasets/dataset.hdf5'
    print_structure_by_path(FILE_PATH)
