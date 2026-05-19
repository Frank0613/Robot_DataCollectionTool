import h5py
import numpy as np
import os

def print_attrs(name, obj):
    indent = "    " * (name.count('/') + 1)
    for key, val in obj.attrs.items():
        print(f"{indent}[Attribute] {key}: {val}")

def print_structure(name, obj):
    level = name.count('/')
    indent = "    " * level
    node_name = name.split('/')[-1]

    if isinstance(obj, h5py.Group):
        print(f"{indent}[Group] {node_name}/")
        print_attrs(name, obj)
    elif isinstance(obj, h5py.Dataset):
        print(f"{indent}[Dataset] {node_name} | Shape: {obj.shape} | Type: {obj.dtype}")
        print_attrs(name, obj)

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
            f.visititems(print_structure)
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