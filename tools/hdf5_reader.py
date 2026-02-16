# I need to run pip install h5py inside the Isaac Sim environment.

import h5py
import numpy as np

# HDF5 file path (Change this to your dataset path if needed)
FILE_PATH = 'datasets/dataset.hdf5' #Change this to your dataset path if needed

def print_attrs(name, obj):
    """ Helper function: print all attributes for a given node """
    indent = "    " * (name.count('/') + 1)
    for key, val in obj.attrs.items():
        print(f"{indent}[Attribute] {key}: {val}")

def print_structure(name, obj):
    """ Core function: recursively visit each node in the HDF5 file """
    level = name.count('/')
    indent = "    " * level
    node_name = name.split('/')[-1]

    if isinstance(obj, h5py.Group):
        print(f"{indent}[Group] {node_name}/")
        print_attrs(name, obj)

    elif isinstance(obj, h5py.Dataset):
        print(f"{indent}[Dataset] {node_name} | Shape: {obj.shape} | Type: {obj.dtype}")
        print_attrs(name, obj)

print(f"Analyzing file: {FILE_PATH}\n" + "="*60)

try:
    with h5py.File(FILE_PATH, 'r') as f:
        # Print global attributes (often contains metadata such as env_name)
        print("[Global Attributes]")
        for key, val in f.attrs.items():
            print(f"  {key}: {val}")
        print("-" * 60)

        # Recursively visit all nodes
        f.visititems(print_structure)

except FileNotFoundError:
    print(f"File not found: {FILE_PATH}")
except Exception as e:
    print(f"Error occurred: {e}")
