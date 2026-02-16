#  # Visualize all camera datasets under this obs group

import h5py
import matplotlib.pyplot as plt
import numpy as np
import os

# HDF5 file path (Change this to your dataset path if needed)
FILE_PATH = 'datasets/dataset.hdf5' 

def visualize_hdf5_cameras(file_path=FILE_PATH, demo_idx=0, frame_idx=0):
    if not os.path.exists(file_path):
        print(f"No file path: {file_path}")
        return

    with h5py.File(file_path, 'r') as f:
        demo_path = f"data/demo_{demo_idx}/obs"
        if demo_path not in f:
            print(f"No demo path: {demo_path}")
            return
        
        obs = f[demo_path]
        camera_data = {}
        
        def find_datasets(name, obj):
            if isinstance(obj, h5py.Dataset):
                if "rgb" in name or "depth" in name:
                    camera_data[name] = obj[frame_idx]

        obs.visititems(find_datasets)

        if not camera_data:
            print("No RGBD Dataset。")
            return

        # RGB & Depth
        rgb_keys = sorted([k for k in camera_data.keys() if "rgb" in k])
        depth_keys = sorted([k for k in camera_data.keys() if "depth" in k])
        
        num_cams = len(rgb_keys)
        rows = 2 if depth_keys else 1
        
        fig, axes = plt.subplots(rows, num_cams, figsize=(5 * num_cams, 5 * rows))
        if num_cams == 1:
            axes = np.atleast_2d(axes).T
        
        # RGB
        for i, key in enumerate(rgb_keys):
            ax = axes[0, i] if rows > 1 else axes[i]
            ax.imshow(camera_data[key])
            ax.set_title(f"RGB: {key.split('/')[-2]}") # Camera Name
            ax.axis('off')

        # Depth
        for i, key in enumerate(depth_keys):
            ax = axes[1, i]
            d_data = camera_data[key].astype(np.float32)
            d_min, d_max = np.min(d_data), np.max(d_data)
            d_viz = (d_data - d_min) / (d_max - d_min + 1e-5)
            ax.imshow(d_viz, cmap='magma')
            ax.set_title(f"Depth: {key.split('/')[-2]}\n{int(d_min)}-{int(d_max)}mm")
            ax.axis('off')

        plt.tight_layout()
        plt.savefig("camera_view.png")
        print(f"Save to camera_view.png, total view : {num_cams}")

if __name__ == "__main__":
    visualize_hdf5_cameras()