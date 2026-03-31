import h5py
import matplotlib.pyplot as plt
import numpy as np
import os

def visualize_hdf5_cameras_by_path(file_path, demo_idx=0, frame_idx=0):
    """Function for main.py to call"""
    if not os.path.exists(file_path):
        print(f"[Error] File not found: {file_path}")
        return

    with h5py.File(file_path, 'r') as f:
        total_demos = f['data'].attrs.get('total', 0)
        if demo_idx >= total_demos:
            print(f"[Error] Demo index {demo_idx} out of range. This file only has {total_demos} demos.")
            return

        demo_path = f"data/demo_{demo_idx}/obs"
        if demo_path not in f:
            print(f"[Error] In demo_{demo_idx}, obs group not found.")
            return
        
        obs = f[demo_path]
        
        max_frames = obs['actions'].shape[0]
        if frame_idx >= max_frames:
            print(f"[Error] Frame index {frame_idx} out of range. This Demo only has {max_frames} frames (0-{max_frames-1}).")
            return
        
        camera_data = {}
        def find_datasets(name, obj):
            if isinstance(obj, h5py.Dataset):
                if "rgb" in name or "depth" in name:
                    camera_data[name] = obj[frame_idx]

        obs.visititems(find_datasets)
        if not camera_data:
            print("No RGBD data found.")
            return

        rgb_keys = sorted([k for k in camera_data.keys() if "rgb" in k])
        depth_keys = sorted([k for k in camera_data.keys() if "depth" in k])
        num_cams = len(rgb_keys)
        rows = 2 if depth_keys else 1
        
        fig, axes = plt.subplots(rows, num_cams, figsize=(5 * num_cams, 5 * rows))
        if num_cams == 1:
            axes = np.atleast_2d(axes).T
        
        for i, key in enumerate(rgb_keys):
            ax = axes[0, i] if rows > 1 else axes[i]
            ax.imshow(camera_data[key])
            ax.set_title(f"RGB: {key.split('/')[-2]}")
            ax.axis('off')

        for i, key in enumerate(depth_keys):
            ax = axes[1, i]
            d_data = camera_data[key].astype(np.float32)
            d_min, d_max = np.min(d_data), np.max(d_data)
            d_viz = (d_data - d_min) / (d_max - d_min + 1e-5)
            ax.imshow(d_viz, cmap='magma')
            ax.set_title(f"Depth: {key.split('/')[-2]}\n{int(d_min)}-{int(d_max)}mm")
            ax.axis('off')

        plt.tight_layout()
        save_name = f"check_{os.path.basename(file_path)}_frame_{frame_idx}.png"
        plt.savefig(save_name)
        print(f"Visualization results have been saved to {save_name}, Total of {num_cams} views")

if __name__ == "__main__":
    visualize_hdf5_cameras_by_path('datasets/dataset.hdf5')