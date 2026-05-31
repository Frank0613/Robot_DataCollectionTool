import h5py
import numpy as np
import os
from matplotlib import cm


def compose_camera_grid(cam_data, include_depth=True):
    """Build one composite image from a {"<cam>/rgb": HxWx3 uint8,
    "<cam>/depth": HxW uint16} dict — i.e. the output of
    CameraManager.get_all_camera_data(). With include_depth, a colorized depth
    row is stacked below the RGB row. Returns an RGB uint8 array, or None if
    there are no RGB cameras.
    """
    rgb_keys = sorted(k for k in cam_data if k.endswith("/rgb"))
    depth_keys = sorted(k for k in cam_data if k.endswith("/depth"))
    if not rgb_keys:
        return None

    rgb_strip = np.hstack([cam_data[k].astype(np.uint8) for k in rgb_keys])
    if not include_depth or not depth_keys:
        return rgb_strip

    magma = cm.get_cmap('magma')
    depth_row = []
    for k in depth_keys:
        d = cam_data[k].astype(np.float32)
        d_min, d_max = float(d.min()), float(d.max())
        d_norm = (d - d_min) / (d_max - d_min + 1e-5)
        depth_row.append((magma(d_norm)[..., :3] * 255).astype(np.uint8))
    depth_strip = np.hstack(depth_row)
    return np.vstack([rgb_strip, depth_strip])


def visualize_hdf5_demo_as_video(file_path, demo_idx=0, output_path=None, fps=30):
    """Render every frame of a demo as a side-by-side RGB + depth grid and
    stitch them into a single MP4. Same layout as tools/hdf5_checker.py but
    iterated across all frames of one demo.
    """
    if not os.path.exists(file_path):
        print(f"[Error] File not found: {file_path}")
        return

    with h5py.File(file_path, 'r') as f:
        total_demos = int(f['data'].attrs.get('total', 0))
        if demo_idx >= total_demos:
            print(f"[Error] Demo index {demo_idx} out of range. File has {total_demos} demos.")
            return

        demo_path = f"data/demo_{demo_idx}/obs"
        if demo_path not in f:
            print(f"[Error] In demo_{demo_idx}, obs group not found.")
            return

        obs = f[demo_path]
        num_frames = int(obs['actions'].shape[0])

        # Collect camera datasets (full slice — we'll index per-frame below)
        rgb_datasets = {}
        depth_datasets = {}

        def find_datasets(name, obj):
            if isinstance(obj, h5py.Dataset):
                if "/rgb" in f"/{name}":
                    rgb_datasets[name] = obj
                elif "/depth" in f"/{name}":
                    depth_datasets[name] = obj

        obs.visititems(find_datasets)
        if not rgb_datasets:
            print("[Error] No RGB cameras in this demo.")
            return

        rgb_keys = sorted(rgb_datasets.keys())
        depth_keys = sorted(depth_datasets.keys())

        # Frame dimensions from the first RGB sample
        sample = rgb_datasets[rgb_keys[0]][0]
        h, w = sample.shape[:2]

        cam_label_names = [k.split('/')[0] for k in rgb_keys]
        print(f"[Video] Demo {demo_idx} | {num_frames} frames | "
              f"{len(rgb_keys)} RGB cams, {len(depth_keys)} depth cams")
        print(f"[Video] Camera order (left -> right): {cam_label_names}")

        magma = cm.get_cmap('magma')

        # Pre-load all frames into memory for speed (256x256 x 5 cams x N frames is OK)
        # If memory is a concern for huge demos, switch to streamed read.
        rgb_arrays = {k: rgb_datasets[k][:num_frames] for k in rgb_keys}
        depth_arrays = {k: depth_datasets[k][:num_frames] for k in depth_keys}

        frames = []
        for fi in range(num_frames):
            rgb_row = [rgb_arrays[k][fi].astype(np.uint8) for k in rgb_keys]
            rgb_strip = np.hstack(rgb_row) if rgb_row else None

            if depth_keys:
                depth_row = []
                for k in depth_keys:
                    d = depth_arrays[k][fi].astype(np.float32)
                    d_min, d_max = float(d.min()), float(d.max())
                    d_norm = (d - d_min) / (d_max - d_min + 1e-5)
                    d_rgb = (magma(d_norm)[..., :3] * 255).astype(np.uint8)
                    depth_row.append(d_rgb)
                depth_strip = np.hstack(depth_row)
                composite = np.vstack([rgb_strip, depth_strip])
            else:
                composite = rgb_strip

            frames.append(composite)

        # Default output: ./check_<dataset>_demo_<idx>.mp4
        if output_path is None:
            stem = os.path.splitext(os.path.basename(file_path))[0]
            output_path = f"check_{stem}_demo_{demo_idx}.mp4"

        if _write_mp4(frames, output_path, fps):
            return

        # Last-resort PNG sequence so the data isn't lost
        print("[Fallback] Writing PNG sequence instead")
        png_dir = output_path.replace('.mp4', '_frames')
        os.makedirs(png_dir, exist_ok=True)
        for i, img in enumerate(frames):
            try:
                import imageio.v2 as imageio
                imageio.imwrite(os.path.join(png_dir, f"frame_{i:04d}.png"), img)
            except Exception:
                from matplotlib import image as mpimg
                mpimg.imsave(os.path.join(png_dir, f"frame_{i:04d}.png"), img)
        print(f"[Fallback] Wrote {len(frames)} PNGs to {png_dir}/")


def _write_mp4(frames, output_path, fps):
    """Try multiple MP4 backends in order of preference. Returns True on success."""
    # 1) imageio + ffmpeg (highest quality, needs `imageio[ffmpeg]`)
    try:
        import imageio.v2 as imageio
        imageio.mimsave(output_path, frames, fps=fps, codec='libx264', quality=8)
        print(f"[Video] (imageio/ffmpeg) Saved {len(frames)} frames @ {fps}fps -> {output_path}")
        return True
    except Exception as e:
        print(f"[Video] imageio backend unavailable: {e}")

    # 2) cv2 mp4v writer — opencv is usually bundled with Isaac Sim
    try:
        import cv2
        h, w = frames[0].shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(output_path, fourcc, float(fps), (w, h))
        if not writer.isOpened():
            print("[Video] cv2 VideoWriter could not be opened (codec mismatch?)")
            return False
        for img in frames:
            # frames are RGB; cv2 expects BGR
            writer.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        writer.release()
        print(f"[Video] (cv2/mp4v) Saved {len(frames)} frames @ {fps}fps -> {output_path}")
        return True
    except Exception as e:
        print(f"[Video] cv2 backend unavailable: {e}")

    return False


if __name__ == "__main__":
    visualize_hdf5_demo_as_video('datasets/dataset.hdf5', demo_idx=0)
