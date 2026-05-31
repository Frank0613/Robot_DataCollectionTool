import os

from tools.hdf5_video import compose_camera_grid, _write_mp4


class TrialVideoRecorder:
    """Buffers composited camera frames for one eval trial and writes them to an
    MP4 when the trial ends. Used by `--eval --save_video`.

    Lifecycle (driven from the main loop, mirrors InferenceStats):
      start_trial()      -> clear the frame buffer for a new trial
      capture(cam_data)  -> append one composite frame (call after world.step)
      save(outcome)      -> write buffer to eval_videos/eval_trial_<N>_<outcome>.mp4
      discard()          -> drop the buffer (redo / interrupted trial)
    """

    def __init__(self, out_dir="eval_videos", fps=30):
        self.out_dir = out_dir
        self.fps = fps
        self._frames = []
        self._counter = 0  # number of saved trials so far (for file naming)
        os.makedirs(out_dir, exist_ok=True)

    def start_trial(self):
        self._frames = []

    def capture(self, cam_data):
        # RGB only — depth is not useful for eyeballing eval rollouts and
        # halves memory + skips the per-frame colormap cost.
        frame = compose_camera_grid(cam_data, include_depth=False)
        if frame is not None:
            self._frames.append(frame)

    def save(self, outcome):
        """Write the buffered frames to an MP4 named by trial index + outcome."""
        if not self._frames:
            self._frames = []
            return None
        self._counter += 1
        path = os.path.join(self.out_dir, f"eval_trial_{self._counter}_{outcome}.mp4")
        if not _write_mp4(self._frames, path, self.fps):
            print(f"[Eval][Video] Failed to write {path} (no MP4 backend available)")
            path = None
        self._frames = []
        return path

    def discard(self):
        self._frames = []
