import os
import time
from datetime import datetime
from collections import defaultdict


class _Profiler:
    """Lightweight per-phase timer. Accumulates elapsed time per label and
    prints a rolling average every `report_every` frames. Zero cost when
    disabled (the stop() early-return), so it can stay wired into the hot loop.

    Reports go to stdout AND are appended to a log file (default
    profile_logs/profile_<timestamp>.log) so runs can be reviewed afterwards.

    Usage:
        from core.profiler import profiler
        profiler.enabled = True            # turn on (e.g. from --profile)
        t = profiler.start()
        ...work...
        profiler.stop("label", t)
        profiler.tick_frame()              # call once per main-loop iteration
    """

    def __init__(self):
        self.enabled = False
        self.report_every = 60
        self.sums = defaultdict(float)
        self.counts = defaultdict(int)
        self.frame_count = 0
        self._frame_t0 = None
        self.log_dir = "profile_logs"
        self._log_path = None

    @property
    def log_path(self):
        """Lazily create the timestamped log file on first report."""
        if self._log_path is None:
            os.makedirs(self.log_dir, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._log_path = os.path.join(self.log_dir, f"profile_{stamp}.log")
        return self._log_path

    def _write_log(self, text):
        try:
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(text + "\n")
        except Exception as e:
            print(f"[Profiler] Failed to write log: {e}")

    def start(self):
        # Cheap even when disabled; caller passes the returned value back to stop().
        return time.perf_counter()

    def stop(self, label, t0):
        if not self.enabled or t0 is None:
            return
        self.sums[label] += (time.perf_counter() - t0) * 1000.0
        self.counts[label] += 1

    def tick_frame(self):
        """Call once per main-loop iteration. Tracks whole-frame wall time and
        periodically prints the rolling averages + effective FPS."""
        if not self.enabled:
            return
        now = time.perf_counter()
        if self._frame_t0 is not None:
            self.sums["_frame_total"] += (now - self._frame_t0) * 1000.0
            self.counts["_frame_total"] += 1
        self._frame_t0 = now

        self.frame_count += 1
        if self.frame_count % self.report_every == 0:
            self.report()

    def report(self):
        frame_n = self.counts.get("_frame_total", 0)
        frame_avg = self.sums.get("_frame_total", 0.0) / frame_n if frame_n else 0.0
        fps = 1000.0 / frame_avg if frame_avg > 0 else 0.0

        ts = datetime.now().strftime("%H:%M:%S")
        lines = [f"[{ts}] rolling avg over last {frame_n} frames  "
                 f"(frame={frame_avg:.1f} ms -> {fps:.1f} FPS):"]
        for label in sorted(self.sums):
            if label == "_frame_total":
                continue
            n = self.counts[label]
            avg = self.sums[label] / n if n else 0.0
            pct = (self.sums[label] / self.sums["_frame_total"] * 100.0
                   if self.sums.get("_frame_total") else 0.0)
            lines.append(f"    {label:34s} {avg:8.2f} ms  ({pct:5.1f}%  n={n})")
        block = "\n".join(lines)
        print(block)
        self._write_log(block)

        self.sums.clear()
        self.counts.clear()


profiler = _Profiler()
