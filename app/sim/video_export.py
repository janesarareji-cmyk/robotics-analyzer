"""U6.5: playback video export (MP4 / GIF / PNG frames) and screenshots.

Frames are rendered on the GUI thread (one per timer tick, so the window stays alive) and encoded in a
worker (``TaskRunner``) through a bounded queue. Frame ``i`` shows the playback time ``t = i / fps``;
the frame count is ``round(duration * fps)``.

Formats, best first:

-   ``mp4``: needs ``imageio-ffmpeg`` OR an ``ffmpeg`` program on the PATH (H.264 through raw frames on
    stdin). Neither is installed by the app and no dependency is added: the entry only appears when found.
-   ``gif``: Pillow (already installed with PyVista). GIF stores the delay in 10 ms steps, so 30 fps plays
    at 33 fps (3 cs per frame).
-   ``png``: a folder of numbered PNG files, always available (Qt image writer or Pillow).

``grab_fn() -> (H, W, 3) uint8`` and ``seek_fn(t)`` are given by the caller (the main window grabs the
viewport; tests give fakes).
"""
from __future__ import annotations

import math
import queue
import shutil
import subprocess
from pathlib import Path
from typing import Any, Callable

import numpy as np
from PySide6.QtCore import QObject, Qt, QTimer, Signal

DEFAULT_FPS = 30
RESOLUTIONS = {"Viewport size": None, "1280 x 720": (1280, 720), "1920 x 1080": (1920, 1080)}
_ABORT = object()


def frame_count(duration: float, fps: float) -> int:
    """Frames of a ``duration`` s video at ``fps``: duration * fps rounded, at least 1."""
    return max(1, int(round(float(duration) * float(fps))))


def ffmpeg_command() -> str | None:
    """Path of an ffmpeg program (imageio-ffmpeg's or one on the PATH), or None."""
    try:
        import imageio_ffmpeg
        exe = imageio_ffmpeg.get_ffmpeg_exe()
        if exe and Path(exe).exists():
            return str(exe)
    except Exception:
        pass
    return shutil.which("ffmpeg")


def available_formats() -> dict[str, str]:
    """key -> label of the formats that work on this machine, best first."""
    out: dict[str, str] = {}
    if ffmpeg_command():
        out["mp4"] = "MP4 video (.mp4)"
    try:
        import PIL  # noqa: F401
        out["gif"] = "Animated GIF (.gif)"
    except Exception:
        pass
    out["png"] = "PNG frames (a folder)"
    return out


def normalize_path(path: str, fmt: str) -> str:
    """Put the right suffix on ``path`` (a folder name for png)."""
    p = Path(path)
    if fmt == "png":
        return str(p.with_suffix("")) if p.suffix.lower() == ".png" else str(p)
    want = ".mp4" if fmt == "mp4" else ".gif"
    return str(p) if p.suffix.lower() == want else str(p.with_suffix(want))


# ---------------------------------------------------------------------------- encoders
class Encoder:
    """open(path, w, h, fps) / add(frame) / close() -> path / abort()."""

    def open(self, path: str, w: int, h: int, fps: float) -> None:
        raise NotImplementedError

    def add(self, frame: np.ndarray) -> None:
        raise NotImplementedError

    def close(self) -> str:
        raise NotImplementedError

    def abort(self) -> None:
        pass


def _save_png(path: Path, frame: np.ndarray) -> None:
    try:
        from PIL import Image
        Image.fromarray(frame).save(str(path))
    except ImportError:
        from PySide6.QtGui import QImage
        f = np.ascontiguousarray(frame)
        img = QImage(f.data, f.shape[1], f.shape[0], f.strides[0], QImage.Format.Format_RGB888)
        if not img.copy().save(str(path)):
            raise OSError(f"cannot write {path}")


class PngEncoder(Encoder):
    def open(self, path, w, h, fps):
        self.dir = Path(path)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.n = 0

    def add(self, frame):
        self.n += 1
        _save_png(self.dir / f"frame_{self.n:04d}.png", frame)

    def close(self):
        return str(self.dir)

    def abort(self):
        shutil.rmtree(self.dir, ignore_errors=True)


class GifEncoder(Encoder):
    def open(self, path, w, h, fps):
        self.path = Path(path)
        self.fps = float(fps)
        self.frames: list = []

    def add(self, frame):
        from PIL import Image
        self.frames.append(Image.fromarray(frame).convert("P", palette=Image.Palette.ADAPTIVE, colors=256))

    def close(self):
        if not self.frames:
            raise ValueError("no frames")
        dur = max(10, int(round(1000.0 / self.fps / 10.0)) * 10)
        self.frames[0].save(str(self.path), save_all=True, append_images=self.frames[1:], duration=dur,
                            loop=0, optimize=False, disposal=2)
        return str(self.path)


class FfmpegEncoder(Encoder):
    def open(self, path, w, h, fps):
        exe = ffmpeg_command()
        if not exe:
            raise RuntimeError("ffmpeg not found")
        self.path = path
        self.w, self.h = w - (w % 2), h - (h % 2)            # H.264 wants even sizes
        self.proc = subprocess.Popen(
            [exe, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s",
             f"{w}x{h}", "-r", f"{fps}", "-i", "-", "-vf", f"crop={self.w}:{self.h}:0:0",
             "-c:v", "libx264", "-pix_fmt", "yuv420p", path],
            stdin=subprocess.PIPE, stderr=subprocess.PIPE)

    def add(self, frame):
        self.proc.stdin.write(np.ascontiguousarray(frame).tobytes())

    def close(self):
        self.proc.stdin.close()
        err = self.proc.stderr.read().decode("utf-8", "replace")
        if self.proc.wait() != 0:
            raise RuntimeError(f"ffmpeg failed: {err.strip()[-200:]}")
        return self.path

    def abort(self):
        try:
            self.proc.stdin.close()
            self.proc.kill()
        except Exception:
            pass
        Path(self.path).unlink(missing_ok=True)


def make_encoder(fmt: str) -> Encoder:
    if fmt == "mp4":
        return FfmpegEncoder()
    if fmt == "gif":
        return GifEncoder()
    if fmt == "png":
        return PngEncoder()
    raise ValueError(f"unknown format '{fmt}'")


# ---------------------------------------------------------------------------- exporter
class VideoExporter(QObject):
    """Render ``frame_count`` frames on the GUI thread, encode them in a worker."""

    progress = Signal(int, int)          # frames rendered, total
    finished = Signal(str, int)          # output path, frames
    failed = Signal(str)
    cancelled = Signal()
    _done = Signal(object)               # worker result (kind, payload)

    def __init__(self, task_runner: Any, parent: QObject | None = None):
        super().__init__(parent)
        self._runner = task_runner
        self._timer = QTimer(self)
        self._timer.setInterval(0)
        self._timer.timeout.connect(self._tick)
        self._done.connect(self._on_done, Qt.ConnectionType.QueuedConnection)
        self.running = False
        self._q: queue.Queue = queue.Queue(maxsize=6)
        self._abort = False

    def start(self, path: str, fmt: str, fps: float, duration: float, grab_fn: Callable[[], np.ndarray],
              seek_fn: Callable[[float], None]) -> None:
        if self.running:
            raise RuntimeError("an export is already running")
        self._path, self._fmt, self._fps = path, fmt, float(fps)
        self._grab, self._seek = grab_fn, seek_fn
        self._n = frame_count(duration, fps)
        self._i = 0
        self._abort = False
        self._enc = make_encoder(fmt)
        self._q = queue.Queue(maxsize=6)
        self._opened = False
        self.running = True
        self._pending: np.ndarray | None = None
        self._task = None
        self._timer.start()

    def cancel(self) -> None:
        if not self.running:
            return
        self._abort = True

    # ------------------------------------------------------------------ GUI thread
    def _start_worker(self, w: int, h: int) -> None:
        enc, q, path, fps = self._enc, self._q, self._path, self._fps

        def work():
            try:
                enc.open(path, w, h, fps)
                while True:
                    item = q.get()
                    if item is None:
                        return ("done", enc.close())
                    if item is _ABORT:
                        enc.abort()
                        return ("cancelled", None)
                    enc.add(item)
            except Exception as e:
                try:
                    enc.abort()
                except Exception:
                    pass
                while not q.empty():            # unblock the GUI thread's put
                    try:
                        q.get_nowait()
                    except queue.Empty:
                        break
                return ("failed", f"{type(e).__name__}: {e}")

        from app.task_runner import FunctionTask
        task = FunctionTask(work)
        task.signals.result.connect(self._done, Qt.ConnectionType.QueuedConnection)
        self._task = task
        self._runner.submit(task)

    def _tick(self) -> None:
        try:
            if self._abort:
                self._timer.stop()
                self._finish_abort()
                return
            if self._pending is None:
                if self._i >= self._n:
                    self._timer.stop()
                    self._put(None)
                    return
                self._seek(self._i / self._fps)
                frame = np.ascontiguousarray(self._grab())
                if frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
                    raise ValueError("frames must be (H, W, 3) uint8")
                if not self._opened:
                    self._start_worker(frame.shape[1], frame.shape[0])
                    self._opened = True
                self._pending = frame
            try:
                self._q.put_nowait(self._pending)
            except queue.Full:
                return                          # the worker is behind: try again next tick
            self._pending = None
            self._i += 1
            self.progress.emit(self._i, self._n)
        except Exception as e:                  # render failed
            self._timer.stop()
            self.running = False
            if self._opened:
                self._put(_ABORT)
            self.failed.emit(f"{type(e).__name__}: {e}")

    def _put(self, item: Any) -> None:
        if not self._opened:
            return
        while True:
            try:
                self._q.put(item, timeout=0.05)
                return
            except queue.Full:
                if self._task is None:
                    return

    def _finish_abort(self) -> None:
        if self._opened:
            while not self._q.empty():
                try:
                    self._q.get_nowait()
                except queue.Empty:
                    break
            self._put(_ABORT)
        else:
            self.running = False
            self.cancelled.emit()

    def _on_done(self, payload: Any) -> None:
        kind, value = payload
        self._timer.stop()
        self.running = False
        if kind == "done":
            self.finished.emit(value, self._n)
        elif kind == "cancelled":
            self.cancelled.emit()
        else:
            self.failed.emit(str(value))


def image_to_array(img: Any) -> np.ndarray:
    """QImage -> (H, W, 3) uint8 RGB."""
    from PySide6.QtGui import QImage
    img = img.convertToFormat(QImage.Format.Format_RGB888)
    w, h, bpl = img.width(), img.height(), img.bytesPerLine()
    buf = np.frombuffer(img.constBits(), dtype=np.uint8, count=bpl * h).reshape(h, bpl)
    return np.ascontiguousarray(buf[:, :w * 3].reshape(h, w, 3))
