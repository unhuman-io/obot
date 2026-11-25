"""Camera capture for the operator UI: each camera runs in its own thread and keeps only
its newest frame as a JPEG, so a slow viewer never builds up a queue. Depth and colour
cameras also keep the newest picture itself (Camera.raw), which the 3D world view is built
from.

Sources:
    test                        a moving test pattern (no hardware)
    sim:<link>:color            what a camera on that URDF link would see of the robot model
    sim:<link>:depth            and a few props, drawn by the operator page itself (needs
                                [model]); for trying the page without cameras
    v4l2:/dev/video0            a V4L2 device: USB cameras, RealSense colour streams
    v4l2depth:/dev/video1       a 16 bit depth stream in millimetres (V4L2 format Z16), drawn
                                in false colour. This is how a RealSense D435 shows up on
                                kernels that know its formats (the Jetson L4T kernels do):
                                depth, infrared and colour as separate video devices, with
                                nothing from Intel to install. Needs v4l2-ctl (v4l-utils).
    scene:depth                 a small made-up room (floor, boxes, a wall) seen from where
    scene:color                 the camera is mounted, drawn here: for trying the 3D world
                                view without a depth camera
    realsense:color             an Intel RealSense (D435, ...) through librealsense
    realsense:depth             (pip install pyrealsense2): the colour image, or the depth
    realsense:<serial>:depth    image drawn in false colour, near = warm. Add the serial
                                number when more than one camera is connected.
    gst:<pipeline>              a GStreamer pipeline that ends in a JPEG encoder, run with
                                gst-launch-1.0. A CSI camera on a Jetson Nano:
                                gst:nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1280,
                                height=720,framerate=15/1 ! nvjpegenc
"""

from __future__ import annotations

import threading
import time

import numpy as np

from ..config import CameraConfig


class _RealSense:
    """One librealsense pipeline per device, shared by the cameras that show its streams."""

    _devices: dict[str, _RealSense] = {}
    _guard = threading.Lock()

    @classmethod
    def get(cls, serial: str, width: int, height: int, fps: int) -> _RealSense:
        with cls._guard:
            if serial not in cls._devices:
                cls._devices[serial] = cls(serial, width, height, fps)
            return cls._devices[serial]

    def __init__(self, serial: str, width: int, height: int, fps: int):
        import pyrealsense2 as rs

        self._rs = rs
        self._lock = threading.Lock()
        self._frames = None
        self._stamp = 0.0
        config = rs.config()
        if serial:
            config.enable_device(serial)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self._pipeline = rs.pipeline()
        self._scale = self._pipeline.start(config).get_device().first_depth_sensor().get_depth_scale()

    def read(self, stream: str, max_depth_m: float = 4.0):
        """The newest colour image, or the newest depth image in metres."""
        with self._lock:
            if time.monotonic() - self._stamp > 0.005:
                self._frames = self._pipeline.wait_for_frames(2000)
                self._stamp = time.monotonic()
            frames = self._frames
        if stream == "color":
            return np.asanyarray(frames.get_color_frame().get_data())
        depth = np.asanyarray(frames.get_depth_frame().get_data()).astype(np.float32) * self._scale
        return np.clip(depth, 0, max_depth_m)


_depth_colours: dict[float, np.ndarray] = {}


def depth_mm_to_image(depth_mm: np.ndarray, max_depth_m: float = 4.0) -> np.ndarray:
    """Depth in millimetres as a false colour picture: near is warm, far is cool, no reading
    (0, or the top of the sensor's range) is black."""
    table = _depth_colours.get(max_depth_m)
    if table is None:
        import cv2

        mm = np.arange(65536, dtype=np.float32)
        near = (255 * (1.0 - np.clip(mm / (max_depth_m * 1000), 0, 1))).astype(np.uint8)
        table = cv2.applyColorMap(near.reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)
        table[0] = 0
        table[65000:] = 0
        _depth_colours[max_depth_m] = table
    return table[depth_mm]


def depth_to_image(depth_m: np.ndarray, max_depth_m: float = 4.0) -> np.ndarray:
    """The same for depth in metres."""
    return depth_mm_to_image(np.clip(depth_m * 1000, 0, 65535).astype(np.uint16), max_depth_m)


class _GstJpeg:
    """JPEG frames from a gst-launch-1.0 pipeline writing to its standard output."""

    def __init__(self, pipeline: str):
        import shlex
        import subprocess

        self._process = subprocess.Popen(
            ["gst-launch-1.0", "-q", *shlex.split(pipeline), "!", "fdsink", "fd=1"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        )
        self._buffer = b""

    def read_jpeg(self) -> bytes:
        """The next complete JPEG (start marker FFD8 to end marker FFD9)."""
        while True:
            start = self._buffer.find(b"\xff\xd8")
            end = self._buffer.find(b"\xff\xd9", start + 2) if start >= 0 else -1
            if end >= 0:
                frame, self._buffer = self._buffer[start : end + 2], self._buffer[end + 2 :]
                return frame
            chunk = self._process.stdout.read1(1 << 16)
            if not chunk:
                raise OSError("gst-launch-1.0 ended")
            self._buffer += chunk

    def release(self) -> None:
        self._process.kill()
        self._process.wait()


class _V4l2Raw:
    """16 bit frames from v4l2-ctl streaming to its standard output.

    A thread takes every frame off the pipe as it arrives and keeps the newest. Reading
    them one by one at the pace of whoever draws them would leave the picture several
    frames behind as soon as drawing is slower than the camera: the frames in between wait
    in the driver's buffers and the pipe.
    """

    def __init__(self, device: str, width: int, height: int, fps: int, pixelformat: str = "Z16 "):
        import shutil
        import subprocess

        self._shape = (height, width)
        self._size = width * height * 2
        # Without stdbuf the end of each frame can sit in v4l2-ctl's output buffer until
        # the next frame pushes it out.
        unbuffered = ["stdbuf", "-o0"] if shutil.which("stdbuf") else []
        self._process = subprocess.Popen(
            [*unbuffered, "v4l2-ctl", "-d", device,
             f"--set-fmt-video=width={width},height={height},pixelformat={pixelformat}",
             f"--set-parm={fps}", "--stream-mmap", "--stream-to=-"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        )
        self._ready = threading.Condition()
        self._frame: np.ndarray | None = None
        self._stamp = 0.0
        self._count = 0
        self._taken = 0
        self._ended = False
        threading.Thread(target=self._drain, name=f"v4l2-{device}", daemon=True).start()

    def _drain(self) -> None:
        stream = self._process.stdout
        while True:
            data = stream.read(self._size)
            with self._ready:
                if len(data) < self._size:
                    self._ended = True
                else:
                    self._frame = np.frombuffer(data, dtype="<u2").reshape(self._shape)
                    self._stamp = time.monotonic()
                    self._count += 1
                self._ready.notify_all()
            if len(data) < self._size:
                return

    def read(self, timeout: float = 3.0) -> tuple[float, np.ndarray]:
        """(arrival time, newest frame), waiting for one that was not handed out before."""
        with self._ready:
            if not self._ready.wait_for(lambda: self._ended or self._count != self._taken, timeout):
                raise OSError("no frame from v4l2-ctl")
            if self._count == self._taken:
                raise OSError("v4l2-ctl ended (is the device in use, or v4l-utils missing?)")
            self._taken = self._count
            return self._stamp, self._frame

    def release(self) -> None:
        self._process.kill()
        self._process.wait()


class Camera:
    def __init__(self, config: CameraConfig, pose=None):
        import cv2  # imported here so the rest of the package works without OpenCV

        self._cv2 = cv2
        self.config = config
        self.name = config.name
        # For scene: sources: a function returning where the camera's optical frame is in
        # the world (4 x 4). The 3D world view sets it; without it the base is at the origin.
        self.pose = pose
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._sequence = 0
        self._stamp = 0.0
        self._raw: np.ndarray | None = None
        self._raw_stamp = 0.0
        self.fps = 0.0  # measured
        self.error: str | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"camera-{config.name}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)

    def latest(self) -> tuple[int, bytes | None]:
        """(sequence number, JPEG bytes). The sequence changes with every new frame."""
        with self._lock:
            return self._sequence, self._jpeg

    def raw(self) -> tuple[float, np.ndarray | None]:
        """(time it arrived, newest picture): depth in millimetres (uint16) from a depth
        camera, BGR from a colour camera, None from sources that only deliver JPEG. The
        array is not to be written to."""
        with self._lock:
            return self._raw_stamp, self._raw

    @property
    def age(self) -> float:
        return time.monotonic() - self._stamp if self._stamp else float("inf")

    def _open(self):
        cv2 = self._cv2
        c = self.config
        if c.source == "test" or c.source.startswith(("realsense", "scene:")):
            return None
        if c.source.startswith("v4l2:"):
            cap = cv2.VideoCapture(c.source[5:], cv2.CAP_V4L2)
            # MJPG where the camera has it (more frames over USB 2); cameras without it,
            # like the D435's colour device, keep their own format.
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, c.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c.height)
            cap.set(cv2.CAP_PROP_FPS, c.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        elif c.source.startswith("gst:"):
            return _GstJpeg(c.source[4:])
        elif c.source.startswith("v4l2depth:"):
            return _V4l2Raw(c.source[10:], c.width, c.height, c.fps)
        else:
            raise ValueError(f"camera {c.name}: unknown source '{c.source}'")
        if not cap.isOpened():
            raise OSError(f"camera {c.name}: cannot open {c.source}")
        return cap

    def _test_frame(self, t: float) -> np.ndarray:
        c = self.config
        x = np.linspace(0, 255, c.width, dtype=np.uint8)
        frame = np.empty((c.height, c.width, 3), np.uint8)
        frame[..., 0] = x
        frame[..., 1] = np.linspace(0, 255, c.height, dtype=np.uint8)[:, None]
        frame[..., 2] = 80
        bar = int((t * 0.25 % 1.0) * c.width)
        frame[:, max(0, bar - 4) : bar + 4] = 255
        self._cv2.putText(frame, f"{c.name} {time.strftime('%H:%M:%S')}", (10, 30),
                          self._cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        return frame

    def _scene_pose(self) -> np.ndarray:
        from .. import world

        if self.pose is not None:
            return self.pose()
        return world.transform(self.config.xyz, self.config.rpy) @ world.OPTICAL

    def _read(self, cap, start: float):
        """One frame: (JPEG bytes or None, picture to encode or None, raw picture or None,
        arrival time)."""
        c = self.config
        if c.source == "test":
            return None, self._test_frame(start), None, start
        if c.source.startswith("gst:"):
            return cap.read_jpeg(), None, None, time.monotonic()   # already encoded
        if c.source.startswith("v4l2depth:"):
            stamp, depth = cap.read()
            return None, depth_mm_to_image(depth), depth, stamp
        if c.source.startswith("scene:"):
            from . import scene

            depth, colour = scene.render(self._scene_pose(), c.intrinsics())
            return (None, depth_mm_to_image(depth), depth, start) if c.is_depth else (None, colour, colour, start)
        if c.source.startswith("realsense"):
            parts = c.source.split(":")[1:]
            serial = parts[0] if parts and parts[0] not in ("color", "depth") else ""
            data = _RealSense.get(serial, c.width, c.height, c.fps).read("depth" if c.is_depth else "color")
            self.error = None
            if c.is_depth:
                depth = np.clip(data * 1000, 0, 65535).astype(np.uint16)
                return None, depth_mm_to_image(depth), depth, time.monotonic()
            return None, data, data, time.monotonic()
        ok, frame = cap.read()
        if not ok:
            raise OSError(f"camera {c.name}: read failed")
        return None, frame, frame, time.monotonic()

    def _run(self) -> None:
        cv2 = self._cv2
        c = self.config
        period = 1.0 / max(1, c.fps)
        paced = c.source == "test" or c.source.startswith("scene:")   # the others pace themselves
        cap = None
        while not self._stop.is_set():
            try:
                if cap is None:
                    cap = self._open()
                    self.error = None
                start = time.monotonic()
                jpeg, frame, raw, stamp = self._read(cap, start)
                if jpeg is None:
                    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, c.quality])
                    jpeg = encoded.tobytes() if ok else None
                if jpeg is not None:
                    now = time.monotonic()
                    with self._lock:
                        if self._stamp:
                            rate = 1.0 / max(1e-3, now - self._stamp)
                            self.fps = rate if not self.fps else 0.9 * self.fps + 0.1 * rate
                        self._jpeg = jpeg
                        self._sequence += 1
                        self._stamp = now
                        if raw is not None:
                            self._raw, self._raw_stamp = raw, stamp
                if paced:
                    self._stop.wait(max(0.0, period - (time.monotonic() - start)))
            except Exception as e:  # keep trying: cameras get unplugged and come back
                self.error = str(e)
                if cap is not None:
                    cap.release()
                    cap = None
                self._stop.wait(1.0)
        if cap is not None:
            cap.release()
