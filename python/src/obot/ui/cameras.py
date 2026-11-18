"""Camera capture for the operator UI: each camera runs in its own thread and keeps only
its newest frame as a JPEG, so a slow viewer never builds up a queue.

Sources:
    test                        a moving test pattern (no hardware)
    v4l2:/dev/video0            a V4L2 device: USB cameras, RealSense colour streams
    realsense:color             an Intel RealSense (D435, ...) through librealsense
    realsense:depth             (pip install pyrealsense2): the colour image, or the depth
    realsense:<serial>:depth    image drawn in false colour, near = warm. Add the serial
                                number when more than one camera is connected.
    gst:<pipeline>              a GStreamer pipeline ending in appsink, e.g. a CSI camera on
                                a Jetson: gst:nvarguscamerasrc ! nvvidconv ! video/x-raw,
                                format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink
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


def depth_to_image(depth_m: np.ndarray, max_depth_m: float = 4.0) -> np.ndarray:
    """Depth in metres as a false colour picture: near is warm, far is cool, no reading is black."""
    import cv2

    near = (255 * (1.0 - np.clip(depth_m / max_depth_m, 0, 1))).astype(np.uint8)
    image = cv2.applyColorMap(near, cv2.COLORMAP_TURBO)
    image[depth_m <= 0] = 0
    return image


class Camera:
    def __init__(self, config: CameraConfig):
        import cv2  # imported here so the rest of the package works without OpenCV

        self._cv2 = cv2
        self.config = config
        self.name = config.name
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._sequence = 0
        self._stamp = 0.0
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

    @property
    def age(self) -> float:
        return time.monotonic() - self._stamp if self._stamp else float("inf")

    def _open(self):
        cv2 = self._cv2
        c = self.config
        if c.source == "test" or c.source.startswith("realsense"):
            return None
        if c.source.startswith("v4l2:"):
            cap = cv2.VideoCapture(c.source[5:], cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, c.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c.height)
            cap.set(cv2.CAP_PROP_FPS, c.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        elif c.source.startswith("gst:"):
            cap = cv2.VideoCapture(c.source[4:], cv2.CAP_GSTREAMER)
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

    def _run(self) -> None:
        cv2 = self._cv2
        c = self.config
        period = 1.0 / max(1, c.fps)
        cap = None
        while not self._stop.is_set():
            try:
                if cap is None and c.source != "test" and not c.source.startswith("realsense"):
                    cap = self._open()
                    self.error = None
                start = time.monotonic()
                if c.source == "test":
                    frame = self._test_frame(start)
                elif c.source.startswith("realsense"):
                    parts = c.source.split(":")[1:]
                    stream = parts[-1] if parts and parts[-1] in ("color", "depth") else "color"
                    serial = parts[0] if parts and parts[0] not in ("color", "depth") else ""
                    data = _RealSense.get(serial, c.width, c.height, c.fps).read(stream)
                    frame = depth_to_image(data) if stream == "depth" else data
                    self.error = None
                else:
                    ok, frame = cap.read()
                    if not ok:
                        raise OSError(f"camera {c.name}: read failed")
                ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, c.quality])
                if ok:
                    with self._lock:
                        self._jpeg = encoded.tobytes()
                        self._sequence += 1
                        self._stamp = time.monotonic()
                if c.source == "test":
                    self._stop.wait(max(0.0, period - (time.monotonic() - start)))
            except Exception as e:  # keep trying: cameras get unplugged and come back
                self.error = str(e)
                if cap is not None:
                    cap.release()
                    cap = None
                self._stop.wait(1.0)
        if cap is not None:
            cap.release()
