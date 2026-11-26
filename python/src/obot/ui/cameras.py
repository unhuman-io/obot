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
    realsense:color             an Intel RealSense (D435, ...): the colour image, or the depth
    realsense:depth             image drawn in false colour, near = warm. Add the serial
    realsense:<serial>:depth    number when more than one camera is connected. Read through
                                librealsense where it is installed (pip install
                                pyrealsense2), otherwise through the kernel's video devices
                                as with v4l2: and v4l2depth:, found by what they are, so
                                their numbers do not have to be known.
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


def _roomy(pipe) -> None:
    """A 1 MB pipe instead of the usual 64 kB: a whole picture fits, so the process that
    writes it and the thread that reads it do not have to take turns ten times per frame."""
    import fcntl

    try:
        fcntl.fcntl(pipe.fileno(), getattr(fcntl, "F_SETPIPE_SZ", 1031), 1 << 20)
    except (OSError, ValueError, AttributeError):
        pass  # not a pipe, or not allowed: the default size works too


def depth_mm_to_image(depth_mm: np.ndarray, max_depth_m: float = 4.0) -> np.ndarray:
    """Depth in millimetres as a false colour picture: near is warm, far is cool, no reading
    (0, or the top of the sensor's range) is black.

    Done with OpenCV calls only. They are several times faster than indexing a colour table
    with numpy, and they do not hold the interpreter lock, which the other camera threads
    and the page server need (on a Jetson Nano that is the difference between 9 and 15
    depth frames per second)."""
    import cv2

    limit = int(max_depth_m * 1000)
    near = cv2.convertScaleAbs(cv2.min(depth_mm, limit), alpha=-255.0 / limit, beta=255.0)
    image = cv2.applyColorMap(near, cv2.COLORMAP_TURBO)
    return cv2.bitwise_and(image, image, mask=cv2.inRange(depth_mm, 1, 64999))


def depth_to_image(depth_m: np.ndarray, max_depth_m: float = 4.0) -> np.ndarray:
    """The same for depth in metres."""
    return depth_mm_to_image(np.clip(depth_m * 1000, 0, 65535).astype(np.uint16), max_depth_m)


def find_realsense_video(stream: str, serial: str = "", sysfs: str = "/sys/class/video4linux") -> str:
    """The video device the kernel made for a RealSense's "depth" or "color" stream (of the
    camera with that serial number, if given). The camera is several USB interfaces: depth
    is the first video device of interface 0, colour that of interface 3."""
    import glob
    import os

    def read(path: str) -> str:
        try:
            with open(path) as f:
                return f.read().strip()
        except OSError:
            return ""

    interface = "03" if stream == "color" else "00"
    entries = glob.glob(os.path.join(sysfs, "video*"))
    for entry in sorted(entries, key=lambda e: int(os.path.basename(e)[5:] or 0)):
        usb_interface = os.path.join(entry, "device")
        if ("RealSense" in read(os.path.join(entry, "name")) and read(os.path.join(entry, "index")) == "0"
                and read(os.path.join(usb_interface, "bInterfaceNumber")) == interface
                and (not serial or read(os.path.join(usb_interface, "..", "serial")) == serial)):
            return "/dev/" + os.path.basename(entry)
    which = f" with serial number {serial}" if serial else ""
    raise OSError(f"no RealSense{which} found (no {stream} video device, and pyrealsense2 is not installed)")


class _GstJpeg:
    """JPEG frames from a gst-launch-1.0 pipeline writing to its standard output."""

    def __init__(self, pipeline: str):
        import shlex
        import subprocess

        self._process = subprocess.Popen(
            ["gst-launch-1.0", "-q", *shlex.split(pipeline), "!", "fdsink", "fd=1"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        )
        _roomy(self._process.stdout)
        self._buffer = b""

    def read_jpeg(self) -> bytes:
        """The next complete JPEG (start marker FFD8 to end marker FFD9)."""
        while True:
            start = self._buffer.find(b"\xff\xd8")
            end = self._buffer.find(b"\xff\xd9", start + 2) if start >= 0 else -1
            if end >= 0:
                frame, self._buffer = self._buffer[start : end + 2], self._buffer[end + 2 :]
                return frame
            chunk = self._process.stdout.read1(1 << 20)
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

    Each frame comes with the time it was taken, from the kernel (the start of its
    exposure, on the clock of time.monotonic()): v4l2-ctl prints it with --verbose. When a
    frame arrives here says little about that. It is some 50 ms later on a Jetson Nano, and
    more whenever this process is busy, and to a robot that is turning, 50 ms more or less
    is a few degrees.
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
             f"--set-parm={fps}", "--stream-mmap", "--stream-to=-", "--verbose"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        _roomy(self._process.stdout)
        self._taken_at: dict[int, float] = {}   # frame number -> when the kernel says it was taken
        self.stamps_capture = False             # are the stamps handed out those times?
        threading.Thread(target=self._times, name=f"v4l2-times-{device}", daemon=True).start()
        self._ready = threading.Condition()
        self._frame: np.ndarray | None = None
        self._stamp = 0.0
        self._count = 0
        self._taken = 0
        self._ended = False
        threading.Thread(target=self._drain, name=f"v4l2-{device}", daemon=True).start()

    def _times(self) -> None:
        """What v4l2-ctl says about each frame, as it says it: one "Timestamp: 1234.5678s"
        per frame, in the order of the frames."""
        import re

        number = 0
        for line in getattr(self._process, "stderr", None) or ():
            found = re.search(rb"Timestamp:\s*([0-9]+\.[0-9]+)s", line)
            if found:
                with self._ready:
                    self._taken_at[number] = float(found.group(1))
                    self._taken_at.pop(number - 64, None)
                    self._ready.notify_all()
                number += 1

    def _drain(self) -> None:
        stream = self._process.stdout
        # With --verbose, v4l2-ctl also describes the format it set, as text, on the same
        # pipe as the frames and before the first of them: a block of lines whose last
        # starts with a tab and "Flags". (Other versions may go on with what they did,
        # "VIDIOC_...: ok", up to VIDIOC_STREAMON.) Every byte of it has to go: left in, it
        # shifts all the pictures sideways.
        start = stream.read(6)
        if start in (b"Format", b"VIDIOC"):
            def text_upto(mark: bytes) -> None:
                nonlocal start
                while not start.endswith(mark) and len(start) < 65536:
                    more = stream.read(1)
                    if not more:
                        return
                    start += more

            if start == b"Format":
                text_upto(b"\n\tFlags")
                text_upto(b"\n")
                start = stream.read(6)   # (the next six bytes: more text, or the first frame)
            if start == b"VIDIOC":
                text_upto(b"VIDIOC_STREAMON: ok\n")
                start = b""
        while True:
            data = start + stream.read(self._size - len(start))
            start = b""
            arrived = time.monotonic()
            with self._ready:
                if len(data) < self._size:
                    self._ended = True
                else:
                    # (its time is printed before the frame is written, but another thread reads it)
                    self._ready.wait_for(lambda: self._count in self._taken_at, 0.05)
                    taken = self._taken_at.pop(self._count, None)
                    self.stamps_capture = taken is not None and 0.0 <= arrived - taken < 1.0
                    self._frame = np.frombuffer(data, dtype="<u2").reshape(self._shape)
                    self._stamp = taken if self.stamps_capture else arrived
                    self._count += 1
                self._ready.notify_all()
            if len(data) < self._size:
                return

    def read(self, timeout: float = 3.0) -> tuple[float, np.ndarray]:
        """(when it was taken, or failing that when it arrived: see stamps_capture; the
        newest frame), waiting for one that was not handed out before."""
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
        self._source = None   # what the pictures come from, once it is open
        self._kind = config.source.partition(":")[0]
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
    def stamps_capture(self) -> bool:
        """Are raw()'s stamps the times the pictures were taken (and not when they arrived)?"""
        return bool(getattr(self._source, "stamps_capture", False))

    def raw(self) -> tuple[float, np.ndarray | None]:
        """(its time, newest picture): depth in millimetres (uint16) from a depth
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
        self._kind, _, device = c.source.partition(":")
        if self._kind == "realsense":
            try:
                import pyrealsense2  # noqa: F401
            except ImportError:
                # No librealsense here (there is none for the Jetson Nano's Ubuntu 18.04):
                # read the video devices the kernel makes of the camera instead. Looked up
                # every time the camera is opened, because their numbers change when it is
                # plugged in again.
                parts = device.split(":")
                device = find_realsense_video("depth" if c.is_depth else "color", parts[0] if len(parts) > 1 else "")
                self._kind = "v4l2depth" if c.is_depth else "v4l2"
        if self._kind in ("test", "realsense", "scene"):
            return None
        if self._kind == "v4l2":
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            # MJPG where the camera has it (more frames over USB 2); cameras without it,
            # like the D435's colour device, keep their own format.
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, c.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c.height)
            cap.set(cv2.CAP_PROP_FPS, c.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        elif self._kind == "gst":
            return _GstJpeg(device)
        elif self._kind == "v4l2depth":
            return _V4l2Raw(device, c.width, c.height, c.fps)
        else:
            raise ValueError(f"camera {c.name}: unknown source '{c.source}'")
        if not cap.isOpened():
            raise OSError(f"camera {c.name}: cannot open {device}")
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
        arrival time), or None for a frame that is not to be shown."""
        c = self.config
        if self._kind == "test":
            return None, self._test_frame(start), None, start
        if self._kind == "gst":
            return cap.read_jpeg(), None, None, time.monotonic()   # already encoded
        if self._kind == "v4l2depth":
            stamp, depth = cap.read()
            # Some cameras send more frames than they were asked for (a D435 on USB 2 sends
            # 30 per second whatever is set): drawing those would only cost time.
            if stamp - self._raw_stamp < 0.7 / max(1, c.fps):
                return None
            return None, depth_mm_to_image(depth), depth, stamp
        if self._kind == "scene":
            from . import scene

            depth, colour = scene.render(self._scene_pose(), c.intrinsics())
            return (None, depth_mm_to_image(depth), depth, start) if c.is_depth else (None, colour, colour, start)
        if self._kind == "realsense":
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
                    self._source = cap
                    self.error = None
                start = time.monotonic()
                picture = self._read(cap, start)
                if picture is None:
                    continue
                jpeg, frame, raw, stamp = picture
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
