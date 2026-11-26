"""What is in a picture, said by a network on the GPU: obot-segd, and what is done with it.

obot-segd (rt/gpu) runs a segmentation network with TensorRT. It is a program of its own
for the reason obot-navd is, and for one more: NVIDIA's Python bindings on a Jetson are for
the Python of its OS. This module starts it and talks to it (Segd), and keeps what it says
about the things around the robot where the map has them (Labels).

The network that this was made with is the FCN-ResNet18 of the jetson-inference project,
trained on SUN RGB-D (rooms: floor, wall, furniture, clothes, person and so on, 21 classes).
It looks at a 512 x 400 picture and says one class for each 32 x 32 pixels of it, 16 x 13 in
all: enough to know a chair from a wall, not to find a sock.
"""

from __future__ import annotations

import contextlib
import os
import shutil
import struct
import subprocess
import threading

import numpy as np

SETUP, PICTURE = 1, 2


def find() -> str | None:
    """The obot-segd to use: $OBOT_SEGD, or one on the PATH."""
    given = os.environ.get("OBOT_SEGD")
    if given is not None:
        return given if given and os.access(given, os.X_OK) else None   # (set but empty: do without)
    return shutil.which("obot-segd")


def _text(text: str) -> bytes:
    raw = text.encode()
    return struct.pack("<I", len(raw)) + raw


def _lines(path: str) -> list[str]:
    with contextlib.suppress(OSError), open(path) as f:
        return [line.strip() for line in f if line.strip()]
    return []


class Segd:
    """A running obot-segd with its network loaded. Making a network ready for a GPU the
    first time takes minutes (obot-segd --build does it ahead); this waits for it."""

    def __init__(self, program: str, model: str, engine: str = ""):
        self._lock = threading.Lock()
        self._process = subprocess.Popen([program], stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        for pipe in (self._process.stdin, self._process.stdout):
            with contextlib.suppress(OSError, AttributeError):   # (a picture in one go)
                import fcntl

                fcntl.fcntl(pipe.fileno(), 1031, 1 << 20)   # F_SETPIPE_SZ
        try:
            self.width, self.height, self.columns, self.rows, classes = struct.unpack(
                "<5i", self._ask(SETUP, _text(model), _text(engine)))
        except RuntimeError:
            self.close()
            raise RuntimeError(f"obot-segd could not load {model}") from None
        # The classes' names and colours, as the jetson-inference models have them beside them.
        folder = os.path.dirname(model)
        names = _lines(os.path.join(folder, "classes.txt"))
        self.names = [names[k] if k < len(names) else f"class {k}" for k in range(classes)]
        colours = [[int(v) for v in line.split()[:3]] for line in _lines(os.path.join(folder, "colors.txt"))]
        self.colours = [colours[k] if k < len(colours) and len(colours[k]) == 3 else [128, 128, 128]
                        for k in range(classes)]

    def _ask(self, what: int, *parts: bytes) -> bytes:
        size = 4 + sum(len(p) for p in parts)
        with self._lock:
            try:
                self._process.stdin.write(struct.pack("<II", size, what))
                for part in parts:
                    self._process.stdin.write(part)
                length, status = struct.unpack("<II", self._read(8))
                answer = self._read(length - 4)
            except (OSError, ValueError, struct.error) as e:
                raise RuntimeError(f"obot-segd is gone ({e})") from None
        if status:
            raise RuntimeError(f"obot-segd refused request {what}")
        return answer

    def _read(self, n: int) -> bytes:
        chunks = []
        while n:
            chunk = self._process.stdout.read(n)
            if not chunk:
                raise OSError("it closed its output")
            chunks.append(chunk)
            n -= len(chunk)
        return b"".join(chunks)

    def look(self, bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        """What the network says of a colour picture (BGR, any size): the class of each part
        of it and how sure it is of that (0..255), both rows x columns; and the ms it took."""
        import cv2

        if bgr.shape[:2] != (self.height, self.width):
            bgr = cv2.resize(bgr, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
        answer = self._ask(PICTURE, np.ascontiguousarray(bgr, np.uint8).tobytes())
        parts = self.rows * self.columns
        said = np.frombuffer(answer, np.uint8, parts).reshape(self.rows, self.columns)
        sure = np.frombuffer(answer, np.uint8, parts, parts).reshape(self.rows, self.columns)
        return said, sure, struct.unpack_from("<f", answer, 2 * parts)[0]

    def close(self) -> None:
        with contextlib.suppress(OSError):
            self._process.stdin.close()   # it ends when its input does
        try:
            self._process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self._process.kill()
        with contextlib.suppress(OSError):
            self._process.stdout.close()


class Labels:
    """What the things on the map are, as far as the network says.

    A few times a second the colour camera's picture goes through the network. What it
    says is kept for the page to lay over the picture, and it is carried over to the map:
    the depth picture of the same moment gives where each part of the colour picture is in
    the room, and every square of the map (the squares of the map from above) collects
    votes for the classes seen standing in it. A square is said to be what most of its
    votes say, once there are a few.

    This does not change where the robot drives. It is what is known about a square, for
    the page now and for whoever plans with it later."""

    LOW, HIGH = 0.04, 1.5   # m above the floor: what stands there (the floor itself is not labelled)
    SURE = 128              # of 255: less sure than this does not vote
    VOTES = 6               # before a square is said to be anything
    STRIDE = 8              # every n-th pixel of the depth picture
    MAX_TURN = 1.0          # rad/s: turning faster than this, the two pictures are not of the same thing
    NOTHING = 0             # the class that means "none of the others"

    def __init__(self, config, cameras: dict, view, program: str, segd=Segd):
        self.config = config.segmentation
        self.cameras = cameras
        self.view = view
        self.program = program
        self._segd_type = segd
        # The colour camera to look through, and the depth camera that goes with it.
        with_colour = [c for c in view.depth if c.color in cameras and not c.link]
        wanted = self.config.camera or (with_colour[0].color if with_colour else "")
        self.depth = next((c for c in with_colour if c.color == wanted), None)
        self.colour = next((c for c in config.cameras if c.name == wanted and c.name in cameras), None)
        self.cell, self.half = view.SEEN_CELL, view.SEEN_HALF
        self.names: list[str] = []
        self.colours: list[list[int]] = []
        self.columns = self.rows = 0
        self.ready = False
        self.error: str | None = None if self.colour else "no colour camera to look through"
        self.ms = 0.0
        self.latest: tuple[float, np.ndarray, np.ndarray] | None = None
        self._votes: np.ndarray | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="labels", daemon=True)

    def start(self) -> None:
        if self.colour is not None:
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=3)

    def clear(self) -> None:
        with self._lock:
            if self._votes is not None:
                self._votes[:] = 0

    def _run(self) -> None:
        import time

        try:
            segd = self._segd_type(self.program, self.config.model, self.config.engine)
        except (RuntimeError, OSError) as e:
            self.error = str(e)
            return
        try:
            self.use(segd)
            period = 1.0 / max(0.1, self.config.rate)
            last = None
            while not self._stop.is_set():
                began = time.monotonic()
                stamp, picture = self.cameras[self.colour.name].raw()
                if picture is not None and stamp != last:
                    last = stamp
                    said, sure, self.ms = segd.look(picture)
                    self.latest = (stamp, said, sure)
                    self.error = None
                    if self.depth is not None:
                        self.vote(stamp, said, sure)
                self._stop.wait(max(0.02, period - (time.monotonic() - began)))
        except (RuntimeError, OSError) as e:
            self.error, self.ready = str(e), False
        finally:
            segd.close()

    def use(self, segd) -> None:
        """Take over what the network is like: its classes, and how finely it speaks."""
        self.names, self.colours, self.columns, self.rows = segd.names, segd.colours, segd.columns, segd.rows
        side = 2 * self.half + 1
        self._votes = np.zeros((side, side, len(self.names)), np.uint16)
        self.ready = True

    def vote(self, stamp: float, said: np.ndarray, sure: np.ndarray) -> int:
        """Carry what the network said of a colour picture over to the map, by the depth
        picture of the same moment. Returns how many readings voted."""
        from .. import world

        taken_at, depth = self.cameras[self.depth.name].raw()
        if depth is None or self._votes is None:
            return 0
        taken = taken_at - self.view.latency(self.depth)
        # (The two cameras' clocks differ by what a picture takes to arrive: a fifth of a
        # second apart is still the same moment for this, standing or driving slowly.)
        if abs(taken_at - stamp) > 0.25 or abs(self.view._pose_at(taken)[4]) > self.MAX_TURN:
            return 0
        k = self.depth.intrinsics()
        points = world.depth_to_points(depth, k, self.STRIDE, self.view.config.near, self.view.config.range)
        if not len(points):
            return 0
        in_map = world.apply(self.view.camera_pose(self.depth, taken), points)
        seen = world.apply(world.invert(self.view.camera_pose(self.colour, taken)), in_map)
        kc = self.colour.intrinsics()
        z = np.maximum(seen[:, 2], 1e-3)
        column = np.floor((seen[:, 0] / z * kc.fx + kc.cx) / kc.width * self.columns).astype(np.int64)
        row = np.floor((seen[:, 1] / z * kc.fy + kc.cy) / kc.height * self.rows).astype(np.int64)
        i = np.floor(in_map[:, 0] / self.cell).astype(np.int64) + self.half
        j = np.floor(in_map[:, 1] / self.cell).astype(np.int64) + self.half
        side = 2 * self.half + 1
        ok = ((seen[:, 2] > 0.05) & (column >= 0) & (column < self.columns) & (row >= 0) & (row < self.rows)
              & (in_map[:, 2] > self.LOW) & (in_map[:, 2] < self.HIGH) & (i >= 0) & (i < side) & (j >= 0) & (j < side))
        ok[ok] = sure[row[ok], column[ok]] >= self.SURE
        if not ok.any():
            return 0
        with self._lock:
            np.add.at(self._votes, (j[ok], i[ok], said[row[ok], column[ok]].astype(np.int64)), 1)
            if self._votes[j[ok], i[ok]].max() > 60000:   # (a long time in one place: halve, keep the shares)
                self._votes >>= 1
        return int(ok.sum())

    def squares(self) -> np.ndarray:
        """The squares that are said to be something: (i, j, class, per cent of the votes),
        N x 4; a square is (i * cell, j * cell) to one cell further, as in the map from above."""
        if self._votes is None:
            return np.zeros((0, 4), np.int64)
        with self._lock:
            total = self._votes.sum(axis=2, dtype=np.int64)
            j, i = np.nonzero(total >= self.VOTES)
            votes = self._votes[j, i].astype(np.int64)
        winner = votes.argmax(axis=1)
        keep = winner != self.NOTHING
        share = votes[np.arange(len(winner)), winner] * 100 // np.maximum(1, total[j, i])
        return np.stack([i - self.half, j - self.half, winner, share], axis=1)[keep]

    def describe(self) -> dict:
        """For the page: what the network said of the newest picture, and the squares."""
        import time

        latest = self.latest
        return {
            "ready": self.ready, "error": self.error, "camera": self.colour.name if self.colour else None,
            "names": self.names, "colours": self.colours, "columns": self.columns, "rows": self.rows,
            "ms": round(self.ms, 1), "cell": self.cell,
            "age": round(time.monotonic() - latest[0], 2) if latest else None,
            "said": latest[1].ravel().tolist() if latest else [],
            "sure": latest[2].ravel().tolist() if latest else [],
            "squares": self.squares()[:, :3].tolist(),
        }
