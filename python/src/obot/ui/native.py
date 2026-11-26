"""obot-navd, the map of the surroundings kept by a C++ process, and how to talk to it.

The page server starts it and speaks to it over its standard input and output (the layout
is in rt/apps/obot_navd.cpp). It does what obot.world.WorldMap and the floor bookkeeping of
WorldView do in Python, with the same rules, about fifty times faster, which on a small
computer is the difference between a map that keeps up with the camera and one that does not.

Without the program (not built, not installed) everything works as before, in Python.
"""

from __future__ import annotations

import contextlib
import math
import os
import shutil
import struct
import subprocess
import threading
from pathlib import Path

import numpy as np

SETUP, PICTURE, DELTA, POSE, CLEAR, GRID, KNOWN, OBSTACLES, SAVE, LOAD, COUNT, THINGS = range(1, 13)
ON_BASE, FOR_MAP, WITH_COLOUR = 1, 2, 4


def find() -> str | None:
    """The obot-navd to use: $OBOT_NAVD, one on the PATH, or the one built in this checkout."""
    given = os.environ.get("OBOT_NAVD")
    if given is not None:
        return given if given and os.access(given, os.X_OK) else None   # (set but empty: do without)
    found = shutil.which("obot-navd")
    if found:
        return found
    built = Path(__file__).resolve().parents[4] / "rt" / "build" / "release" / "obot-navd"
    return str(built) if os.access(built, os.X_OK) else None


def _pose(matrix) -> bytes:
    return np.ascontiguousarray(np.asarray(matrix, np.float64)[:3, :4]).tobytes()


def _intrinsics(k) -> bytes:
    return struct.pack("<iidddd", k.width, k.height, k.fx, k.fy, k.cx, k.cy)


def _text(text: str) -> bytes:
    raw = text.encode()
    return struct.pack("<I", len(raw)) + raw


class Navd:
    """A running obot-navd. Its methods may be called from any thread, one at a time."""

    def __init__(self, program: str, voxel: float, max_voxels: int, cell: float, half: int, way_half_width: float,
                 way_low: float, way_high: float):
        self.voxel = float(voxel)
        self._lock = threading.Lock()
        self._process = subprocess.Popen([program], stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        for pipe in (self._process.stdin, self._process.stdout):
            with contextlib.suppress(OSError, AttributeError):   # (a megabyte picture in one go)
                import fcntl

                fcntl.fcntl(pipe.fileno(), 1031, 1 << 20)   # F_SETPIPE_SZ
        self.ask(SETUP, struct.pack("<dIdiddd", voxel, max_voxels, cell, half, way_half_width, way_low, way_high))

    def ask(self, what: int, *parts: bytes) -> bytes:
        size = 4 + sum(len(p) for p in parts)
        with self._lock:
            try:
                self._process.stdin.write(struct.pack("<II", size, what))
                for part in parts:
                    self._process.stdin.write(part)
                head = self._read(8)
                length, status = struct.unpack("<II", head)
                answer = self._read(length - 4)
            except (OSError, ValueError, struct.error) as e:
                raise RuntimeError(f"obot-navd is gone ({e})") from None
        if status:
            raise RuntimeError(f"obot-navd refused request {what}")
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

    @property
    def alive(self) -> bool:
        return self._process.poll() is None

    def close(self) -> None:
        with contextlib.suppress(OSError):
            self._process.stdin.close()   # it ends when its input does
        try:
            self._process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self._process.kill()
        with contextlib.suppress(OSError):
            self._process.stdout.close()

    # --- what it does ------------------------------------------------------------------------

    def picture(self, depth: np.ndarray, k, stride: int, near: float, far: float, to_base, to_map, base_pose,
                on_base: bool, for_map: bool, colour=None) -> tuple[float | None, int, int, int, tuple]:
        """A depth picture and what to do with it (see obot_navd.cpp). colour: (picture BGR,
        its intrinsics, pose of the depth camera's optical frame in the colour camera's).
        Returns (how far ahead the nearest thing in the base's way is, or None if not
        looked; points used; voxels taken out; voxels shown now; how the picture agrees
        with the floor plan: squares with something in them, of those remembered, of those
        contradicted)."""
        flags = (ON_BASE if on_base else 0) | (FOR_MAP if for_map else 0) | (WITH_COLOUR if colour is not None else 0)
        parts = [struct.pack("<I", flags), _intrinsics(k), struct.pack("<idd", stride, near, far), _pose(to_base),
                 _pose(to_map), struct.pack("<ddd", *base_pose), np.ascontiguousarray(depth, "<u2").tobytes()]
        if colour is not None:
            picture, colour_k, to_colour = colour
            parts += [_intrinsics(colour_k), _pose(to_colour), np.ascontiguousarray(picture, np.uint8).tobytes()]
        ahead, points, removed, shown, *agreement = struct.unpack("<dIIQIII", self.ask(PICTURE, *parts))
        return (None if ahead < 0 else ahead), points, removed, shown, tuple(agreement)

    def delta(self, since: int = 0, epoch: int = 0, trail: int = 0) -> bytes:
        return self.ask(DELTA, struct.pack("<III", since & 0xFFFFFFFF, epoch & 0xFFFFFFFF, trail & 0xFFFFFFFF))

    def pose(self, x: float, y: float, stand: float) -> None:
        self.ask(POSE, struct.pack("<ddd", x, y, stand))

    def clear(self) -> None:
        self.ask(CLEAR)

    def grid(self, keep_clear: float) -> bytes:
        return self.ask(GRID, struct.pack("<d", keep_clear))

    def known(self, origin, shape) -> np.ndarray:
        raw = self.ask(KNOWN, struct.pack("<iiII", int(origin[0]), int(origin[1]), int(shape[1]), int(shape[0])))
        return np.frombuffer(raw, np.uint8).reshape(shape).astype(bool)

    def obstacles(self, z_min: float, z_max: float, least: int) -> np.ndarray:
        raw = self.ask(OBSTACLES, struct.pack("<ddi", z_min, z_max, least))
        (count,) = struct.unpack_from("<I", raw)
        return np.frombuffer(raw, "<i4", count * 2, 4).reshape(-1, 2).astype(np.float64) * self.voxel

    def things(self) -> np.ndarray:
        """The squares of the floor plan in which something is remembered to stand: (i, j)
        each, the square reaching from (i * cell, j * cell)."""
        raw = self.ask(THINGS)
        (count,) = struct.unpack_from("<I", raw)
        return np.frombuffer(raw, "<i4", count * 2, 4).reshape(-1, 2)

    def save(self, path: str, frame: str) -> None:
        self.ask(SAVE, _text(path), _text(frame))

    def load(self, path: str, frame: str) -> bool:
        return self.ask(LOAD, _text(path), _text(frame)) == b"\x01"

    def count(self) -> dict:
        shown, voxels, floor, epoch, rev = struct.unpack("<QQQII", self.ask(COUNT))
        return {"shown": shown, "voxels": voxels, "floor": floor, "epoch": epoch, "rev": rev}


class NativeMap:
    """What the rest of the page server asks of a WorldMap, answered by obot-navd."""

    def __init__(self, navd: Navd, stand: float):
        self.navd = navd
        self.voxel = navd.voxel
        self.stand = stand
        self.shown = 0   # as of the last picture
        self._trail_at: tuple[float, float] | None = None

    def __len__(self) -> int:
        return self.shown

    def add_pose(self, x: float, y: float, step: float = 0.02) -> None:
        # (the map makes a point of the trail every 2 cm; nothing needs telling in between)
        if self._trail_at is None or math.hypot(x - self._trail_at[0], y - self._trail_at[1]) >= step:
            self._trail_at = (x, y)
            self.navd.pose(x, y, self.stand)

    def delta(self, since: int = 0, epoch: int = 0, trail: int = 0) -> bytes:
        return self.navd.delta(since, epoch, trail)

    def obstacles(self, z_min: float = 0.05, z_max: float = 0.7, least: int = 4) -> np.ndarray:
        return self.navd.obstacles(z_min, z_max, least)

    def points(self) -> tuple[np.ndarray, np.ndarray]:
        """Centres (N x 3, metres) and colours (N x 3) of the voxels that are shown."""
        from ..world import WorldMap

        update = WorldMap.decode(self.delta())
        return update["changed"].astype(np.float32) * self.voxel, update["rgb"].copy()

    def clear(self) -> None:
        self.navd.clear()
        self.shown = 0
        self._trail_at = None
