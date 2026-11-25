"""What the depth cameras have seen, kept as a coloured voxel map in the frame the base
odometry is in (x forward, y left, z up, origin where the daemon started, floor at z = 0).

    depth picture --depth_to_points--> points in the camera --camera pose--> points in the world
                                                                   |
                                             WorldMap.integrate: one voxel per few centimetres

The map has history in both directions: a voxel stays after the camera has turned away, and
a voxel the camera now sees *through* (the thing was picked up) is taken out again.

Frames. A camera's mount in the configuration is its body frame, as in a URDF: x is the
viewing direction, y is left, z is up. Pictures are in the optical frame: x right, y down,
z along the view. OPTICAL converts between the two.
"""

from __future__ import annotations

import math
import secrets
import struct
import threading
from dataclasses import dataclass

import numpy as np

# Pose of the optical frame in the camera body frame.
OPTICAL = np.array([[0.0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])


def rpy_matrix(rpy) -> np.ndarray:
    """URDF roll, pitch, yaw (about fixed x, y, z in that order) as a 3 x 3 rotation."""
    r, p, y = (float(v) for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(rpy)
    t[:3, 3] = xyz
    return t


def base_pose(x: float, y: float, yaw: float) -> np.ndarray:
    return transform((x, y, 0.0), (0.0, 0.0, yaw))


def invert(t: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return out


def apply(t: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Points (N x 3) moved by a 4 x 4 pose."""
    return (points @ t[:3, :3].T + t[:3, 3]).astype(np.float32)


@dataclass(frozen=True)
class Intrinsics:
    """A pinhole camera: pixel = focal length * (x / z, y / z) + centre."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_hfov(cls, width: int, height: int, hfov_deg: float) -> Intrinsics:
        focal = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
        return cls(width, height, focal, focal, (width - 1) / 2, (height - 1) / 2)


def depth_to_points(depth_mm: np.ndarray, k: Intrinsics, stride: int = 4, near: float = 0.2,
                    far: float = 3.5) -> np.ndarray:
    """Every stride-th pixel of a depth picture (millimetres, 0 = no reading) as a point in
    the optical frame, in metres. Readings outside near .. far are left out."""
    half = stride // 2
    sub = depth_mm[half::stride, half::stride]
    v, u = np.nonzero((sub >= near * 1000) & (sub <= far * 1000))
    z = sub[v, u].astype(np.float32) * 0.001
    x = ((u * stride + half) - k.cx) / k.fx * z
    y = ((v * stride + half) - k.cy) / k.fy * z
    return np.stack([x, y, z], axis=1).astype(np.float32)


def sample_colors(points: np.ndarray, bgr: np.ndarray, k: Intrinsics) -> np.ndarray:
    """The colour (r, g, b) a colour camera sees at each point, the points being in that
    camera's optical frame. (0, 0, 0) where the point is outside its picture."""
    rgb = np.zeros((len(points), 3), np.uint8)
    z = points[:, 2]
    ok = z > 0.05
    u = np.zeros(len(points), np.int64)
    v = np.zeros(len(points), np.int64)
    u[ok] = np.rint(points[ok, 0] / z[ok] * k.fx + k.cx)
    v[ok] = np.rint(points[ok, 1] / z[ok] * k.fy + k.cy)
    ok &= (u >= 0) & (u < bgr.shape[1]) & (v >= 0) & (v < bgr.shape[0])
    rgb[ok] = np.maximum(bgr[v[ok], u[ok], ::-1], 1)  # never exactly black: that means "no colour"
    return rgb


class LinkPoses:
    """Where each link of a URDF is, given the joint positions (model: ui.model.load())."""

    def __init__(self, model: dict):
        self.root = model.get("root")
        self.links = set(model["links"])
        self._joint_above = {j["child"]: j for j in model["joints"]}

    def pose(self, link: str, q: dict[str, float]) -> np.ndarray:
        """Pose of the link in the root link's frame."""
        if link not in self.links:
            raise KeyError(f"the robot model has no link '{link}'")
        t = np.eye(4)
        while link in self._joint_above:
            joint = self._joint_above[link]
            motion = np.eye(4)
            value = q.get(joint["name"])
            if value is not None and not math.isnan(value):
                axis = np.asarray(joint["axis"], float)
                axis = axis / (np.linalg.norm(axis) or 1.0)
                if joint["type"] in ("revolute", "continuous"):
                    x, y, z = axis
                    c, s = math.cos(value), math.sin(value)
                    motion[:3, :3] = np.array([
                        [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                        [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
                        [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
                    ])
                elif joint["type"] == "prismatic":
                    motion[:3, 3] = axis * value
            t = transform(joint["xyz"], joint["rpy"]) @ motion @ t
            link = joint["parent"]
        return t


def fit_floor(points: np.ndarray, tolerance: float = 0.015, rounds: int = 200, seed: int = 0):
    """The largest flat surface in the points, taken to be the floor.

    The points are in the camera body frame (x forward, y left, z up). Returns (height of
    the camera above the surface, roll, pitch of the camera, share of the points on it), or
    None if no plane holds at least 30 % of the points."""
    if len(points) < 50:
        return None
    rng = np.random.default_rng(seed)
    best, best_count = None, 0
    for _ in range(rounds):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal /= length
        count = int((np.abs((points - a) @ normal) < tolerance).sum())
        if count > best_count:
            best, best_count = (normal, a), count
    if best is None or best_count < 0.3 * len(points):
        return None
    normal, a = best
    on_plane = points[np.abs((points - a) @ normal) < tolerance]
    centre = on_plane.mean(axis=0)
    normal = np.linalg.svd(on_plane - centre, full_matrices=False)[2][2]  # least squares on the inliers
    if normal @ centre > 0:
        normal = -normal  # pointing from the floor towards the camera: "up"
    height = float(-(normal @ centre))
    pitch = -math.asin(max(-1.0, min(1.0, float(normal[0]))))
    roll = math.atan2(float(normal[1]), float(normal[2]))
    return height, roll, pitch, best_count / len(points)


_OFFSET = 32768
_LIMIT = 32760  # voxel indices travel as 16 bit numbers


class WorldMap:
    """A sparse voxel map: one entry per `voxel`-sized cube that something was seen in.

    A voxel is shown once it has been seen `confirm` times (single stray readings never
    appear), and is removed again when the camera has looked through it as often as it had
    been seen (at most `max_hits` times). Thread safe.
    """

    MAGIC = 0x3157424F  # "OBW1"
    HEADER = struct.Struct("<8If")

    def __init__(self, voxel: float = 0.03, max_voxels: int = 250_000, confirm: int = 2, max_hits: int = 6):
        self.voxel = float(voxel)
        self.max_voxels = int(max_voxels)
        self.confirm = confirm
        self.max_hits = max_hits
        self._lock = threading.Lock()
        self.epoch = secrets.randbits(32) or 1  # tells a client that this is a different map
        self.rev = 0
        self._horizon = 0  # clients that last asked before this are sent everything
        self._removed: list[tuple[int, np.ndarray]] = []
        self._trail: list[tuple[float, float]] = []
        self._trail_start = 0  # how many trail points were dropped from the front
        self._empty()

    def _empty(self) -> None:
        self._keys = np.empty(0, np.int64)
        self._index = np.empty((0, 3), np.int16)
        self._rgb = np.empty((0, 3), np.uint8)
        self._hits = np.empty(0, np.uint8)
        self._shown = np.empty(0, bool)
        self._changed = np.empty(0, np.uint32)  # revision in which what a client draws last changed
        self._seen = np.empty(0, np.uint32)  # revision in which it was last observed

    def __len__(self) -> int:
        return int(self._shown.sum())

    def clear(self) -> None:
        with self._lock:
            self._empty()
            self._removed.clear()
            self._trail_start += len(self._trail)
            self._trail.clear()
            self.rev += 1
            self._horizon = self.rev

    def points(self) -> tuple[np.ndarray, np.ndarray]:
        """Centres (N x 3, metres) and colours (N x 3) of the voxels that are shown."""
        with self._lock:
            return (self._index[self._shown].astype(np.float32) * self.voxel, self._rgb[self._shown].copy())

    # --- building ------------------------------------------------------------------------

    def integrate(self, points: np.ndarray, rgb: np.ndarray | None = None) -> None:
        """Add points seen now (world frame, metres) with their colours ((0, 0, 0) = unknown)."""
        # Cells are centred on multiples of the voxel size, so that a floor at z = 0 is in the
        # middle of a layer of cells instead of on the border between two.
        index = np.floor(np.asarray(points, np.float64) / self.voxel + 0.5).astype(np.int64)
        inside = np.all(np.abs(index) < _LIMIT, axis=1)
        index = index[inside]
        rgb = np.zeros((len(index), 3), np.uint8) if rgb is None else np.asarray(rgb, np.uint8)[inside]
        keys = ((index[:, 0] + _OFFSET) << 32) | ((index[:, 1] + _OFFSET) << 16) | (index[:, 2] + _OFFSET)
        keys, first = np.unique(keys, return_index=True)
        index, rgb = index[first].astype(np.int16), rgb[first]
        with self._lock:
            self.rev += 1
            count = len(self._keys)
            at = np.searchsorted(self._keys, keys)
            known = np.zeros(len(keys), bool)
            if count:
                known = (at < count) & (self._keys[np.minimum(at, count - 1)] == keys)
            # Voxels that are in the map already.
            e = at[known]
            self._hits[e] = np.minimum(self._hits[e] + 1, self.max_hits)
            self._seen[e] = self.rev
            appeared = ~self._shown[e] & (self._hits[e] >= self.confirm)
            colour = rgb[known]
            # Repainting on every small change of brightness would resend the whole view
            # each time; a visible difference is enough.
            differs = np.abs(colour.astype(np.int16) - self._rgb[e]).sum(axis=1) > 40
            repaint = colour.any(axis=1) & differs
            self._rgb[e[repaint]] = colour[repaint]
            self._shown[e[appeared]] = True
            self._changed[e[appeared | (repaint & self._shown[e])]] = self.rev
            # New ones. np.insert places each before the given position of the old array,
            # which keeps the keys sorted.
            new = ~known
            if new.any():
                where = at[new]
                shown = self.confirm <= 1
                self._keys = np.insert(self._keys, where, keys[new])
                self._index = np.insert(self._index, where, index[new], axis=0)
                self._rgb = np.insert(self._rgb, where, rgb[new], axis=0)
                self._hits = np.insert(self._hits, where, 1)
                self._shown = np.insert(self._shown, where, shown)
                self._changed = np.insert(self._changed, where, self.rev if shown else 0)
                self._seen = np.insert(self._seen, where, self.rev)
            if len(self._keys) > self.max_voxels:
                # Full: forget what has not been looked at for longest.
                keep = len(self._keys) - self.max_voxels // 10
                cutoff = np.partition(self._seen, len(self._keys) - keep)[len(self._keys) - keep]
                self._drop(self._seen < cutoff)

    def carve(self, camera: np.ndarray, depth_mm: np.ndarray, k: Intrinsics, near: float = 0.2,
              far: float = 3.5) -> int:
        """Take out what the camera now sees through: voxels in front of what it measures
        along the same line of sight. `camera` is the pose of its optical frame in the world.
        Returns how many voxels were removed."""
        with self._lock:
            if not len(self._keys):
                return 0
            centres = self._index.astype(np.float32) * self.voxel
            p = (centres - camera[:3, 3].astype(np.float32)) @ camera[:3, :3].astype(np.float32)
            z = p[:, 2]
            candidates = np.nonzero((z > near) & (z < far))[0]
            zc = z[candidates]
            u = np.rint(p[candidates, 0] / zc * k.fx + k.cx).astype(np.int64)
            v = np.rint(p[candidates, 1] / zc * k.fy + k.cy).astype(np.int64)
            inside = (u >= 0) & (u < depth_mm.shape[1]) & (v >= 0) & (v < depth_mm.shape[0])
            candidates, zc = candidates[inside], zc[inside]
            measured = depth_mm[v[inside], u[inside]].astype(np.float32) * 0.001
            # Only a reading counts (no reading says nothing), and it has to be clearly
            # behind the voxel: depth noise grows with distance.
            through = candidates[(measured > 0) & (measured - zc > 0.08 + 0.05 * zc)]
            self._hits[through] -= 1
            gone = np.zeros(len(self._keys), bool)
            gone[through[self._hits[through] == 0]] = True
            removed = int(gone.sum())
            if removed:
                self.rev += 1
                self._drop(gone)
            return removed

    def _drop(self, gone: np.ndarray) -> None:
        told = gone & self._shown
        if told.any():
            self._removed.append((self.rev, self._index[told].copy()))
            if len(self._removed) > 400:
                self._horizon = self._removed[0][0]
                del self._removed[0]
        keep = ~gone
        self._keys, self._index, self._rgb = self._keys[keep], self._index[keep], self._rgb[keep]
        self._hits, self._shown = self._hits[keep], self._shown[keep]
        self._changed, self._seen = self._changed[keep], self._seen[keep]

    def add_pose(self, x: float, y: float, step: float = 0.02, limit: int = 20_000) -> None:
        """Remember where the robot has been (a point for every `step` metres travelled)."""
        with self._lock:
            if self._trail and math.hypot(x - self._trail[-1][0], y - self._trail[-1][1]) < step:
                return
            self._trail.append((x, y))
            if len(self._trail) > limit:
                del self._trail[: limit // 2]
                self._trail_start += limit // 2

    # --- for the operator page ---------------------------------------------------------------

    def delta(self, since: int = 0, epoch: int = 0, trail: int = 0) -> bytes:
        """What changed after revision `since` of map `epoch`, for a client that has `trail`
        trail points. Layout (little endian):

            u32 magic, epoch, rev, flags (1 = forget what you have), changed, removed,
                trail_start, trail_count; f32 voxel size
            f32 x, y            per trail point, the first being number trail_start
            i16 ix, iy, iz      per changed voxel: its centre is i * voxel size
            i16 ix, iy, iz      per removed voxel
            u8  r, g, b         per changed voxel; 0, 0, 0 = no colour known
        """
        with self._lock:
            reset = epoch != self.epoch or since > self.rev or since < self._horizon
            if reset:
                pick = self._shown
                removed = np.empty((0, 3), np.int16)
                trail = 0
            else:
                pick = self._shown & (self._changed > since)
                gone = [index for rev, index in self._removed if rev > since]
                removed = np.concatenate(gone) if gone else np.empty((0, 3), np.int16)
            first = min(max(trail, self._trail_start), self._trail_start + len(self._trail))
            path = np.asarray(self._trail[first - self._trail_start:], np.float32).reshape(-1, 2)
            changed, rgb = self._index[pick], self._rgb[pick]
            header = self.HEADER.pack(self.MAGIC, self.epoch, self.rev, 1 if reset else 0, len(changed),
                                      len(removed), first, len(path), self.voxel)
            return b"".join((header, path.tobytes(), changed.tobytes(), removed.tobytes(), rgb.tobytes()))

    @classmethod
    def decode(cls, data: bytes) -> dict:
        """The inverse of delta(), for tests and tools."""
        magic, epoch, rev, flags, changed, removed, first, count, voxel = cls.HEADER.unpack_from(data)
        if magic != cls.MAGIC:
            raise ValueError("not a world map update")
        at = cls.HEADER.size
        path = np.frombuffer(data, "<f4", count * 2, at).reshape(-1, 2)
        at += count * 8
        index = np.frombuffer(data, "<i2", changed * 3, at).reshape(-1, 3)
        at += changed * 6
        gone = np.frombuffer(data, "<i2", removed * 3, at).reshape(-1, 3)
        at += removed * 6
        rgb = np.frombuffer(data, np.uint8, changed * 3, at).reshape(-1, 3)
        return {"epoch": epoch, "rev": rev, "reset": bool(flags & 1), "voxel": voxel, "trail_start": first,
                "trail": path, "changed": index, "removed": gone, "rgb": rgb}
