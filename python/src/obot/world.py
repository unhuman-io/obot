"""What the depth cameras have seen, kept as a coloured voxel map in the frame the base
odometry is in (x forward, y left, z up, origin where the daemon started, floor at z = 0).

    depth picture --depth_to_points--> points in the camera --camera pose--> points in the world
                                                                   |
                                             WorldMap.integrate: one voxel per few centimetres

The map has history in both directions: a voxel stays after the camera has turned away, and
a voxel the camera now sees *through* (the thing was picked up) is taken out again.

What the cameras see also tells the robot where it is: what stands on the floor is kept as
a floor plan (scan_2d, FloorPlan) and each new picture is laid over it, which corrects the
wheel odometry for slipping wheels and a wrong track width.

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


def compose(a, b) -> tuple[float, float, float]:
    """Planar poses (x, y, yaw): b, given in the frame a, in the frame a is given in."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inverse(a) -> tuple[float, float, float]:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


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


def scan_2d(points: np.ndarray, centre=(0.0, 0.0), step: float = 0.01, z_min: float = 0.10, z_max: float = 1.5,
            min_height: float = 0.05) -> np.ndarray:
    """What stands on the floor, seen from above, as a scan around `centre` (where the
    camera is): from points in the base frame (x forward, y left, z up from the floor)
    between z_min and z_max, one point (x, y) for each upright stretch of surface in each
    direction, the directions being `step` radians apart.

    Something upright is seen in one direction at one distance over a range of heights, at
    least `min_height`; its points are averaged, which takes out most of a depth camera's
    noise. That leaves out the floor and what hangs from the ceiling, and also everything
    that lies flat (a table top, the top of a box): of a flat surface a camera sees rows,
    and where those rows are depends on where the camera is, not on where the surface is."""
    band = (points[:, 2] > z_min) & (points[:, 2] < z_max)
    if not band.any():
        return np.empty((0, 2))
    xy, z = points[band, :2].astype(np.float64), points[band, 2].astype(np.float64)
    dx, dy = xy[:, 0] - centre[0], xy[:, 1] - centre[1]
    distance = np.hypot(dx, dy)
    direction = np.rint(np.arctan2(dy, dx) / step).astype(np.int64)
    order = np.lexsort((distance, direction))
    direction, distance, xy, z = direction[order], distance[order], xy[order], z[order]
    # A new stretch where the direction changes, or the distance jumps by more than the
    # camera's noise (which grows with the square of the distance).
    jump = np.diff(distance) > 0.06 + 0.02 * distance[:-1] ** 2
    first = np.nonzero(np.r_[True, (direction[1:] != direction[:-1]) | jump])[0]
    count = np.diff(np.r_[first, len(z)])
    # Of each stretch, what is at about its middle distance: real things are not as neat as
    # a wall (a shelf, a curtain, a skirting board in front of the wall).
    middle = np.repeat(distance[first + count // 2], count)
    core = np.abs(distance - middle) < 0.04 + 0.015 * middle ** 2
    points = np.add.reduceat(core.astype(np.float64), first)
    height = (np.maximum.reduceat(np.where(core, z, -np.inf), first)
              - np.minimum.reduceat(np.where(core, z, np.inf), first))
    upright = (points >= 3) & (height >= min_height)
    total = np.stack([np.add.reduceat(xy[:, 0] * core, first), np.add.reduceat(xy[:, 1] * core, first)], axis=1)
    return (total / np.maximum(points, 1)[:, None])[upright]


def scan_surfaces(scan: np.ndarray, radius: float = 0.1) -> tuple[np.ndarray, np.ndarray]:
    """The surfaces a scan (scan_2d) is of. For each scan point, from its neighbours within
    `radius`: the direction across the surface it is part of (a unit vector, either way
    round), and the point itself moved onto the line through those neighbours, which takes
    out most of the depth camera's noise. Where the neighbours do not form a line (a lone
    point, a corner) the direction is (0, 0) and the point is left where it is.
    Returns (points, directions)."""
    if not len(scan):
        return np.empty((0, 2)), np.empty((0, 2))
    offset = scan[:, None, :] - scan[None, :, :]
    near = ((offset ** 2).sum(axis=2) < radius ** 2).astype(np.float64)
    count = near.sum(axis=1)
    mean = (near @ scan) / count[:, None]
    xx = (near @ (scan[:, 0] ** 2)) / count - mean[:, 0] ** 2
    yy = (near @ (scan[:, 1] ** 2)) / count - mean[:, 1] ** 2
    xy = (near @ (scan[:, 0] * scan[:, 1])) / count - mean[:, 0] * mean[:, 1]
    # the neighbours' spread: along the surface it is large, across it small
    half_trace, root = (xx + yy) / 2, np.sqrt(((xx - yy) / 2) ** 2 + xy ** 2)
    along, across = half_trace + root, half_trace - root
    angle = 0.5 * np.arctan2(2 * xy, xx - yy)  # direction of the largest spread
    normal = np.stack([-np.sin(angle), np.cos(angle)], axis=1)
    normal *= ((count >= 4) & (across < 0.1 * along))[:, None]
    return scan - normal * ((scan - mean) * normal).sum(axis=1)[:, None], normal


def scan_normals(scan: np.ndarray, radius: float = 0.1) -> np.ndarray:
    return scan_surfaces(scan, radius)[1]


@dataclass
class Match:
    pose: tuple[float, float, float]  # where the scan fits the floor plan best
    matched: int  # scan points that lie on something in the plan there
    share: float  # ... as a share of the scan points that could be compared
    directions: int  # of x, y and yaw, how many the plan pins down (a bare wall: 2)


class FloorPlan:
    """What stands on the floor, from above, where it was first seen: points (x, y) in the
    map, each with the direction across the surface it is on, a point every 10 to 20 cm
    along a wall.

    It is what the robot finds its place by (match), so it must not move with the robot's
    mistakes: a point, once in, stays where it is, and nothing new is added next to it.
    A wall seen a second time from a slightly wrong place therefore lands *off* the plan,
    by exactly the mistake, instead of thickening it.
    """

    def __init__(self, spacing: float = 0.05):
        self.spacing = spacing
        self.points = np.empty((0, 2))
        self.normals = np.empty((0, 2))
        self._cells: dict[tuple[int, int], int] = {}  # one point per spacing-sized square

    def __len__(self) -> int:
        return len(self.points)

    ACROSS, ALONG = 0.10, 0.10  # m: nearer than this to a point of the plan is the same surface

    def add(self, points: np.ndarray, normals: np.ndarray) -> int:
        """Put in the points (map frame, with the direction across their surface) that are
        not on a surface the plan already has there: within ACROSS of one of its points
        across that point's surface, within ALONG along it, and facing the same way.
        Returns how many were new."""
        size = self.spacing
        have, facing = self.points.tolist(), self.normals.tolist()
        before = len(have)
        for (x, y), (nx, ny) in zip(np.asarray(points).tolist(), np.asarray(normals).tolist(), strict=True):
            i, j = math.floor(x / size), math.floor(y / size)
            taken = (i, j) in self._cells or (nx == 0 and ny == 0)  # (no surface to speak of: not for the plan)
            for di in (0, -1, 1, -2, 2):
                if taken:
                    break
                for dj in (0, -1, 1, -2, 2):
                    at = self._cells.get((i + di, j + dj))
                    if at is None:
                        continue
                    (qx, qy), (mx, my) = have[at], facing[at]
                    across, along = (x - qx) * mx + (y - qy) * my, (y - qy) * mx - (x - qx) * my
                    if abs(across) < self.ACROSS and abs(along) < self.ALONG and abs(nx * mx + ny * my) > 0.8:
                        taken = True
                        break
            if not taken:
                self._cells[(i, j)] = len(have)
                have.append([x, y])
                facing.append([nx, ny])
        if len(have) > before:
            self.points, self.normals = np.asarray(have, np.float64), np.asarray(facing, np.float64)
        return len(have) - before

    def _nearest(self, centre, half: float):
        """For looking up the plan's nearest point around `centre`: (index[iy, ix] of the
        nearest point for every spacing-sized square within `half`, (ix0, iy0))."""
        import cv2  # here so that the rest of the module works without OpenCV

        n = int(half / self.spacing)
        ix0, iy0 = math.floor(centre[0] / self.spacing) - n, math.floor(centre[1] / self.spacing) - n
        ix = np.floor(self.points[:, 0] / self.spacing).astype(np.int64) - ix0
        iy = np.floor(self.points[:, 1] / self.spacing).astype(np.int64) - iy0
        inside = np.nonzero((ix >= 0) & (ix <= 2 * n) & (iy >= 0) & (iy <= 2 * n))[0]
        if not len(inside):
            return None, (ix0, iy0)
        empty = np.ones((2 * n + 1, 2 * n + 1), np.uint8)
        empty[iy[inside], ix[inside]] = 0
        _, label = cv2.distanceTransformWithLabels(empty, cv2.DIST_L2, cv2.DIST_MASK_PRECISE,
                                                   labelType=cv2.DIST_LABEL_PIXEL)
        which = np.zeros(int(label.max()) + 1, np.int64)
        which[label[iy[inside], ix[inside]]] = inside
        return which[label], (ix0, iy0)

    def match(self, scan: np.ndarray, normals: np.ndarray, guess, reach: float = 0.3,
              iterations: int = 12) -> tuple[Match | None, np.ndarray]:
        """Lay a scan (scan_surfaces of a scan_2d, in the base frame) over the plan, starting
        from the pose `guess` = (x, y, yaw) of the base.

        Each scan point is compared with the plan's point nearest to it, if that is within
        `reach` and on a surface facing the same way, and only *across* that surface: along
        a wall one stretch is like the next, so there is nothing to compare. Far points
        count for less (a depth camera's error grows with the square of the distance).
        Directions in which the plan does not hold the scan (along a bare wall) are left as
        guessed.

        Returns (the match, or None if nothing of the scan is near anything in the plan;
        which scan points have nothing of the plan within reach: they are new)."""
        new = np.ones(len(scan), bool)
        usable = np.any(normals != 0, axis=1)
        if not len(self.points) or usable.sum() < 3:
            return None, new
        nearest, (ix0, iy0) = self._nearest(guess[:2], float(np.abs(scan).max()) + reach + 2 * self.spacing)
        if nearest is None:
            return None, new
        height, width = nearest.shape
        px, py = scan[:, 0], scan[:, 1]
        distance2 = px ** 2 + py ** 2
        certain = 1.0 / (1.0 + (0.4 * distance2) ** 2)  # 1 cm close by, 4 cm at 3 m
        tolerance = 0.03 / np.sqrt(certain)  # how far off a point may be and still be the same thing

        def look(pose):
            c, s = math.cos(pose[2]), math.sin(pose[2])
            wx, wy = pose[0] + c * px - s * py, pose[1] + s * px + c * py
            gx = np.floor(wx / self.spacing).astype(np.int64) - ix0
            gy = np.floor(wy / self.spacing).astype(np.int64) - iy0
            inside = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
            at = nearest[np.clip(gy, 0, height - 1), np.clip(gx, 0, width - 1)]
            qx, qy = self.points[at, 0], self.points[at, 1]
            nx, ny = self.normals[at, 0], self.normals[at, 1]
            near = inside & ((wx - qx) ** 2 + (wy - qy) ** 2 < reach ** 2)
            # the same way round as the scan point's own surface?
            facing = (c * normals[:, 0] - s * normals[:, 1]) * nx + (s * normals[:, 0] + c * normals[:, 1]) * ny
            across = nx * (wx - qx) + ny * (wy - qy)
            turn = nx * (-s * px - c * py) + ny * (c * px - s * py)  # change of `across` with yaw
            return across, np.stack([nx, ny, turn], axis=1), near, near & usable & (np.abs(facing) > 0.8)

        pose = [float(v) for v in guess]
        directions = 0
        for _ in range(iterations):
            across, jacobian, _, comparable = look(pose)
            if comparable.sum() < 3:
                break
            weight = comparable * certain / (1 + (across / tolerance) ** 2)
            hessian = (jacobian * weight[:, None]).T @ jacobian
            gradient = jacobian.T @ (weight * across)
            strength, axes = np.linalg.eigh(hessian)
            held = strength > 0.03 * weight.sum()  # a direction that a share of the points pulls in
            directions = int(held.sum())
            if not directions:
                break
            step = -axes[:, held] @ ((axes[:, held].T @ gradient) / strength[held])
            pose = [pose[0] + step[0], pose[1] + step[1], pose[2] + step[2]]
            if abs(step[0]) + abs(step[1]) < 1e-4 and abs(step[2]) < 1e-4:
                break
        across, _, near, comparable = look(pose)
        new = ~near
        if not comparable.any():
            return None, new
        matched = int((comparable & (np.abs(across) < tolerance)).sum())
        return Match((pose[0], pose[1], pose[2]), matched, matched / int(usable.sum()), directions), new


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
        self._trail_lock = threading.Lock()  # its own: add_pose is called from the page's main loop
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
            with self._trail_lock:
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
        """Remember where the robot has been (a point for every `step` metres travelled).
        Never waits for a map update."""
        with self._trail_lock:
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
            with self._trail_lock:
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
