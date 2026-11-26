"""Keeps the 3D map of the surroundings up to date for the operator page: takes the newest
picture of each depth camera, works out where that camera was when it was taken (odometry,
joint angles, the camera's mount) and adds what it saw to a WorldMap.

The cameras in turn correct the odometry. The wheels say how the base moved from one moment
to the next, and over time that drifts: wheels slip, and the track width and wheel radius
are never quite right. What a depth camera on the base sees is laid over a floor plan of
what it saw before, and the difference becomes `correction`, the pose of the odometry's
frame in the map's. It only ever changes slowly: the wheels stay in charge of the short
term.
"""

from __future__ import annotations

import math
import os
import threading
import time
from collections import deque

import numpy as np

from .. import world
from ..config import CameraConfig, ToolsConfig
from . import model
from . import native as native_map

# A camera picture is about this old when it arrives. Measured on a Jetson Nano with a D435
# over V4L2: turning on the spot, the pictures line up best with the wheels 50 ms earlier
# (clearly better than 35 or 65 ms).
DEFAULT_LATENCY_S = 0.05
# A camera that says when each picture was taken (the kernel's stamp, "start of exposure")
# is still not quite believed: turning on the spot six times, the D435's depth pictures line
# up best with the wheels 30 to 40 ms before that stamp (10 600 cells of map, against 12 700
# with the stamp as it is).
CAPTURE_DELAY_S = 0.035


class WorldView:
    def __init__(self, config: ToolsConfig, cameras: dict, native: bool | None = None):
        """native: keep the map in obot-navd (C++) instead of in Python. None: if the
        program is there. (The experimental odometry correction is Python only.)"""
        self.config = config.world
        self.cameras = cameras
        self.navd = None
        program = None if native is False or self.config.correct_odometry else native_map.find()
        if native and program is None:
            raise RuntimeError("obot-navd is not there (or correct_odometry is on, which it does not do)")
        if program is not None:
            self.navd = native_map.Navd(program, self.config.voxel, self.config.max_voxels, self.SEEN_CELL,
                                        self.SEEN_HALF, self.WAY_HALF_WIDTH, *self.WAY_HEIGHT)
            self.map = native_map.NativeMap(self.navd, self.STAND)
        else:
            self.map = world.WorldMap(self.config.voxel, self.config.max_voxels)
        self.links = None
        if config.urdf and os.path.isfile(config.urdf):
            self.links = world.LinkPoses(model.load(config.urdf))
        by_name = {c.name: c for c in config.cameras}
        self.depth = [c for c in config.cameras if c.is_depth and c.name in cameras]
        self.colour = {c.name: by_name[c.color] for c in self.depth if c.color in cameras}
        self.mounts = {c.name: world.transform(c.xyz, c.rpy) for c in config.cameras}
        for c in config.cameras:
            if c.link and (self.links is None or c.link not in self.links.links):
                raise ValueError(f"camera {c.name}: link '{c.link}' is not in the robot model")
        self.joint_names: tuple[str, ...] = ()  # set by whoever calls update_pose with q
        self._on_links = any(c.link for c in self.depth) or any(c.link for c in self.colour.values())
        self._poses: deque = deque(maxlen=250)  # (time, x, y, yaw, yaw rate, joint angles)
        self._lock = threading.Lock()
        self._last: dict[str, float] = {}
        self.leveled: dict[str, dict] = {}
        self.clock = time.monotonic  # what the cameras' stamps are in (a test may use its own)
        # Pose (x, y, yaw) of the wheel odometry's frame in the map's: where the map says
        # the robot is = correction * where the wheels say it is.
        self.correction: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.match: dict = {}  # how the last comparison with the map went, for the page
        self._difference = np.zeros(3)  # average of what the comparisons found: x, y, yaw
        # For driving: how far ahead of the middle of the wheel axis the nearest thing is that
        # a depth camera on the base sees in the base's way (m; inf: nothing), and when.
        self.ahead: tuple[float, float] = (math.inf, 0.0)
        # Also for driving: where the floor has been seen to be free, from above, as a grid
        # of SEEN_CELL-sized squares around the origin of the map (see known()).
        self._seen = np.zeros((2 * self.SEEN_HALF + 1, 2 * self.SEEN_HALF + 1), bool)
        # The long memory of the map, on the same grid: how sure it is that something stands
        # in each square. It goes up by UP (to MOST) with every picture that shows something
        # there, and down by DOWN with every picture that shows floor there instead; from
        # THERE on the square counts as having something in it. Unlike the 3D map it has no
        # size to run out of, and it lets go of what it has seen only slowly.
        self._things = np.zeros(self._seen.shape, np.uint8)
        # How the pictures agree with what was remembered, added up: squares with something
        # in them, of those remembered, of those contradicted (see agreement()).
        self.agree = [0, 0, 0]
        self.note = ""  # for the page: what became of the map from before
        self.plan = world.FloorPlan()
        self.update_ms = 0.0
        self.error: str | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="world", daemon=True)
        for c in config.cameras:  # scene: cameras draw from where the robot currently is
            if c.name in cameras and c.source.startswith("scene:"):
                # ... by its wheels: in the simulation those are the truth
                cameras[c.name].pose = lambda c=c: self.camera_pose(c, corrected=False)

    @property
    def active(self) -> bool:
        return bool(self.depth)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2)

    def close(self) -> None:
        """Stop, and end obot-navd if it keeps the map."""
        self.stop()
        if self.navd is not None:
            self.navd.close()

    # --- where things are --------------------------------------------------------------------

    def update_pose(self, x: float, y: float, yaw: float, yaw_rate: float, q=None, t: float | None = None) -> None:
        """Called with every state the page server reads; q are the joint angles in the
        order of joint_names (only looked at if a camera is on a link of the arm); t is
        when the state was true (on the cameras' clock; default: now)."""
        q = np.array(q, np.float32) if self._on_links and q is not None else None
        with self._lock:
            if t is None or (self._poses and t <= self._poses[-1][0]):
                t = self.clock()
            self._poses.append((t, x, y, yaw, yaw_rate, q))
        here = world.compose(self.correction, (x, y, yaw))
        self.map.add_pose(here[0], here[1])   # (obot-navd notes where the base stands with it)
        if self.navd is None:
            self._stand(here[0], here[1])

    def _pose_at(self, t: float | None):
        """(time, x, y, yaw, yaw rate, joint angles) of the odometry at time t (None: now),
        between the two states it was read at before and after."""
        with self._lock:
            if not self._poses:
                return (0.0, 0.0, 0.0, 0.0, 0.0, None)
            if t is None or t >= self._poses[-1][0]:
                return self._poses[-1]
            after = next((k for k, sample in enumerate(self._poses) if sample[0] >= t), 0)
            if after == 0:
                return self._poses[0]
            a, b = self._poses[after - 1], self._poses[after]
            share = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
            near = a if share < 0.5 else b
            # (the odometry's yaw runs on: it does not jump from pi to -pi)
            return (t, a[1] + (b[1] - a[1]) * share, a[2] + (b[2] - a[2]) * share, a[3] + (b[3] - a[3]) * share,
                    near[4], near[5])

    def base_pose(self, t: float | None = None) -> tuple[float, float, float]:
        """Where the base is in the map (x, y, yaw) at time t (None: now): the odometry,
        corrected."""
        _, x, y, yaw, _, _ = self._pose_at(t)
        return world.compose(self.correction, (x, y, yaw))

    def camera_pose(self, camera: CameraConfig, t: float | None = None, corrected: bool = True) -> np.ndarray:
        """Pose of the camera's optical frame in the world at time t (None: now)."""
        _, x, y, yaw, _, q = self._pose_at(t)
        if corrected:
            x, y, yaw = world.compose(self.correction, (x, y, yaw))
        pose = world.base_pose(x, y, yaw)
        if camera.link:
            joints = {} if q is None else {name: float(v) for name, v in zip(self.joint_names, q, strict=False)}
            pose = pose @ self.links.pose(camera.link, joints)
        return pose @ self.mounts[camera.name] @ world.OPTICAL

    def describe(self) -> dict:
        """For the page: how big the map is and where the cameras are (rows of the 3 x 4
        pose of each optical frame)."""
        return {
            "voxels": len(self.map),
            "update_ms": round(self.update_ms, 1),
            "error": self.error,
            "leveled": self.leveled,
            "note": self.note,
            "correction": [round(v, 4) for v in self.correction],
            "match": self.match,
            "cameras": {c.name: [round(float(v), 4) for v in self.camera_pose(c)[:3].ravel()] for c in self.depth},
        }

    def frustums(self) -> list[dict]:
        out = []
        for c in self.depth:
            k = c.intrinsics()
            out.append({"name": c.name, "tan_x": k.width / 2 / k.fx, "tan_y": k.height / 2 / k.fy})
        return out

    # --- building the map ----------------------------------------------------------------------

    def step(self) -> int:
        """Use every depth picture that is new. Returns how many were used."""
        used = 0
        for c in self.depth:
            stamp, depth = self.cameras[c.name].raw()
            if depth is None or stamp == self._last.get(c.name):
                continue
            self._last[c.name] = stamp
            # When was it taken? The stamp says, if the camera knows; otherwise it is when the
            # picture arrived, which is later.
            latency = c.latency
            if latency is None:
                if c.source.startswith("scene:"):
                    latency = 0.0
                else:
                    stamped = getattr(self.cameras[c.name], "stamps_capture", False)
                    latency = CAPTURE_DELAY_S if stamped else DEFAULT_LATENCY_S
            taken = stamp - latency
            k = c.intrinsics()
            if self.navd is not None:
                used += self._step_native(c, depth, k, stamp, taken)
                continue
            points = world.depth_to_points(depth, k, self.config.stride, self.config.near, self.config.range)
            if not c.link:
                self._look_ahead(c, points, stamp, depth)
            if abs(self._pose_at(taken)[4]) > self.config.max_turn_rate:
                continue  # a small error in time is a large error in place while turning
            if self.config.correct_odometry and not c.link:
                self._localize(c, points, taken)
            pose = self.camera_pose(c, taken)
            if not c.link:
                self._see_floor(c, points, taken, self.config.stride / k.fx)
            self.map.carve(pose, depth, k, self.config.near, self.config.range)
            rgb = None
            colour = self.colour.get(c.name)
            if colour is not None:
                colour_stamp, picture = self.cameras[colour.name].raw()
                if picture is not None and abs(colour_stamp - stamp) < 0.5:
                    to_colour = world.invert(self.camera_pose(colour, taken)) @ pose
                    rgb = world.sample_colors(world.apply(to_colour, points), picture, colour.intrinsics())
            self.map.integrate(world.apply(pose, points), rgb)
            used += 1
        return used

    UP, DOWN, MOST, THERE = 8, 2, 80, 16
    SEEN_CELL = 0.06  # m
    SEEN_HALF = 250  # cells: the grid reaches 15 m each way
    STAND = 0.24  # m: where the base stands, this much around the middle of its wheel axis is free

    def _stand(self, x: float, y: float) -> None:
        """Where the base is, there is nothing else."""
        n = int(self.STAND / self.SEEN_CELL) + 1
        i = int(math.floor(x / self.SEEN_CELL)) + self.SEEN_HALF
        j = int(math.floor(y / self.SEEN_CELL)) + self.SEEN_HALF
        limit = 2 * self.SEEN_HALF - n
        if getattr(self, "_stood", None) == (i, j) or not (n <= i < limit and n <= j < limit):
            return
        self._stood = (i, j)
        rows, columns = np.ogrid[-n:n + 1, -n:n + 1]
        disc = (rows ** 2 + columns ** 2) * self.SEEN_CELL ** 2 <= self.STAND ** 2
        self._seen[j - n:j + n + 1, i - n:i + n + 1] |= disc

    def _see_floor(self, camera: CameraConfig, points: np.ndarray, taken: float, step: float) -> None:
        """Note which floor a depth camera on the base sees to be free.

        Where the camera sees the floor, the floor is free. And so is the stretch between
        two places where it sees the floor one behind the other in the same direction with
        nothing standing in between: the line of sight to the farther one passes so low
        over that stretch that anything on it would have been in the way. The same goes for
        the stretch from the camera to the first floor it sees in a direction."""
        mount = self.mounts[camera.name]
        p = world.apply(mount @ world.OPTICAL, points)
        floor = np.abs(p[:, 2]) < 0.04
        thing = (p[:, 2] > self.WAY_HEIGHT[0]) & (p[:, 2] < self.WAY_HEIGHT[1])
        use = floor | thing
        x, y, yaw = self.base_pose(taken)
        c, s = math.cos(yaw), math.sin(yaw)
        side = 2 * self.SEEN_HALF + 1

        def squares(bx, by):
            """Which squares of the grid places in the base frame are in (those inside it)."""
            i = np.floor((x + c * bx - s * by) / self.SEEN_CELL).astype(np.int64) + self.SEEN_HALF
            j = np.floor((y + s * bx + c * by) / self.SEEN_CELL).astype(np.int64) + self.SEEN_HALF
            inside = (i >= 0) & (i < side) & (j >= 0) & (j < side)
            return j[inside] * side + i[inside]

        # Where something stands in this picture: the squares with at least two readings of
        # it. How that sits with what was remembered, and then: more sure of it.
        tx, ty = p[thing, 0].astype(np.float64), p[thing, 1].astype(np.float64)
        ti = np.floor((x + c * tx - s * ty) / self.SEEN_CELL).astype(np.int64) + self.SEEN_HALF
        tj = np.floor((y + s * tx + c * ty) / self.SEEN_CELL).astype(np.int64) + self.SEEN_HALF
        inside = (ti >= 0) & (ti < side) & (tj >= 0) & (tj < side)
        far = np.hypot(tx - mount[0, 3], ty - mount[1, 3])[inside]
        standing, where, readings = np.unique(tj[inside] * side + ti[inside], return_inverse=True, return_counts=True)
        nearest = np.full(len(standing), np.inf)
        np.minimum.at(nearest, where.ravel(), far)
        standing, nearest = standing[readings >= 2], nearest[readings >= 2]
        things = self._things.reshape(-1)
        if len(standing):
            there = self._things >= self.THERE
            j, i = np.divmod(standing, side)
            near = np.zeros(len(standing), bool)
            for dj in (-1, 0, 1):
                for di in (-1, 0, 1):
                    nj, ni = j + dj, i + di
                    ok = (nj >= 0) & (nj < side) & (ni >= 0) & (ni < side)
                    near[ok] |= there[nj[ok], ni[ok]]
            self.agree[0] += len(standing)
            self.agree[1] += int(near.sum())
            self.agree[2] += int((~near & self._seen.reshape(-1)[standing] & (things[standing] == 0)).sum())
            # What is seen from close by counts for more: the camera is good to half a
            # centimetre at 1 m and to 4 cm at 3 m, which is most of a square.
            up = np.where(nearest < 1.5, self.UP, np.where(nearest < 2.5, self.UP // 2, self.UP // 4))
            things[standing] = np.minimum(self.MOST, things[standing].astype(np.int64) + up)
        if not floor.any():
            return
        dx, dy = p[use, 0] - mount[0, 3], p[use, 1] - mount[1, 3]
        is_floor = floor[use]
        distance = np.hypot(dx, dy)
        direction = np.rint(np.arctan2(dy, dx) / step).astype(np.int64)
        order = np.lexsort((distance, direction))
        direction, distance, is_floor = direction[order], distance[order], is_floor[order]
        dx, dy = dx[order], dy[order]
        # pairs of floor readings that follow each other in one direction
        pair = np.nonzero((direction[1:] == direction[:-1]) & is_floor[1:] & is_floor[:-1]
                          & (distance[1:] - distance[:-1] < 1.0))[0]
        ends = [np.stack([dx[is_floor], dy[is_floor]], axis=1)]
        t = np.linspace(0.0, 1.0, int(1.0 / (self.SEEN_CELL / 2)) + 2)[None, :, None]
        long = pair[distance[pair + 1] - distance[pair] > self.SEEN_CELL / 2]
        if len(long):
            a = np.stack([dx[long], dy[long]], axis=1)[:, None, :]
            b = np.stack([dx[long + 1], dy[long + 1]], axis=1)[:, None, :]
            ends.append((a + (b - a) * t).reshape(-1, 2))
        first = np.nonzero(np.r_[True, direction[1:] != direction[:-1]] & is_floor & (distance < 1.0))[0]
        if len(first):
            ends.append((np.stack([dx[first], dy[first]], axis=1)[:, None, :] * t).reshape(-1, 2))
        local = np.concatenate(ends) + mount[:2, 3]
        free = np.unique(squares(local[:, 0], local[:, 1]))
        self._seen.reshape(-1)[free] = True
        # floor where something was remembered (and is not seen now): a little less sure of it
        fading = free[~np.isin(free, standing)]
        things[fading] = np.maximum(0, things[fading].astype(np.int64) - self.DOWN)

    def remembered(self) -> np.ndarray:
        """The squares of the floor plan in which something is remembered to stand: (i, j)
        each, the square reaching from (i * SEEN_CELL, j * SEEN_CELL)."""
        if self.navd is not None:
            return self.navd.things()
        rows, columns = np.nonzero(self._things >= self.THERE)
        return np.stack([columns, rows], axis=1) - self.SEEN_HALF

    def obstacles(self) -> np.ndarray:
        """Where the base cannot drive, (x, y) each: what stands there in the 3D map now, and
        what the floor plan remembers to stand there."""
        now = self.map.obstacles(*self.WAY_HEIGHT)
        remembered = self.remembered()
        # Where the 3D map has the thing, it says where to 3 cm. What is only remembered is
        # known to a square of 6 cm: all of the square counts (nine places, a voxel apart).
        if len(now) and len(remembered):
            # (a voxel is 3 cm wide: it is in every square that one of its corners is in)
            corners = np.array([(-0.5, -0.5), (-0.5, 0.5), (0.5, -0.5), (0.5, 0.5)]) * self.map.voxel * 0.999
            seen_now = np.floor((now[:, None, :] + corners[None]) / self.SEEN_CELL).astype(np.int64).reshape(-1, 2)
            key = lambda squares: squares[:, 0] * 100_003 + squares[:, 1]  # noqa: E731
            remembered = remembered[~np.isin(key(remembered.astype(np.int64)), key(seen_now))]
        spread = np.array([(a, b) for a in (0.0, 0.5, 1.0) for b in (0.0, 0.5, 1.0)])
        return np.concatenate([now, ((remembered[:, None, :] + spread[None]) * self.SEEN_CELL).reshape(-1, 2)])

    GRID_MAGIC = 0x3147424F  # "OBG1"

    def grid(self, keep_clear: float = 0.29) -> bytes:
        """The map from above, for the page: what stands on the floor, where the floor has
        been seen to be free, and where the base cannot be for being too close to something.

        Layout (little endian): u32 magic; i32 i0, j0; u32 width, height; f32 cell size;
        then height rows of width bytes, the square [j][i] reaching from ((i + i0) * cell,
        (j + j0) * cell): 0 not seen, 1 free floor, 2 something there, 3 free but closer to
        something than `keep_clear` (the middle of the base cannot be there), 4 something
        remembered to be there that the 3D map does not have now."""
        if self.navd is not None:
            return self.navd.grid(keep_clear)
        import struct

        import cv2  # here so that the rest of the module works without OpenCV

        cell, half = self.SEEN_CELL, self.SEEN_HALF
        occupied = np.zeros_like(self._seen)
        things = self.map.obstacles(*self.WAY_HEIGHT)
        if len(things):
            index = np.floor(things / cell).astype(np.int64) + half
            ok = np.all((index >= 0) & (index <= 2 * half), axis=1)
            occupied[index[ok, 1], index[ok, 0]] = True
        remembered = (self._things >= self.THERE) & ~occupied
        rows, columns = np.nonzero(self._seen | occupied | remembered)
        if not len(rows):
            return struct.pack("<IiiIIf", self.GRID_MAGIC, 0, 0, 0, 0, cell)
        pad = int(0.5 / cell)
        a, b = max(0, rows.min() - pad), min(2 * half, rows.max() + pad) + 1
        c, d = max(0, columns.min() - pad), min(2 * half, columns.max() + pad) + 1
        occupied, seen, remembered = occupied[a:b, c:d], self._seen[a:b, c:d], remembered[a:b, c:d]
        distance = cv2.distanceTransform((~(occupied | remembered)).astype(np.uint8), cv2.DIST_L2,
                                         cv2.DIST_MASK_PRECISE)
        near = distance * cell < keep_clear
        cells = np.where(occupied, 2, np.where(remembered, 4, np.where(seen, np.where(near, 3, 1), 0)))
        cells = cells.astype(np.uint8)
        return struct.pack("<IiiIIf", self.GRID_MAGIC, c - half, a - half, d - c, b - a, cell) + cells.tobytes()

    def known(self, origin, shape) -> np.ndarray:
        """Which squares of a grid of SEEN_CELL have been seen to be free floor: the grid's
        square [j, i] reaches from ((i + origin[0]) * SEEN_CELL, (j + origin[1]) * SEEN_CELL)."""
        if self.navd is not None:
            return self.navd.known(origin, shape)
        out = np.zeros(shape, bool)
        i0, j0 = int(origin[0]) + self.SEEN_HALF, int(origin[1]) + self.SEEN_HALF
        a, b = max(0, -j0), min(shape[0], 2 * self.SEEN_HALF + 1 - j0)
        c, d = max(0, -i0), min(shape[1], 2 * self.SEEN_HALF + 1 - i0)
        if a < b and c < d:
            out[a:b, c:d] = self._seen[j0 + a:j0 + b, i0 + c:i0 + d]
        return out

    WAY_HALF_WIDTH = 0.245  # m: half the width of the base (0.224), and a little
    WAY_HEIGHT = (0.05, 0.7)  # m above the floor: what is lower the wheels roll over

    def _look_ahead(self, camera: CameraConfig, points: np.ndarray, stamp: float,
                    depth: np.ndarray) -> None:
        """Note how far ahead the nearest thing in the base's way is (self.ahead)."""
        # A depth camera gives no reading for what is too close to it. So when the middle
        # of the picture is mostly without readings, something may be right in front.
        rows, columns = depth.shape
        middle = depth[rows // 4: rows * 9 // 10: 4, columns * 3 // 10: columns * 7 // 10: 4]
        if (middle == 0).mean() > 0.5:
            self.ahead = (0.0, stamp)
            return
        p = world.apply(self.mounts[camera.name] @ world.OPTICAL, points)
        height = (p[:, 2] > self.WAY_HEIGHT[0]) & (p[:, 2] < self.WAY_HEIGHT[1])
        way = p[height & (np.abs(p[:, 1]) < self.WAY_HALF_WIDTH) & (p[:, 0] > 0.0), 0]
        # a dozen readings, not one: single wrong readings are common
        self.ahead = (float(np.partition(way, 11)[11]) if len(way) > 11 else math.inf, stamp)

    # --- where the robot is in the map -------------------------------------------------------

    MIN_MATCHED = 30  # scan points that have to lie on the plan for a comparison to count
    MIN_SHARE = 0.3  # ... and as a share of the scan: less is mostly something new
    MAX_STEP = (0.25, 0.25)  # m, rad: a comparison that disagrees with the wheels by more is not believed
    # One comparison is good to about a map cell, and which way it is off changes from
    # picture to picture. So the differences found are averaged (each new one counts this
    # much), and the average is only acted on once it is more than a comparison could be
    # wrong by: m, rad. Drift is steady and gets through; the scatter does not.
    SMOOTH = 0.3
    SAME = (0.015, 0.005)

    def _localize(self, camera: CameraConfig, points: np.ndarray, taken: float) -> None:
        """Compare what a depth camera on the base sees (points in its optical frame) with
        the map as it is, and move `correction` towards where they agree."""
        _, x, y, yaw, _, _ = self._pose_at(taken)
        guess = world.compose(self.correction, (x, y, yaw))
        mount = self.mounts[camera.name]
        k = camera.intrinsics()
        scan, normals = world.scan_surfaces(world.scan_2d(
            world.apply(mount @ world.OPTICAL, points), mount[:2, 3], self.config.stride / k.fx))
        on_a_surface = np.any(normals != 0, axis=1)
        report = {"scan": int(on_a_surface.sum()), "used": False}
        plan = self.plan
        new = np.ones(len(scan), bool)
        try:
            if on_a_surface.sum() < self.MIN_MATCHED:
                return
            found, new = plan.match(scan, normals, guess)
            if found is None:
                return
            off = math.hypot(found.pose[0] - guess[0], found.pose[1] - guess[1])
            turned = abs(world.wrap_angle(found.pose[2] - guess[2]))
            report.update(share=round(found.share, 2), directions=found.directions,
                          off=[round(off, 3), round(turned, 4)])
            if (found.matched < self.MIN_MATCHED or found.share < self.MIN_SHARE or off > self.MAX_STEP[0]
                    or turned > self.MAX_STEP[1]):
                return
            report["used"] = True
            found_now = np.array([found.pose[0] - guess[0], found.pose[1] - guess[1],
                                  world.wrap_angle(found.pose[2] - guess[2])])
            self._difference += self.SMOOTH * (found_now - self._difference)
            dx, dy, dyaw = self._difference
            if math.hypot(dx, dy) < self.SAME[0] and abs(dyaw) < self.SAME[1]:
                return
            target = (guess[0] + dx, guess[1] + dy, guess[2] + dyaw)
            cx, cy, cyaw = world.compose(target, world.inverse((x, y, yaw)))
            self.correction = (cx, cy, world.wrap_angle(cyaw))
            self._difference[:] = 0
        finally:
            # What is new goes into the plan, where the robot now takes itself to be. Where
            # the comparison was believed that is everything the plan has nothing next to;
            # where it was not, only what is well away from all the plan has.
            here = world.compose(self.correction, (x, y, yaw))
            c, s = math.cos(here[2]), math.sin(here[2])
            add = on_a_surface & (new if not report["used"] and len(plan) else True)
            turned_to_map = np.array([[c, -s], [s, c]])
            report["plan"] = len(plan) + plan.add(scan[add] @ turned_to_map.T + here[:2],
                                                  normals[add] @ turned_to_map.T)
            self.match = report  # whole, in one go: the page's thread reads it

    def _step_native(self, c: CameraConfig, depth: np.ndarray, k, stamp: float, taken: float) -> int:
        """One depth picture, handed to obot-navd with where it was taken from."""
        for_map = abs(self._pose_at(taken)[4]) <= self.config.max_turn_rate
        on_base = not c.link
        if not for_map and not on_base:
            return 0
        pose = self.camera_pose(c, taken)
        colour = None
        partner = self.colour.get(c.name)
        if for_map and partner is not None:
            colour_stamp, picture = self.cameras[partner.name].raw()
            if picture is not None and abs(colour_stamp - stamp) < 0.5:
                colour = (picture, partner.intrinsics(), world.invert(self.camera_pose(partner, taken)) @ pose)
        ahead, _, _, shown, agreement = self.navd.picture(
            depth, k, self.config.stride, self.config.near, self.config.range, self.mounts[c.name] @ world.OPTICAL,
            pose, self.base_pose(taken), on_base, for_map, colour)
        self.map.shown = shown
        for k in range(3):
            self.agree[k] += agreement[k]
        if ahead is not None:
            self.ahead = (ahead, stamp)
        return int(for_map)

    def _run(self) -> None:
        period = 1.0 / max(0.1, self.config.rate)
        while not self._stop.is_set():
            start = time.monotonic()
            try:
                if self.step():
                    self.error = None
                    elapsed = (time.monotonic() - start) * 1000
                    self.update_ms = elapsed if not self.update_ms else 0.9 * self.update_ms + 0.1 * elapsed
            except Exception as e:  # a camera that changed its picture size, ...: say so and go on
                self.error = str(e)
            self._stop.wait(max(0.01, period - (time.monotonic() - start)))

    # --- keeping the map ---------------------------------------------------------------------

    def save(self, path: str, frame: str) -> None:
        """Write the map to a file. `frame` names the odometry it is in (a run of obot-rtd):
        with other odometry the same numbers mean other places."""
        if self.navd is not None:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            self.navd.save(path, frame)
            return
        saved = self.map.dump()
        rows, columns = np.nonzero(self._seen)
        thing_rows, thing_columns = np.nonzero(self._things)
        saved.update(frame=np.array(frame), seen=np.stack([rows, columns], axis=1).astype(np.int32),
                     things=np.stack([thing_rows, thing_columns, self._things[thing_rows, thing_columns]],
                                     axis=1).astype(np.int32),
                     correction=np.asarray(self.correction))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path + ".part", "wb") as f:
            np.savez_compressed(f, **saved)
            f.flush()
            os.fsync(f.fileno())
        os.replace(path + ".part", path)

    def load(self, path: str, frame: str) -> bool:
        """Take up the map saved in a file, if it is in the odometry `frame` ("*": whichever
        it is in; whoever asks knows how that relates to the odometry now). Returns whether
        it was."""
        if self.navd is not None:
            if not self.navd.load(path, frame):
                return False
            self.map.shown = self.navd.count()["shown"]
            return True
        try:
            with np.load(path) as saved:
                if frame != "*" and str(saved["frame"]) != frame:
                    return False
                self.map.restore(saved)
                self._seen[:] = False
                self._seen[saved["seen"][:, 0], saved["seen"][:, 1]] = True
                self._things[:] = 0
                self._things[saved["things"][:, 0], saved["things"][:, 1]] = saved["things"][:, 2]
            return True
        except Exception:  # not there, not a map, or a map of another kind (obot-navd's own)
            return False

    def clear(self) -> None:
        self.map.clear()
        self.plan = world.FloorPlan()
        self._seen[:] = False
        self._things[:] = 0
        self._stood = None

    def floor_seen(self) -> int:
        """How many squares of floor have been seen to be free."""
        return self.navd.count()["floor"] if self.navd is not None else int(self._seen.sum())

    def level(self) -> dict[str, dict]:
        """Work out how each base-mounted depth camera is tilted and how high it is from the
        floor it sees, use that from now on, and start the map again. Returns, per camera,
        the xyz and rpy to put in the configuration (or an error)."""
        result = {}
        for c in self.depth:
            if c.link:
                result[c.name] = {"error": "is on a moving link; only base cameras can be levelled"}
                continue
            _, depth = self.cameras[c.name].raw()
            if depth is None:
                result[c.name] = {"error": "no picture yet"}
                continue
            points = world.depth_to_points(depth, c.intrinsics(), self.config.stride, self.config.near,
                                           self.config.range)
            fit = world.fit_floor(world.apply(world.OPTICAL, points))
            if fit is None:
                result[c.name] = {"error": "no floor found: it has to fill a good part of the picture"}
                continue
            height, roll, pitch, share = fit
            old = self.mounts[c.name]
            yaw = math.atan2(old[1, 0], old[0, 0])
            xyz = [float(old[0, 3]), float(old[1, 3]), height]
            rpy = [roll, pitch, yaw]
            new = world.transform(xyz, rpy)
            # A colour camera that belongs to this one keeps its place relative to it.
            partner = self.colour.get(c.name)
            if partner is not None and not partner.link:
                self.mounts[partner.name] = new @ world.invert(old) @ self.mounts[partner.name]
            self.mounts[c.name] = new
            result[c.name] = {"xyz": [round(v, 3) for v in xyz], "rpy": [round(v, 3) for v in rpy],
                              "floor_share": round(share, 2)}
        self.leveled = result
        self.clear()
        return result
