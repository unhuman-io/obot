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

DEFAULT_LATENCY_S = 0.08  # a camera picture is about this old when it arrives


class WorldView:
    def __init__(self, config: ToolsConfig, cameras: dict):
        self.config = config.world
        self.cameras = cameras
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
        # Pose (x, y, yaw) of the wheel odometry's frame in the map's: where the map says
        # the robot is = correction * where the wheels say it is.
        self.correction: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.match: dict = {}  # how the last comparison with the map went, for the page
        self._difference = np.zeros(3)  # average of what the comparisons found: x, y, yaw
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

    # --- where things are --------------------------------------------------------------------

    def update_pose(self, x: float, y: float, yaw: float, yaw_rate: float, q=None) -> None:
        """Called with every state the page server reads; q are the joint angles in the
        order of joint_names (only looked at if a camera is on a link of the arm)."""
        q = np.array(q, np.float32) if self._on_links and q is not None else None
        with self._lock:
            self._poses.append((time.monotonic(), x, y, yaw, yaw_rate, q))
        here = world.compose(self.correction, (x, y, yaw))
        self.map.add_pose(here[0], here[1])

    def _pose_at(self, t: float | None):
        with self._lock:
            if not self._poses:
                return (0.0, 0.0, 0.0, 0.0, 0.0, None)
            if t is None:
                return self._poses[-1]
            return min(self._poses, key=lambda sample: abs(sample[0] - t))

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
            latency = DEFAULT_LATENCY_S if c.latency is None else c.latency
            if c.source.startswith("scene:") and c.latency is None:
                latency = 0.0
            taken = stamp - latency
            if abs(self._pose_at(taken)[4]) > self.config.max_turn_rate:
                continue  # a small error in time is a large error in place while turning
            k = c.intrinsics()
            points = world.depth_to_points(depth, k, self.config.stride, self.config.near, self.config.range)
            if self.config.correct_odometry and not c.link:
                self._localize(c, points, taken)
            pose = self.camera_pose(c, taken)
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

    def clear(self) -> None:
        self.map.clear()
        self.plan = world.FloorPlan()

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
