"""obot-navd keeps the same map as the Python code it replaces: the same pictures and poses
go to both, and what they make of them is compared."""

import math
import struct
import time

import numpy as np
import pytest

from obot import world
from obot.config import CameraConfig, ToolsConfig
from obot.world import WorldMap

pytest.importorskip("cv2")
from obot.ui import native as native_map  # noqa: E402
from obot.ui import scene  # noqa: E402
from obot.ui.worldview import WorldView  # noqa: E402

pytestmark = pytest.mark.skipif(native_map.find() is None,
                                reason="obot-navd is not built (cmake --preset release in rt/)")


class StillCamera:
    def __init__(self):
        self.stamp, self.frame = 0.0, None

    def raw(self):
        return self.stamp, self.frame


def both(max_voxels=250_000):
    config = ToolsConfig()
    mount = {"xyz": (0.116, 0.0175, 0.143), "rpy": (0.0, 0.0, 0.0), "width": 320, "height": 240, "latency": 0.0}
    config.cameras = [CameraConfig(name="colour", source="realsense:color", **mount),
                      CameraConfig(name="depth", source="realsense:depth", color="colour", **mount)]
    config.world.stride = 2
    config.world.max_voxels = max_voxels
    views = []
    for native in (False, True):
        cameras = {"colour": StillCamera(), "depth": StillCamera()}
        view = WorldView(config, cameras, native=native)
        clock = {"t": 0.0}
        view.clock = lambda clock=clock: clock["t"]
        views.append((view, cameras, clock))
    return config, views


def show(config, views, pose, turning=0.0):
    """Both views get the same pictures of the made-up room from the same place."""
    depth_camera = config.cameras[1]
    mount = world.transform(depth_camera.xyz, depth_camera.rpy) @ world.OPTICAL
    depth, picture = scene.render(world.base_pose(*pose) @ mount, depth_camera.intrinsics())
    for view, cameras, clock in views:
        clock["t"] += 0.2
        view.update_pose(*pose, turning)
        cameras["depth"].frame, cameras["depth"].stamp = depth, clock["t"]
        cameras["colour"].frame, cameras["colour"].stamp = picture, clock["t"]
        view.step()


def shown(view):
    update = WorldMap.decode(view.map.delta())
    return {tuple(i): tuple(c) for i, c in zip(update["changed"].tolist(), update["rgb"].tolist(), strict=True)}


def same_share(a: set, b: set) -> float:
    return len(a & b) / max(1, len(a | b))


@pytest.fixture
def room(monkeypatch):
    walls = (((3.0, 0.0, 0.75), (0.1, 3.7, 1.5), (216, 212, 204)),
             ((1.0, 1.85, 0.75), (4.1, 0.1, 1.5), (216, 212, 204)),
             ((1.0, -1.85, 0.75), (4.1, 0.1, 1.5), (216, 212, 204)))
    monkeypatch.setattr(scene, "BOXES", scene.BOXES + walls)


def test_the_same_map_from_the_same_pictures(room, tmp_path):
    config, views = both()
    (python, _, _), (native, _, _) = views
    assert native.navd is not None and python.navd is None
    # a drive through the room: forward, a turn on the spot (some of it too fast to map), on
    path = [(x, 0.0, 0.0, 0.0) for x in np.arange(0, 0.5, 0.05)]
    path += [(0.45, 0.0, yaw, 1.5 if 0.5 < yaw < 0.8 else 0.3) for yaw in np.arange(0.05, 1.6, 0.05)]
    path += [(0.45, d, math.pi / 2, 0.0) for d in np.arange(0.05, 0.5, 0.05)]
    for x, y, yaw, turning in path:
        show(config, views, (x, y, yaw), turning)
        assert native.ahead[0] == pytest.approx(python.ahead[0], abs=1e-3) or native.ahead[0] == python.ahead[0]
    try:
        # The voxels shown: the same ones (a reading that falls on the border between two
        # cells may go either way, the two computing in slightly different order), in the same colours.
        a, b = shown(python), shown(native)
        assert len(a) > 5000
        assert same_share(set(a), set(b)) > 0.999
        common = set(a) & set(b)
        assert sum(a[v] == b[v] for v in common) / len(common) > 0.995
        assert len(native.map) == len(b) and abs(len(native.map) - len(python.map)) < 10
        # What stands in the way of the base, and where the floor has been seen to be free.
        things = lambda view: {tuple(p) for p in np.rint(view.map.obstacles() / 0.03).astype(int).tolist()}  # noqa: E731
        assert same_share(things(python), things(native)) > 0.999
        known_a, known_b = python.known((-60, -60), (130, 130)), native.known((-60, -60), (130, 130))
        assert known_a.sum() > 1500 and (known_a != known_b).sum() <= 3
        assert abs(python.floor_seen() - native.floor_seen()) <= 3
        # The map from above, as the page draws it.
        grid_a, grid_b = python.grid(), native.grid()
        assert grid_a[:24] == grid_b[:24]
        cells_a, cells_b = np.frombuffer(grid_a, np.uint8, offset=24), np.frombuffer(grid_b, np.uint8, offset=24)
        assert (cells_a != cells_b).mean() < 0.001
        # The trail, and an update for a page that has part of the map already.
        update_a, update_b = WorldMap.decode(python.map.delta()), WorldMap.decode(native.map.delta())
        assert np.allclose(update_a["trail"], update_b["trail"], atol=1e-6) and len(update_a["trail"]) > 15
        show(config, views, (0.45, 0.5, math.pi / 2 - 0.3))
        later_a = WorldMap.decode(python.map.delta(update_a["rev"], update_a["epoch"], len(update_a["trail"])))
        later_b = WorldMap.decode(native.map.delta(update_b["rev"], update_b["epoch"], len(update_b["trail"])))
        assert not later_b["reset"] and abs(len(later_a["changed"]) - len(later_b["changed"])) < 15
        assert 0 < len(later_b["changed"]) < len(b) / 2
        assert abs(len(later_a["removed"]) - len(later_b["removed"])) < 15

        # Kept across a restart, for the same odometry only.
        path_saved = str(tmp_path / "state" / "map.bin")
        native.save(path_saved, "run 1")
        before = shown(native)
        native.clear()
        assert len(native.map) == 0 and native.floor_seen() == 0 and shown(native) == {}
        assert not native.load(path_saved, "run 2") and len(native.map) == 0
        assert native.load(path_saved, "run 1")
        assert shown(native) == before and len(native.map) == len(before)
        assert (native.known((-60, -60), (130, 130)) != known_b).sum() < 200   # (one more picture since)
        assert not python.load(path_saved, "run 1")                            # the Python map does not read it
        assert not native.load(str(tmp_path / "nothing"), "run 1")
    finally:
        native.close()


def test_a_full_map_forgets_the_same(room):
    config, views = both(max_voxels=20000)
    (python, _, _), (native, _, _) = views
    try:
        for yaw in np.arange(0, 2 * math.pi, 0.1):
            show(config, views, (0.0, 0.0, yaw), 0.2)
        a, b = set(shown(python)), set(shown(native))
        # (far fewer than the turn saw, which is some 60 000; which ones go is the same)
        assert 2000 < len(a) < 40000 and same_share(a, b) > 0.99, (len(a), len(b), same_share(a, b))
    finally:
        native.close()


def test_obot_navd_is_much_faster(room):
    config, views = both()
    (python, cameras, clock), (native, _, _) = views
    try:
        poses = [(0.0, 0.0, yaw) for yaw in np.arange(0, 2 * math.pi, 0.05)]
        for pose in poses:
            show(config, views, pose, 0.2)
        took = {}
        depth_camera = config.cameras[1]
        mount = world.transform(depth_camera.xyz, depth_camera.rpy) @ world.OPTICAL
        frames = [scene.render(world.base_pose(*pose) @ mount, depth_camera.intrinsics()) for pose in poses[:40]]
        for name, (view, cams, clk) in zip(("python", "obot-navd"), views, strict=True):
            start = time.perf_counter()
            for pose, (depth, picture) in zip(poses, frames, strict=False):
                clk["t"] += 0.2
                view.update_pose(*pose, 0.2)
                cams["depth"].frame, cams["depth"].stamp = depth, clk["t"]
                cams["colour"].frame, cams["colour"].stamp = picture, clk["t"]
                view.step()
            took[name] = (time.perf_counter() - start) / len(frames) * 1e3
        print(f"a map update with {len(python.map)} voxels: Python {took['python']:.1f} ms, "
              f"obot-navd {took['obot-navd']:.1f} ms")
        assert took["obot-navd"] < took["python"] / 3
        assert struct.unpack_from("<I", native.grid())[0] == 0x3147424F
    finally:
        native.close()
