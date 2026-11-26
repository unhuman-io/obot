"""The 3D map of the surroundings: geometry, the voxel map, and the whole path from depth
pictures of a made-up room to what the operator page is sent."""

import asyncio
import json
import math
import time

import numpy as np
import pytest

from obot import world
from obot.config import CameraConfig, ToolsConfig
from obot.world import Intrinsics, WorldMap

K = Intrinsics.from_hfov(160, 120, 79.6)


def test_a_camera_looking_straight_down_sees_the_floor_below_it():
    # Body frame: x is the view. Pitch +90 degrees turns x from forward to down.
    pose = world.base_pose(2.0, 1.0, 0.0) @ world.transform((0, 0, 1.0), (0, math.pi / 2, 0)) @ world.OPTICAL
    depth = np.full((120, 160), 1000, np.uint16)
    depth[0, :] = 0                                    # no reading
    depth[1, :] = 9000                                 # too far
    points = world.depth_to_points(depth, K, stride=1, near=0.2, far=3.5)
    assert len(points) == 118 * 160
    seen = world.apply(pose, points)
    assert np.allclose(seen[:, 2], 0.0, atol=1e-5)     # 1 m below a camera 1 m up
    assert seen[:, 0].mean() == pytest.approx(2.0, abs=0.02) and seen[:, 1].mean() == pytest.approx(1.0, abs=0.01)
    assert np.ptp(seen[:, 1]) == pytest.approx(2 * math.tan(math.radians(79.6 / 2)), rel=0.02)   # the picture's width
    # The top of the picture is ahead of the robot (+x), its left is the robot's left (+y).
    top_left = world.apply(pose, np.array([[(0 - K.cx) / K.fx, (2 - K.cy) / K.fy, 1.0]]))[0]
    assert top_left[0] > 2.0 and top_left[1] > 1.0


def test_link_poses_follow_the_joints():
    model = {"root": "base", "links": {"base": [], "upper": [], "tool": [], "rail": []}, "joints": [
        {"name": "turn", "type": "revolute", "parent": "base", "child": "upper", "xyz": [0, 0, 1], "rpy": [0, 0, 0],
         "axis": [0, 0, 1]},
        {"name": "nod", "type": "continuous", "parent": "upper", "child": "tool", "xyz": [1, 0, 0], "rpy": [0, 0, 0],
         "axis": [0, 2, 0]},
        {"name": "slide", "type": "prismatic", "parent": "base", "child": "rail", "xyz": [0, 0, 0],
         "rpy": [0, 0, math.pi / 2], "axis": [1, 0, 0]},
    ]}
    links = world.LinkPoses(model)
    assert np.allclose(links.pose("tool", {}), world.transform((1, 0, 1)))
    turned = links.pose("tool", {"turn": math.pi / 2, "nod": math.pi / 2})
    assert np.allclose(turned[:3, 3], (0, 1, 1), atol=1e-9)          # the elbow swung to +y
    assert np.allclose(turned[:3, 0], (0, 0, -1), atol=1e-9)         # and the tool points down
    assert np.allclose(links.pose("rail", {"slide": 0.3})[:3, 3], (0, 0.3, 0), atol=1e-9)
    assert np.allclose(links.pose("tool", {"turn": float("nan")}), world.transform((1, 0, 1)))   # no reading yet
    with pytest.raises(KeyError):
        links.pose("nose", {})


def test_map_shows_what_was_seen_twice_and_sends_only_changes():
    m = WorldMap(voxel=0.1, confirm=2)
    red, blue = (200, 10, 10), (10, 10, 200)
    m.integrate(np.array([[0.01, 0.02, -0.01], [0.04, -0.03, 0.01], [1.0, 0.0, 0.0]]), np.array([red, red, blue]))
    assert len(m) == 0                                               # seen once: could be noise
    first = WorldMap.decode(m.delta())
    assert first["reset"] and len(first["changed"]) == 0             # a new client starts from nothing
    m.integrate(np.array([[0.0, 0.0, 0.0]]), np.array([red]))
    assert len(m) == 1
    update = WorldMap.decode(m.delta(first["rev"], first["epoch"]))
    assert not update["reset"] and update["voxel"] == pytest.approx(0.1)
    assert update["changed"].tolist() == [[0, 0, 0]] and update["rgb"].tolist() == [list(red)]
    # Nothing new: nothing is sent, however often the same thing is seen in the same colour.
    m.integrate(np.array([[0.0, 0.0, 0.0]]), np.array([(205, 12, 8)]))
    assert len(WorldMap.decode(m.delta(update["rev"], update["epoch"]))["changed"]) == 0
    # A different colour is news; a sighting without colour does not erase the colour.
    m.integrate(np.array([[0.0, 0.0, 0.0], [-0.1, -0.1, 0.2]]), np.array([blue, (0, 0, 0)]))
    m.integrate(np.array([[0.0, 0.0, 0.0], [-0.1, -0.1, 0.2]]), None)
    latest = WorldMap.decode(m.delta(update["rev"], update["epoch"]))
    assert sorted(latest["changed"].tolist()) == [[-1, -1, 2], [0, 0, 0]]
    centres, colours = m.points()
    assert sorted(colours.tolist()) == [[0, 0, 0], list(blue)]
    assert sorted(centres.astype(float).round(3).tolist()) == [[-0.1, -0.1, 0.2], [0.0, 0.0, 0.0]]
    # A client from another run of the server, or one that is ahead of this map, starts again.
    assert WorldMap.decode(m.delta(latest["rev"], latest["epoch"] ^ 1))["reset"]
    assert WorldMap.decode(m.delta(latest["rev"] + 100, latest["epoch"]))["reset"]
    # Clearing reaches clients as "forget what you have".
    m.clear()
    cleared = WorldMap.decode(m.delta(latest["rev"], latest["epoch"]))
    assert cleared["reset"] and len(cleared["changed"]) == 0 and len(m) == 0


def test_map_forgets_what_the_camera_now_sees_through():
    m = WorldMap(voxel=0.05, confirm=2, max_hits=3)
    camera = world.transform((0, 0, 0.5)) @ world.OPTICAL            # looking along +x
    thing = np.array([[1.0, 0.0, 0.5]])
    wall = np.array([[2.0, y, z] for y in np.arange(-0.5, 0.5, 0.05) for z in np.arange(0.1, 0.9, 0.05)])
    for _ in range(5):
        m.integrate(np.concatenate([thing, wall]))
    rev = WorldMap.decode(m.delta())
    shown = len(m)
    assert shown == len(wall) + 1

    # The thing is still there: a depth picture with it in front of the wall removes nothing.
    still = np.full((120, 160), 2000, np.uint16)
    still[55:65, 75:85] = 1000
    assert m.carve(camera, still, K) == 0
    # A picture without a reading there says nothing either.
    blind = np.full((120, 160), 2000, np.uint16)
    blind[55:65, 75:85] = 0
    assert m.carve(camera, blind, K) == 0
    # Picked up: the camera measures the wall behind it. It goes after being seen through
    # as often as it had been seen (at most max_hits), and the wall stays.
    empty = np.full((120, 160), 2000, np.uint16)
    assert [m.carve(camera, empty, K) for _ in range(3)] == [0, 0, 1]
    assert len(m) == shown - 1
    update = WorldMap.decode(m.delta(rev["rev"], rev["epoch"]))
    assert update["removed"].tolist() == [[20, 0, 10]] and not update["reset"]


def test_map_is_bounded_and_keeps_what_was_seen_last():
    m = WorldMap(voxel=0.1, max_voxels=100, confirm=1)
    for i in range(30):
        m.integrate(np.array([[x * 0.1, i * 0.1, 0.0] for x in range(10)]))
    assert 80 <= len(m) <= 100
    centres, _ = m.points()
    assert centres[:, 1].min() > 1.5 and centres[:, 1].max() == pytest.approx(2.9)
    # Points far outside what 16 bit cell numbers can address are left out, not wrapped.
    m.integrate(np.array([[5000.0, 0, 0]]))
    assert m.points()[0][:, 0].max() < 2


def test_trail_is_sent_in_pieces():
    m = WorldMap()
    for i in range(10):
        m.add_pose(i * 0.005, 0.0)                                   # 4.5 cm in half-centimetre steps
    first = WorldMap.decode(m.delta())
    assert first["trail"][:, 0].tolist() == pytest.approx([0.0, 0.02, 0.04])
    m.add_pose(1.0, 2.0)
    more = WorldMap.decode(m.delta(first["rev"], first["epoch"], trail=3))
    assert more["trail_start"] == 3 and more["trail"].tolist() == [[1.0, 2.0]]
    m.clear()
    assert len(WorldMap.decode(m.delta(first["rev"], first["epoch"], trail=4))["trail"]) == 0


def test_floor_fit_finds_height_and_tilt():
    rng = np.random.default_rng(1)
    mount = world.transform((0, 0, 0.42), (0.05, 0.6, 0))            # 42 cm up, looking down, slightly rolled
    floor = np.column_stack([rng.uniform(0.2, 1.5, 3000), rng.uniform(-0.6, 0.6, 3000), rng.normal(0, 0.003, 3000)])
    clutter = np.column_stack([rng.uniform(0.5, 0.8, 600), rng.uniform(-0.2, 0.2, 600), rng.uniform(0.0, 0.4, 600)])
    in_camera = world.apply(world.invert(mount), np.concatenate([floor, clutter]))
    height, roll, pitch, share = world.fit_floor(in_camera)
    assert height == pytest.approx(0.42, abs=0.005)
    assert roll == pytest.approx(0.05, abs=0.01) and pitch == pytest.approx(0.6, abs=0.01)
    assert 0.75 < share < 0.95
    assert world.fit_floor(rng.uniform(-1, 1, (2000, 3))) is None    # no surface: no answer


cv2 = pytest.importorskip("cv2")


def test_scene_camera_conventions():
    from obot.ui import scene

    # Half a metre up, looking along +x, level. The box at x = 1.6, y = +0.9 is to the left.
    depth, colour = scene.render(world.transform((0, 0, 0.5)) @ world.OPTICAL, K)
    box = np.all(colour == (82, 111, 138), axis=2)                   # BGR of the box
    rows, columns = np.nonzero(box)
    assert len(columns) > 50 and columns.max() < K.cx
    assert depth[box].min() == pytest.approx(1400, abs=15)           # its near face is at x = 1.4
    assert depth[100, 80] == pytest.approx(500 / ((100 - K.cy) / K.fy), rel=0.01)   # the floor, by similar triangles
    assert depth[50, 80] == pytest.approx(4450, abs=5)               # the wall
    assert depth[10, 80] == 0                                        # over the wall: nothing
    assert (colour[100, 80] == (166, 178, 185)).all() or (colour[100, 80] == (151, 163, 170)).all()


def wait_for(condition, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.05)
    return False


def scene_config() -> ToolsConfig:
    mount = {"xyz": (0.15, 0.0, 0.5), "rpy": (0.0, 0.5, 0.0), "width": 160, "height": 120, "fps": 30}
    config = ToolsConfig()
    config.cameras = [CameraConfig(name="room", source="scene:color", **mount),
                      CameraConfig(name="room depth", source="scene:depth", color="room", **mount)]
    config.world.stride = 2
    config.world.correct_odometry = True
    return config


def test_world_view_maps_the_room_while_the_robot_moves():
    from obot.ui.cameras import Camera
    from obot.ui.worldview import WorldView

    config = scene_config()
    cameras = {}
    view = WorldView(config, cameras)                                # cameras not started yet: nothing to map
    assert not view.active
    cameras.update({c.name: Camera(c) for c in config.cameras})
    view = WorldView(config, cameras)
    try:
        assert view.active
        # Drive 0.6 m forward, then turn on the spot towards the box, in small steps as the
        # odometry would report them.
        path = [(x, 0.0, 0.0) for x in np.linspace(0, 0.6, 13)] + [(0.6, 0.0, yaw) for yaw in np.linspace(0, 0.7, 8)]
        for x, y, yaw in path:
            view.update_pose(x, y, yaw, 0.0)
            before = view.cameras["room depth"].raw()[0]
            # A depth and a colour picture taken from the new place, then one map update.
            assert wait_for(lambda before=before: min(c.raw()[0] for c in cameras.values()) > before + 0.01)
            time.sleep(0.08)
            assert view.step() == 1
        centres, colours = view.map.points()
        assert len(centres) > 2000
        z = centres[:, 2]
        # Everything is where the room has something: the floor, or a box. Nothing floats,
        # nothing is below the floor, and nothing was smeared along the way.
        floor = np.abs(z) < 0.02
        from obot.ui import scene

        in_a_box = np.zeros(len(centres), bool)
        for centre, size, _ in scene.BOXES:
            in_a_box |= np.all(np.abs(centres - centre) <= np.divide(size, 2) + 0.045, axis=1)
        assert (floor | in_a_box).mean() > 0.995
        assert floor.sum() > 1000
        # The box's top (0.45 m up) was found. So was the shirt, in its colour, and the floor
        # in the colours of its tiles. The colour camera sees less than the depth camera,
        # so not everything has a colour.
        top = (np.abs(z - 0.45) < 0.03) & (np.abs(centres[:, 0] - 1.6) < 0.2) & (np.abs(centres[:, 1] - 0.9) < 0.3)
        assert top.sum() > 30
        coloured = colours.any(axis=1)
        assert 0.3 < coloured.mean() < 0.95
        tiles = np.all(colours == (185, 178, 166), axis=1) | np.all(colours == (170, 163, 151), axis=1)
        assert tiles[floor & coloured].mean() > 0.9                  # the rest is the foot of the things on it
        shirt = (np.abs(centres[:, 0] - 0.9) < 0.1) & (np.abs(centres[:, 1] - 0.25) < 0.06) & (z > 0.06) & (z < 0.12)
        assert shirt.sum() > 5 and np.median(colours[shirt], axis=0).tolist() == [201, 79, 79]
        # The trail is the way driven.
        trail = WorldMap.decode(view.map.delta())["trail"]
        assert trail[0].tolist() == [0.0, 0.0] and trail[-1].tolist() == pytest.approx([0.6, 0.0])
        # The odometry was exact, and comparing the pictures with the map left it alone.
        assert view.match["plan"] > 10 and view.correction == (0.0, 0.0, 0.0)
        # While turning fast, pictures are not used.
        view.update_pose(0.6, 0.0, 0.7, 2.0)
        before = view.cameras["room depth"].raw()[0]
        assert wait_for(lambda: view.cameras["room depth"].raw()[0] > before + 0.01)
        assert view.step() == 0
    finally:
        view.stop()
        for camera in cameras.values():
            camera.stop()


def test_level_to_floor_recovers_the_mount():
    from obot.ui.cameras import Camera
    from obot.ui.worldview import WorldView

    config = scene_config()
    cameras = {c.name: Camera(c) for c in config.cameras}
    assert wait_for(lambda: all(c.raw()[1] is not None for c in cameras.values()))
    # The configuration is wrong about the mount: the pictures are of a camera 0.5 m up and
    # pitched 0.5 rad, the configuration says flat on the floor.
    wrong = scene_config()
    for c in wrong.cameras:
        c.xyz, c.rpy = (0.15, 0.0, 0.0), (0.0, 0.0, 0.0)
    view = WorldView(wrong, cameras)
    for camera in cameras.values():
        camera.pose = None                                           # keep drawing from the true mount
    try:
        result = view.level()["room depth"]
        assert result["xyz"] == pytest.approx([0.15, 0.0, 0.5], abs=0.01)
        assert result["rpy"] == pytest.approx([0.0, 0.5, 0.0], abs=0.01)
        # The colour camera moved with it, and the map now puts the floor at z = 0.
        assert np.allclose(view.mounts["room"], view.mounts["room depth"])
        assert view.step() == 1 and view.step() == 0
        time.sleep(0.2)
        view.step()
        centres, _ = view.map.points()
        assert len(centres) > 500 and np.percentile(np.abs(centres[:, 2]), 60) < 0.02
    finally:
        view.stop()
        for camera in cameras.values():
            camera.stop()


def test_configuration_of_mounts_and_world(tmp_path):
    from obot.config import load

    path = tmp_path / "robot.toml"
    path.write_text("""
[[camera]]
name = "colour"
source = "v4l2:/dev/video3"
xyz = [0.1, 0.015, 0.4]
rpy = [0, 0.6, 0]

[[camera]]
name = "depth"
source = "v4l2depth:/dev/video1"
xyz = [0.1, 0.0, 0.4]
rpy = [0, 0.6, 0]
color = "colour"
hfov = 87

[world]
voxel = 0.05
range = 2.5
max_voxels = 1000
""")
    config = load(str(path))
    colour, depth = config.cameras
    assert depth.is_depth and not colour.is_depth and depth.color == "colour"
    assert depth.xyz == (0.1, 0.0, 0.4) and colour.rpy == (0.0, 0.6, 0.0)
    assert depth.intrinsics().fx == pytest.approx(320 / math.tan(math.radians(43.5)))
    assert colour.intrinsics().fx == pytest.approx(614, abs=2)        # the D435's colour camera at 640x480
    assert (config.world.voxel, config.world.range, config.world.max_voxels) == (0.05, 2.5, 1000)
    path.write_text('[[camera]]\nname = "depth"\nsource = "scene:depth"\ncolor = "nope"\n')
    with pytest.raises(ValueError, match="nope"):
        load(str(path))


async def test_page_server_serves_the_map(robot, daemon):
    aiohttp_utils = pytest.importorskip("aiohttp.test_utils")
    from test_ui import make_server, next_state

    ui = make_server(daemon)
    ui.config.cameras += scene_config().cameras
    async with aiohttp_utils.TestClient(aiohttp_utils.TestServer(ui.app())) as client:
        ws = await client.ws_connect("/ws")
        info = json.loads((await ws.receive()).data)
        assert [f["name"] for f in info["world"]] == ["room depth"]
        state = await next_state(ws, lambda s: s["world"] and s["world"]["voxels"] > 500, timeout=15)
        assert len(state["world"]["cameras"]["room depth"]) == 12
        assert state["cameras"]["room depth"]["fps"] > 1
        update = WorldMap.decode(await (await client.get("/api/world")).read())
        assert update["reset"] and len(update["changed"]) > 500
        again = await client.get("/api/world", params={"since": update["rev"], "epoch": update["epoch"]})
        assert len(WorldMap.decode(await again.read())["changed"]) < 200      # only what is new
        assert (await client.get("/api/world", params={"since": "x"})).status == 400

        await ws.send_json({"type": "world", "action": "level"})
        state = await next_state(ws, lambda s: s["world"] and s["world"]["leveled"], timeout=10)
        assert state["world"]["leveled"]["room depth"]["xyz"] == pytest.approx([0.15, 0.0, 0.5], abs=0.01)
        await ws.send_json({"type": "world", "action": "clear"})
        await asyncio.sleep(0.05)
        cleared = WorldMap.decode(await (await client.get(
            "/api/world", params={"since": update["rev"], "epoch": update["epoch"]})).read())
        assert cleared["reset"]


def test_a_camera_on_the_arm_moves_with_the_joints():
    from conftest import CONFIG

    from obot.config import load
    from obot.ui.worldview import WorldView

    config = load(str(CONFIG))
    config.cameras = [CameraConfig(name="wrist depth", source="scene:depth", link="wrist_camera_link",
                                   xyz=(0.01, 0.0, 0.0))]
    view = WorldView(config, {"wrist depth": type("NoCamera", (), {"pose": None})()})
    view.joint_names = ("shoulder_pitch_joint", "elbow_pitch_joint")
    camera = config.cameras[0]
    view.update_pose(1.0, 2.0, 0.0, 0.0, [0.0, 0.0])
    straight = view.camera_pose(camera)
    expected = (world.base_pose(1.0, 2.0, 0.0) @ view.links.pose("wrist_camera_link", {})
                @ world.transform((0.01, 0, 0)) @ world.OPTICAL)
    assert np.allclose(straight, expected)
    view.update_pose(1.0, 2.0, 0.0, 0.0, [0.5, 0.0])
    bent = view.camera_pose(camera)
    assert np.linalg.norm(bent[:3, 3] - straight[:3, 3]) > 0.05      # the shoulder carried it along
    assert np.allclose(bent, world.base_pose(1.0, 2.0, 0.0)
                       @ view.links.pose("wrist_camera_link", {"shoulder_pitch_joint": 0.5})
                       @ world.transform((0.01, 0, 0)) @ world.OPTICAL, atol=1e-6)
    bad = load(str(CONFIG))
    bad.cameras = [CameraConfig(name="d", source="scene:depth", link="no_such_link")]
    with pytest.raises(ValueError, match="no_such_link"):
        WorldView(bad, {"d": None})


# --- the map corrects the odometry -----------------------------------------------------------


def walls_seen_from(true, walls, rng):
    """What points on walls (x, y), 1 m tall, look like from the base at the pose `true`,
    with 1 cm of noise: a scan and its normals."""
    w = np.asarray(walls) + rng.normal(0, 0.01, (len(walls), 2))
    back = world.inverse(true)
    c, s = math.cos(back[2]), math.sin(back[2])
    x, y = back[0] + c * w[:, 0] - s * w[:, 1], back[1] + s * w[:, 0] + c * w[:, 1]
    local = np.concatenate([np.stack([x, y, np.full(len(w), z)], axis=1) for z in (0.2, 0.5, 0.8)])
    return world.scan_surfaces(world.scan_2d(local))


def test_scan_keeps_what_stands_and_drops_what_lies_flat():
    wall = [(1.0, y, z) for y in np.arange(-0.5, 0.5, 0.01) for z in np.arange(0.0, 2.0, 0.05)]
    table = [(x, y, 0.7) for x in np.arange(0.2, 0.8, 0.01) for y in np.arange(-0.3, 0.3, 0.01)]
    scan = world.scan_2d(np.array(wall + table))
    assert len(scan) > 25 and np.allclose(scan[:, 0], 1.0, atol=0.001)     # the wall, not the table or the floor
    normals = world.scan_normals(scan)
    middle = np.abs(scan[:, 1]) < 0.3
    assert np.allclose(np.abs(normals[middle]), (1.0, 0.0), atol=0.05)     # across the wall is along x
    assert world.scan_normals(np.array([(0.0, 0.0), (1.0, 1.0), (2.0, 0.5)])).tolist() == [[0, 0]] * 3   # lone points


def test_floor_plan_match_finds_the_pose_and_leaves_alone_what_it_cannot_tell():
    pytest.importorskip("cv2")
    rng = np.random.default_rng(1)
    # A corner of two walls: one ahead at x = 2, one to the left at y = 1.5.
    ahead = [(2.0, y) for y in np.arange(-1.0, 1.0, 0.005)]
    corner = ahead + [(2.0, y) for y in np.arange(1.0, 1.5, 0.005)] + [(x, 1.5) for x in np.arange(0.8, 2.0, 0.005)]
    plan = world.FloorPlan()
    scan, normals = walls_seen_from((0.0, 0.0, 0.0), corner, rng)
    assert 25 < plan.add(scan, normals) < 60                            # 4.2 m of wall, a point every 10 to 20 cm
    assert plan.add(scan, normals) == 0                                 # nothing goes in twice,
    other = world.FloorPlan()
    other.add(scan, normals)
    for shift in ((0.02, 0.02), (-0.03, 0.03), (0.04, -0.01)):          # nor next to what is there (bar a wall's end):
        assert other.add(scan + shift, normals) <= 1                    # the same wall seen from a wrong place
    assert other.add(scan + (0.3, 0.0), normals) > 15                   # but another wall 30 cm behind does

    # Both walls in view: the pose is found from 15 cm and 7 degrees off, to a centimetre.
    true = (0.3, 0.1, 0.2)
    for guess in ((0.22, 0.17, 0.26), (0.18, 0.2, 0.1), true):
        found, new = plan.match(*walls_seen_from(true, corner, rng), guess)
        assert found.directions == 3 and found.share > 0.9 and not new.any()
        assert found.pose[:2] == pytest.approx(true[:2], abs=0.006)
        assert found.pose[2] == pytest.approx(true[2], abs=0.004)

    # Only the wall ahead: the distance to it and the heading are found; where along the
    # wall the robot is cannot be told, and stays as guessed.
    guess = (0.22, 0.17, 0.26)
    found, _ = plan.match(*walls_seen_from(true, ahead, rng), guess)
    assert found.directions == 2
    assert found.pose[0] == pytest.approx(true[0], abs=0.015) and found.pose[2] == pytest.approx(true[2], abs=0.006)
    along = math.hypot(found.pose[0] - guess[0], found.pose[1] - guess[1])
    assert found.pose[1] == pytest.approx(guess[1], abs=0.02), along

    # Something the plan does not have (a wall 1 m nearer): nothing to hold on to, all new.
    nearer = [(1.0, y) for y in np.arange(-0.5, 0.5, 0.005)]
    found, new = plan.match(*walls_seen_from((0.0, 0.0, 0.0), nearer, rng), (0, 0, 0))
    assert found is None and new.all()
    # The stretch of a wall that has just come into view, beyond where the plan's ends, is
    # next to what the plan has, not misplaced along it: the pose stays.
    longer = [(2.0, y) for y in np.arange(-1.6, 1.0, 0.005)]
    found, new = plan.match(*walls_seen_from(true, longer, rng), true)
    assert found.pose[1] == pytest.approx(true[1], abs=0.01) and found.pose[2] == pytest.approx(true[2], abs=0.006)
    assert 0 < new.sum() < len(new)


@pytest.fixture
def walled_room(monkeypatch):
    """The made-up room with walls around it, 3 m ahead and 1.8 m to either side: what a
    camera near the floor sees of a real room."""
    from obot.ui import scene

    walls = (((3.0, 0.0, 0.75), (0.1, 3.7, 1.5), (216, 212, 204)),
             ((1.0, 1.85, 0.75), (4.1, 0.1, 1.5), (216, 212, 204)),
             ((1.0, -1.85, 0.75), (4.1, 0.1, 1.5), (216, 212, 204)))
    monkeypatch.setattr(scene, "BOXES", scene.BOXES + walls)


class StillCamera:
    """A depth camera for WorldView whose pictures the test draws."""

    def __init__(self):
        self.stamp, self.depth = 0.0, None

    def raw(self):
        return self.stamp, self.depth


def drive(view, camera, config, steps, odometry_of):
    """Take the robot through the true poses `steps` in the made-up room: at each, the
    wheels report odometry_of(pose) and the depth camera a picture from where it really is."""
    from obot.ui import scene

    c = config.cameras[0]
    mount = world.transform(c.xyz, c.rpy) @ world.OPTICAL
    for true in steps:
        view.update_pose(*odometry_of(true), 0.0)
        camera.depth = scene.render(world.base_pose(*true) @ mount, c.intrinsics())[0]
        camera.stamp = time.monotonic()
        assert view.step() == 1


def base_camera_config(correct: bool = True) -> ToolsConfig:
    config = ToolsConfig()
    # as on the robot: 14 cm above the floor, level, facing forward
    config.cameras = [CameraConfig(name="d435_depth", source="realsense:depth", width=320, height=240,
                                   xyz=(0.116, 0.0175, 0.143), rpy=(0.0, 0.0, 0.0), latency=0.0)]
    config.world.stride = 2
    config.world.correct_odometry = correct
    return config


def test_the_map_corrects_wheels_that_misreport_the_way(walled_room):
    pytest.importorskip("cv2")
    from obot.ui.worldview import WorldView

    # 0.9 m forward, a turn of 0.5 rad to the left, 0.4 m on. The wheels report 6 % more
    # distance and 12 % more turning than there was (slip, a wrong track width).
    steps = [(x, 0.0, 0.0) for x in np.arange(0, 0.9001, 0.02)]
    steps += [(0.9, 0.0, yaw) for yaw in np.arange(0.02, 0.5001, 0.02)]
    steps += [(0.9 + d * math.cos(0.5), d * math.sin(0.5), 0.5) for d in np.arange(0.02, 0.4001, 0.02)]

    travelled = {"d": 0.0, "last": steps[0]}

    def wheels(true):
        last = travelled["last"]
        travelled["d"] += math.hypot(true[0] - last[0], true[1] - last[1])
        travelled["last"] = true
        yaw = 1.12 * true[2]
        if true[2] == 0.0:
            return (1.06 * true[0], 0.0, 0.0)
        if true[:2] == (0.9, 0.0):
            return (1.06 * 0.9, 0.0, yaw)
        d = 1.06 * (travelled["d"] - 0.9)
        return (1.06 * 0.9 + d * math.cos(1.12 * 0.5), d * math.sin(1.12 * 0.5), 1.12 * 0.5)

    def error(pose):
        true = steps[-1]
        return math.hypot(pose[0] - true[0], pose[1] - true[1]), abs(pose[2] - true[2])

    results = {}
    for correct in (True, False):
        travelled.update(d=0.0, last=steps[0])
        config = base_camera_config(correct)
        camera = StillCamera()
        view = WorldView(config, {"d435_depth": camera})
        drive(view, camera, config, steps, wheels)
        results[correct] = (error(view.base_pose()), view)
    (off, turned), view = results[True]
    (raw_off, raw_turned), raw_view = results[False]
    print(f"with the map: {off * 100:.1f} cm, {math.degrees(turned):.2f} deg off; "
          f"wheels alone: {raw_off * 100:.1f} cm, {math.degrees(raw_turned):.2f} deg")
    # The wheels alone end up 7 cm and 3.4 degrees from where the robot is; with the map it
    # is within 2 cm and half a degree.
    assert raw_view.correction == (0.0, 0.0, 0.0)
    assert raw_off > 0.06 and raw_turned > 0.05
    assert off < 0.02 and turned < 0.009
    assert view.match["used"] and view.match["share"] > 0.5
    # And the map is of the room as it is: the basket's near face is where the basket is
    # (x = 1.14), not smeared towards where the wheels put it.
    for which, limit in ((view, 0.03), (raw_view, None)):
        centres, _ = which.map.points()
        face = centres[(np.abs(centres[:, 1] + 0.8) < 0.1) & (centres[:, 2] > 0.1) & (centres[:, 0] < 1.3)]
        spread = float(np.abs(face[:, 0] - 1.14).max())
        if limit is not None:
            assert len(face) > 20 and spread < limit + 0.03
    # The trail is of the corrected way.
    trail = WorldMap.decode(view.map.delta())["trail"]
    assert math.hypot(trail[-1][0] - steps[-1][0], trail[-1][1] - steps[-1][1]) < 0.05


def test_wheels_turning_in_the_air_do_not_move_the_robot_in_the_map(walled_room):
    pytest.importorskip("cv2")
    from obot.ui.worldview import WorldView

    config = base_camera_config()
    camera = StillCamera()
    view = WorldView(config, {"d435_depth": camera})
    count = {"n": 0}

    def wheels(true):
        count["n"] += 1
        n = max(0, count["n"] - 5)                                   # a few pictures first, to have a map
        return (0.01 * n, 0.0, 0.005 * n)                           # then 1 cm and 0.3 degrees per picture

    drive(view, camera, config, [(0.0, 0.0, 0.0)] * 65, wheels)
    x, y, yaw = view.base_pose()
    print(f"wheels in the air: map pose {x * 100:.1f} cm, {y * 100:.1f} cm, {math.degrees(yaw):.2f} deg")
    # The wheels say 0.6 m and 17 degrees. The robot has not moved, and the map agrees.
    assert view._pose_at(None)[1] == pytest.approx(0.6)
    assert math.hypot(x, y) < 0.04 and abs(yaw) < 0.02


def test_nothing_to_compare_with_leaves_the_odometry_alone():
    pytest.importorskip("cv2")
    from obot.ui.worldview import WorldView

    config = base_camera_config()
    camera = StillCamera()
    view = WorldView(config, {"d435_depth": camera})
    # Facing away from everything in the room: only floor in view.
    drive(view, camera, config, [(-x, 0.0, math.pi) for x in np.arange(0, 0.3, 0.02)], lambda true: true)
    assert view.correction == (0.0, 0.0, 0.0)
    assert view.match == {"scan": 0, "used": False, "plan": 0}
    assert len(view.map) > 500                                       # the floor was mapped all the same
