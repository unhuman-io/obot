"""Driving to a point: the plan, and the whole thing in the made-up room."""

import math

import numpy as np
import pytest

from obot import world
from obot.config import CameraConfig, ToolsConfig

pytest.importorskip("cv2")
from obot.ui import navigate, scene  # noqa: E402
from obot.ui.navigate import Costmap, Navigator  # noqa: E402
from obot.ui.worldview import WorldView  # noqa: E402


def wall(x0, y0, x1, y1, step=0.03):
    n = int(math.hypot(x1 - x0, y1 - y0) / step) + 1
    return [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n + 1)]


def length(start, path):
    points = [start, *path]
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:], strict=False))


def clearance(obstacles, start, path):
    """The nearest the way comes to anything."""
    points = [start, *path]
    nearest = math.inf
    for a, b in zip(points, points[1:], strict=False):
        for k in np.linspace(0, 1, 50):
            p = (a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k)
            nearest = min(nearest, float(np.hypot(obstacles[:, 0] - p[0], obstacles[:, 1] - p[1]).min()))
    return nearest


def test_plan_goes_around_and_through_what_it_can():
    start, goal = (0.0, 0.0), (3.0, 0.0)
    # Nothing in the way: straight there.
    assert Costmap(np.empty((0, 2)), np.array(start), np.array(goal)).plan(start, goal) == [goal]

    # A wall across the way with a door 1 m wide at y = 1.5 .. 2.5: through the door.
    across = np.array(wall(1.5, -3.0, 1.5, 1.5) + wall(1.5, 2.5, 1.5, 4.0))
    costmap = Costmap(across, np.array(start), np.array(goal))
    path = costmap.plan(start, goal)
    assert path[-1] == goal and 2 <= len(path) <= 6
    assert any(abs(x - 1.5) < 0.4 and 1.5 < y < 2.5 for x, y in path) or costmap.free_line((1.2, 2.0), (1.8, 2.0))
    assert clearance(across, start, path) > navigate.RADIUS - 0.06
    assert 5.0 < length(start, path) < 7.0                              # (the shortest possible is 5.0)

    # The door is 0.4 m wide: the base does not fit, and there is no other way.
    narrow = np.array(wall(1.5, -4.0, 1.5, 1.8) + wall(1.5, 2.2, 1.5, 5.0))
    assert Costmap(narrow, np.array(start), np.array(goal), pad=1.0).plan(start, goal) is None

    # A goal inside something: the nearest place the base can be, if that is close.
    costmap = Costmap(across, np.array(start), np.array((1.5, 0.0)))
    assert costmap.plan(start, (1.5, 0.0)) is None
    near = costmap.nearest_free((1.5, 0.0))
    assert near is not None and abs(abs(near[0] - 1.5) - navigate.RADIUS) < 0.08 and abs(near[1]) < 0.08
    assert costmap.plan(start, near)[-1] == near

    # Starting closer to something than a plan would go (driven there by hand): out and away.
    costmap = Costmap(across, np.array((1.3, 0.0)), np.array((0.0, 0.0)))
    assert costmap.plan((1.3, 0.0), (0.0, 0.0)) == [(0.0, 0.0)]


class StillCamera:
    def __init__(self):
        self.stamp, self.depth = 0.0, None

    def raw(self):
        return self.stamp, self.depth


@pytest.fixture
def room(monkeypatch):
    """The made-up room with walls all round it: 3 m ahead, 1 m behind and 1.85 m to either side."""
    walls = (((3.0, 0.0, 0.75), (0.1, 3.7, 1.5), (216, 212, 204)),
             ((-1.05, 0.0, 0.75), (0.1, 3.7, 1.5), (216, 212, 204)),
             ((1.0, 1.85, 0.75), (4.1, 0.1, 1.5), (216, 212, 204)),
             ((1.0, -1.85, 0.75), (4.1, 0.1, 1.5), (216, 212, 204)))
    monkeypatch.setattr(scene, "BOXES", scene.BOXES + walls)


class Sim:
    """The base in the made-up room, moving exactly as commanded, with its depth camera
    where the robot has it."""

    def __init__(self, start=(0.0, 0.0, 0.0), native=False):
        config = ToolsConfig()
        config.cameras = [CameraConfig(name="d435_depth", source="realsense:depth", width=320, height=240,
                                       xyz=(0.116, 0.0175, 0.143), rpy=(0.0, 0.0, 0.0), latency=0.0)]
        config.world.stride = 2
        self.c = config.cameras[0]
        self.mount = world.transform(self.c.xyz, self.c.rpy) @ world.OPTICAL
        self.camera = StillCamera()
        self.view = WorldView(config, {self.c.name: self.camera}, native=native)
        self.nav = Navigator(self.view, threaded=False)
        self.t = 0.0                                                     # the test's own clock: it runs
        self.view.clock = self.nav.clock = lambda: self.t                # faster than a real one
        self.x, self.y, self.yaw = start
        self.w = 0.0
        self.went = [start]
        for _ in range(3):                                               # a few pictures before setting off
            self.t += 0.2
            self.look()

    def look(self):
        self.view.update_pose(self.x, self.y, self.yaw, self.w)
        self.camera.depth = scene.render(world.base_pose(self.x, self.y, self.yaw) @ self.mount, self.c.intrinsics())[0]
        self.camera.stamp = self.t
        self.view.step()

    def run(self, seconds=90.0, dt=0.02):
        """Until the navigator is done. Returns the way gone (x, y, yaw per step)."""
        first = len(self.went)
        for tick in range(int(seconds / dt)):
            if not self.nav.active:
                break
            self.t += dt
            v, self.w = self.nav.step(dt)
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
            self.yaw += self.w * dt
            if tick % 10 == 0:
                self.look()
            else:
                self.view.update_pose(self.x, self.y, self.yaw, self.w)
            self.went.append((self.x, self.y, self.yaw))
        return np.array(self.went[first - 1:])

    def go(self, x, y, seconds=90.0):
        self.nav.goto(x, y)
        return self.run(seconds)


def nearest_box(went):
    """How close the middle of the wheel axis came to anything in the room."""
    nearest = math.inf
    for centre, size, _ in scene.BOXES:
        out = np.maximum(np.abs(went[:, :2] - centre[:2]) - np.divide(size[:2], 2), 0.0)
        nearest = min(nearest, float(np.hypot(out[:, 0], out[:, 1]).min()))
    return nearest


def forwards_only(went):
    step = np.diff(went[:, 0]) * np.cos(went[1:, 2]) + np.diff(went[:, 1]) * np.sin(went[1:, 2])
    return bool(np.all(step > -1e-9))


@pytest.fixture(params=["python", "obot-navd"])
def native(request):
    """Whether the map is kept in Python or by obot-navd: the robot drives the same."""
    from obot.ui import native as native_map

    if request.param == "obot-navd" and native_map.find() is None:
        pytest.skip("obot-navd is not built (cmake --preset release in rt/)")
    return request.param == "obot-navd"


def body_gap(went):
    """The least room there ever was between the base (its real outline, turned as it was)
    and anything in the room: a collision is 0."""
    outline = navigate._outline(grow=0.0, step=0.02)
    least = math.inf
    for x, y, yaw in went[::4]:
        c, s = math.cos(yaw), math.sin(yaw)
        placed = outline @ np.array([[c, s], [-s, c]]) + (x, y)
        for centre, size, _ in scene.BOXES:
            out = np.maximum(np.abs(placed - centre[:2]) - np.divide(size[:2], 2), 0.0)
            least = min(least, float(np.hypot(out[:, 0], out[:, 1]).min()))
    return least


def door(width, y=0.0):
    """Two rooms, 3 m by 4 m each, with a door of that width between them at x = 2."""
    low, high = y - width / 2, y + width / 2
    return tuple(((cx, cy, 0.75), (dx, dy, 1.5), (216, 212, 204)) for cx, cy, dx, dy in (
        (2.0, 2.05, 6.2, 0.1), (2.0, -2.05, 6.2, 0.1), (-1.05, 0.0, 0.1, 4.2), (5.05, 0.0, 0.1, 4.2),
        (2.0, (low - 2.0) / 2, 0.1, low + 2.0), (2.0, (high + 2.0) / 2, 0.1, 2.0 - high)))


def test_goes_through_a_gap_it_fits_through_and_not_through_one_it_does_not(monkeypatch):
    # The base is 0.45 m wide. A door of 0.66 m, off to one side so that it has to line up for it.
    monkeypatch.setattr(scene, "BOXES", door(0.66, y=0.7))
    sim = Sim()
    went = sim.go(3.6, -0.5, seconds=400)
    assert sim.nav.state == "arrived", sim.nav.describe()
    print(f"through a 0.66 m door in {len(went) * 0.02:.0f} s: least room around the base "
          f"{body_gap(went) * 100:.0f} cm")
    assert went[:, 0].max() > 3.0 and body_gap(went) > 0.03 and forwards_only(went)
    # In the doorway it is slow.
    inside = np.abs(went[:, 0] - 2.0) < 0.2
    speed = np.hypot(*np.diff(went[:, :2], axis=0).T) / 0.02
    assert inside.sum() > 50 and speed[inside[1:]].max() < 0.2 < speed.max()
    # A door of 0.56 m: it would fit with 5 cm to spare either side, and does not try.
    monkeypatch.setattr(scene, "BOXES", door(0.56, y=0.7))
    sim = Sim()
    went = sim.go(3.6, -0.5, seconds=400)
    assert sim.nav.state == "failed" and went[:, 0].max() < 1.8 and body_gap(went) > 0.03


def test_does_not_turn_where_its_tail_would_hit(monkeypatch):
    # A passage 0.7 m wide and 1.6 m long with a wall at its end. The base drives in (it has
    # seen the walls on the way), and is then asked to come back out. It cannot turn round
    # in there (the tail swings out 0.375 m) and it does not drive backwards (the camera
    # looks forwards): it says so, and touches nothing trying.
    passage = (((0.5, 0.36, 0.5), (1.6, 0.02, 1.0), (90, 90, 90)), ((0.5, -0.36, 0.5), (1.6, 0.02, 1.0), (90, 90, 90)),
               ((1.31, 0.0, 0.5), (0.02, 0.74, 1.0), (90, 90, 90)))
    monkeypatch.setattr(scene, "BOXES", passage)
    sim = Sim(start=(-1.5, 0.0, 0.0))
    went = sim.go(0.85, 0.0, seconds=300)
    assert sim.nav.state == "arrived" and went[-1, 0] > 0.7, sim.nav.describe()
    assert body_gap(went) > 0.03
    went = sim.go(-1.5, 0.0, seconds=300)
    print(f"in the passage: {sim.nav.state}, {sim.nav.note!r}, turned {math.degrees(np.ptp(went[:, 2])):.0f} degrees, "
          f"least room {body_gap(np.array(sim.went)) * 100:.1f} cm")
    assert sim.nav.state == "failed" and "no room" in sim.nav.note
    assert body_gap(np.array(sim.went)) > 0.005 and went[-1, 0] > 0.5


def test_turns_the_long_way_round_when_the_tail_would_hit_the_short_way(monkeypatch):
    # Something small stands behind the base to its left, where its tail would swing
    # through if it turned right. The goal is to the right. It turns left, three quarters
    # of a turn, and goes.
    monkeypatch.setattr(scene, "BOXES", (((-0.233, 0.233, 0.2), (0.08, 0.08, 0.4), (90, 90, 90)),))
    sim = Sim()
    post = np.array([(x, y, z) for x in (-0.26, -0.23, -0.2) for y in (0.2, 0.23, 0.26)
                     for z in (0.09, 0.15, 0.21)])
    for _ in range(5):                                                   # (it cannot see behind itself)
        sim.view.map.integrate(post)
    sim.view._seen[:] = True
    went = sim.go(0.0, -1.5, seconds=120)
    assert sim.nav.state == "arrived", sim.nav.describe()
    print(f"turned {math.degrees(went[:, 2].max()):.0f} degrees left to face right; "
          f"least room {body_gap(went) * 100:.1f} cm")
    assert went[:, 2].max() > 4.0 and went[:, 2].min() > -0.3            # never right
    assert body_gap(went) > 0.01


def test_drives_around_things_and_finds_its_way_as_it_sees_more(room, native):
    sim = Sim(native=native)
    # Behind the box, where the base has not looked, with the shirt and the sock straight
    # ahead too close together to pass between. It goes as far towards the place as it has
    # seen the floor to be free, sees more from there, and so on until it is there.
    goal = (2.4, 0.9)
    went = sim.go(*goal, seconds=240)
    assert sim.nav.state == "arrived", sim.nav.describe()
    assert math.hypot(went[-1, 0] - goal[0], went[-1, 1] - goal[1]) < navigate.ARRIVED + 0.02
    print(f"went {np.hypot(*np.diff(went[:, :2], axis=0).T).sum():.1f} m in {len(went) * 0.02:.0f} s, "
          f"never nearer than {nearest_box(went):.2f} m to anything")
    assert nearest_box(went) > 0.24 and body_gap(went) > 0.03             # the base is 0.45 m wide
    assert forwards_only(went)
    sim.view.close()


def test_a_point_inside_something_and_a_point_out_of_reach(room, monkeypatch):
    # (a pen behind the robot, closed all round)
    sides = ((-2.0, 0.7, (1.5, 0.1, 0.6)), (-2.0, -0.7, (1.5, 0.1, 0.6)),
             (-1.3, 0.0, (0.1, 1.5, 0.6)), (-2.7, 0.0, (0.1, 1.5, 0.6)))
    pen = tuple(((x, y, 0.3), size, (90, 90, 90)) for x, y, size in sides)
    monkeypatch.setattr(scene, "BOXES", scene.BOXES + pen)
    sim = Sim()
    # In the middle of the basket: the base goes as near as it can get.
    went = sim.go(1.3, -0.8, seconds=240)
    assert sim.nav.state == "arrived", sim.nav.describe()
    assert 0.24 < nearest_box(went[-1:]) < 0.5 and body_gap(went) > 0.03
    # Inside the pen: it goes as near as it can, looks around, and says that there is no way.
    went = sim.go(-2.0, 0.0, seconds=400)
    assert sim.nav.state == "failed", sim.nav.describe()
    assert went[-1, 0] < -0.4
    assert body_gap(np.array(sim.went)) > 0.03


TWO_ROOMS = tuple(((x, y, 0.75), (dx, dy, 1.5), (216, 212, 204)) for x, y, dx, dy in (
    (2.0, 2.05, 6.2, 0.1), (2.0, -2.05, 6.2, 0.1), (-1.05, 0.0, 0.1, 4.2), (5.05, 0.0, 0.1, 4.2),   # 6 m by 4 m
    (2.0, -1.15, 0.1, 1.7), (2.0, 1.45, 0.1, 1.1),             # a wall across, with a door 1.2 m wide
)) + (((0.8, -1.2, 0.2), (0.4, 0.4, 0.4), (138, 111, 82)),     # a box in the first room
      ((3.9, 1.0, 0.2), (0.4, 0.4, 0.4), (63, 127, 191)))      # and one in the second


def test_maps_two_rooms_by_itself(monkeypatch):
    from obot.ui import native as native_map

    monkeypatch.setattr(scene, "BOXES", TWO_ROOMS)
    sim = Sim(native=native_map.find() is not None)
    sim.nav.explore(radius=8.0)
    assert sim.nav.describe()["exploring"]
    went = sim.run(seconds=1500)
    described = sim.nav.describe()
    assert described["state"] == "arrived" and described["note"] == "mapped all it can reach", described
    assert not described["exploring"] and not sim.nav.active
    travelled = float(np.hypot(*np.diff(went[:, :2], axis=0).T).sum())
    # How much of each room's floor has been seen? (Not all: the camera does not see the
    # floor right behind a box, and nothing sends the robot there.)
    cell = 0.06
    first = sim.view.known((int(-0.9 / cell), int(-1.9 / cell)), (int(3.8 / cell), int(2.8 / cell)))
    second = sim.view.known((int(2.2 / cell), int(-1.9 / cell)), (int(3.8 / cell), int(2.7 / cell)))
    print(f"mapped two rooms in {len(went) * 0.02:.0f} s, {travelled:.1f} m: {first.mean() * 100:.0f} % and "
          f"{second.mean() * 100:.0f} % of their floors seen, never nearer than {nearest_box(went):.2f} m to anything")
    assert first.mean() > 0.85 and second.mean() > 0.85
    assert went[:, 0].max() > 2.0                                         # it went into the doorway
    assert body_gap(went) > 0.03 and forwards_only(went)
    sim.view.close()
    # Told to stay within a metre and a half of where it starts, it does: the second room
    # stays as the door showed it.
    near = Sim(native=native_map.find() is not None)
    near.nav.explore(radius=1.5)
    went = near.run(seconds=900)
    assert near.nav.state == "arrived" and np.hypot(went[:, 0], went[:, 1]).max() < 1.6
    # Someone takes over: that is the end of it.
    near.nav.explore()
    near.run(seconds=3)
    assert near.nav.active
    near.nav.cancel("stopped: someone took the controls")
    assert not near.nav.active and near.nav.step(0.02)[0] == 0.0
    near.view.close()


def test_stops_for_what_was_not_there_before(room, monkeypatch):
    sim = Sim()
    sim.nav.look_around()
    sim.run()
    assert sim.nav.state == "arrived" and 5.0 < sim.yaw < 2 * math.pi       # (the camera sees 80 degrees itself)
    # A way that is free, as far as the map knows. Then something is put down across it.
    monkeypatch.setattr(scene, "BOXES", scene.BOXES + (((0.6, -0.9, 0.2), (0.1, 1.0, 0.4), (10, 10, 10)),))
    went = sim.go(1.9, -1.3, seconds=400)
    # It never touches it: it found another way, or gave up.
    assert body_gap(np.array(sim.went)) > 0.03
    assert sim.nav.state in ("arrived", "failed")
    print("with something put in the way:", sim.nav.state, sim.nav.note, "nearest", round(nearest_box(went), 2))


def test_no_pictures_no_driving(room):
    config = ToolsConfig()
    config.cameras = [CameraConfig(name="d435_depth", source="realsense:depth", width=320, height=240,
                                   xyz=(0.116, 0.0175, 0.143), rpy=(0.0, 0.0, 0.0), latency=0.0)]
    view = WorldView(config, {"d435_depth": StillCamera()}, native=False)
    view.update_pose(0.0, 0.0, 0.0, 0.0)
    view._seen[:] = True                                                 # (as if it had seen everything)
    nav = Navigator(view, threaded=False)
    nav.goto(1.0, 0.0)
    assert nav.state == "driving"
    speeds = [nav.step(0.02) for _ in range(50)]
    assert all(v == 0.0 for v, _ in speeds) and nav.note == "no pictures from the depth camera"
    nav.cancel("operator took over")
    assert nav.state == "idle" and nav.step(0.02)[0] == 0.0


async def test_page_server_drives_to_a_clicked_point_and_keeps_the_map(robot, daemon, tmp_path, monkeypatch):
    """The whole chain with obot-rtd on simulated motors: a page asks for a point, the base
    is enabled and driven there, a controller takes over at once, and the map is saved."""
    import asyncio
    import json

    aiohttp_utils = pytest.importorskip("aiohttp.test_utils")
    from test_ui import gamepad, make_server, next_state

    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path))
    ui = make_server(daemon)
    ui.config.cameras.append(CameraConfig(name="low depth", source="scene:depth", width=320, height=240, fps=15,
                                          xyz=(0.116, 0.0175, 0.143), rpy=(0.0, 0.0, 0.0)))
    ui.config.world.stride = 2
    async with aiohttp_utils.TestClient(aiohttp_utils.TestServer(ui.app())) as client:
        ws = await client.ws_connect("/ws")
        await next_state(ws, lambda s: s["world"] and s["world"]["voxels"] > 300, timeout=20)
        assert robot.state()["mode"] == 1                                # idle: nobody has enabled it

        await ws.send_json({"type": "nav", "action": "goto", "x": 0.3, "y": -0.5})
        state = await next_state(ws, lambda s: s["nav"]["state"] in ("arrived", "failed"), timeout=60)
        assert state["nav"]["state"] == "arrived", state["nav"]
        at = robot.state()
        assert math.hypot(at["odom_x"] - 0.3, at["odom_y"] + 0.5) < 0.1
        assert at["mode"] == 2                                           # it enabled the robot itself
        await asyncio.sleep(0.3)
        assert abs(robot.state()["base_v"]) < 0.02                       # and stands still now

        # On the way to another point someone moves a stick: the drive is over.
        await ws.send_json({"type": "nav", "action": "goto", "x": 0.0, "y": -1.0})
        await next_state(ws, lambda s: s["nav"]["state"] == "driving" and abs(s["base_omega"]) > 0.1, timeout=20)
        for _ in range(10):
            await ws.send_json(gamepad(axes=(0, -1, 0, 0), lb=1))
            await asyncio.sleep(0.03)
        state = await next_state(ws, lambda s: s["nav"]["state"] == "idle")
        assert "took the controls" in state["nav"]["note"]

        # Nonsense is ignored.
        for bad in ({"action": "goto", "x": "a", "y": 0}, {"action": "goto", "x": 1e9, "y": 0}, {"action": "fly"}):
            await ws.send_json({"type": "nav", **bad})
        await asyncio.sleep(0.2)
        assert ui.nav.state == "idle"

        # The map from above, as the page draws it: floor seen free, things, and the margin around them.
        import struct

        body = await (await client.get("/api/grid")).read()
        magic, i0, j0, width, height, cell = struct.unpack_from("<IiiIIf", body)
        cells = np.frombuffer(body, np.uint8, offset=24).reshape(height, width)
        assert magic == 0x3147424F and cell == pytest.approx(0.06) and width > 30 and height > 30
        assert (cells == 1).sum() > 200 and (cells == 2).sum() > 20 and (cells == 3).sum() > 50
        here = robot.state()
        i, j = int(here["odom_x"] / cell) - i0, int(here["odom_y"] / cell) - j0
        assert cells[j, i] in (1, 3)                                     # where the base stands is free
        box = cells[int(0.9 / cell) - j0, int(1.6 / cell) - i0 - 3: int(1.6 / cell) - i0 + 4]
        assert 2 in box or 0 in box                                      # and the box is not

        await ws.send_json({"type": "world", "action": "save"})
        await asyncio.sleep(1.0)
    saved = list(tmp_path.glob("obot/map-*.npz"))
    assert len(saved) == 1
    where = json.loads(next(tmp_path.glob("obot/*.where.json")).read_text())
    assert math.hypot(where["pose"][0], where["pose"][1]) < 1.5 and len(where["wheels"]) == 2

    # The page server is started again, obot-rtd still running: the map is taken up as it was.
    again = make_server(daemon)
    again.config.cameras.append(ui.config.cameras[-1])
    async with aiohttp_utils.TestClient(aiohttp_utils.TestServer(again.app())) as client:
        ws = await client.ws_connect("/ws")
        json.loads((await ws.receive()).data)
        await next_state(ws, lambda s: True)
        assert len(again.world.map) > 300 and again.world.floor_seen() > 100
        assert again.world.correction == (0.0, 0.0, 0.0) and again._check is None
        again.world.stop()                                               # (no new pictures from here on)
        state = robot.state()

        def restarted(seconds, wheels):
            """What obot-rtd says after it has been started again: odometry at zero."""
            return {"t_real_ns": int(state["t_real_ns"]) + seconds * 1_000_000_000, "cycle": 5,
                    "q": wheels, "odom_x": 0.0, "odom_y": 0.0, "odom_yaw": 0.0}

        # obot-rtd is started again (a deploy does that) and the wheels stand as they did:
        # the robot is where it was, and the map goes on from there, odometry at zero or not.
        was = again.world.base_pose()
        voxels = len(again.world.map)
        again._take_up_map(restarted(60, state["q"]))
        assert len(again.world.map) == voxels
        assert again.world.correction == pytest.approx(was, abs=2e-3) and "took up the map" in again.world.note
        # ... which is then checked against what the camera sees. Things where the map has
        # them: it stays.
        again.world.agree = [a + b for a, b in zip(again.world.agree, (200, 180, 3), strict=True)]
        again._check_place()
        assert again._check is None and again.world.note == "took up the map from before" and len(again.world.map)
        # The same again, but the camera sees things where the map has none: the robot was
        # carried somewhere else while it was off. A new map, and the old one put aside.
        again._take_up_map(restarted(120, state["q"]))
        again.world.agree = [a + b for a, b in zip(again.world.agree, (200, 30, 90), strict=True)]
        again._check_place()
        assert len(again.world.map) == 0 and again.world.floor_seen() == 0 and "not where it was" in again.world.note
        assert again.world.correction == (0.0, 0.0, 0.0)
        assert list(tmp_path.glob("obot/map-*.npz.previous"))
    # A page server that finds the wheels turned since the map was made (the robot was
    # pushed with obot-rtd off): where it is on the old map is not known. A new map.
    pushed = make_server(daemon)
    pushed.config.cameras.append(ui.config.cameras[-1])
    async with aiohttp_utils.TestClient(aiohttp_utils.TestServer(pushed.app())) as client:
        ws = await client.ws_connect("/ws")
        await next_state(ws, lambda s: s["world"] and s["world"]["voxels"] > 100, timeout=20)
        turned = np.array(robot.state()["q"], float)
        turned[np.nonzero(robot.info.continuous)[0]] += 1.0
        pushed._take_up_map({"t_real_ns": int(robot.state()["t_real_ns"]) + 300_000_000_000, "cycle": 5, "q": turned,
                             "odom_x": 0.0, "odom_y": 0.0, "odom_yaw": 0.0})
        assert len(pushed.world.map) == 0 and "new map" in pushed.world.note
