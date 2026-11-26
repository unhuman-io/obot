"""What a segmentation network says of the colour picture, and how that gets onto the map.
The network itself needs a GPU and is not here: tests/fake/obot-segd speaks as obot-segd
does and says something simple."""

import asyncio
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("cv2")
from obot import world  # noqa: E402
from obot.ui import scene, segment  # noqa: E402
from obot.ui.worldview import WorldView  # noqa: E402

FAKE = str(Path(__file__).parent / "fake" / "obot-segd")


class StillCamera:
    def __init__(self):
        self.stamp, self.frame = 0.0, None

    def raw(self):
        return self.stamp, self.frame


@pytest.fixture
def model(tmp_path):
    """A network's files as the jetson-inference models come: the names and colours of the
    classes beside it (of five of the fake's seven)."""
    (tmp_path / "net.onnx").write_bytes(b"not looked at by the stand-in")
    (tmp_path / "classes.txt").write_text("other\nwall\nfloor\nchair/stool\ntable\nclothes\n")
    (tmp_path / "colors.txt").write_text("0 0 0\n128 0 0\n0 128 0\n128 0 128\n128 128 128\n0 0 255\n")
    return str(tmp_path / "net.onnx")


def test_obot_segd_is_asked_and_answers(model):
    segd = segment.Segd(FAKE, model)
    try:
        assert (segd.width, segd.height, segd.columns, segd.rows) == (64, 48, 8, 6)
        assert segd.names[:6] == ["other", "wall", "floor", "chair/stool", "table", "clothes"]
        assert segd.names[6] == "class 6" and segd.colours[5] == [0, 0, 255] and segd.colours[6] == [128, 128, 128]
        picture = np.full((480, 640, 3), 200, np.uint8)            # (any size: it is made the network's)
        picture[:240] = 0                                          # the upper half black
        said, sure, ms = segd.look(picture)
        assert said.shape == (6, 8) and sure.shape == (6, 8) and ms == pytest.approx(1.5)
        assert (said[:3] == 0).all() and (said[3:, :4] == 5).all() and (said[3:, 4:] == 3).all()
        assert (sure[:, :7] == 230).all() and (sure[:, 7] == 40).all()
    finally:
        segd.close()
    with pytest.raises(RuntimeError, match="could not load"):
        segment.Segd(FAKE, model.replace("net.onnx", "broken.onnx"))


def view_of_the_room(monkeypatch, model):
    from test_world import scene_config

    config = scene_config()
    config.segmentation.model = model
    cameras = {c.name: StillCamera() for c in config.cameras}
    view = WorldView(config, cameras, native=False)
    view.clock = lambda: 10.0
    view.update_pose(0.0, 0.0, 0.0, 0.0)
    depth_camera = next(c for c in config.cameras if c.is_depth)
    for c in config.cameras:
        depth, colour = scene.render(view.camera_pose(c), c.intrinsics())
        cameras[c.name].frame, cameras[c.name].stamp = (depth if c.is_depth else colour), 10.0
    return config, cameras, view, depth_camera


def test_what_the_network_says_is_carried_to_the_squares_of_the_map(monkeypatch, model):
    config, cameras, view, depth_camera = view_of_the_room(monkeypatch, model)
    monkeypatch.setattr(segment.Labels, "STRIDE", 2)                # (the test's pictures are small)
    labels = segment.Labels(config, cameras, view, FAKE)
    assert labels.colour.name == depth_camera.color and labels.depth is depth_camera and labels.error is None
    segd = segment.Segd(FAKE, model)
    try:
        labels.use(segd)
        assert len(labels.squares()) == 0
        said, sure, _ = segd.look(cameras[labels.colour.name].frame)
        for _ in range(labels.VOTES):
            assert labels.vote(10.0, said, sure) > 50
    finally:
        segd.close()
    squares = labels.squares()
    x, y, kind = (squares[:, 0] + 0.5) * labels.cell, (squares[:, 1] + 0.5) * labels.cell, squares[:, 2]
    assert len(squares) > 20 and set(kind.tolist()) == {3, 5} and (squares[:, 3] > 50).all()
    # The stand-in calls the left half of the picture clothes (5) and the right half a
    # chair (3): on the map that is what stands to the left and to the right of the camera.
    assert (kind[y > 0.3] == 5).all() and (kind[y < -0.3] == 3).all() and (x > 0.2).all()
    # ... of what stands there: every labelled square has something in the depth picture's points.
    points = world.apply(view.camera_pose(depth_camera), world.depth_to_points(
        cameras[depth_camera.name].frame, depth_camera.intrinsics(), 2, 0.2, 3.5))
    standing = points[(points[:, 2] > labels.LOW) & (points[:, 2] < labels.HIGH)]
    has = {(int(i), int(j)) for i, j in np.floor(standing[:, :2] / labels.cell)}
    assert all((int(i), int(j)) in has for i, j in squares[:, :2])

    # Pictures that are not of the same moment say nothing about each other, and neither
    # does anything while the base turns fast.
    before = labels.squares()
    assert labels.vote(11.0, said, sure) == 0
    view.clock = lambda: 12.0
    view.update_pose(0.0, 0.0, 0.0, 2.0)
    for camera in cameras.values():
        camera.stamp = 12.0
    assert labels.vote(12.0, said, sure) == 0
    assert (labels.squares() == before).all()
    described = labels.describe()
    assert described["ready"] and described["columns"] == 8 and len(described["squares"]) == len(before)
    labels.clear()
    assert len(labels.squares()) == 0


async def test_page_server_serves_what_the_network_says(robot, daemon, model, monkeypatch):
    aiohttp_utils = pytest.importorskip("aiohttp.test_utils")
    from test_ui import make_server
    from test_world import scene_config

    monkeypatch.setenv("OBOT_SEGD", FAKE)
    ui = make_server(daemon)
    ui.config.cameras += scene_config().cameras
    ui.config.segmentation.model = model
    ui.config.segmentation.rate = 20.0
    async with aiohttp_utils.TestClient(aiohttp_utils.TestServer(ui.app())) as client:
        assert (await (await client.get("/api/info")).json())["labels"] is True
        for _ in range(100):
            said = await (await client.get("/api/labels")).json()
            if said["ready"] and said["squares"]:
                break
            await asyncio.sleep(0.1)
        assert said["error"] is None and said["camera"] == "room" and len(said["said"]) == 48
        assert said["names"][5] == "clothes" and {square[2] for square in said["squares"]} <= {3, 5}
        ws = await client.ws_connect("/ws")
        await ws.send_json({"type": "world", "action": "clear"})      # the map forgotten: what was on it too
        await asyncio.sleep(0.05)
        assert ui.labels is not None
    # Without the program, or without the model: the page server goes without.
    monkeypatch.setenv("OBOT_SEGD", "")
    again = make_server(daemon)
    again.config.cameras += scene_config().cameras
    again.config.segmentation.model = model
    async with aiohttp_utils.TestClient(aiohttp_utils.TestServer(again.app())) as client:
        assert (await (await client.get("/api/info")).json())["labels"] is False
        assert (await client.get("/api/labels")).status == 404
