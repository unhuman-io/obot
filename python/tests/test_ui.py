import asyncio
import json

import pytest

aiohttp = pytest.importorskip("aiohttp")
pytest.importorskip("cv2")

from aiohttp.test_utils import TestClient, TestServer  # noqa: E402

from obot import RobotMode  # noqa: E402
from obot.config import CameraConfig, load  # noqa: E402
from obot.ui.server import UiServer  # noqa: E402


def make_server(daemon, **kwargs) -> UiServer:
    from conftest import CONFIG

    tools = load(str(CONFIG))
    tools.shm_name = daemon
    tools.cameras = [CameraConfig(name="test", source="test", width=160, height=120, fps=20)]
    return UiServer(tools, **kwargs)


async def next_state(ws, predicate, timeout=5.0):
    async def wait():
        while True:
            message = json.loads((await ws.receive()).data)
            if message.get("type") == "state" and "mode" in message and predicate(message):
                return message

    return await asyncio.wait_for(wait(), timeout)


def gamepad(axes=(0, 0, 0, 0), **buttons):
    names = ["a", "b", "x", "y", "lb", "rb", "lt", "rt", "back", "start", "ls", "rs", "up", "down", "left", "right"]
    values = [0.0] * 17
    for name, value in buttons.items():
        values[names.index(name)] = value
    return {"type": "gamepad", "axes": list(axes), "buttons": values}


async def test_page_info_and_camera(robot, daemon):
    async with TestClient(TestServer(make_server(daemon).app())) as client:
        page = await client.get("/")
        assert page.status == 200
        assert "obot operator" in await page.text()

        await asyncio.sleep(1.5)                   # the server connects to the robot, a frame is captured
        info = await (await client.get("/api/info")).json()
        assert info["robot"] == "obot3"
        assert info["joints"][0] == "shoulder_pitch_joint"
        assert info["cameras"] == ["test"]
        assert info["has_base"]

        frame = await client.get("/camera/test.jpg")
        assert frame.status == 200
        assert (await frame.read())[:2] == b"\xff\xd8"          # a JPEG
        assert (await client.get("/camera/nope.jpg")).status == 404

        stream = await client.get("/camera/test.mjpg")
        assert stream.headers["Content-Type"].startswith("multipart/x-mixed-replace")
        chunk = await asyncio.wait_for(stream.content.read(64), 5)
        assert chunk.startswith(b"--frame")
        stream.close()


async def test_driving_from_the_browser(robot, daemon):
    async with TestClient(TestServer(make_server(daemon).app())) as client:
        ws = await client.ws_connect("/ws")
        first = json.loads((await ws.receive()).data)
        assert first["type"] == "info"
        state = await next_state(ws, lambda s: True)
        assert state["mode"] == "idle"
        assert state["status"]["operator"] is None

        # An idle page writes no commands: other clients are not overridden.
        await asyncio.sleep(0.3)
        assert robot.state()["command_age_us"] == -1

        await ws.send_json({"type": "request", "request": "enable"})
        await next_state(ws, lambda s: s["mode"] == "active")

        async def hold(message, seconds):
            for _ in range(int(seconds / 0.03)):
                await ws.send_json(message)
                await asyncio.sleep(0.03)

        await hold(gamepad(axes=(0, -1, 0, 0), lb=1), 1.5)       # deadman + stick forward
        state = await next_state(ws, lambda s: s["base_v"] > 0.2 and s["status"]["moving"])
        assert state["status"]["operator"].startswith("browser")

        # The operator's connection drops while driving: the robot stops by itself.
        await ws.close()
        await asyncio.sleep(1.0)
        after = robot.state()
        assert abs(after["base_v"]) < 0.03
        assert after["mode"] == RobotMode.ACTIVE
        assert after["odom_x"] > 0.1


async def test_garbage_from_a_client_is_ignored(robot, daemon):
    async with TestClient(TestServer(make_server(daemon).app())) as client:
        ws = await client.ws_connect("/ws")
        await ws.send_str("not json")
        await ws.send_str("[1, 2]")
        await ws.send_json({"type": "request", "request": "self_destruct"})
        await ws.send_json({"type": "gamepad", "axes": ["x", None, 1e9], "buttons": "nope"})
        state = await next_state(ws, lambda s: True)
        assert state["mode"] == "idle"
        await ws.close()


async def test_token(robot, daemon):
    async with TestClient(TestServer(make_server(daemon, token="s3cret").app())) as client:
        assert (await client.get("/")).status == 401
        assert (await client.get("/api/info?token=wrong")).status == 401
        with pytest.raises(aiohttp.WSServerHandshakeError):
            await client.ws_connect("/ws")
        ok = await client.get("/?token=s3cret")
        assert ok.status == 200
        assert (await client.get("/api/info")).status == 200      # the cookie from the page carries on


async def test_control_is_released_and_taken_over_cleanly(robot, daemon):
    """An operator who lets go stops writing commands; taking over later starts from where the
    arm is, even if something else moved it in between."""
    from obot import JointMode

    async with TestClient(TestServer(make_server(daemon).app())) as client:
        ws = await client.ws_connect("/ws")
        await ws.send_json({"type": "request", "request": "enable"})
        await next_state(ws, lambda s: s["mode"] == "active")
        for _ in range(10):                                   # a touch on the deadman, then hands off
            await ws.send_json(gamepad(lb=1))
            await asyncio.sleep(0.03)
        await next_state(ws, lambda s: s["status"]["operator"] is not None)
        for _ in range(60):                                   # the page keeps reporting a resting controller
            await ws.send_json(gamepad())
            await asyncio.sleep(0.03)
        state = await next_state(ws, lambda s: s["status"]["operator"] is None)
        assert robot.state()["command_age_us"] > 300_000      # nothing has been written for a while

        # A script moves the shoulder while nobody operates.
        command = robot.new_command()
        command["joint_mode"][:4] = JointMode.POSITION
        command["q"][0] = 0.5
        for _ in range(150):
            robot.send(command)
            await asyncio.sleep(0.01)
        await asyncio.sleep(0.5)                              # the script stops; the daemon holds
        before = robot.state()
        assert before["q"][0] == pytest.approx(0.5, abs=0.1)  # there, less the sag of a plain PD loop

        # The operator touches the deadman again: the arm must stay exactly where it is held.
        for _ in range(30):
            await ws.send_json(gamepad(lb=1))
            await asyncio.sleep(0.03)
        state = robot.state()
        assert 0 <= state["command_age_us"] < 200_000         # the page is commanding again
        assert state["q_cmd"][0] == pytest.approx(float(before["q_cmd"][0]), abs=0.01)
        assert state["q"][0] == pytest.approx(float(before["q"][0]), abs=0.02)
        await ws.close()


async def test_any_operator_can_stop_the_robot(robot, daemon):
    async with TestClient(TestServer(make_server(daemon).app())) as client:
        driver = await client.ws_connect("/ws")
        bystander = await client.ws_connect("/ws")
        await driver.send_json({"type": "request", "request": "enable"})
        await next_state(driver, lambda s: s["mode"] == "active")

        async def drive():
            for _ in range(60):
                await driver.send_json(gamepad(axes=(0, -1, 0, 0), lb=1))
                await asyncio.sleep(0.03)

        task = asyncio.create_task(drive())
        await next_state(driver, lambda s: s["base_v"] > 0.1)
        await bystander.send_json(gamepad(y=1))               # not in control, presses stop
        await next_state(bystander, lambda s: s["mode"] == "estop")
        await task                                            # the driver keeps pushing the stick: no effect
        await asyncio.sleep(1.0)
        assert robot.state()["mode"] == RobotMode.ESTOP
        assert abs(robot.state()["base_v"]) < 0.1             # braked by the idle drives
        await driver.close()
        await bystander.close()


async def test_robot_model_is_served(robot, daemon):
    async with TestClient(TestServer(make_server(daemon).app())) as client:
        model = await (await client.get("/api/model")).json()
        assert model["root"] == "base_footprint"
        joints = {j["name"]: j for j in model["joints"]}
        assert joints["shoulder_pitch_joint"]["axis"] == [0, -1, 0]
        assert joints["elbow_pitch_joint"]["xyz"] == [0, 0, 0.25]
        mesh = model["links"]["upper_arm_link"][0]["mesh"]
        response = await client.get("/model/" + mesh)
        assert response.status == 200
        assert len(await response.read()) > 1000
        assert (await client.get("/model/../../config/obot3.toml")).status == 404     # stays inside the model


def test_depth_is_drawn_in_false_colour():
    import numpy as np

    from obot.ui.cameras import depth_to_image

    depth = np.array([[0.0, 0.3, 2.0, 9.0]], np.float32)       # no reading, near, mid, beyond range
    image = depth_to_image(depth)
    assert image.shape == (1, 4, 3)
    assert image[0, 0].tolist() == [0, 0, 0]                    # no reading: black
    assert image[0, 1][2] > image[0, 1][0]                      # near: more red than blue (BGR)
    assert image[0, 3][0] > image[0, 3][2] or image[0, 3].sum() < 200   # far: cool


def test_gstreamer_jpeg_stream_is_split_into_frames(monkeypatch):
    import io
    import subprocess

    from obot.ui import cameras

    frames = [b"\xff\xd8" + bytes([i]) * 50 + b"\xff\xd9" for i in range(3)]

    class FakeProcess:
        stdout = io.BufferedReader(io.BytesIO(b"".join(frames)), buffer_size=16)   # arrives in small pieces

        def kill(self): ...
        def wait(self): ...

    monkeypatch.setattr(subprocess, "Popen", lambda *a, **k: FakeProcess())
    source = cameras._GstJpeg("videotestsrc ! jpegenc")
    assert [source.read_jpeg() for _ in range(3)] == frames
    with pytest.raises(OSError):
        source.read_jpeg()
