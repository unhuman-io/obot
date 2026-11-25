import asyncio
import json
import time

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


def test_raw_depth_reader_hands_out_the_newest_frame(monkeypatch):
    import os
    import subprocess
    import time

    import numpy as np

    from obot.ui import cameras

    read_end, write_end = os.pipe()

    class FakeProcess:
        stdout = os.fdopen(read_end, "rb")

        def kill(self): ...
        def wait(self): ...

    started = {}
    monkeypatch.setattr(subprocess, "Popen", lambda args, **k: started.setdefault("args", args) and FakeProcess())
    source = cameras._V4l2Raw("/dev/video9", 3, 2, 15)
    assert "--set-fmt-video=width=3,height=2,pixelformat=Z16 " in started["args"]

    def arrived(count):
        end = time.monotonic() + 2
        while source._count < count and time.monotonic() < end:
            time.sleep(0.005)
        return source._count == count

    # Two frames arrive before anyone asks: the reader is behind, and gets the newer one.
    os.write(write_end, np.full((2, 3), 500, "<u2").tobytes() + np.full((2, 3), 1500, "<u2").tobytes())
    assert arrived(2)
    stamp, frame = source.read()
    assert frame.dtype == np.uint16 and frame.shape == (2, 3) and (frame == 1500).all()
    assert time.monotonic() - stamp < 1
    # Nothing new: the reader waits instead of being given the same frame again.
    with pytest.raises(OSError, match="no frame"):
        source.read(timeout=0.1)
    os.write(write_end, np.full((2, 3), 2500, "<u2").tobytes())
    assert (source.read()[1] == 2500).all()
    os.close(write_end)
    with pytest.raises(OSError, match="ended"):
        source.read()


def test_depth_pictures_are_coloured_by_distance():
    import numpy as np

    from obot.ui.cameras import depth_mm_to_image, depth_to_image

    depth = np.array([[0, 300, 2000, 3900, 65535]], np.uint16)
    image = depth_mm_to_image(depth)
    assert image.shape == (1, 5, 3) and image.dtype == np.uint8
    assert image[0, 0].tolist() == [0, 0, 0] and image[0, 4].tolist() == [0, 0, 0]     # no reading: black
    assert image[0, 1, 2] > 150 and image[0, 1, 0] < 80                               # near: red (BGR)
    assert image[0, 3, 0] > 100 and image[0, 3, 2] < 100                              # far: blue
    assert (depth_to_image(depth.astype(np.float32) / 1000)[0, :4] == image[0, :4]).all()


def test_realsense_video_devices_are_found_by_what_they_are(tmp_path, monkeypatch):
    from obot.ui import cameras

    def device(number, name, index, interface, serial):
        usb = tmp_path / f"usb-{serial}"
        entry = tmp_path / "v4l" / f"video{number}"
        (usb / f"if{interface}").mkdir(parents=True, exist_ok=True)
        entry.mkdir(parents=True)
        (usb / "serial").write_text(serial + "\n")
        (usb / f"if{interface}" / "bInterfaceNumber").write_text(interface + "\n")
        (entry / "name").write_text(name + "\n")
        (entry / "index").write_text(f"{index}\n")
        (entry / "device").symlink_to(usb / f"if{interface}")

    sysfs = str(tmp_path / "v4l")
    device(0, "vi-output, imx219 7-0010", 0, "", "csi")
    with pytest.raises(OSError, match="no RealSense found"):
        cameras.find_realsense_video("depth", sysfs=sysfs)
    # As a Jetson Nano shows a D435 after it was plugged in again: colour is video4 now.
    device(1, "Intel(R) RealSense(TM) Depth Ca", 0, "00", "111")
    device(2, "Intel(R) RealSense(TM) Depth Ca", 1, "00", "111")
    device(4, "Intel(R) RealSense(TM) Depth Ca", 0, "03", "111")
    device(10, "Intel(R) RealSense(TM) Depth Ca", 0, "00", "222")
    device(12, "Intel(R) RealSense(TM) Depth Ca", 0, "03", "222")
    assert cameras.find_realsense_video("depth", sysfs=sysfs) == "/dev/video1"
    assert cameras.find_realsense_video("color", sysfs=sysfs) == "/dev/video4"
    assert cameras.find_realsense_video("color", "222", sysfs=sysfs) == "/dev/video12"
    assert cameras.find_realsense_video("depth", "222", sysfs=sysfs) == "/dev/video10"
    with pytest.raises(OSError, match="333"):
        cameras.find_realsense_video("depth", "333", sysfs=sysfs)

    # A realsense: camera uses them when librealsense is not installed.
    import builtins

    real_import = builtins.__import__

    def no_librealsense(name, *args, **kwargs):
        if name == "pyrealsense2":
            raise ImportError("no pyrealsense2 here")
        return real_import(name, *args, **kwargs)

    opened = []

    class FakeRaw:
        def __init__(self, device, width, height, fps):
            opened.append(device)

        def read(self):
            raise OSError("stop here")

        def release(self): ...

    monkeypatch.setattr(builtins, "__import__", no_librealsense)
    monkeypatch.setattr(cameras, "_V4l2Raw", FakeRaw)
    monkeypatch.setattr(cameras, "find_realsense_video", lambda stream, serial="": f"/dev/found-{stream}-{serial}")
    camera = cameras.Camera(CameraConfig(name="d", source="realsense:222:depth"))
    try:
        for _ in range(100):
            if opened:
                break
            time.sleep(0.02)
        assert opened[0] == "/dev/found-depth-222"
    finally:
        camera.stop()
