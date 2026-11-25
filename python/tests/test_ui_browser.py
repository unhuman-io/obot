"""The operator page in a real browser. Needs `pip install playwright` and
`playwright install chromium`; skipped otherwise."""

import os

import pytest

pytest.importorskip("aiohttp")
pytest.importorskip("cv2")
playwright_api = pytest.importorskip("playwright.async_api")

from aiohttp.test_utils import TestServer  # noqa: E402
from test_ui import make_server  # noqa: E402

from obot import RobotMode  # noqa: E402
from obot.config import CameraConfig  # noqa: E402


async def test_page_shows_the_robot_and_drives_it(robot, daemon):
    ui = make_server(daemon)
    ui.config.cameras += [
        CameraConfig(name=f"{where} {mode}", source=f"sim:{link}:{mode}")
        for where, link in (("base", "base_camera_link"), ("gripper", "wrist_camera_link"))
        for mode in ("color", "depth")
    ]
    from test_world import scene_config

    ui.config.cameras += scene_config().cameras       # real depth and colour pictures of a made-up room
    server = TestServer(ui.app())
    await server.start_server()
    errors = []
    async with playwright_api.async_playwright() as p:
        try:
            browser = await p.chromium.launch()
        except Exception as e:
            await server.close()
            pytest.skip(f"chromium cannot start: {str(e).splitlines()[0]}")
        page = await browser.new_page(viewport={"width": 1280, "height": 800})
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.on("console", lambda m: errors.append(m.text) if m.type == "error" else None)
        await page.goto(str(server.make_url("/")))

        # State arrives: the mode, one row per joint, the camera picture.
        await page.wait_for_function("document.getElementById('mode').textContent === 'idle'", timeout=10000)
        assert await page.text_content("#robot") == "obot3"
        assert await page.locator("#joints tbody tr").count() == 6
        assert "shoulder_pitch_joint" in await page.text_content("#joint-0")
        await page.wait_for_function(
            "(() => { const i = document.querySelector('#cameras img'); return i && i.naturalWidth > 0; })()",
            timeout=10000,
        )

        # The robot is drawn from its URDF and follows the joints.
        await page.wait_for_function("window.obotModel && window.obotModel.links > 5", timeout=15000)
        assert await page.locator("#model canvas").count() == 1
        # Simulated base and gripper cameras, colour and depth, drawn from the same model.
        assert await page.evaluate("window.obotModel.views") == 4
        assert await page.locator(".simcam canvas").count() == 4
        assert "shoulder_pitch_joint" in await page.evaluate("window.obotModel.joints")

        # What the depth camera has seen is drawn around the robot: the floor, and the red
        # shirt lying on it where the room has one.
        await page.wait_for_function("window.obotModel.world && window.obotModel.world.count > 500", timeout=15000)
        await page.wait_for_function("window.obotModel.world.near(0.9, 0.25, 0.09, 0.07) !== null", timeout=5000)
        shirt = await page.evaluate("window.obotModel.world.near(0.9, 0.25, 0.09, 0.07)")
        assert shirt[3:] == [201, 79, 79]
        await page.wait_for_function("document.getElementById('world-note').textContent.includes('map cells')")
        await page.click("#world-colour")
        assert await page.text_content("#world-colour") == "Colour: height"
        await page.click("#world-colour")
        assert await page.evaluate("window.obotModel.world.failed") == 0

        # Buttons are requests.
        await page.click("button[data-request=enable]")
        await page.wait_for_function("document.getElementById('mode').textContent === 'active'", timeout=5000)
        assert robot.state()["mode"] == RobotMode.ACTIVE

        # Keyboard driving: W alone does nothing, Shift (deadman) + W drives forward.
        await page.keyboard.down("KeyW")
        await page.wait_for_timeout(500)
        assert abs(robot.state()["base_v"]) < 0.01
        await page.keyboard.down("ShiftLeft")
        await page.wait_for_function("document.getElementById('operator').textContent.includes('moving')", timeout=5000)
        await page.wait_for_timeout(1200)
        assert robot.state()["base_v"] > 0.2
        assert await page.text_content("#pad") == "Keyboard"
        shot = os.environ.get("OBOT_UI_SCREENSHOT")
        if shot:
            await page.screenshot(path=shot, full_page=True)

        # The way driven is drawn, and the map has grown with it.
        await page.wait_for_function("window.obotModel.world.trail > 3", timeout=5000)

        # Plots: chosen signals arrive at the loop's rate and are drawn one panel per unit.
        await page.wait_for_function("window.obotPlots && document.querySelectorAll('.plot-picker input').length > 20")
        await page.evaluate("['qd/4', 'qd_cmd/4', 'base_v'].forEach((id) => window.obotPlots.select(id))")
        await page.wait_for_function("window.obotPlots.samples('qd/4') > 50", timeout=5000)
        assert await page.evaluate("window.obotPlots.panels") == 2            # rad/s and m/s
        assert await page.locator(".plot-canvas").count() == 2
        assert await page.evaluate("window.obotPlots.last('base_v')") > 0.2   # it is driving
        assert "left_wheel_joint velocity" in await page.text_content(".plot-legend")
        # Pointing at a plot reads off every line at that instant.
        await page.locator(".plot-canvas").first.scroll_into_view_if_needed()
        box = await page.locator(".plot-canvas").first.bounding_box()
        await page.mouse.move(box["x"] + box["width"] * 0.8, box["y"] + 60)
        await page.wait_for_function("getComputedStyle(document.querySelector('.plot-tip')).display === 'block'")
        assert "left_wheel_joint velocity" in await page.text_content(".plot-tip")
        await page.mouse.move(5, 5)
        await page.evaluate("window.obotPlots.select('base_v', false)")
        assert await page.evaluate("window.obotPlots.panels") == 1

        # Letting go stops the robot.
        await page.keyboard.up("ShiftLeft")
        await page.keyboard.up("KeyW")
        await page.wait_for_timeout(1200)
        assert abs(robot.state()["base_v"]) < 0.03

        # STOP is an emergency stop.
        await page.click("button[data-request=estop]")
        await page.wait_for_function("document.getElementById('mode').textContent === 'estop'", timeout=5000)
        assert "estop" in await page.text_content("#faults")
        await browser.close()
    await server.close()
    assert errors == []
