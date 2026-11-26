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


async def test_map_from_above_and_driving_to_a_click(robot, daemon):
    ui = make_server(daemon)
    ui.config.cameras.append(CameraConfig(name="low depth", source="scene:depth", width=320, height=240, fps=15,
                                          xyz=(0.116, 0.0175, 0.143), rpy=(0.0, 0.0, 0.0)))
    ui.config.world.stride = 2
    server = TestServer(ui.app())
    await server.start_server()
    errors = []
    async with playwright_api.async_playwright() as p:
        try:
            browser = await p.chromium.launch()
        except Exception as e:
            await server.close()
            pytest.skip(f"chromium cannot start: {str(e).splitlines()[0]}")
        page = await browser.new_page(viewport={"width": 1280, "height": 1400})
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.on("console", lambda m: errors.append(m.text) if m.type == "error" else None)
        await page.goto(str(server.make_url("/")))

        # The map from above appears and is drawn: free floor, things, and the robot on it.
        await page.wait_for_function("window.obotPlan && window.obotPlan.grid && window.obotPlan.grid.width > 20 "
                                     "&& window.obotPlan.pose && window.obotPlan.view", timeout=20000)
        drawn = await page.evaluate("""() => {
            const c = document.getElementById('plan');
            const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
            let n = 0; for (let i = 3; i < d.length; i += 4) if (d[i] > 0) n++;
            return n / (d.length / 4); }""")
        assert drawn > 0.02
        # ... and it lies on the floor of the 3D view too
        await page.wait_for_function("window.obotModel && window.obotModel.world && window.obotModel.world.floorPlan")

        # Without "Drive to a click" a click on the map does nothing.
        on_page = """([x, y]) => {
            const p = window.obotPlan, g = p.grid, v = p.view;
            const box = document.getElementById('plan').getBoundingClientRect();
            return [box.left + (v.left + (x / g.cell - g.i0) * v.scale) / v.ratio,
                    box.top + (v.top + (g.height - (y / g.cell - g.j0)) * v.scale) / v.ratio]; }"""
        await page.locator("#plan").scroll_into_view_if_needed()
        where = await page.evaluate(on_page, [0.3, -0.5])
        await page.mouse.click(*where)
        await page.wait_for_timeout(500)
        assert ui.nav.state == "idle" and robot.state()["mode"] == RobotMode.IDLE

        # The wheel zooms in about where the pointer is (the robot stays under it), a drag
        # moves the map, and neither sends the robot anywhere.
        await page.click("#nav-goto")
        robot_at = await page.evaluate(on_page, [0.0, 0.0])
        await page.mouse.move(*robot_at)
        await page.mouse.wheel(0, -500)
        await page.wait_for_function("window.obotPlan.zoom > 1.8")
        after = await page.evaluate(on_page, [0.0, 0.0])
        assert abs(after[0] - robot_at[0]) < 2 and abs(after[1] - robot_at[1]) < 2
        far = await page.evaluate(on_page, [1.0, 0.0])
        assert far[0] - after[0] > 1.8 * (where[0] - robot_at[0]) / 0.3               # a metre is twice as long
        await page.mouse.down()
        await page.mouse.move(after[0] - 40, after[1] + 25, steps=4)
        await page.mouse.up()
        moved = await page.evaluate(on_page, [0.0, 0.0])
        assert abs(moved[0] - (after[0] - 40)) < 3 and abs(moved[1] - (after[1] + 25)) < 3
        await page.wait_for_timeout(300)
        assert ui.nav.state == "idle"                                                  # the drag was not a click

        # A click now: the robot is enabled and drives to where the click was.
        where = await page.evaluate(on_page, [0.3, -0.5])
        await page.mouse.click(*where)
        await page.wait_for_function("document.getElementById('world-note').textContent.includes('arrived')",
                                     timeout=60000)
        at = robot.state()
        assert ((at["odom_x"] - 0.3) ** 2 + (at["odom_y"] + 0.5) ** 2) ** 0.5 < 0.12
        await page.dblclick("#plan")                                                    # the whole map again
        assert await page.evaluate("window.obotPlan.zoom") == 1

        # "Map by itself": the robot looks around and goes on from there until told to stop.
        await page.fill("#nav-radius", "1.5")
        await page.click("#nav-explore")
        await page.wait_for_function("document.getElementById('world-note').textContent.includes('mapping by itself')",
                                     timeout=10000)
        assert ui.nav.describe()["exploring"] and ui.nav._explore["radius"] == 1.5
        await page.wait_for_timeout(1500)
        assert abs(robot.state()["base_omega"]) > 0.2                                   # looking around
        await page.click("#nav-stop")
        await page.wait_for_function("!document.getElementById('world-note').textContent.includes('mapping by itself')")
        assert not ui.nav.active
        shot = os.environ.get("OBOT_UI_SCREENSHOT_MAP")
        if shot:
            await page.wait_for_timeout(1500)
            await page.locator("#plan-section").screenshot(path=shot)
        await browser.close()
    await server.close()
    assert not errors, errors
