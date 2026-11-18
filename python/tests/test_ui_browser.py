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


async def test_page_shows_the_robot_and_drives_it(robot, daemon):
    server = TestServer(make_server(daemon).app())
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
