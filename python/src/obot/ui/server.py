"""The operator UI: one web page with the robot's cameras and state, driven with a gamepad.

    obot ui --config /etc/obot/robot.toml          then open http://<robot>:8080/

The page reads an Xbox (or any standard layout) controller through the browser and sends
its sticks and buttons here over a WebSocket; a controller on the robot itself
(--joystick) feeds the same mapper. Commands are written only while someone is actually
operating, so other clients (a policy, a script) are not overridden by an idle page.

Whoever can reach the port can drive the robot. On a network you do not control, pass
--token and open the page as http://<robot>:8080/?token=<token>.
"""

from __future__ import annotations

import asyncio
import contextlib
import hmac
import json
import os
import time
from pathlib import Path

from aiohttp import WSMsgType, web

from ..client import NotRunning, Request, Robot, describe
from ..config import ToolsConfig
from ..teleop.mapper import GamepadState, InputWatch, TeleopMapper, button_request
from . import model

STATIC = Path(__file__).parent / "static"
GAMEPAD_TIMEOUT_S = 0.3  # a silent operator is treated as having let go of everything
RELEASE_S = 1.0  # an operator who touches nothing for this long gives up control
TICK_HZ = 50
STATE_HZ = 15
_REQUESTS = {
    "enable": Request.ENABLE,
    "disable": Request.DISABLE,
    "clear_faults": Request.CLEAR_FAULTS,
    "estop": Request.ESTOP,
}


class Operator:
    """A source of gamepad input: a browser tab or the local controller."""

    def __init__(self, label: str, controller_timeout_s: float):
        self.label = label
        self.pad = GamepadState()
        self._raw = GamepadState()  # as received, before the stuck-input filter
        self.stamp = 0.0
        self.engaged_at = 0.0
        self._watch = InputWatch(controller_timeout_s)

    def update(self, pad: GamepadState) -> Request:
        """Take new input. Returns the request a newly pressed button stands for."""
        now = time.monotonic()
        request = button_request(self._raw, pad)
        self._raw = pad
        self.pad = self._watch.filter(pad, now)
        self.stamp = now
        if not self.pad.neutral():
            self.engaged_at = now
        return request

    @property
    def fresh(self) -> bool:
        return time.monotonic() - self.stamp < GAMEPAD_TIMEOUT_S

    def engaged(self) -> bool:
        """Is the operator doing something right now (any button, any stick off centre)?"""
        return self.fresh and not self.pad.neutral()

    def holding_control(self) -> bool:
        """Still counts as operating: input is arriving and was not neutral for long."""
        return self.fresh and time.monotonic() - self.engaged_at < RELEASE_S


class UiServer:
    def __init__(self, config: ToolsConfig, token: str | None = None, joystick_path: str | None = None,
                 cameras: bool = True):
        self.config = config
        self.token = token
        self.joystick_path = joystick_path
        self.enable_cameras = cameras
        self.robot: Robot | None = None
        self.mapper: TeleopMapper | None = None
        self.cameras: dict = {}
        self.world = None  # the 3D map of the surroundings, if there is a depth camera
        self.sockets: dict[web.WebSocketResponse, Operator] = {}
        self.local: Operator | None = None
        self.joystick = None
        self.controlling: Operator | None = None
        self.pending_requests: list[Request] = []
        self.status = {"robot": "connecting", "operator": None, "group": 0, "moving": False}

    # --- web -----------------------------------------------------------------------------

    def app(self) -> web.Application:
        app = web.Application(middlewares=[self._auth])
        app.router.add_get("/", self._index)
        app.router.add_get("/api/info", self._info)
        app.router.add_get("/ws", self._ws)
        app.router.add_get("/camera/{name}.mjpg", self._mjpeg)
        app.router.add_get("/camera/{name}.jpg", self._jpeg)
        app.router.add_get("/api/model", self._model)
        app.router.add_get("/api/world", self._world)
        app.router.add_get("/model/{path:.*}", self._mesh)
        app.router.add_static("/static", STATIC)
        app.on_startup.append(self._start)
        app.on_cleanup.append(self._stop)
        return app

    @web.middleware
    async def _auth(self, request: web.Request, handler):
        if self.token:
            given = request.query.get("token") or request.cookies.get("obot_token") or ""
            if not hmac.compare_digest(given, self.token):
                raise web.HTTPUnauthorized(text="token required: open the page with ?token=...")
        response = await handler(request)
        if self.token and request.query.get("token") and isinstance(response, web.Response):
            response.set_cookie("obot_token", self.token, httponly=True, samesite="Strict")
        return response

    async def _index(self, request: web.Request) -> web.Response:
        return web.Response(text=(STATIC / "index.html").read_text(), content_type="text/html")

    def _info_dict(self) -> dict:
        info = self.robot.info if self.robot else None
        return {
            "robot": info.name if info else None,
            "joints": list(info.joint_names) if info else [],
            "arm_joints": list(info.arm_joints) if info else [],
            "q_min": [float(x) for x in info.q_min] if info else [],
            "q_max": [float(x) for x in info.q_max] if info else [],
            "continuous": [bool(x) for x in info.continuous] if info else [],
            "has_base": bool(info and info.has_base),
            "cameras": list(self.cameras),
            "sim_cameras": [
                {"name": c.name, "link": c.source.split(":")[1], "mode": (c.source.split(":") + ["color"])[2]}
                for c in self.config.cameras if c.source.startswith("sim:") and self.enable_cameras
            ],
            "model": bool(self.config.urdf and os.path.isfile(self.config.urdf)),
            "world": self.world.frustums() if self.world else None,
        }

    async def _info(self, request: web.Request) -> web.Response:
        return web.json_response(self._info_dict())

    async def _model(self, request: web.Request) -> web.Response:
        if not self.config.urdf or not os.path.isfile(self.config.urdf):
            raise web.HTTPNotFound(text="no robot model configured ([model] urdf = ...)")
        return web.json_response(model.load(self.config.urdf))

    async def _world(self, request: web.Request) -> web.Response:
        """What changed in the 3D map since the revision the page has (see WorldMap.delta)."""
        if self.world is None:
            raise web.HTTPNotFound(text="no depth camera configured")
        try:
            since, epoch, trail = (int(request.query.get(key, 0)) for key in ("since", "epoch", "trail"))
        except ValueError:
            raise web.HTTPBadRequest(text="since, epoch and trail are numbers") from None
        return web.Response(body=self.world.map.delta(since, epoch, trail),
                            content_type="application/octet-stream", headers={"Cache-Control": "no-store"})

    async def _mesh(self, request: web.Request) -> web.FileResponse:
        path = model.mesh_path(self.config.urdf, request.match_info["path"]) if self.config.urdf else None
        if path is None:
            raise web.HTTPNotFound()
        return web.FileResponse(path, headers={"Cache-Control": "max-age=3600"})

    async def _ws(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=5)
        await ws.prepare(request)
        operator = Operator(f"browser {request.remote}", self.config.teleop.controller_timeout_s)
        self.sockets[ws] = operator
        try:
            await ws.send_json({"type": "info", **self._info_dict()})
            async for message in ws:
                if message.type != WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(message.data)
                except ValueError:
                    continue
                if not isinstance(data, dict):
                    continue
                if data.get("type") == "gamepad":
                    pad = GamepadState.from_lists(data.get("axes", []), data.get("buttons", []))
                    self._queue(operator.update(pad))
                elif data.get("type") == "request" and data.get("request") in _REQUESTS:
                    self._queue(_REQUESTS[data["request"]])
                elif data.get("type") == "world" and self.world is not None:
                    if data.get("action") == "clear":
                        self.world.clear()
                    elif data.get("action") == "level":   # fits a plane: not on the event loop
                        await asyncio.get_running_loop().run_in_executor(None, self.world.level)
        finally:
            self.sockets.pop(ws, None)
            if self.controlling is operator:
                self.controlling = None
        return ws

    async def _jpeg(self, request: web.Request) -> web.Response:
        camera = self.cameras.get(request.match_info["name"])
        if camera is None:
            raise web.HTTPNotFound()
        _, jpeg = camera.latest()
        if jpeg is None:
            raise web.HTTPServiceUnavailable(text=camera.error or "no frame yet")
        return web.Response(body=jpeg, content_type="image/jpeg", headers={"Cache-Control": "no-store"})

    async def _mjpeg(self, request: web.Request) -> web.StreamResponse:
        camera = self.cameras.get(request.match_info["name"])
        if camera is None:
            raise web.HTTPNotFound()
        response = web.StreamResponse(headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame",
            "Cache-Control": "no-store",
        })
        await response.prepare(request)
        last = -1
        period = 1.0 / max(1, camera.config.fps)
        with contextlib.suppress(ConnectionError, asyncio.CancelledError):
            while True:
                sequence, jpeg = camera.latest()
                if jpeg is not None and sequence != last:
                    last = sequence
                    await response.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n"
                    )
                await asyncio.sleep(period / 2)
        return response

    # --- control loop ----------------------------------------------------------------------

    async def _start(self, app: web.Application) -> None:
        if self.enable_cameras and self.config.cameras:
            from .cameras import Camera

            for camera_config in self.config.cameras:
                if not camera_config.drawn_by_page:
                    self.cameras[camera_config.name] = Camera(camera_config)
            if self.config.world.enabled and any(c.is_depth and c.name in self.cameras for c in self.config.cameras):
                from .worldview import WorldView

                self.world = WorldView(self.config, self.cameras)
                self.world.start()
        self._task = asyncio.create_task(self._loop())

    async def _stop(self, app: web.Application) -> None:
        self._task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await self._task
        if self.world:
            self.world.stop()
        for camera in self.cameras.values():
            camera.stop()
        if self.robot:
            self.robot.close()

    def _connect(self) -> None:
        try:
            self.robot = Robot(self.config.shm_name)
        except NotRunning as e:
            self.status["robot"] = str(e)
            return
        self.mapper = TeleopMapper(self.robot.info, self.config.teleop, self.config.base_v_max,
                                   self.config.base_omega_max, self.config.arm)
        self.status["robot"] = "connected"

    def _poll_joystick(self) -> None:
        if not self.joystick_path:
            return
        from ..teleop.joystick import Joystick, find_joystick

        if self.joystick is None or not self.joystick.connected:
            path = find_joystick() if self.joystick_path == "auto" else self.joystick_path
            if not path:
                return
            try:
                if self.joystick:
                    self.joystick.close()
                self.joystick = Joystick(path)
                self.local = Operator(f"controller {path}", self.config.teleop.controller_timeout_s)
            except OSError:
                return
        self._queue(self.local.update(self.joystick.poll()))

    def _queue(self, request: Request) -> None:
        """Requests come from any operator's buttons and any page, not only from whoever drives."""
        if request != Request.NONE:
            self.pending_requests.append(request)

    def _next_request(self) -> Request:
        if not self.pending_requests:
            return Request.NONE
        if Request.ESTOP in self.pending_requests:  # a stop is never queued behind anything
            self.pending_requests.clear()
            return Request.ESTOP
        return self.pending_requests.pop(0)

    def _choose_operator(self) -> Operator | None:
        """One operator at a time: whoever is in control keeps it until they let go of everything."""
        if self.controlling and self.controlling.holding_control():
            return self.controlling
        candidates = ([self.local] if self.local else []) + list(self.sockets.values())
        self.controlling = next((o for o in candidates if o.engaged()), None)
        if self.controlling is not None and self.mapper is not None:
            # Something else may have moved the arm since this page last drove it.
            self.mapper.resync()
        return self.controlling

    async def _broadcast(self, payload: dict) -> None:
        text = json.dumps(payload)
        for ws in list(self.sockets):
            with contextlib.suppress(ConnectionError, RuntimeError):
                await ws.send_str(text)

    async def _loop(self) -> None:
        dt = 1.0 / TICK_HZ
        tick = 0
        told_clients = False
        while True:
            await asyncio.sleep(dt)
            tick += 1
            if self.robot is not None and not self.robot.alive():
                self.robot.close()
                self.robot = self.mapper = None
                self.status["robot"] = "daemon stopped"
                told_clients = False
            if self.robot is None:
                if tick % TICK_HZ == 1:
                    self._connect()
                if self.robot is None:
                    if tick % (TICK_HZ // 2) == 0:
                        await self._broadcast({"type": "state", "status": self.status})
                    self.pending_requests.clear()
                    continue
            if not told_clients:
                await self._broadcast({"type": "info", **self._info_dict()})
                told_clients = True

            self._poll_joystick()
            state = self.robot.state()
            if self.world is not None and state is not None:
                joints = {name: float(q) for name, q in zip(self.robot.info.joint_names, state["q"], strict=False)}
                self.world.update_pose(float(state["odom_x"]), float(state["odom_y"]), float(state["odom_yaw"]),
                                       float(state["base_omega"]), joints)
            operator = self._choose_operator()
            pad = operator.pad if operator else GamepadState()
            out = self.mapper.step(pad, state, dt)
            request = self._next_request()
            if operator is not None or request != Request.NONE:
                self.robot.send(out.command, request)
            self.status.update(operator=operator.label if operator else None, group=out.group, moving=out.moving,
                               cartesian=out.cartesian)

            if tick % (TICK_HZ // STATE_HZ) == 0 and state is not None:
                cameras = {name: {"age": None if c.age == float("inf") else round(c.age, 2), "error": c.error,
                                  "fps": round(c.fps, 1)}
                           for name, c in self.cameras.items()}
                await self._broadcast({"type": "state", "status": self.status, "cameras": cameras,
                                       "world": self.world.describe() if self.world else None,
                                       **describe(state, self.robot.info)})


def run(config: ToolsConfig, host: str = "0.0.0.0", port: int = 8080, token: str | None = None,
        joystick: str | None = None, cameras: bool = True) -> None:
    server = UiServer(config, token=token, joystick_path=joystick, cameras=cameras)
    # Camera streams are responses that never end by themselves; do not wait a minute for
    # them when asked to stop.
    web.run_app(server.app(), host=host, port=port, print=lambda *_: None, shutdown_timeout=1.0)
