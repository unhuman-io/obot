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
import math
import os
import sys
import time
from pathlib import Path

from aiohttp import WSMsgType, web

from .. import world as wm
from ..client import NotRunning, Request, Robot, RobotMode, describe
from ..config import ToolsConfig
from ..layout import BaseMode
from ..teleop.mapper import GamepadState, InputWatch, TeleopMapper, button_request
from . import model, plots

STATIC = Path(__file__).parent / "static"
GAMEPAD_TIMEOUT_S = 0.3  # a silent operator is treated as having let go of everything
RELEASE_S = 1.0  # an operator who touches nothing for this long gives up control
TICK_HZ = 50
STATE_HZ = 15
PLOT_BATCH_HZ = 10
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
        self.plot_signals: list[str] = []  # what this page plots, and at what rate
        self.plot_rate = 100

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
        self.nav = None  # drives the base to a point on that map
        self.labels = None  # what a network on the GPU says the things on it are
        self._nav_sent = False
        self._nav_enable = 0.0
        self._map_frame: str | None = None
        self._run_started: float | None = None  # when the obot-rtd that was last connected started
        self._where: dict | None = None  # where the robot is on the map, and how its wheels stand
        self._where_saved: dict | None = None
        self._check: dict | None = None  # a map taken up again is being checked against the camera
        self._plot_t0: int | None = None
        self.sockets: dict[web.WebSocketResponse, Operator] = {}
        self.local: Operator | None = None
        self.joystick = None
        self._joystick_looked = 0.0
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
        app.router.add_get("/api/grid", self._grid)
        app.router.add_get("/api/labels", self._labels)
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
            # Depth cameras' pictures come grey (nearest is brightest), and with them the
            # colours to paint them in: [r, g, b] for each of the 256 greys.
            "depth_cameras": [name for name, camera in self.cameras.items() if camera.config.is_depth],
            "depth_colours": self._depth_colours(),
            "labels": self.labels is not None,
            "sim_cameras": [
                {"name": c.name, "link": c.source.split(":")[1], "mode": (c.source.split(":") + ["color"])[2]}
                for c in self.config.cameras if c.source.startswith("sim:") and self.enable_cameras
            ],
            "model": bool(self.config.urdf and os.path.isfile(self.config.urdf)),
            "world": self.world.frustums() if self.world else None,
            "plot_signals": plots.signals(info.joint_names) if info else [],
            "plot_rates": [r for r in plots.RATES if info and r <= round(1 / info.period_s)],
        }

    async def _labels(self, request: web.Request) -> web.Response:
        """What the segmentation network says: of the newest picture, and of the map's squares."""
        if self.labels is None:
            raise web.HTTPNotFound(text="no segmentation network is running")
        body = await asyncio.get_running_loop().run_in_executor(None, self.labels.describe)
        return web.json_response(body, headers={"Cache-Control": "no-store"})

    def _depth_colours(self) -> list:
        if not any(camera.config.is_depth for camera in self.cameras.values()):
            return []
        from .cameras import depth_colours  # (needs OpenCV, as the cameras do)

        return depth_colours()[:, ::-1].tolist()

    async def _info(self, request: web.Request) -> web.Response:
        return web.json_response(self._info_dict())

    async def _model(self, request: web.Request) -> web.Response:
        if not self.config.urdf or not os.path.isfile(self.config.urdf):
            raise web.HTTPNotFound(text="no robot model configured ([model] urdf = ...)")
        return web.json_response(model.load(self.config.urdf))

    async def _grid(self, request: web.Request) -> web.Response:
        """The map from above (see WorldView.grid)."""
        if self.world is None:
            raise web.HTTPNotFound(text="no depth camera configured")
        body = await asyncio.get_running_loop().run_in_executor(None, self.world.grid)
        return web.Response(body=body, content_type="application/octet-stream")

    async def _world(self, request: web.Request) -> web.Response:
        """What changed in the 3D map since the revision the page has (see WorldMap.delta)."""
        if self.world is None:
            raise web.HTTPNotFound(text="no depth camera configured")
        try:
            since, epoch, trail = (int(request.query.get(key, 0)) for key in ("since", "epoch", "trail"))
        except ValueError:
            raise web.HTTPBadRequest(text="since, epoch and trail are numbers") from None
        # Not on the event loop: this waits while the map is being updated, and the loop
        # is what passes the operator's controller on to the robot.
        body = await asyncio.get_running_loop().run_in_executor(None, self.world.map.delta, since, epoch, trail)
        return web.Response(body=body, content_type="application/octet-stream", headers={"Cache-Control": "no-store"})

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
                elif data.get("type") == "plot":
                    joints = len(self.robot.info.joint_names) if self.robot else 0
                    wanted = data.get("signals") if isinstance(data.get("signals"), list) else []
                    operator.plot_signals = [s for s in wanted if isinstance(s, str) and plots.valid(s, joints)
                                             ][:plots.MAX_SIGNALS]
                    if data.get("rate") in plots.RATES:
                        operator.plot_rate = data["rate"]
                elif data.get("type") == "world" and self.world is not None:
                    if data.get("action") == "clear":
                        self.world.clear()
                        if self.labels:
                            self.labels.clear()
                    elif data.get("action") == "level":   # fits a plane: not on the event loop
                        await asyncio.get_running_loop().run_in_executor(None, self.world.level)
                    elif data.get("action") == "save":
                        await asyncio.get_running_loop().run_in_executor(None, self._save_map)
                elif data.get("type") == "nav" and self.nav is not None:
                    if data.get("action") == "goto" and self._number(data.get("x")) and self._number(data.get("y")):
                        self.nav.goto(float(data["x"]), float(data["y"]))
                    elif data.get("action") == "look":
                        turns = data.get("turns")
                        if turns is None:
                            self.nav.look_around()
                        elif self._number(turns) and 0.1 <= abs(turns) <= 10:
                            self.nav.look_around(float(turns))
                    elif data.get("action") == "explore":
                        radius = data.get("radius", 4.0)
                        if self._number(radius) and 0.5 <= radius <= 20:
                            self.nav.explore(float(radius))
                    elif data.get("action") == "cancel":
                        self.nav.cancel("stopped")
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
        for _ in range(30):   # (pictures are only encoded once somebody asks: the next one, then)
            if jpeg is not None or camera.error:
                break
            await asyncio.sleep(0.05)
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
                from .navigate import Navigator

                self.nav = Navigator(self.world)
                self._start_labels()
        self._task = asyncio.create_task(self._loop())

    def _start_labels(self) -> None:
        """The network that says what things are, if one is configured and can run here."""
        wanted = self.config.segmentation
        if not wanted.model:
            return
        from . import segment

        program = segment.find()
        if program is None or not os.path.exists(wanted.model):
            print("obot ui: [segmentation] is configured but "
                  + ("there is no obot-segd" if program is None else f"{wanted.model} is not there")
                  + ": going without", file=sys.stderr)
            return
        self.labels = segment.Labels(self.config, self.cameras, self.world, program)
        self.labels.start()

    async def _stop(self, app: web.Application) -> None:
        self._task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await self._task
        if self.labels:
            self.labels.stop()
        if self.world:
            self.world.stop()
            self._save_map()
            self.world.close()
        for camera in self.cameras.values():
            camera.stop()
        if self.robot:
            self.robot.close()

    @staticmethod
    def _number(value) -> bool:
        return isinstance(value, (int, float)) and not isinstance(value, bool) and abs(value) < 1000

    @property
    def map_path(self) -> str:
        """Where the map is kept between runs."""
        base = os.environ.get("XDG_STATE_HOME") or os.path.join(os.path.expanduser("~"), ".local", "state")
        return os.path.join(base, "obot", f"map-{self.config.shm_name}.npz")

    @property
    def where_path(self) -> str:
        """Next to the map: where on it the robot is, and how its wheels stand."""
        return self.map_path[:-len(".npz")] + ".where.json"

    def _save_map(self) -> None:
        if self.world is not None and self._map_frame is not None and len(self.world.map):
            with contextlib.suppress(OSError):
                self.world.save(self.map_path, self._map_frame)
            self._save_where(force=True)

    def _save_where(self, force: bool = False) -> None:
        """Where the robot is on its map, to a file: often, because it is what lets the map
        be taken up again when the robot is started the next time (see _take_up_map)."""
        where = self._where
        if where is None or (not force and where == self._where_saved):
            return
        self._where_saved = where
        with contextlib.suppress(OSError):
            os.makedirs(os.path.dirname(self.where_path), exist_ok=True)
            with open(self.where_path + ".part", "w") as f:
                json.dump(where, f)
            os.replace(self.where_path + ".part", self.where_path)

    def _note_where(self, state) -> None:
        if self.world is None or self._map_frame is None:
            return
        pose = [round(float(v), 4) for v in self.world.base_pose()]
        self._where = {"run": self._map_frame, "pose": pose, "wheels": self._wheels(state),
                       "correction": [round(float(v), 6) for v in self.world.correction]}

    def _wheels(self, state) -> list[float]:
        """How the wheels stand: the angle of every joint that turns without end, within a turn."""
        turning = self.robot.info.continuous
        return [round(math.remainder(float(q), 2 * math.pi), 4) for q, endless in zip(state["q"], turning, strict=False)
                if endless]

    def _wheels_as(self, before, state) -> bool:
        """Do the wheels stand as they did (to 0.03 rad, 4 mm of a 250 mm wheel's way)?"""
        now = self._wheels(state)
        return bool(now) and len(now) == len(before) and all(
            abs(math.remainder(a - b, 2 * math.pi)) < 0.03 for a, b in zip(now, before, strict=True))

    def _take_up_map(self, state) -> None:
        """Once the daemon is there: how does its odometry relate to the map?

        The map is in the frame of the odometry it was made with, and the odometry starts
        at zero with every run of obot-rtd. But a robot whose wheels stand exactly as they
        did when it was last heard of has not driven anywhere in between: it is where it
        was, and the map is taken up again from there (and then checked against what the
        camera sees, in case the robot was carried: _check_place). Otherwise there is no
        telling where on the old map the robot is, and a new one is started."""
        started = (int(state["t_real_ns"]) - int(state["cycle"]) * round(self.robot.info.period_s * 1e9)) / 1e9
        run = str(round(started))
        same = self._run_started is not None and abs(started - self._run_started) <= 2.0
        first = self._run_started is None
        self._run_started = started
        self._map_frame = run
        if self.world is None or same:
            return
        odom = (float(state["odom_x"]), float(state["odom_y"]), float(state["odom_yaw"]))
        if first:
            # The page server has just started: what does the file say?
            try:
                with open(self.where_path) as f:
                    where = json.load(f)
                pose, wheels, correction = where["pose"], where["wheels"], where["correction"]
                before = float(where["run"])
            except (OSError, ValueError, KeyError, TypeError):
                return
            if len(self.world.map):
                return
            if abs(before - started) <= 2.0:   # the same run of obot-rtd: nothing has changed
                if self.world.load(self.map_path, "*"):
                    self.world.correction = tuple(float(v) for v in correction)
                    self.world.note = ""
                return
        else:
            # obot-rtd was started again under this page server (every deploy does that).
            where = self._where
            if where is None:
                self.world.clear()
                return
            pose, wheels = where["pose"], where["wheels"]
            if self.nav is not None:
                self.nav.cancel("stopped: the robot's odometry started again")
        if self._wheels_as(wheels, state) and (not first or self.world.load(self.map_path, "*")):
            # where it was on the map, with the odometry as it is now
            self.world.correction = wm.compose(tuple(float(v) for v in pose), wm.inverse(odom))
            self.world.note = "took up the map from before: checking it against what the camera sees"
            self._check = {"until": time.monotonic() + 45.0, "from": list(self.world.agree)}
        else:
            if not first:
                self.world.clear()
            with contextlib.suppress(OSError):   # (the old map is kept once, in case)
                os.replace(self.map_path, self.map_path + ".previous")
            self.world.correction = (0.0, 0.0, 0.0)
            self.world.note = ("new map: the wheels have turned since the last one was made, so where the robot "
                               "is on it is not known")

    def _check_place(self) -> None:
        """After taking up a map on the strength of the wheels: do the pictures agree with
        it? If the robot was carried somewhere else, they show things where the map has
        none, and the map is of no use here."""
        check = self._check
        if check is None or self.world is None:
            return
        things, remembered = (self.world.agree[k] - check["from"][k] for k in (0, 1))
        if things >= 150:
            self._check = None
            if remembered < 0.4 * things:
                self.world.clear()
                self.world.correction = (0.0, 0.0, 0.0)
                with contextlib.suppress(OSError):
                    os.replace(self.map_path, self.map_path + ".previous")
                self.world.note = "new map: the robot is not where it was when the last one was made"
                if self.nav is not None:
                    self.nav.cancel("stopped: the map did not fit")
            else:
                self.world.note = "took up the map from before"
        elif time.monotonic() > check["until"]:
            self._check = None
            self.world.note = "took up the map from before (nothing in view to check it against)"

    def _drive_to_point(self, operator, request: Request, state, command, dt: float) -> bool:
        """While the base is being driven to a point on the map: put the speeds for that
        into the command. Returns whether the command is to be sent."""
        nav = self.nav
        if nav is None:
            return False
        mode = int(state["mode"])
        if nav.active:
            if operator is not None:
                nav.cancel("stopped: someone took the controls")
            elif request in (Request.DISABLE, Request.ESTOP):
                nav.cancel("stopped")
            elif mode in (RobotMode.FAULT, RobotMode.ESTOP):
                nav.cancel("stopped: the robot is not ready")
        if not nav.active:
            if self._nav_sent and operator is None:   # once more, with the speeds at zero
                self._nav_sent = False
                command["base_mode"], command["base_v"], command["base_omega"] = BaseMode.TWIST, 0.0, 0.0
                return True
            return False
        v = w = 0.0
        if mode == RobotMode.ACTIVE:
            v, w = nav.step(dt)
        elif mode == RobotMode.IDLE and time.monotonic() - self._nav_enable > 1.0:
            self._nav_enable = time.monotonic()
            self._queue(Request.ENABLE)
        command["base_mode"], command["base_v"], command["base_omega"] = BaseMode.TWIST, v, w
        self._nav_sent = True
        return True

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
            now = time.monotonic()
            if now - self._joystick_looked < 1.0:   # looking is a directory scan: not on every tick
                return
            self._joystick_looked = now
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

    async def _send_plots(self) -> None:
        """Each page's chosen signals from the state records published since the last batch."""
        plotting = [(ws, o) for ws, o in self.sockets.items() if o.plot_signals]
        records = self.robot.read()   # also when nobody plots: a plot starts with now, not with old data
        if not plotting or not len(records):
            return
        loop_hz = 1 / self.robot.info.period_s
        joints = len(self.robot.info.joint_names)
        if self._plot_t0 is None:
            self._plot_t0 = int(records["t_mono_ns"][0])
        for ws, operator in plotting:
            stride = max(1, round(loop_hz / operator.plot_rate))
            message = plots.batch(records, operator.plot_signals, stride, joints, self._plot_t0)
            if message is not None:
                with contextlib.suppress(ConnectionError, RuntimeError):
                    await ws.send_str(json.dumps(message))

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
                self._map_frame = None
                self.status["robot"] = "daemon stopped"
                told_clients = False
            if self.robot is None:
                if tick % TICK_HZ == 1:
                    self._connect()
                if self.robot is None:
                    if tick % (TICK_HZ // 2) == 0:   # cameras and the map work without the daemon
                        await self._broadcast({"type": "state", "status": self.status,
                                               "world": self.world.describe() if self.world else None})
                    self.pending_requests.clear()
                    continue
            if not told_clients:
                await self._broadcast({"type": "info", **self._info_dict()})
                told_clients = True

            self._poll_joystick()
            state = self.robot.state()
            if state is not None and self._map_frame is None:
                self._take_up_map(state)
            if tick % (TICK_HZ * 120) == 0:
                await asyncio.get_running_loop().run_in_executor(None, self._save_map)
            if tick % TICK_HZ == 0:
                self._check_place()
                asyncio.get_running_loop().run_in_executor(None, self._save_where)
            if self.world is not None and state is not None:
                self.world.joint_names = self.robot.info.joint_names
                self.world.update_pose(float(state["odom_x"]), float(state["odom_y"]), float(state["odom_yaw"]),
                                       float(state["base_omega"]), state["q"], int(state["t_mono_ns"]) * 1e-9)
                self._note_where(state)
            operator = self._choose_operator()
            pad = operator.pad if operator else GamepadState()
            out = self.mapper.step(pad, state, dt)
            request = self._next_request()
            driving = self._drive_to_point(operator, request, state, out.command, dt)
            if operator is not None or request != Request.NONE or driving:
                self.robot.send(out.command, request)
            self.status.update(operator=operator.label if operator else None, group=out.group, moving=out.moving,
                               cartesian=out.cartesian)

            if tick % (TICK_HZ // PLOT_BATCH_HZ) == 0:
                await self._send_plots()

            if tick % (TICK_HZ // STATE_HZ) == 0 and state is not None:
                cameras = {name: {"age": None if c.age == float("inf") else round(c.age, 2), "error": c.error,
                                  "fps": round(c.fps, 1), "capture_time": c.stamps_capture}
                           for name, c in self.cameras.items()}
                await self._broadcast({"type": "state", "status": self.status, "cameras": cameras,
                                       "nav": self.nav.describe() if self.nav else None,
                                       # Where the cameras are does not need the state's rate.
                                       "world": self.world.describe() if self.world and tick % 9 == 0 else None,
                                       **describe(state, self.robot.info)})


def run(config: ToolsConfig, host: str = "0.0.0.0", port: int = 8080, token: str | None = None,
        joystick: str | None = None, cameras: bool = True) -> None:
    server = UiServer(config, token=token, joystick_path=joystick, cameras=cameras)
    # Camera and map threads spend their time in OpenCV and numpy and only briefly need the
    # interpreter. With the default of 5 ms a thread that asks for it can wait that long
    # every time; on a small computer that made each of them several times slower.
    sys.setswitchinterval(0.001)
    # Camera streams are responses that never end by themselves; do not wait a minute for
    # them when asked to stop.
    web.run_app(server.app(), host=host, port=port, print=lambda *_: None, shutdown_timeout=1.0)
