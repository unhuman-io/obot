"""A test of the map on the real robot, to be started by hand: `obot map-test`.

It talks to the operator page's server as a page does, so everything is in it that is in
normal use: the camera, obot-rtd's odometry, the map (obot-navd), the navigation.

A map is good if things are in the same place in it whenever the robot looks at them. So
the robot makes a map of what is around it several times, each time from nothing and by
turning once on the spot, and the maps are laid over each other: how far does one have to
be turned and moved to fit the other?

  A   one turn to the left
      (some more turns to the left)
  B   one turn to the left         B on A: the base turned more, or less, than the odometry
                                   says. Both were seen the same way, only turns apart.
  C   one turn to the right        C on B: the pictures are older, or newer, than they are
                                   taken for. Then everything is drawn a little ahead of
                                   where it is, or behind: to the left turning left, to the
                                   right turning right.
      (a drive to a free place nearby and back)
  D   one turn to the left         D on B: what driving adds.

The first two come out as numbers that can be put right in the configuration: a factor for
the base's `track_width`, and milliseconds for the camera's `latency`.

The limits are from the robot this was made on (a Jetson Nano with a D435 in a cluttered
garage), with room to spare; `obot map-test --help` shows how to change them.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import math
import os
import struct
import time

import numpy as np

LIMITS = {
    "scale_pct": (0.3, "at most: per cent that the base turns more or less than the odometry says"),
    "delay_ms": (15.0, "at most: ms that the depth pictures are older or newer than they are taken for"),
    "overlap": (0.75, "at least: share of one map's places with something that another map has too, fitted"),
    "drive_shift_m": (0.12, "at most: how far the map has moved after the drive"),
    "drive_turn_deg": (3.0, "at most: how far the map has turned after the drive"),
    "update_ms": (120.0, "at most: one map update"),
    "camera_fps": (4.0, "at least: depth pictures a second"),
}
ACTIVE = ("planning", "driving", "looking")
NEAR, FAR = 0.4, 4.0  # m: what is compared of a map is between these distances from the robot
LOW, HIGH = 0.05, 1.5  # m: ... and between these heights above the floor


class Failed(Exception):
    """The test could not be made."""


# --- laying one map over another -----------------------------------------------------------


def things(body: bytes, centre) -> tuple[np.ndarray, float]:
    """Where in the map there is something that stands on the floor, around `centre`: the
    (x, y) of its voxels between LOW and HIGH above the floor, each place once (N x 2); and
    the size of a voxel. `body` is the map as the page gets it."""
    from ..world import WorldMap

    data = WorldMap.decode(body)
    voxel = float(data["voxel"])
    index = data["changed"].astype(np.int64)
    z = index[:, 2] * voxel
    at = np.unique(index[(z > LOW) & (z < HIGH), :2], axis=0) * voxel
    away = np.hypot(at[:, 0] - centre[0], at[:, 1] - centre[1])
    return at[(away > NEAR) & (away < FAR)], voxel


def fit(a: np.ndarray, b: np.ndarray, centre, cell: float, most_turn: float = math.radians(14),
        most_shift: float = 0.18) -> dict:
    """How map `b` has to be turned (about `centre`) and moved to lie on map `a`: the turn
    (rad, to the left), the move (m), and the share of b's squares that then lie on one of
    a's or next to one.

    Map a is drawn on a grid three times as fine and blurred, which makes how well b lies
    on it a smooth thing of the turn and the move, with one top that can be climbed: places
    that only count as on or off give an answer that is a fraction of a degree out, and a
    fraction of a degree is what is asked for here."""
    if len(a) < 300 or len(b) < 300:
        raise Failed(f"too little in the map to compare ({len(a)} and {len(b)} places with something within "
                     f"{FAR:.0f} m): the test needs walls or furniture around the robot")
    centre = np.asarray(centre[:2], float)
    fine = cell / 3
    low = np.minimum(a.min(axis=0), b.min(axis=0)) - most_shift - 1.0
    size = np.ceil((np.maximum(a.max(axis=0), b.max(axis=0)) + most_shift + 1.0 - low) / fine).astype(int) + 2
    drawn = np.zeros((size[0], size[1]), np.float32)
    index = np.floor((a - low) / fine).astype(int)
    for di in (-1, 0, 1):
        for dj in (-1, 0, 1):
            drawn[index[:, 0] + di, index[:, 1] + dj] = 1.0
    squares = drawn.copy()
    for _ in range(3):   # (three times a box of five is close to a bell)
        for axis in (0, 1):
            drawn = sum(np.roll(drawn, k, axis=axis) for k in (-2, -1, 0, 1, 2)) / 5

    def placed(turn: float, dx: float, dy: float) -> np.ndarray:
        c, s = math.cos(turn), math.sin(turn)
        return ((b - centre) @ np.array([[c, s], [-s, c]]) + centre + (dx, dy) - low) / fine - 0.5

    def lying(turn: float, dx: float, dy: float) -> float:
        p = placed(turn, dx, dy)
        i = np.clip(np.floor(p).astype(int), 0, size - 2)
        u, v = np.clip(p[:, 0] - i[:, 0], 0, 1), np.clip(p[:, 1] - i[:, 1], 0, 1)
        return float((drawn[i[:, 0], i[:, 1]] * (1 - u) * (1 - v) + drawn[i[:, 0] + 1, i[:, 1]] * u * (1 - v)
                      + drawn[i[:, 0], i[:, 1] + 1] * (1 - u) * v + drawn[i[:, 0] + 1, i[:, 1] + 1] * u * v).sum())

    # Roughly everywhere it could be (with a part of b: there is a lot of everywhere), then
    # uphill from the best of that in ever smaller steps.
    whole, b = b, b[:: max(1, len(b) // 1500)]
    moves = np.arange(-most_shift, most_shift + 1e-9, 0.06)
    at = max(((turn, dx, dy) for turn in np.arange(-most_turn, most_turn + 1e-9, math.radians(1.0))
              for dx in moves for dy in moves), key=lambda place: lying(*place))
    b = whole
    at, top = list(at), lying(*at)
    steps = [math.radians(0.4), 0.02, 0.02]
    while steps[0] > math.radians(0.005):
        better = False
        for k in range(3):
            for way in (-1.0, 1.0):
                there = list(at)
                there[k] += way * steps[k]
                value = lying(*there)
                if value > top:
                    at, top, better = there, value, True
        if not better:
            steps = [step / 2 for step in steps]
    p = np.clip(np.floor(placed(*at) + 0.5).astype(int), 0, size - 1)
    near = np.zeros(len(b), bool)   # on one of a's places, or next to one
    for di in (-3, 0, 3):
        for dj in (-3, 0, 3):
            near |= squares[np.clip(p[:, 0] + di, 0, size[0] - 1), np.clip(p[:, 1] + dj, 0, size[1] - 1)] > 0
    return {"turn": float(at[0]), "move": (float(at[1]), float(at[2])), "overlap": float(near.mean()),
            "squares": (len(a), len(b))}


# --- talking to the page server ---------------------------------------------------------------


class Session:
    def __init__(self, session, ws, url: str, say):
        self.session, self.ws, self.url, self.say = session, ws, url, say
        self.cell = 0.03
        self.state: dict = {}
        self.world: dict = {}
        self.worst_update = 0.0
        self.moved = False
        self.driven = 0.0    # m
        self.turned = 0.0    # rad, either way
        self._last = None
        self._fresh = asyncio.Event()
        self._closed = False

    async def read(self) -> None:
        """Take what the server sends, for as long as the test runs: a connection that is
        not read from does not answer the server's pings either, and is closed by it."""
        import aiohttp

        try:
            async for message in self.ws:
                if message.type != aiohttp.WSMsgType.TEXT:
                    continue
                data = json.loads(message.data)
                if data.get("type") != "state" or "mode" not in data or not data.get("nav"):
                    continue
                self.state = data
                if data.get("world"):
                    self.world = data["world"]
                    self.worst_update = max(self.worst_update, float(self.world.get("update_ms") or 0.0))
                odom = data["odom"]
                if self._last is not None:
                    self.driven += math.hypot(odom[0] - self._last[0], odom[1] - self._last[1])
                    self.turned += abs(odom[2] - self._last[2])
                self._last = odom
                self._fresh.set()
        finally:
            self._closed = True
            self._fresh.set()

    async def next(self, timeout: float = 5.0) -> dict:
        """The next state the server sends."""
        self._fresh.clear()
        if not self._closed:
            with contextlib.suppress(asyncio.TimeoutError):
                await asyncio.wait_for(self._fresh.wait(), timeout)
        if self._closed:
            raise Failed("the page server closed the connection")
        if not self._fresh.is_set():
            raise Failed("no state from the page server: is obot-rtd running?")
        return self.state

    def pose(self) -> tuple[float, float, float]:
        """Where the robot is on the map."""
        cx, cy, cyaw = self.world.get("correction", (0.0, 0.0, 0.0))
        x, y, yaw = self.state["odom"]
        c, s = math.cos(cyaw), math.sin(cyaw)
        return cx + c * x - s * y, cy + s * x + c * y, yaw + cyaw

    async def settle(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            await self.next()

    async def drive(self, what: str, message: dict, timeout: float) -> dict:
        """Ask for a drive (a turn, a place to go to) and wait until it is over. Returns how
        it ended, with the rate it turned at (rad/s, signed; the median)."""
        self.moved = True
        await self.ws.send_json({"type": "nav", **message})
        began, started, rates = time.monotonic(), False, []
        while time.monotonic() - began < timeout:
            state = await self.next()
            nav = state["nav"]
            if nav["state"] in ACTIVE:
                started = True
                if nav["state"] == "looking" and abs(state["base_omega"]) > 0.1:
                    rates.append(state["base_omega"])
            elif started or time.monotonic() - began > 3.0:
                self.say(f"  {what}: {nav['state']}{' (' + nav['note'] + ')' if nav['note'] else ''}, "
                         f"{time.monotonic() - began:.0f} s")
                return {**nav, "rate": float(np.median(rates)) if rates else 0.0}
        raise Failed(f"{what} was not over after {timeout:.0f} s")

    async def stop(self) -> None:
        with contextlib.suppress(Exception):
            await self.ws.send_json({"type": "nav", "action": "cancel"})
            await self.ws.send_json({"type": "request", "request": "disable"})
            end = time.monotonic() + 3.0
            while time.monotonic() < end and (await self.next())["mode"] != "idle":
                pass

    async def grid(self) -> dict:
        async with self.session.get(self.url + "/api/grid") as response:
            body = await response.read()
        if len(body) < 24:
            raise Failed("no map from above from the page server")
        magic, i0, j0, width, height, cell = struct.unpack_from("<IiiIIf", body)
        if magic != 0x3147424F:
            raise Failed("the page server's map from above is not what this test knows")
        cells = np.frombuffer(body, np.uint8, offset=24).reshape(height, width)
        return {"i0": i0, "j0": j0, "cell": cell, "cells": cells}

    async def map_of_a_turn(self, name: str, turns: float) -> dict:
        """Forget the map, turn on the spot, and return the map that makes, with where the
        odometry had the robot facing when the turn began and the rate it turned at."""
        await self.ws.send_json({"type": "world", "action": "clear"})
        await self.settle(1.0)
        yaw = self.state["odom"][2]
        way = "left" if turns > 0 else "right"
        nav = await self.drive(f"map {name}, a turn to the {way}", {"action": "look", "turns": turns},
                               60.0 + 30.0 * abs(turns))
        if nav["state"] != "arrived":
            raise Failed(f"the turn for map {name} ended as '{nav['state']}'"
                         + (f" ({nav['note']})" if nav["note"] else "")
                         + ("; the test needs a place where the robot can turn all the way round"
                            if "no room" in nav["note"] else ""))
        await self.settle(1.5)   # (the last pictures of the turn)
        async with self.session.get(self.url + "/api/world") as response:
            body = await response.read()
        centre = self.pose()
        places, self.cell = things(body, centre)
        return {"name": name, "grid": await self.grid(), "things": places, "centre": centre, "yaw": yaw,
                "rate": nav["rate"], "voxels": int(self.world.get("voxels", 0))}


def _a_free_place(grid: dict, pose, near: float, far: float):
    """A place to drive to: free floor between `near` and `far` metres away, with as much
    room around it as there is."""
    cells, cell = grid["cells"], grid["cell"]
    rows, columns = np.nonzero(cells == 1)
    if not len(rows):
        return None
    x = (columns + grid["i0"] + 0.5) * cell
    y = (rows + grid["j0"] + 0.5) * cell
    away = np.hypot(x - pose[0], y - pose[1])
    pick = np.nonzero((away >= near) & (away <= far))[0]
    if not len(pick):
        return None
    other_rows, other_columns = np.nonzero(cells != 1)
    best, most = None, -1.0
    for k in pick[:: max(1, len(pick) // 300)]:
        room = float(np.hypot(other_rows - rows[k], other_columns - columns[k]).min()) * cell
        if room > most:
            best, most = (float(x[k]), float(y[k])), room
    return best


# --- the test --------------------------------------------------------------------------------


async def run(url: str = "http://127.0.0.1:8080", turns: int = 3, drive: bool = True, out: str | None = None,
              limits: dict | None = None, say=print) -> int:
    """Run the test. Returns 0 if everything was within its limit, 1 if not, 2 if the test
    could not be made. `report` (the last one made) has the numbers."""
    import aiohttp

    limit = {name: value for name, (value, _) in LIMITS.items()} | (limits or {})
    results: list[tuple[str, bool, str]] = []
    maps: list[dict] = []
    report: dict = {"url": url, "started": time.strftime("%Y-%m-%d %H:%M:%S"), "limits": limit}
    run.report = report

    def check(name: str, ok: bool, what: str) -> None:
        results.append((name, bool(ok), what))
        say(f"  {'ok  ' if ok else 'FAIL'}  {name}: {what}")

    async def lay(b: dict, a: dict, most_shift: float = 0.18) -> dict:
        found = await asyncio.get_running_loop().run_in_executor(   # (seconds, on a small computer)
            None, lambda: fit(a["things"], b["things"], b["centre"], t.cell, most_shift=most_shift))
        check(f"map {b['name']} is map {a['name']} again", found["overlap"] >= limit["overlap"],
              f"{found['overlap'] * 100:.0f} % of its {found['squares'][1]} places with something lie on "
              f"{a['name']}'s once fitted (at least {limit['overlap'] * 100:.0f} %)")
        return found

    try:
        async with aiohttp.ClientSession() as session, session.ws_connect(url.rstrip("/") + "/ws") as ws:
            t = Session(session, ws, url.rstrip("/"), say)
            reading = asyncio.create_task(t.read())
            try:
                await t.settle(2.0)
                state = t.state
                if state.get("mode") in ("fault", "estop"):
                    raise Failed(f"the robot is in mode {state.get('mode')}: see obot status")
                if not t.world:
                    raise Failed("the page server has no map: is there a depth camera?")
                say("before starting:")
                for name in t.world.get("cameras", {}):
                    camera = state.get("cameras", {}).get(name, {})
                    fps = camera.get("fps", 0)
                    check(f"camera '{name}' delivers", not camera.get("error") and fps >= limit["camera_fps"],
                          f"{camera.get('fps', 0)} pictures a second (at least {limit['camera_fps']:.0f})"
                          + (f", error: {camera['error']}" if camera.get("error") else ""))
                    check(f"camera '{name}' says when its pictures were taken", camera.get("capture_time", False),
                          "yes" if camera.get("capture_time") else "no: their time is when they arrive, which is later "
                          "and not always by the same")

                say(f"turning on the spot ({turns + 3} turns in all), making a map of each of four of them:")
                a = await t.map_of_a_turn("A", 1)
                maps.append(a)
                more = await t.drive(f"{turns} more to the left", {"action": "look", "turns": turns},
                                     60.0 + 30.0 * turns)
                if more["state"] != "arrived":
                    raise Failed("the turns ended as '" + more["state"] + "' " + more["note"])
                b = await t.map_of_a_turn("B", 1)
                maps.append(b)
                c = await t.map_of_a_turn("C", -1)
                maps.append(c)

                between = b["yaw"] - a["yaw"]   # by the odometry
                found = await lay(b, a)
                scale, by = found["turn"] / between, math.degrees(found["turn"])
                report.update(turned_deg=math.degrees(between), b_on_a=found, scale_pct=scale * 100)
                check("the base turns as far as the odometry says", abs(scale * 100) <= limit["scale_pct"],
                      f"map B, {math.degrees(between):.0f} degrees of turning after A, has to be turned "
                      f"{by + 0.0:+.2f} degrees to lie on A: the base turns {abs(scale) * 100:.2f} % "
                      f"{'more' if scale >= 0 else 'less'} than the odometry says (at most {limit['scale_pct']} %). "
                      f"To put right: [base] track_width times {1 / (1 + scale):.4f}")

                found = await lay(c, b)
                rate = (abs(b["rate"]) + abs(c["rate"])) / 2
                if rate < 0.1:
                    raise Failed("no rate of turning in the states")
                delay, by = found["turn"] / (2 * rate), math.degrees(found["turn"])
                report.update(c_on_b=found, delay_ms=delay * 1000, rate=rate)
                check("the pictures are from when they are taken to be", abs(delay * 1000) <= limit["delay_ms"],
                      f"map C, made turning right at {rate:.2f} rad/s, has to be turned {by:+.2f} degrees to lie "
                      f"on B, made turning left: the pictures are {abs(delay) * 1000:.0f} ms "
                      f"{'older' if delay > 0 else 'newer'} than they are taken for "
                      f"(at most {limit['delay_ms']:.0f} ms)")

                if drive:
                    say("driving to a free place nearby and back:")
                    start = t.pose()
                    place = _a_free_place(c["grid"], start, 0.8, 1.6)
                    if place is None:
                        check("a free place to drive to", False, "none between 0.8 and 1.6 m away in the map")
                    else:
                        driven, turned = t.driven, t.turned
                        there = await t.drive(f"to ({place[0]:+.2f}, {place[1]:+.2f})",
                                              {"action": "goto", "x": place[0], "y": place[1]}, 120.0)
                        off = math.hypot(t.pose()[0] - place[0], t.pose()[1] - place[1])
                        check("it got there", there["state"] == "arrived" and off < 0.15,
                              f"{off * 100:.0f} cm from the place" + (f" ({there['note']})" if there["note"] else ""))
                        back = await t.drive("back", {"action": "goto", "x": start[0], "y": start[1]}, 120.0)
                        off = math.hypot(t.pose()[0] - start[0], t.pose()[1] - start[1])
                        check("it came back", back["state"] == "arrived" and off < 0.15,
                              f"{off * 100:.0f} cm from where it started"
                              + (f" ({back['note']})" if back["note"] else ""))
                        driven, turned = t.driven - driven, t.turned - turned
                        d = await t.map_of_a_turn("D", 1)
                        maps.append(d)
                        found = await lay(d, b, most_shift=0.36)
                        shift, by = math.hypot(*found["move"]), math.degrees(found["turn"])
                        report.update(d_on_b=found, driven_m=driven, drive_turned_deg=math.degrees(turned))
                        check("the map is where it was after driving",
                              shift <= limit["drive_shift_m"] and abs(by) <= limit["drive_turn_deg"],
                              f"after {driven:.1f} m and {math.degrees(turned):.0f} degrees of turning, map D has "
                              f"to be moved {shift * 100:.0f} cm and turned {by:+.1f} degrees to lie on B (at most "
                              f"{limit['drive_shift_m'] * 100:.0f} cm and {limit['drive_turn_deg']:.0f} degrees)")
                check("a map update is quick enough", t.worst_update <= limit["update_ms"],
                      f"{t.worst_update:.0f} ms at the most (at most {limit['update_ms']:.0f})")
                report["update_ms"] = t.worst_update
            finally:
                if t.moved:
                    await t.stop()   # however it ended: the robot stands and is disabled
                reading.cancel()
    except Failed as e:
        say(f"the test could not be made: {e}")
        return 2
    except (OSError, asyncio.TimeoutError) as e:
        say(f"the test could not be made: no answer from the page server at {url} ({type(e).__name__} {e})")
        return 2

    failed = [name for name, ok, _ in results if not ok]
    report["results"] = [{"check": name, "ok": ok, "what": what} for name, ok, what in results]
    report["passed"] = not failed
    if out:
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, "report.json"), "w") as f:
            json.dump(report, f, indent=1, default=float)
        _picture(maps, os.path.join(out, "maps.png"))
        say(f"report and a picture of the maps over each other: {out}")
    say("MAP TEST PASSED" if not failed else "MAP TEST FAILED: " + "; ".join(failed))
    return 1 if failed else 0


def _picture(maps: list[dict], path: str) -> None:
    """The maps over each other as they are (not fitted), each in its colour, as a PNG; if
    OpenCV is there to write it. Where they agree it is dark."""
    try:
        import cv2
    except ImportError:
        return
    if not maps:
        return
    colours = ((200, 120, 0), (0, 0, 220), (0, 160, 0), (160, 0, 160))   # BGR: A blue, B red, C green, D purple
    cell, scale = 0.03, 3
    every = np.concatenate([m["things"] for m in maps if len(m["things"])] or [np.zeros((1, 2))])
    low, high = every.min(axis=0) - 0.3, every.max(axis=0) + 0.3
    width, height = (np.ceil((high - low) / cell).astype(int) * scale).tolist()
    picture = np.full((height, width, 3), 255, np.float32)
    for m, colour in zip(maps, colours, strict=False):
        layer = np.zeros((height, width), np.float32)
        for x, y in m["things"]:
            u, v = int((x - low[0]) / cell) * scale, height - int((y - low[1]) / cell + 1) * scale
            layer[max(0, v): v + scale, max(0, u): u + scale] = 1.0
        picture -= layer[:, :, None] * (255 - np.array(colour, np.float32)) * 0.55
    picture = np.ascontiguousarray(np.clip(picture, 0, 255).astype(np.uint8))
    for k, (m, colour) in enumerate(zip(maps, colours, strict=False)):
        cv2.putText(picture, m["name"], (8 + 22 * k, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
    cv2.imwrite(path, picture)
