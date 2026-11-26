"""What the robot's software costs in processor time, measured on the robot: `obot cpu-test`.

It finds the page server and what belongs to it (obot-navd, the programs that read the
cameras) and obot-rtd, and measures how much of a processor each uses, twice:

  1. with nobody on the operator page: what the robot costs when it is just switched on;
  2. with a page open, as a browser has it: the state and plots over the websocket, every
     camera's stream, the 3D map's changes three times a second and the map from above once
     a second.

It moves nothing. The numbers are per cent of one processor; a Jetson Nano has four, of
which one is usually kept for obot-rtd alone. The limits are for that computer, with r1's
cameras; `obot cpu-test --help` shows how to change them.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import os
import time

# name: (limit, what it is)
LIMITS = {
    "idle_pct": (45.0, "at most: the page server and what belongs to it, nobody on the page"),
    "page_pct": (90.0, "at most: the same with a page open"),
    "rtd_pct": (15.0, "at most: obot-rtd"),
    "state_hz": (12.0, "at least: states a second reaching the page"),
}
TICKS = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100


class Failed(Exception):
    """The test could not be made."""


def _processes() -> dict[int, dict]:
    """Every process: its parent, name, arguments and processor time so far (s)."""
    found = {}
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        with contextlib.suppress(OSError, ValueError, IndexError):
            with open(f"/proc/{entry}/stat") as f:
                stat = f.read()
            name = stat[stat.index("(") + 1: stat.rindex(")")]
            fields = stat[stat.rindex(")") + 2:].split()
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                arguments = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
            found[int(entry)] = {"parent": int(fields[1]), "name": name, "arguments": arguments,
                                 "seconds": (int(fields[11]) + int(fields[12])) / TICKS}
    return found


def find(port: int) -> dict[str, list[int]]:
    """The processes to measure, by what they are."""
    all_of_them = _processes()
    def is_server(arguments: str) -> bool:   # ".../python .../obot --config ... ui ...", or "obot ui"
        words = arguments.split()
        return "ui" in words and any(os.path.basename(word) == "obot" for word in words[:2])

    servers = [pid for pid, p in all_of_them.items() if is_server(p["arguments"])]
    if len(servers) > 1:   # (more than one page server on this computer: the one on this port)
        named = [pid for pid in servers if f"--port {port}" in all_of_them[pid]["arguments"]]
        servers = named or servers[:1]
    if not servers:
        raise Failed("no page server (obot ui) is running on this computer: the test measures processes, "
                     "so it has to be run where they are")
    server = servers[0]

    def below(pid: int) -> list[int]:
        children = [child for child, p in all_of_them.items() if p["parent"] == pid]
        return children + [grand for child in children for grand in below(child)]

    groups: dict[str, list[int]] = {"page server": [server], "obot-navd": [], "reading cameras": [], "obot-rtd": []}
    for pid in below(server):
        groups["obot-navd" if all_of_them[pid]["name"] == "obot-navd" else "reading cameras"].append(pid)
    groups["obot-rtd"] = [pid for pid, p in all_of_them.items() if p["name"] == "obot-rtd"]
    return groups


def _seconds(groups: dict[str, list[int]]) -> dict[str, float]:
    all_of_them = _processes()
    return {name: sum(all_of_them[pid]["seconds"] for pid in pids if pid in all_of_them)
            for name, pids in groups.items()}


def _machine() -> tuple[float, float]:
    """(busy, all) processor time of the whole computer so far, in ticks."""
    with open("/proc/stat") as f:
        numbers = [int(v) for v in f.readline().split()[1:]]
    return float(sum(numbers) - numbers[3] - numbers[4]), float(sum(numbers))


async def measure(groups: dict[str, list[int]], seconds: float) -> dict[str, float]:
    """Per cent of one processor that each group used over the next `seconds`; and under
    "computer", per cent of all of the computer that was in use."""
    before, busy, everything, began = _seconds(groups), *_machine(), time.monotonic()
    await asyncio.sleep(seconds)
    after, busy_after, everything_after, took = _seconds(groups), *_machine(), time.monotonic() - began
    used = {name: 100.0 * (after[name] - before[name]) / took for name in groups}
    used["computer"] = 100.0 * (busy_after - busy) / max(1.0, everything_after - everything)
    return used


class Page:
    """What a browser with the operator page open asks of the server, without the browser."""

    def __init__(self, url: str):
        self.url = url
        self.states = 0
        self.bytes = 0
        self.pictures: dict[str, int] = {}
        self._tasks: list[asyncio.Task] = []

    async def __aenter__(self) -> Page:
        import aiohttp

        self._session = aiohttp.ClientSession()
        try:
            async with self._session.get(self.url + "/api/info") as response:
                self.info = await response.json()
        except (OSError, asyncio.TimeoutError, ValueError) as e:
            await self._session.close()
            raise Failed(f"no answer from the page server at {self.url} ({type(e).__name__})") from None
        self._tasks = [asyncio.create_task(self._socket()), asyncio.create_task(self._grid())]
        self._tasks += [asyncio.create_task(self._stream(name)) for name in self.info.get("cameras", [])]
        if self.info.get("world"):
            self._tasks.append(asyncio.create_task(self._world()))
        return self

    async def __aexit__(self, *_) -> None:
        for task in self._tasks:
            task.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        await self._session.close()

    async def _socket(self) -> None:
        import aiohttp

        async with self._session.ws_connect(self.url + "/ws") as ws:
            # Four plots of the first joint at 100 points a second: an ordinary look at a motor.
            await ws.send_json({"type": "plot", "rate": 100, "signals": ["q/0", "qd/0", "tau/0", "q_cmd/0"]})
            async for message in ws:
                if message.type == aiohttp.WSMsgType.TEXT:
                    self.bytes += len(message.data)
                    if message.data.startswith('{"type": "state"'):
                        self.states += 1

    async def _stream(self, name: str) -> None:
        from urllib.parse import quote

        self.pictures[name] = 0
        async with self._session.get(f"{self.url}/camera/{quote(name)}.mjpg", timeout=None) as response:
            async for chunk in response.content.iter_any():
                self.bytes += len(chunk)
                self.pictures[name] += chunk.count(b"--frame\r\n")

    async def _world(self) -> None:
        from ..world import WorldMap

        revision = epoch = trail = 0
        while True:
            async with self._session.get(f"{self.url}/api/world?since={revision}&epoch={epoch}&trail={trail}") as r:
                body = await r.read()
            self.bytes += len(body)
            with contextlib.suppress(ValueError):
                change = WorldMap.decode(body)
                revision, epoch = change["rev"], change["epoch"]
                trail = change["trail_start"] + len(change["trail"])
            await asyncio.sleep(0.3)

    async def _grid(self) -> None:
        while True:
            async with self._session.get(self.url + "/api/grid") as response:
                self.bytes += len(await response.read())
            await asyncio.sleep(1.0)


async def run(url: str = "http://127.0.0.1:8080", seconds: float = 15.0, out: str | None = None,
              limits: dict | None = None, say=print) -> int:
    """Run the test. Returns 0 if everything was within its limit, 1 if not, 2 if the test
    could not be made. `run.report` (the last one made) has the numbers."""
    limit = {name: value for name, (value, _) in LIMITS.items()} | (limits or {})
    url = url.rstrip("/")
    report: dict = {"url": url, "started": time.strftime("%Y-%m-%d %H:%M:%S"), "limits": limit,
                    "processors": os.cpu_count(), "seconds": seconds}
    run.report = report
    results: list[tuple[str, bool, str]] = []

    def check(name: str, ok: bool, what: str) -> None:
        results.append((name, bool(ok), what))
        say(f"  {'ok  ' if ok else 'FAIL'}  {name}: {what}")

    def table(used: dict[str, float]) -> None:
        for name, pids in groups.items():
            if pids:
                say(f"        {name:<16} {used[name]:5.1f} %")
        say(f"        (the whole computer, {os.cpu_count()} processors: {used['computer']:.0f} % in use)")

    try:
        port = int(url.rsplit(":", 1)[1].split("/")[0]) if url.count(":") > 1 else 80
        groups = find(port)
        ours = [name for name in groups if name != "obot-rtd"]
        async with Page(url) as page:   # (is it there at all? and once round, so that nothing is being started)
            await asyncio.sleep(2.0)
            cameras = page.info.get("cameras", [])
        await asyncio.sleep(3.0)
        groups = find(port)   # (the cameras' programs may have been started since)

        say(f"nobody on the page, {seconds:.0f} s:")
        idle = await measure(groups, seconds)
        table(idle)
        say(f"a page open ({len(cameras)} camera streams, plots, the 3D map), {seconds:.0f} s:")
        async with Page(url) as page:
            await asyncio.sleep(3.0)   # (the first of everything is the whole of everything)
            states, pictures, received, began = page.states, dict(page.pictures), page.bytes, time.monotonic()
            open_page = await measure(groups, seconds)
            took = time.monotonic() - began
            state_hz = (page.states - states) / took
            rates = {name: (page.pictures[name] - pictures.get(name, 0)) / took for name in page.pictures}
            sent = (page.bytes - received) / took
        table(open_page)
        say("        the page got " + ", ".join(f"{rate:.0f} pictures a second of '{name}'"
                                                 for name, rate in rates.items())
            + f"; {state_hz:.0f} states a second; {sent / 1e6:.2f} MB a second in all")

        idle_ours, page_ours = sum(idle[name] for name in ours), sum(open_page[name] for name in ours)
        report.update(idle=idle, page=open_page, idle_pct=idle_ours, page_pct=page_ours, state_hz=state_hz,
                      pictures_hz=rates, bytes_per_s=sent,
                      rtd_pct=max(idle["obot-rtd"], open_page["obot-rtd"]))
        say("of one processor:")
        check("the page server and what belongs to it, nobody on the page", idle_ours <= limit["idle_pct"],
              f"{idle_ours:.0f} % (at most {limit['idle_pct']:.0f} %)")
        check("the same with a page open", page_ours <= limit["page_pct"],
              f"{page_ours:.0f} % (at most {limit['page_pct']:.0f} %)")
        if groups["obot-rtd"]:
            check("obot-rtd", report["rtd_pct"] <= limit["rtd_pct"],
                  f"{report['rtd_pct']:.0f} % (at most {limit['rtd_pct']:.0f} %)")
        check("the page is kept up to date", state_hz >= limit["state_hz"],
              f"{state_hz:.0f} states a second (at least {limit['state_hz']:.0f})")
    except Failed as e:
        say(f"the test could not be made: {e}")
        return 2

    failed = [name for name, ok, _ in results if not ok]
    report["results"] = [{"check": name, "ok": ok, "what": what} for name, ok, what in results]
    report["passed"] = not failed
    if out:
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        with open(out, "w") as f:
            json.dump(report, f, indent=1, default=float)
        say(f"report: {out}")
    say("CPU TEST PASSED" if not failed else "CPU TEST FAILED: " + "; ".join(failed))
    return 1 if failed else 0
