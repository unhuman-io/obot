"""Driving the base to a point on the map.

Three layers, each simple:

- A plan: the shortest way from where the base is to the goal that keeps the base clear of
  everything in the map (Costmap.plan, A* on a grid). What the map has not seen counts as
  free: the depth camera looks where the base is going and will see it before the base is
  there.
- Following it: turn towards a point a little way along the plan, drive when facing it.
  The base only ever drives forwards, because forwards is where the camera looks.
- Not hitting things: whatever the plan says, the base does not drive on when the depth
  camera sees something close ahead (WorldView.ahead), or when the camera has stopped
  delivering pictures. It then waits for a new plan, which goes around what is now in the map.

Whoever touches a controller takes over at once (the page server cancels the drive).
"""

from __future__ import annotations

import contextlib
import heapq
import math
import threading
import time

import numpy as np

from .. import world

# The base from above, in its own frame (x forward from the middle of the wheel axis, y to the
# left), as rectangles (from x, to x, half the width), from the CAD model: 0.45 m wide at the
# wheels, with a narrow tail that carries the caster.
FOOTPRINT = ((-0.125, 0.125, 0.224),   # the wheels and what is between them
             (-0.26, -0.125, 0.16),    # the deck behind them
             (-0.375, -0.26, 0.05))    # the tail with the caster
# How close the middle of the wheel axis may come to things. Driving straight, the base
# needs half its width; a plan is followed to a few centimetres and the map is good to a
# few more. A gap has to be twice this wide to be driven through, and a little for where
# the squares of the plan happen to lie: 0.64 m. (Whichever way the base faces, its wheels
# and all between them stay within 0.26 m of the middle of the axis; only the tail does not.)
RADIUS = 0.30  # m
# Turning on the spot the tail swings round at 0.375 m: where there is less room than this
# the base can pass, but a plan avoids having to turn there (and before every move the
# base's whole outline is checked against the map, see Navigator._fits).
TURN = 0.40  # m
MARGIN = 0.30  # m: where there is room, a way keeps this much farther from things still
UNSEEN = 0.32  # m: floor that has not been seen may have anything on it
CELL = 0.06  # m: squares for choosing where to look next
PLAN_CELL = 0.03  # m: squares for the way to a place
FINE = 0.03  # m: the grid things are placed on (the map's voxels)
SEEN_CELL = 0.06  # m: the squares WorldView keeps "seen to be free" on
V_MAX, W_MAX = 0.25, 0.7  # m/s, rad/s
V_ACCEL, W_ACCEL = 0.4, 2.0  # m/s^2, rad/s^2
LOOKAHEAD = 0.25  # m: the point steered for is this far along the plan
ARRIVED = 0.06  # m
FRONT = 0.13  # m: the front edge of the base, from the middle of the wheel axis
STOP_GAP = 0.22  # m: nearer than this to the front edge, the base does not drive on
SLOW_GAP = 0.60  # m: from here on it slows down
STALE_S = 0.7  # s: pictures older than this are no pictures


class Costmap:
    """The map from above, around a start and a goal: where the base cannot be, and how
    close to things each place is."""

    def __init__(self, obstacles: np.ndarray, start, goal, known=None, cell: float = CELL, pad: float = 2.5):
        """obstacles: (x, y) of what stands on the floor. known(origin, shape): which squares
        of the grid have been seen to be free floor (WorldView.known); the base only goes
        where that is so. None: everywhere."""
        import cv2  # here so that the rest of the package works without OpenCV

        # The plan is made on squares of `cell`: 3 cm for a way to a place (a gap the base
        # just fits through must not be lost between two squares), 6 cm for choosing where
        # to look next, which goes over everything that can be reached and need not be
        # exact. Things are placed on a grid of 3 cm either way, the size of the map's
        # voxels and centred like them. With 3 cm squares the two grids are the same; a
        # 6 cm square [j, i] has its middle in the fine square [2 j + 1, 2 i + 1].
        self.cell = cell
        self.fine = FINE
        self.k = 2 if cell > 1.5 * FINE else 1
        low = np.minimum(start, goal) - pad
        high = np.maximum(start, goal) + pad
        if self.k == 2:
            self.origin = np.floor(low / cell).astype(np.int64)
            self.shape = tuple((np.ceil(high / cell).astype(np.int64) - self.origin + 1)[::-1])  # rows (y), columns (x)
            self.fine_origin = 2 * self.origin
            fine = np.zeros((2 * self.shape[0] + 1, 2 * self.shape[1] + 1), bool)
        else:
            self.origin = np.rint(low / cell).astype(np.int64)
            self.shape = tuple((np.rint(high / cell).astype(np.int64) - self.origin + 1)[::-1])
            self.fine_origin = self.origin
            fine = np.zeros(self.shape, bool)
        if len(obstacles):
            index = np.rint(np.asarray(obstacles, np.float64) / self.fine).astype(np.int64) - self.fine_origin
            i, j = index[:, 0], index[:, 1]
            inside = (i >= 0) & (i < fine.shape[1]) & (j >= 0) & (j < fine.shape[0])
            fine[j[inside], i[inside]] = True
        self.fine_occupied = fine
        self.fine_clear = cv2.distanceTransform((~fine).astype(np.uint8), cv2.DIST_L2,
                                                cv2.DIST_MASK_PRECISE) * self.fine
        # per square of the plan: is anything in it, and how far is its middle from things
        if self.k == 2:
            occupied = (fine[:-1:2, :-1:2] | fine[1::2, :-1:2] | fine[2::2, :-1:2] | fine[:-1:2, 1::2]
                        | fine[1::2, 1::2] | fine[2::2, 1::2] | fine[:-1:2, 2::2] | fine[1::2, 2::2] | fine[2::2, 2::2])
            self.clearance = self.fine_clear[1::2, 1::2].astype(np.float64)
        else:
            occupied = fine
            self.clearance = self.fine_clear.astype(np.float64)
        self.occupied = occupied
        from_things = self.clearance
        self.from_things = from_things
        self.blocked = from_things < RADIUS
        self.unknown = np.zeros(self.shape, bool)
        from_unknown = np.full(self.shape, np.inf)
        if known is not None:
            # What has been seen is kept on squares of 6 cm (WorldView.known). For a plan on
            # finer squares: each of them is in one of those.
            if self.k == 2:
                seen = known(self.origin, self.shape).astype(np.uint8)
            else:
                ratio = round(SEEN_CELL / cell)
                coarse_origin = self.origin // ratio
                coarse_shape = ((self.origin[1] + self.shape[0] - 1) // ratio - coarse_origin[1] + 1,
                                (self.origin[0] + self.shape[1] - 1) // ratio - coarse_origin[0] + 1)
                seen = known(coarse_origin, coarse_shape).astype(np.uint8)
            # (closing the small gaps that the rows of a picture leave between them far away)
            seen = cv2.morphologyEx(seen, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)).astype(bool)
            if self.k != 2:
                rows = (self.origin[1] + np.arange(self.shape[0])) // ratio - coarse_origin[1]
                columns = (self.origin[0] + np.arange(self.shape[1])) // ratio - coarse_origin[0]
                seen = seen[np.ix_(rows, columns)]
            self.unknown = ~seen & ~occupied
            # What has not been seen may be anything, the back of the box the camera saw the
            # front of for one: the base keeps clear of it too.
            from_unknown = cv2.distanceTransform((~self.unknown).astype(np.uint8), cv2.DIST_L2,
                                                 cv2.DIST_MASK_PRECISE) * cell
            # (for the price of a step, the unseen counts like a thing)
            self.clearance = np.minimum(from_things, np.where(self.unknown, from_things, from_unknown))
        beside_unknown = from_unknown < UNSEEN
        # Standing closer to something, or to unseen floor, than a plan would go (the base
        # was driven there by hand; where it starts, little has been seen): the way out is
        # allowed, but only through places at least as clear as where it stands. Never
        # closer: a base that may come a little closer with every new plan ends up touching.
        si, sj = self.index(start)
        if self.inside(si, sj):
            rows, columns = np.ogrid[: self.shape[0], : self.shape[1]]
            around = (rows - sj) ** 2 + (columns - si) ** 2 < (0.6 / cell) ** 2
            if self.blocked[sj, si]:
                self.blocked &= ~(around & (from_things >= from_things[sj, si]))
            if beside_unknown[sj, si]:
                beside_unknown &= ~(around & (from_unknown >= from_unknown[sj, si]))
        # `solid`: where the base cannot be for what the map knows. `blocked`: where it may
        # not go now, which is that and everything not yet seen to be free.
        self.solid = self.blocked.copy()
        self.blocked |= self.unknown | beside_unknown
        # Each step costs its length, times this: more near things, and much more where the
        # base should not be at all, so that a way out of there leads away from them.
        # And where there is no room to turn it costs more again: narrow places are for
        # going straight through, when the way round is much longer.
        self.price = (1.0 + 4.0 * np.clip((RADIUS + MARGIN - self.clearance) / MARGIN, 0.0, 1.0)
                      + 6.0 * np.clip((TURN - self.clearance) / (TURN - RADIUS), 0.0, 1.0)
                      + 40.0 * np.clip((RADIUS - self.clearance) / RADIUS, 0.0, 1.0))
        # A way through what has not been seen may turn out not to be one.
        self.price[self.blocked & ~self.solid] *= 3.0

    def index(self, point) -> tuple[int, int]:
        if self.k == 2:
            return (int(math.floor(point[0] / self.cell)) - int(self.origin[0]),
                    int(math.floor(point[1] / self.cell)) - int(self.origin[1]))
        return (int(round(point[0] / self.cell)) - int(self.origin[0]),
                int(round(point[1] / self.cell)) - int(self.origin[1]))

    def centre(self, i: int, j: int) -> tuple[float, float]:
        middle = 0.5 if self.k == 2 else 0.0
        return ((i + self.origin[0] + middle) * self.cell, (j + self.origin[1] + middle) * self.cell)

    def inside(self, i: int, j: int) -> bool:
        return 0 <= i < self.shape[1] and 0 <= j < self.shape[0]

    def clear_at(self, x: float, y: float) -> float:
        """How far a place is from the nearest thing in the map, m."""
        i = int(round(x / self.fine)) - int(self.fine_origin[0])
        j = int(round(y / self.fine)) - int(self.fine_origin[1])
        if not (0 <= i < self.fine_clear.shape[1] and 0 <= j < self.fine_clear.shape[0]):
            return math.inf
        return float(self.fine_clear[j, i])

    def touching(self, points: np.ndarray) -> int:
        """How many of the places (N x 2, map frame) have something in the map at them."""
        index = np.rint(points / self.fine).astype(np.int64) - self.fine_origin
        i, j = index[:, 0], index[:, 1]
        inside = (i >= 0) & (i < self.fine_occupied.shape[1]) & (j >= 0) & (j < self.fine_occupied.shape[0])
        return int(self.fine_occupied[j[inside], i[inside]].sum())

    def free_line(self, a, b, clear: float = RADIUS) -> bool:
        """Can the base go straight from a to b, staying `clear` of everything?"""
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (self.fine / 2)))
        for k in range(n + 1):
            x, y = a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
            i, j = self.index((x, y))
            if not self.inside(i, j) or self.blocked[j, i] or self.clear_at(x, y) < clear:
                return False
        return True

    def nearest_free(self, point, within: float = 0.6):
        """The place nearest to `point` where the base can be, if there is one within reach."""
        i, j = self.index(point)
        if self.inside(i, j) and not self.blocked[j, i]:
            return (float(point[0]), float(point[1]))
        if not self.inside(i, j):
            return None
        rows, columns = np.nonzero(~self.blocked)
        if not len(rows):
            return None
        d = np.hypot(rows - j, columns - i)
        best = int(d.argmin())
        return self.centre(int(columns[best]), int(rows[best])) if d[best] * self.cell <= within else None

    def frontier(self, start, centre, radius: float, been=()) -> tuple[float, float] | None:
        """Where to go to see more: the place nearest by the way there, among those the base
        can reach now, that lies at the edge of what has been seen. None if there is none.

        Only edges that are worth a visit count: not the unseen inside or right behind a
        thing (within 0.4 m of something in the map), not specks, nothing farther than
        `radius` from `centre`, and nothing within half a metre of the places in `been`."""
        import cv2  # here so that the rest of the package works without OpenCV

        si, sj = self.index(start)
        if not self.inside(si, sj):
            return None
        width, height = self.shape[1], self.shape[0]
        # everything that can be reached, and at what price
        blocked, price = self.blocked.ravel(), self.price.ravel()
        start_at = sj * width + si
        cost = {start_at: 0.0}
        queue = [(0.0, start_at)]
        steps = [(di, dj, math.hypot(di, dj)) for di in (-1, 0, 1) for dj in (-1, 0, 1) if di or dj]
        while queue:
            so_far, at = heapq.heappop(queue)
            if so_far > cost[at]:
                continue
            j, i = divmod(at, width)
            for di, dj, length in steps:
                ni, nj = i + di, j + dj
                if not (0 <= ni < width and 0 <= nj < height):
                    continue
                to = nj * width + ni
                if blocked[to]:
                    continue
                new = so_far + length * price[to]
                if new < cost.get(to, math.inf):
                    cost[to] = new
                    heapq.heappush(queue, (new, to))
        reached = np.zeros(self.shape, bool)
        reached.ravel()[list(cost)] = True
        open_unseen = self.unknown & (self.from_things > 0.4)
        if not open_unseen.any():
            return None
        to_unseen = cv2.distanceTransform((~open_unseen).astype(np.uint8), cv2.DIST_L2,
                                          cv2.DIST_MASK_PRECISE) * self.cell
        edge = reached & (to_unseen < UNSEEN + 0.2)
        # not a speck: half a dozen such squares within 20 cm
        edge &= cv2.boxFilter(edge.astype(np.float32), -1, (7, 7), normalize=False) >= 6
        rows, columns = np.nonzero(edge)
        best, best_cost = None, math.inf
        for j, i in zip(rows.tolist(), columns.tolist(), strict=True):
            x, y = self.centre(i, j)
            if math.hypot(x - centre[0], y - centre[1]) > radius:
                continue
            if any(math.hypot(x - bx, y - by) < 0.5 for bx, by in been):
                continue
            c = cost[j * width + i]
            if c < best_cost:
                best, best_cost = (x, y), c
        return best

    def plan(self, start, goal) -> list[tuple[float, float]] | None:
        """The way from start to goal as a few corner points (the last is the goal), or
        None if there is none."""
        path, whole = self.plan_towards(start, goal)
        return path if whole else None

    def plan_towards(self, start, goal) -> tuple[list[tuple[float, float]] | None, bool]:
        """(the way from start to goal, True) if all of it has been seen to be free.
        Otherwise (as much of the likeliest way as has been seen to be free, False): the
        way is looked for through what has not been seen as if it were free, only dearer,
        and cut off where the unseen begins. From there the camera will have seen more.
        If the goal cannot be reached even so: the way to the place nearest to it.
        (None, False) if start or goal are outside the map."""
        si, sj = self.index(start)
        gi, gj = self.index(goal)
        if not (self.inside(si, sj) and self.inside(gi, gj)):
            return None, False
        width = self.shape[1]
        blocked, price = self.solid.ravel(), self.price.ravel()
        start_at, goal_at = sj * width + si, gj * width + gi
        steps = [(di, dj, math.hypot(di, dj)) for di in (-1, 0, 1) for dj in (-1, 0, 1) if di or dj]
        cost = {start_at: 0.0}
        came = {}
        queue = [(math.hypot(gi - si, gj - sj), start_at)]
        done = set()
        whole = False
        while queue:
            _, at = heapq.heappop(queue)
            if at == goal_at:
                whole = True
                break
            if at in done:
                continue
            done.add(at)
            j, i = divmod(at, width)
            for di, dj, length in steps:
                ni, nj = i + di, j + dj
                if not (0 <= ni < width and 0 <= nj < self.shape[0]):
                    continue
                to = nj * width + ni
                if blocked[to] or to in done:
                    continue
                new = cost[at] + length * price[to]
                if new < cost.get(to, math.inf):
                    cost[to] = new
                    came[to] = at
                    heapq.heappush(queue, (new + math.hypot(gi - ni, gj - nj), to))
        end = goal_at
        if not whole:
            # everything that can be reached has been visited: the place among it nearest the goal
            end = min(done, key=lambda at: (at % width - gi) ** 2 + (at // width - gj) ** 2)
        cells = [end]
        while cells[-1] != start_at:
            cells.append(came[cells[-1]])
        cells.reverse()
        unseen = self.blocked.ravel()
        for k, at in enumerate(cells[1:], 1):
            if unseen[at]:
                cells, whole = cells[:k], False
                break
        cells.reverse()
        points = [self.centre(at % width, at // width) for at in reversed(cells)]
        points[0] = (float(start[0]), float(start[1]))
        if whole:
            points[-1] = (float(goal[0]), float(goal[1]))
        # Straighten: from each corner, go straight to the farthest point that can be reached
        # so without coming nearer to anything than the way found does on that stretch
        # (where the way starts too close to something, it leads away from it first, and a
        # short cut must not undo that).
        room = [float(self.clearance[at // width, at % width]) for at in reversed(cells)]
        corners, at = [points[0]], 0
        while at < len(points) - 1:
            reach = len(points) - 1
            while reach > at + 1 and not self.free_line(points[at], points[reach],
                                                        min(RADIUS, min(room[at:reach + 1])) - 0.01):
                reach -= 1
            corners.append(points[reach])
            at = reach
        return corners[1:], whole


def _outline(grow: float = 0.005, step: float = 0.025) -> np.ndarray:
    """Places all over the base's outline (FOOTPRINT, grown a little), in its own frame: N x 2."""
    places = []
    for x0, x1, half in FOOTPRINT:
        xs = np.arange(x0 - grow, x1 + grow + 1e-9, step)
        ys = np.arange(-half - grow, half + grow + 1e-9, step)
        places.append(np.stack(np.meshgrid(xs, ys), axis=-1).reshape(-1, 2))
    return np.concatenate(places)


OUTLINE = _outline()


def _approach(now: float, wanted: float, step: float) -> float:
    return now + max(-step, min(step, wanted - now))


class Navigator:
    """Takes the base to a point: goto(x, y), then step() on every tick gives the speeds to
    command. `view` is the WorldView: the map, where the base is in it, and what the depth
    camera sees ahead."""

    REPLAN_S = 2.0

    def __init__(self, view, threaded: bool = True):
        self.view = view
        self.threaded = threaded
        self.clock = time.monotonic  # (a test may run on a clock of its own)
        self.state = "idle"  # idle, planning, driving, arrived, failed
        self.note = ""
        self.goal: tuple[float, float] | None = None
        self.path: list[tuple[float, float]] = []
        self._v = self._w = 0.0
        self._planned = 0.0
        self._planning = False
        self._from: tuple[float, float] = (0.0, 0.0)  # where the stretch to path[0] began
        self._blocked_since: float | None = None
        self._costmap: Costmap | None = None  # the map as the last plan was made on
        self._looked_at = -math.inf
        self._look_way, self._swept = 1.0, (0.0, 0.0)
        self._look_far: float | None = None  # rad to turn when looking around by whole turns
        self.plan_ms = 0.0  # how long the last plan took to make
        self._turn_way = 0.0  # turning on the spot the long way round (+1 left, -1 right), or 0
        self._tight_since: float | None = None
        self._serial = 0  # goto() and cancel() make plans that are under way out of date
        self._looked = False  # has looked around from about here for this goal
        self._whole = True  # the plan goes all the way to the goal
        self._legs = 0  # plans driven for this goal
        self._turned, self._last_yaw = 0.0, None
        self._explore: dict | None = None  # mapping by itself: where from, how far, where it has been

    @property
    def active(self) -> bool:
        return self.state in ("planning", "driving", "looking") or self._explore is not None

    def describe(self) -> dict:
        return {"state": self.state, "note": self.note, "goal": self.goal, "exploring": self._explore is not None,
                "plan_ms": round(self.plan_ms),
                "path": [[round(x, 3), round(y, 3)] for x, y in self.path]}

    def goto(self, x: float, y: float) -> None:
        self._explore = None
        self._goto(x, y)

    def _goto(self, x: float, y: float) -> None:
        self._serial += 1
        self.goal, self.path, self.state, self.note = (float(x), float(y)), [], "planning", ""
        self._blocked_since = None
        self._tight_since = None
        self._turn_way = 0.0
        self._looked = False
        self._legs = 0
        self._plan()

    def look_around(self, turns: float | None = None) -> None:
        """Turn on the spot until the camera has seen all there is to see from here: to the
        left as far as there is room for the tail, and the rest to the right if need be.

        With `turns`, that many whole turns instead, to the left or (negative) to the right,
        or not at all if there is no room: for measuring, where it matters that what is
        compared was seen turning the same way and the same amount."""
        self._explore = None
        self._look("looking around")
        if turns is not None:
            self._look_way, self._look_far = math.copysign(1.0, turns), abs(turns) * 2 * math.pi

    def _around(self) -> None:
        """The map close around the base, for checking that it has room to turn."""
        def make():
            with contextlib.suppress(Exception):
                here = np.asarray(self.view.base_pose()[:2])
                self._costmap = Costmap(self.view.obstacles(), here, here, None, cell=PLAN_CELL, pad=1.0)

        if self.threaded:
            threading.Thread(target=make, name="around", daemon=True).start()
        else:
            make()

    SEES = 1.2  # rad: the camera sees this much without turning (a little less than it does)

    def _look(self, note: str) -> None:
        self._looked_at = -math.inf
        self._look_way, self._swept, self._look_far = 1.0, (0.0, 0.0), None
        self._serial += 1
        self.goal, self.path = None, []
        self._looked, self._turned, self._last_yaw = True, 0.0, None
        self.state, self.note = "looking", note

    def cancel(self, note: str = "stopped") -> None:
        if self.active:
            self._serial += 1
            self._explore = None
            self.state, self.note, self.path = "idle", note, []

    # --- mapping by itself ---------------------------------------------------------------------

    EXPLORE_S = 900.0  # s: after this long it stops by itself

    def explore(self, radius: float = 4.0) -> None:
        """Map what can be reached within `radius` metres of here, without being told where
        to go: look around, drive to the nearest edge of what has been seen, look around
        there, and so on until there is no such edge left to drive to."""
        x, y, _ = self.view.base_pose()
        self._explore = {"from": (x, y), "radius": float(radius), "been": [], "phase": "look",
                         "started": self.clock()}
        self._look("mapping: looking around")

    def _explore_on(self) -> None:
        """One thing done (a look around, a drive): what next?"""
        e = self._explore
        if self.clock() - e["started"] > self.EXPLORE_S:
            self._explore = None
            self.state, self.note = "arrived", f"stopped mapping after {self.EXPLORE_S / 60:.0f} minutes"
            return
        if e["phase"] == "drive":
            e["been"].append(self.goal)  # whether it got there or not: not there again
            if self.state == "arrived":
                e["phase"] = "look"
                self._look("mapping: looking around")
                return
        e["phase"] = "choose"
        self._serial += 1
        self.goal, self.path, self.state, self.note = None, [], "planning", "mapping: choosing where to look next"
        self._planning = True
        serial = self._serial
        if self.threaded:
            threading.Thread(target=self._choose, args=(serial,), name="explore", daemon=True).start()
        else:
            self._choose(serial)

    def _choose(self, serial: int) -> None:
        target = None
        try:
            e = self._explore
            start = self.view.base_pose()[:2]
            costmap = Costmap(self.view.obstacles(), np.asarray(start), np.asarray(start), self.view.known,
                              pad=e["radius"] + math.hypot(start[0] - e["from"][0], start[1] - e["from"][1]) + 1.0)
            target = costmap.frontier(start, e["from"], e["radius"], e["been"])
        except Exception as error:  # (no map to choose from must not leave it hanging)
            if serial == self._serial:
                self._explore = None
                self.state, self.note = "failed", f"mapping failed: {error}"
            return
        finally:
            self._planning = False
        if serial != self._serial or self._explore is None:
            return
        if target is None:
            self._explore = None
            self.state, self.note = "arrived", "mapped all it can reach"
        else:
            self._explore["phase"] = "drive"
            self._goto(*target)

    # --- the plan ------------------------------------------------------------------------

    def _plan(self) -> None:
        if self._planning:
            return
        self._planning = True
        self._planned = self.clock()
        serial = self._serial
        if self.threaded:
            threading.Thread(target=self._plan_now, args=(serial,), name="plan", daemon=True).start()
        else:
            self._plan_now(serial)

    def _plan_now(self, serial: int) -> None:
        began = time.perf_counter()
        try:
            start, goal = self.view.base_pose()[:2], self.goal
            costmap = Costmap(self.view.obstacles(), np.asarray(start), np.asarray(goal), self.view.known,
                              cell=PLAN_CELL)
            self._costmap = costmap
            # A goal in or right next to something in the map: as near to it as the base can
            # be. (Not for a goal that has merely not been seen yet: that is driven towards.)
            gi, gj = costmap.index(goal)
            at_a_thing = costmap.inside(gi, gj) and costmap.solid[gj, gi]
            reachable = (costmap.nearest_free(goal) if at_a_thing else None) or goal
            path, whole = costmap.plan_towards(start, reachable)
            if serial != self._serial:
                return
            far = 0.0
            for a, b in zip([start, *(path or [])], path or [], strict=False):
                far += math.hypot(b[0] - a[0], b[1] - a[1])
            if whole and not path:
                self.state, self.note, self.path = "arrived", "", []  # it is there already
            elif path and (whole or (far > 0.25 and self._legs < 40)):
                # All the way, or as far along the likeliest way as has been seen to be
                # free: from there the camera sees more.
                self._from = (float(start[0]), float(start[1]))
                self.path, self._whole, self.state = path, whole, "driving"
                self._looked = False
            elif not self._looked:
                # Perhaps the way has just not been seen yet: look all around once, then try again.
                self._looked, self._turned, self._last_yaw = True, 0.0, None
                self._look_way, self._swept, self._looked_at, self._look_far = 1.0, (0.0, 0.0), -math.inf, None
                self.state, self.note, self.path = "looking", "looking around", []
            else:
                gi, gj = costmap.index(goal)
                self.state, self.path = "failed", []
                if not costmap.inside(gi, gj) or costmap.unknown[gj, gi]:
                    self.note = "that place has not been seen, and there is no way towards it that has"
                else:
                    self.note = "no way there that has been seen to be free"
        except Exception as e:  # a plan that cannot be made must not leave the base driving
            if serial == self._serial:
                self.state, self.note, self.path = "failed", f"planning failed: {e}", []
        finally:
            self._planning = False
            self.plan_ms = (time.perf_counter() - began) * 1e3

    # --- following it ----------------------------------------------------------------------

    def step(self, dt: float) -> tuple[float, float]:
        """Forward and turning speed to command now."""
        v, w = self._wanted()
        if self._explore is not None and self.state in ("arrived", "failed") and not self._planning:
            self._explore_on()
            v = w = 0.0
        self._v = _approach(self._v, v, V_ACCEL * dt) if v > 0 else 0.0  # stopping is not eased
        self._w = _approach(self._w, w, W_ACCEL * dt)
        return self._v, self._w

    def _target(self, x: float, y: float) -> tuple[float, float]:
        """The point to steer for: LOOKAHEAD along the plan from where the base is beside it.
        (Not simply towards the next corner: that cuts the corner, and the corner is there
        to keep the base clear of something.)"""
        path = self.path  # (a new plan replaces the list, it does not change it)
        while True:
            (ax, ay), (bx, by) = self._from, path[0]
            length = math.hypot(bx - ax, by - ay)
            along = 1.0
            if length > 1e-6:
                along = max(0.0, min(1.0, ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / length ** 2))
            left = (1 - along) * length
            if left < 0.03 and len(path) > 1:
                self._from = path.pop(0)  # that corner is done
                continue
            break
        look = LOOKAHEAD
        if left >= look:
            reach = along + look / length
            return ax + (bx - ax) * reach, ay + (by - ay) * reach
        look -= left
        for (cx, cy), (dx, dy) in zip(path, path[1:], strict=False):
            length = math.hypot(dx - cx, dy - cy)
            if length >= look:
                return cx + (dx - cx) * look / length, cy + (dy - cy) * look / length
            look -= length
        return path[-1]

    W_LOOK = 0.5  # rad/s: slow enough for the map to keep up

    def _wanted(self) -> tuple[float, float]:
        if self.state == "looking":
            # Turn until the camera has looked all round: one way as far as there is room
            # for the tail, then, if that was not all the way, back and round the other way.
            yaw = self.view.base_pose()[2]
            if self._last_yaw is not None:
                self._turned += world.wrap_angle(yaw - self._last_yaw)   # (signed: where it faces now)
                self._swept = (min(self._swept[0], self._turned), max(self._swept[1], self._turned))
            self._last_yaw = yaw
            if self._look_far is not None:
                more = abs(self._turned) < self._look_far
            else:
                more = self._swept[1] - self._swept[0] < 2 * math.pi - self.SEES
            if more:
                now = self.clock()
                if now - self._looked_at > 1.5 and not self._planning:
                    self._looked_at = now   # what is around, as the map has it now
                    self._around()
                x, y, _ = self.view.base_pose()
                w = self._fits(x, y, yaw, 0.0, self._look_way * self.W_LOOK, now)[1]
                if w != 0.0 or self._costmap is None:
                    return 0.0, w if w != 0.0 else self._look_way * self.W_LOOK
                self._tight_since = None
                if self._look_far is not None:
                    self.state, self.note = "failed", "no room to turn here"
                    return 0.0, 0.0
                if self._look_way > 0:
                    self._look_way = -1.0   # no room to turn further this way: the other way
                    return 0.0, 0.0
                # no room that way either: that was the look around
            if self.goal is None:
                self.state, self.note = "arrived", ""
            else:
                self.state, self.note = "planning", ""
                self._plan()
            return 0.0, 0.0
        if self.state != "driving" or not self.path:
            return 0.0, 0.0
        now = self.clock()
        x, y, yaw = self.view.base_pose()
        gx, gy = self.path[-1]
        if math.hypot(gx - x, gy - y) < ARRIVED:
            if self._whole:
                self.state, self.note, self.path = "arrived", "", []
            else:
                self.state, self.path = "planning", []  # as far as was known: what is known now?
                self._legs += 1
                self._plan()
            return 0.0, 0.0
        if now - self._planned > self.REPLAN_S:
            self._plan()
            if self.state != "driving" or not self.path:
                return 0.0, 0.0
        tx, ty = self._target(x, y)
        off = world.wrap_angle(math.atan2(ty - y, tx - x) - yaw)
        w = max(-W_MAX, min(W_MAX, 2.0 * off))
        if abs(off) > 0.45:
            # Turn on the spot first: the short way round, or, if the tail would hit
            # something that way, the long way (and then all the way: no turning back).
            if self._turn_way:
                w = self._turn_way * 0.8 * W_MAX
            if not self._room(x, y, yaw, 0.0, w) and not self._turn_way:
                other = -math.copysign(0.8 * W_MAX, w)
                if self._room(x, y, yaw, 0.0, other):
                    self._turn_way, w = math.copysign(1.0, other), other
            return self._fits(x, y, yaw, 0.0, w, now)
        self._turn_way = 0.0
        v = V_MAX * (1 - abs(off) / 0.45) * min(1.0, 0.3 + math.hypot(gx - x, gy - y) / 0.5)
        # What the camera sees ahead has the last word.
        ahead, seen = self.view.ahead
        gap = ahead - FRONT
        if now - seen > STALE_S:
            self.note = "no pictures from the depth camera"
            v = 0.0
        elif gap < STOP_GAP:
            v = 0.0
            if self._whole and math.hypot(gx - x, gy - y) < 0.2:
                # the goal is at, or in, the very thing: this is as near as it gets
                self.state, self.note, self.path = "arrived", "", []
                return 0.0, 0.0
            self.note = "something in the way"
            if self._blocked_since is None:
                self._blocked_since = now
                self._plan()  # the map has it now: a way around, if there is one
                if self.state != "driving":
                    return 0.0, 0.0
            elif now - self._blocked_since > 8.0:
                self.state, self.path = "failed", []
                return 0.0, 0.0
        else:
            self._blocked_since = None
            self.note = ""
            v *= min(1.0, max(0.25, (gap - STOP_GAP) / (SLOW_GAP - STOP_GAP)))
        return self._fits(x, y, yaw, v, w, now)

    def _room(self, x: float, y: float, yaw: float, v: float, w: float) -> bool:
        """Does the base's whole outline stay clear of what is in the map during the next
        half second of this move? (What is inside the outline already does not count: a
        map is never quite clean.)"""
        costmap = self._costmap
        if costmap is None:
            return True
        c, s = math.cos(yaw), math.sin(yaw)
        most = costmap.touching(OUTLINE @ np.array([[c, s], [-s, c]]) + (x, y))
        for t in (0.1, 0.2, 0.3, 0.4, 0.5):
            a = yaw + w * t
            c, s = math.cos(yaw + w * t / 2), math.sin(yaw + w * t / 2)
            px, py = x + v * t * c, y + v * t * s
            ca, sa = math.cos(a), math.sin(a)
            if costmap.touching(OUTLINE @ np.array([[ca, sa], [-sa, ca]]) + (px, py)) > most:
                return False
        return True

    def _fits(self, x: float, y: float, yaw: float, v: float, w: float, now: float) -> tuple[float, float]:
        """The move, if the base's whole outline stays clear of what is in the map during
        the next half second of it; if not, the part of it that does (straight on without
        the turn, or the turn without driving on); if neither, none.

        A plan treats the base as a disc of half its width. That is right for driving
        straight and wrong for turning, where the tail swings out. So a plan may ask for a
        turn that there is no room for, and this is what stops it."""
        costmap = self._costmap
        if costmap is None or (v == 0.0 and w == 0.0):
            return v, w
        if v > 0:  # slower where it is tight
            v *= max(0.4, min(1.0, (costmap.clear_at(x, y) - RADIUS + 0.04) / 0.14))
        # (turning alone only counts as a way on if it is a turn worth the name)
        moves = ((v, w), (v, 0.0)) + (((0.0, w),) if abs(w) > 0.1 else ()) if v > 0 else ((0.0, w),)
        for move in moves:
            if move != (0.0, 0.0) and self._room(x, y, yaw, *move):
                self._tight_since = None
                return move
        if self.state == "looking":
            return 0.0, 0.0
        self.note = "no room to turn here" if v == 0.0 else "no room to go on"
        if self._tight_since is None:
            self._tight_since = now
            self._plan()  # perhaps there is another way
        elif now - self._tight_since > 8.0:
            self.state, self.path = "failed", []
        return 0.0, 0.0
