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

import heapq
import math
import threading
import time

import numpy as np

from .. import world

# The base is 0.45 m wide. A plan is followed to within a few centimetres, and what the map
# knows of a thing grows while the base drives around it, so the plan keeps well clear:
RADIUS = 0.38  # m: the middle of the wheel axis stays at least this far from everything in the map
MARGIN = 0.30  # m: and where there is room, this much farther away still
UNSEEN = 0.38  # m: floor that has not been seen may have anything on it: the same distance
CELL = 0.06  # m: grid the plan is made on
V_MAX, W_MAX = 0.25, 0.7  # m/s, rad/s
V_ACCEL, W_ACCEL = 0.4, 2.0  # m/s^2, rad/s^2
LOOKAHEAD = 0.25  # m: the point steered for is this far along the plan
ARRIVED = 0.06  # m
FRONT = 0.16  # m: the front edge of the base, from the middle of the wheel axis
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

        self.cell = cell
        low = np.minimum(start, goal) - pad
        high = np.maximum(start, goal) + pad
        self.origin = np.floor(low / cell).astype(np.int64)
        self.shape = tuple((np.ceil(high / cell).astype(np.int64) - self.origin + 1)[::-1])  # rows (y), columns (x)
        occupied = np.zeros(self.shape, bool)
        if len(obstacles):
            index = np.floor(np.asarray(obstacles, np.float64) / cell).astype(np.int64) - self.origin
            i, j = index[:, 0], index[:, 1]
            inside = (i >= 0) & (i < self.shape[1]) & (j >= 0) & (j < self.shape[0])
            occupied[j[inside], i[inside]] = True
        self.occupied = occupied
        # distance from each cell to the nearest thing, m
        self.clearance = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * cell
        from_things = self.clearance
        self.blocked = from_things < RADIUS
        self.unknown = np.zeros(self.shape, bool)
        beside_unknown = np.zeros(self.shape, bool)
        if known is not None:
            # (closing the small gaps that the rows of a picture leave between them far away)
            seen = cv2.morphologyEx(known(self.origin, self.shape).astype(np.uint8), cv2.MORPH_CLOSE,
                                    np.ones((3, 3), np.uint8))
            self.unknown = ~seen.astype(bool) & ~occupied
            # What has not been seen may be anything, the back of the box the camera saw the
            # front of for one: the base keeps clear of it too.
            from_unknown = cv2.distanceTransform((~self.unknown).astype(np.uint8), cv2.DIST_L2,
                                                 cv2.DIST_MASK_PRECISE) * cell
            beside_unknown = from_unknown < UNSEEN
            # (for the price of a step, the unseen counts like a thing)
            self.clearance = np.minimum(from_things, np.where(self.unknown, from_things, from_unknown))
        # Standing closer to something than the plan would allow (it happens: the base was
        # driven there by hand, and around where it starts little has been seen): the way
        # out is allowed, as long as it gets no closer to anything.
        si, sj = self.index(start)
        if self.inside(si, sj):
            rows, columns = np.ogrid[: self.shape[0], : self.shape[1]]
            around = (rows - sj) ** 2 + (columns - si) ** 2 < (0.6 / cell) ** 2
            if self.blocked[sj, si]:
                self.blocked &= ~(around & (from_things >= min(RADIUS, from_things[sj, si]) - cell))
            beside_unknown &= ~around
        # `solid`: where the base cannot be for what the map knows. `blocked`: where it may
        # not go now, which is that and everything not yet seen to be free.
        self.solid = self.blocked.copy()
        self.blocked |= self.unknown | beside_unknown
        # Each step costs its length, times this: more near things, and much more where the
        # base should not be at all, so that a way out of there leads away from them.
        self.price = (1.0 + 4.0 * np.clip((RADIUS + MARGIN - self.clearance) / MARGIN, 0.0, 1.0)
                      + 40.0 * np.clip((RADIUS - self.clearance) / RADIUS, 0.0, 1.0))
        # A way through what has not been seen may turn out not to be one.
        self.price[self.blocked & ~self.solid] *= 3.0

    def index(self, point) -> tuple[int, int]:
        return (int(math.floor(point[0] / self.cell)) - int(self.origin[0]),
                int(math.floor(point[1] / self.cell)) - int(self.origin[1]))

    def centre(self, i: int, j: int) -> tuple[float, float]:
        return ((i + self.origin[0] + 0.5) * self.cell, (j + self.origin[1] + 0.5) * self.cell)

    def inside(self, i: int, j: int) -> bool:
        return 0 <= i < self.shape[1] and 0 <= j < self.shape[0]

    def free_line(self, a, b, clear: float = RADIUS) -> bool:
        """Can the base go straight from a to b, staying `clear` of everything?"""
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (self.cell / 2)))
        for k in range(n + 1):
            i, j = self.index((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))
            if not self.inside(i, j) or self.blocked[j, i] or self.clearance[j, i] < clear:
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
                                                        min(RADIUS, min(room[at:reach + 1])) - self.cell / 2):
                reach -= 1
            corners.append(points[reach])
            at = reach
        return corners[1:], whole


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
        self._serial = 0  # goto() and cancel() make plans that are under way out of date
        self._looked = False  # has looked around from about here for this goal
        self._whole = True  # the plan goes all the way to the goal
        self._legs = 0  # plans driven for this goal
        self._turned, self._last_yaw = 0.0, None

    @property
    def active(self) -> bool:
        return self.state in ("planning", "driving", "looking")

    def describe(self) -> dict:
        return {"state": self.state, "note": self.note, "goal": self.goal,
                "path": [[round(x, 3), round(y, 3)] for x, y in self.path]}

    def goto(self, x: float, y: float) -> None:
        self._serial += 1
        self.goal, self.path, self.state, self.note = (float(x), float(y)), [], "planning", ""
        self._blocked_since = None
        self._looked = False
        self._legs = 0
        self._plan()

    def look_around(self) -> None:
        """Turn once on the spot, so that the camera has seen all there is to see from here."""
        self._serial += 1
        self.goal, self.path = None, []
        self._looked, self._turned, self._last_yaw = True, 0.0, None
        self.state, self.note = "looking", "looking around"

    def cancel(self, note: str = "stopped") -> None:
        if self.active:
            self._serial += 1
            self.state, self.note, self.path = "idle", note, []

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
        try:
            start, goal = self.view.base_pose()[:2], self.goal
            costmap = Costmap(self.view.map.obstacles(), np.asarray(start), np.asarray(goal), self.view.known)
            # (a goal in or right next to something: as near to it as the base can be)
            reachable = costmap.nearest_free(goal) or goal
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

    # --- following it ----------------------------------------------------------------------

    def step(self, dt: float) -> tuple[float, float]:
        """Forward and turning speed to command now."""
        v, w = self._wanted()
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
            yaw = self.view.base_pose()[2]
            if self._last_yaw is not None:
                self._turned += abs(world.wrap_angle(yaw - self._last_yaw))
            self._last_yaw = yaw
            if self._turned < 2 * math.pi:
                return 0.0, self.W_LOOK
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
            return 0.0, w  # turn on the spot first
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
        return v, w
