"""Client for the shared memory region of obot-rtd.

    robot = Robot()                     # attaches to /dev/shm/obot
    state = robot.state()               # newest RobotState record, or None
    cmd = robot.new_command()
    cmd["joint_mode"][:] = JointMode.POSITION
    cmd["q"][0] = 0.3
    robot.send(cmd)                     # repeat faster than the daemon's command timeout

Nothing here blocks the daemon: states are copied out of a ring the daemon overwrites,
and commands go into a mailbox it polls. A command is followed only while it is fresh, so
a client that stops sending makes the robot hold (arm) and stop (base).
"""

from __future__ import annotations

import fcntl
import mmap
import os
import time
from dataclasses import dataclass

import numpy as np

from . import layout
from .layout import JointMode, Request, RobotMode

__all__ = ["Robot", "RobotInfo", "JointMode", "Request", "RobotMode", "NotRunning"]


class NotRunning(RuntimeError):
    """The daemon's shared memory region is missing or was made by another version."""


@dataclass(frozen=True)
class RobotInfo:
    name: str
    num_joints: int
    period_s: float
    joint_names: tuple[str, ...]
    q_min: np.ndarray
    q_max: np.ndarray
    qd_max: np.ndarray
    tau_max: np.ndarray
    continuous: np.ndarray
    left_wheel: int
    right_wheel: int
    wheel_radius: float
    track_width: float

    @property
    def has_base(self) -> bool:
        return self.left_wheel >= 0 and self.right_wheel >= 0

    @property
    def arm_joints(self) -> tuple[int, ...]:
        """Indices of the joints that are not wheels."""
        return tuple(i for i in range(self.num_joints) if i not in (self.left_wheel, self.right_wheel))

    def index(self, joint_name: str) -> int:
        return self.joint_names.index(joint_name)


class Robot:
    def __init__(self, name: str = "obot", source: int = 0):
        """Attach to the daemon. `source` is a free-form id recorded with every command."""
        self.name = name
        self.source = source or (os.getpid() & 0xFFFFFFFF)
        path = f"/dev/shm/{name}"
        try:
            self._fd = os.open(path, os.O_RDWR | os.O_CLOEXEC)
        except OSError as e:
            raise NotRunning(f"{path}: {e.strerror} (is obot-rtd running?)") from e
        try:
            size = os.fstat(self._fd).st_size
            if size < layout.TOTAL_SIZE:
                raise NotRunning(f"{path}: size {size}, expected {layout.TOTAL_SIZE}: different layout version")
            self._mm = mmap.mmap(self._fd, layout.TOTAL_SIZE)
        except Exception:
            os.close(self._fd)
            raise
        self._header = np.frombuffer(self._mm, layout.HEADER, 1, 0)
        h = self._header[0]
        if h["magic"] != layout.MAGIC:
            self.close()
            raise NotRunning(f"{path}: not published yet")
        if (
            h["layout_version"] != layout.LAYOUT_VERSION
            or h["state_size"] != layout.STATE.itemsize
            or h["command_size"] != layout.COMMAND.itemsize
            or h["info_size"] != layout.INFO.itemsize
            or h["ring_capacity"] != layout.RING_CAPACITY
        ):
            version = int(h["layout_version"])
            self.close()
            raise NotRunning(f"{path}: layout version {version}, this client speaks {layout.LAYOUT_VERSION}")
        self._command = np.frombuffer(self._mm, layout.COMMAND_SLOT, 1, layout.COMMAND_OFFSET)
        self._head = np.frombuffer(self._mm, "<u8", 1, layout.RING_OFFSET)
        self._ring = np.frombuffer(self._mm, layout.STATE_SLOT, layout.RING_CAPACITY, layout.RING_SLOTS_OFFSET)
        self._cursor = int(self._head[0])
        self.daemon_pid = int(h["daemon_pid"])

        info = np.frombuffer(self._mm, layout.INFO, 1, layout.INFO_OFFSET)[0]
        n = int(info["num_joints"])
        self.info = RobotInfo(
            name=info["robot_name"].decode(),
            num_joints=n,
            period_s=float(info["period_ns"]) * 1e-9,
            joint_names=tuple(s.decode() for s in info["joint_name"][:n]),
            q_min=info["q_min"][:n].copy(),
            q_max=info["q_max"][:n].copy(),
            qd_max=info["qd_max"][:n].copy(),
            tau_max=info["tau_max"][:n].copy(),
            continuous=info["continuous"][:n].astype(bool),
            left_wheel=int(info["left_wheel"]),
            right_wheel=int(info["right_wheel"]),
            wheel_radius=float(info["wheel_radius"]),
            track_width=float(info["track_width"]),
        )

    # --- lifetime -----------------------------------------------------------------------

    def close(self) -> None:
        # Drop the views before the mapping they point into.
        for attr in ("_header", "_command", "_head", "_ring"):
            self.__dict__.pop(attr, None)
        mm = self.__dict__.pop("_mm", None)
        if mm is not None:
            mm.close()
        fd = self.__dict__.pop("_fd", None)
        if fd is not None:
            os.close(fd)

    def __enter__(self) -> Robot:
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def alive(self) -> bool:
        """False once the daemon that made this region has exited (or been replaced)."""
        try:
            os.kill(self.daemon_pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            pass
        try:
            return os.stat(f"/dev/shm/{self.name}").st_ino == os.fstat(self._fd).st_ino
        except OSError:
            return False

    # --- state --------------------------------------------------------------------------

    def _read_index(self, i: int):
        slot = self._ring[i & (layout.RING_CAPACITY - 1)]
        want = 2 * i + 2
        if int(slot["seq"]) != want:
            return None
        record = slot["state"].copy()
        return record if int(slot["seq"]) == want else None

    def state(self):
        """The newest state record (a numpy structured scalar), or None if there is none yet."""
        for _ in range(4):
            head = int(self._head[0])
            if head == 0:
                return None
            record = self._read_index(head - 1)
            if record is not None:
                return record
        return None

    def read(self, max_records: int = layout.RING_CAPACITY // 2) -> np.ndarray:
        """Every state record since the previous call, oldest first, as a structured array.

        Records that were overwritten before they could be read are skipped (the `cycle`
        field shows the gap). The first call returns only what arrives after attaching.
        """
        head = int(self._head[0])
        oldest = max(0, head - layout.RING_CAPACITY + 64)  # stay clear of the slots being overwritten
        start = max(self._cursor, oldest, head - max_records)
        self._cursor = head
        if start >= head:
            return np.empty(0, layout.STATE)
        indices = np.arange(start, head, dtype=np.uint64)
        slots = (indices & np.uint64(layout.RING_CAPACITY - 1)).astype(np.intp)
        want = indices * np.uint64(2) + np.uint64(2)
        before = self._ring["seq"][slots]
        records = self._ring["state"][slots]  # fancy indexing copies
        after = self._ring["seq"][slots]
        return records[(before == want) & (after == want)]

    def wait_for(self, predicate, timeout: float = 5.0, poll: float = 0.005):
        """Poll until predicate(state) is true; returns that state, or None on timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            s = self.state()
            if s is not None and predicate(s):
                return s
            time.sleep(poll)
        return None

    def wait_mode(self, mode: RobotMode, timeout: float = 5.0):
        return self.wait_for(lambda s: s["mode"] == mode, timeout)

    # --- commands -----------------------------------------------------------------------

    def new_command(self):
        """A zeroed command record: every joint idle, no base motion."""
        return np.zeros((), layout.COMMAND)

    def send(self, command=None, request: Request = Request.NONE) -> None:
        """Write a command (default: an all-idle one), optionally carrying a one-shot request."""
        new = np.zeros((), layout.COMMAND) if command is None else np.array(command, dtype=layout.COMMAND)
        fcntl.flock(self._fd, fcntl.LOCK_EX)
        try:
            slot = self._command[0]
            seq = int(slot["seq"])
            if seq & 1:  # a writer died in the middle of a write
                seq += 1
            previous = slot["command"]
            new["seq"] = int(previous["seq"]) + 1
            new["source"] = self.source
            new["request"] = previous["request"]
            new["request_id"] = previous["request_id"]
            new["estop_id"] = previous["estop_id"]
            if request == Request.ESTOP:
                new["estop_id"] = (int(previous["estop_id"]) + 1) & 0xFFFFFFFF
            elif request != Request.NONE:
                new["request"] = int(request)
                new["request_id"] = (int(previous["request_id"]) + 1) & 0xFFFFFFFF
            new["stamp_ns"] = time.monotonic_ns()
            self._command["seq"][0] = seq + 1
            self._command["command"][0] = new
            self._command["seq"][0] = seq + 2
        finally:
            fcntl.flock(self._fd, fcntl.LOCK_UN)

    def request(self, request: Request) -> None:
        """Send a request with an otherwise idle command."""
        self.send(None, request)

    def enable(self, timeout: float = 2.0) -> bool:
        self.request(Request.ENABLE)
        return self.wait_mode(RobotMode.ACTIVE, timeout) is not None

    def disable(self, timeout: float = 2.0) -> bool:
        self.request(Request.DISABLE)
        return self.wait_for(lambda s: s["mode"] != RobotMode.ACTIVE, timeout) is not None

    def clear_faults(self, timeout: float = 2.0) -> bool:
        self.request(Request.CLEAR_FAULTS)
        time.sleep(0.05)
        return self.wait_mode(RobotMode.IDLE, timeout) is not None

    def estop(self) -> None:
        self.request(Request.ESTOP)


def bit_names(value: int, names: dict[int, str]) -> list[str]:
    return [names.get(bit, f"bit{bit}") for bit in range(32) if value >> bit & 1]


def describe(state, info: RobotInfo) -> dict:
    """A JSON-friendly summary of a state record."""
    n = info.num_joints

    def clean(a):
        return [None if not np.isfinite(x) else round(float(x), 5) for x in a[:n]]

    return {
        "cycle": int(state["cycle"]),
        "t": int(state["t_real_ns"]) * 1e-9,
        "mode": RobotMode(int(state["mode"])).name.lower(),
        "faults": bit_names(int(state["fault"]), layout.ROBOT_FAULT_BITS),
        "warnings": bit_names(int(state["warning"]), layout.ROBOT_WARNING_BITS),
        "command_age_ms": None if state["command_age_us"] < 0 else int(state["command_age_us"]) / 1000,
        "wake_latency_us": int(state["wake_latency_ns"]) / 1000,
        "exec_us": int(state["exec_ns"]) / 1000,
        "overruns": int(state["overruns"]),
        "odom": [float(state["odom_x"]), float(state["odom_y"]), float(state["odom_yaw"])],
        "base_v": float(state["base_v"]),
        "base_omega": float(state["base_omega"]),
        "joints": [
            {
                "name": info.joint_names[i],
                "mode": JointMode(int(state["joint_mode"][i])).name.lower(),
                "faults": bit_names(int(state["joint_fault"][i]), layout.JOINT_FAULT_BITS),
                "drive_error": int(state["drive_error"][i]),
            }
            for i in range(n)
        ],
        "q": clean(state["q"]),
        "qd": clean(state["qd"]),
        "tau": clean(state["tau"]),
        "q_cmd": clean(state["q_cmd"]),
        "temperature": clean(state["temperature"]),
        "bus_voltage": clean(state["bus_voltage"]),
    }
