"""The shared memory layout of obot-rtd as numpy dtypes.

This mirrors rt/include/obot/ipc/layout.h and channels.h by hand. `obot-rtd --dump-layout`
prints the C++ side's offsets, and tests/test_layout.py fails if the two disagree; the
daemon also writes its layout version and struct sizes into the region's header, which
`obot.client.Robot` checks when it attaches.
"""

from __future__ import annotations

import enum

import numpy as np

MAGIC = 0x544F424F
LAYOUT_VERSION = 1
MAX_JOINTS = 16
NAME_LEN = 24
RING_CAPACITY = 4096


class RobotMode(enum.IntEnum):
    INIT = 0
    IDLE = 1
    ACTIVE = 2
    FAULT = 3
    ESTOP = 4


class JointMode(enum.IntEnum):
    IDLE = 0
    DAMPED = 1
    POSITION = 2
    IMPEDANCE = 3
    VELOCITY = 4
    TORQUE = 5


class Request(enum.IntEnum):
    NONE = 0
    ENABLE = 1
    DISABLE = 2
    CLEAR_FAULTS = 3
    ESTOP = 4


class BaseMode(enum.IntEnum):
    JOINTS = 0
    TWIST = 1


ROBOT_FAULT_BITS = {0: "joint", 1: "bus", 2: "estop"}
ROBOT_WARNING_BITS = {0: "command_stale", 1: "command_invalid", 2: "overrun", 3: "tx_dropped"}
JOINT_FAULT_BITS = {0: "comm_timeout", 1: "drive", 2: "position_limit", 3: "velocity_limit", 4: "temperature"}

_J = (MAX_JOINTS,)

HEADER = np.dtype(
    [
        ("magic", "<u4"),
        ("layout_version", "<u4"),
        ("header_size", "<u4"),
        ("info_size", "<u4"),
        ("state_size", "<u4"),
        ("command_size", "<u4"),
        ("ring_capacity", "<u4"),
        ("ring_slot_size", "<u4"),
        ("info_offset", "<u8"),
        ("command_offset", "<u8"),
        ("ring_offset", "<u8"),
        ("total_size", "<u8"),
        ("daemon_pid", "<i8"),
        ("start_time_real_ns", "<i8"),
    ]
)

INFO = np.dtype(
    [
        ("num_joints", "<u4"),
        ("period_ns", "<u4"),
        ("robot_name", f"S{NAME_LEN}"),
        ("joint_name", f"S{NAME_LEN}", _J),
        ("q_min", "<f4", _J),
        ("q_max", "<f4", _J),
        ("qd_max", "<f4", _J),
        ("tau_max", "<f4", _J),
        ("continuous", "u1", _J),
        ("left_wheel", "<i4"),
        ("right_wheel", "<i4"),
        ("wheel_radius", "<f4"),
        ("track_width", "<f4"),
    ]
)

STATE = np.dtype(
    [
        ("cycle", "<u8"),
        ("t_mono_ns", "<i8"),
        ("t_real_ns", "<i8"),
        ("mode", "<u4"),
        ("fault", "<u4"),
        ("warning", "<u4"),
        ("num_joints", "<u4"),
        ("command_seq", "<u8"),
        ("command_source", "<u4"),
        ("command_age_us", "<i4"),
        ("wake_latency_ns", "<i4"),
        ("exec_ns", "<i4"),
        ("rx_wait_ns", "<i4"),
        ("overruns", "<u4"),
        ("odom_x", "<f8"),
        ("odom_y", "<f8"),
        ("odom_yaw", "<f8"),
        ("base_v", "<f4"),
        ("base_omega", "<f4"),
        ("joint_mode", "u1", _J),
        ("joint_fault", "<u4", _J),
        ("drive_error", "<u4", _J),
        ("rx_miss", "<u4", _J),
        ("rx_age_us", "<i4", _J),
        ("q", "<f4", _J),
        ("qd", "<f4", _J),
        ("tau", "<f4", _J),
        ("q_cmd", "<f4", _J),
        ("qd_cmd", "<f4", _J),
        ("tau_cmd", "<f4", _J),
        ("kp_cmd", "<f4", _J),
        ("kd_cmd", "<f4", _J),
        ("temperature", "<f4", _J),
        ("bus_voltage", "<f4", _J),
    ]
)

COMMAND = np.dtype(
    [
        ("stamp_ns", "<i8"),
        ("seq", "<u8"),
        ("source", "<u4"),
        ("request", "<u4"),
        ("request_id", "<u4"),
        ("estop_id", "<u4"),
        ("base_mode", "<u4"),
        ("reserved", "<u4"),
        ("base_v", "<f4"),
        ("base_omega", "<f4"),
        ("joint_mode", "u1", _J),
        ("q", "<f4", _J),
        ("qd", "<f4", _J),
        ("tau", "<f4", _J),
        ("kp", "<f4", _J),
        ("kd", "<f4", _J),
    ]
)

# The channels: a sequence number in front of each record (see channels.h).
COMMAND_SLOT = np.dtype([("seq", "<u8"), ("command", COMMAND)])
STATE_SLOT = np.dtype([("seq", "<u8"), ("state", STATE)])


def _align64(n: int) -> int:
    return (n + 63) // 64 * 64


INFO_OFFSET = HEADER.itemsize
COMMAND_OFFSET = _align64(INFO_OFFSET + INFO.itemsize)
RING_OFFSET = _align64(COMMAND_OFFSET + COMMAND_SLOT.itemsize)
RING_SLOTS_OFFSET = RING_OFFSET + 8  # after the ring's head counter
TOTAL_SIZE = RING_SLOTS_OFFSET + RING_CAPACITY * STATE_SLOT.itemsize

_TYPE_CODES = {
    "u8": np.dtype("u1"),
    "u32": np.dtype("<u4"),
    "i32": np.dtype("<i4"),
    "u64": np.dtype("<u8"),
    "i64": np.dtype("<i8"),
    "f32": np.dtype("<f4"),
    "f64": np.dtype("<f8"),
}


def differences(cpp_layout: dict) -> list[str]:
    """Compare against the parsed output of `obot-rtd --dump-layout`. Empty means identical."""
    problems: list[str] = []
    constants = {
        "layout_version": LAYOUT_VERSION,
        "magic": MAGIC,
        "max_joints": MAX_JOINTS,
        "name_len": NAME_LEN,
        "ring_capacity": RING_CAPACITY,
        "ring_slot_size": STATE_SLOT.itemsize,
        "info_offset": INFO_OFFSET,
        "command_offset": COMMAND_OFFSET,
        "ring_offset": RING_OFFSET,
        "total_size": TOTAL_SIZE,
    }
    for key, mine in constants.items():
        if cpp_layout.get(key) != mine:
            problems.append(f"{key}: C++ {cpp_layout.get(key)}, Python {mine}")
    structs = {"ShmHeader": HEADER, "RobotInfo": INFO, "RobotState": STATE, "RobotCommand": COMMAND}
    for name, dtype in structs.items():
        cpp = cpp_layout.get(name)
        if cpp is None:
            problems.append(f"{name}: missing from the C++ layout")
            continue
        if cpp["size"] != dtype.itemsize:
            problems.append(f"{name}: size C++ {cpp['size']}, Python {dtype.itemsize}")
        cpp_fields = {f[0]: f for f in cpp["fields"]}
        for field in set(cpp_fields) ^ set(dtype.names):
            problems.append(f"{name}.{field}: present on one side only")
        for field in dtype.names:
            if field not in cpp_fields:
                continue
            _, offset, code, shape = cpp_fields[field]
            sub, mine_offset = dtype.fields[field][:2]
            if code == "char":  # char[N] or char[M][N]: the last dimension is the string length
                want = np.dtype((f"S{shape[-1]}", tuple(shape[:-1]))) if len(shape) > 1 else np.dtype(f"S{shape[0]}")
            else:
                want = np.dtype((_TYPE_CODES[code], tuple(shape))) if shape else _TYPE_CODES[code]
            if offset != mine_offset:
                problems.append(f"{name}.{field}: offset C++ {offset}, Python {mine_offset}")
            if want != sub:
                problems.append(f"{name}.{field}: type C++ {want}, Python {sub}")
    return problems
