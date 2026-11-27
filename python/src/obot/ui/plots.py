"""Signals of the robot's state that the operator page can plot, and their samples.

The page asks for some of them at a rate; the server takes every state record the daemon
has published since it last looked (the daemon keeps a few seconds of them, one per control
cycle) and sends the chosen signals of every n-th record. So a plot shows the control loop's
own data, not the page's slower status updates.
"""

from __future__ import annotations

import numpy as np

# field, label, unit, factor from the stored value to that unit
JOINT_SIGNALS = (
    ("q", "position", "rad", 1.0),
    ("q_cmd", "position command", "rad", 1.0),
    ("qd", "velocity", "rad/s", 1.0),
    ("qd_cmd", "velocity command", "rad/s", 1.0),
    ("tau", "torque", "N m", 1.0),
    ("tau_cmd", "torque command", "N m", 1.0),
    ("temperature", "temperature", "°C", 1.0),
    ("bus_voltage", "bus voltage", "V", 1.0),
    ("rx_age_us", "status age", "ms", 1e-3),
)
ROBOT_SIGNALS = (
    ("base_v", "base speed", "m/s", 1.0),
    ("base_omega", "base turn rate", "rad/s", 1.0),
    ("odom_x", "odometry x", "m", 1.0),
    ("odom_y", "odometry y", "m", 1.0),
    ("odom_yaw", "odometry yaw", "rad", 1.0),
    ("wake_latency_ns", "wake-up latency", "µs", 1e-3),
    ("exec_ns", "cycle time", "µs", 1e-3),
    ("rx_wait_ns", "wait for statuses", "µs", 1e-3),
    ("command_age_us", "command age", "ms", 1e-3),
)
MAX_SIGNALS = 8  # as many as there are colours that can be told apart
RATES = (50, 100, 250, 500, 1000)


def signals(joint_names) -> list[dict]:
    """Everything that can be plotted for a robot with these joints."""
    out = []
    for index, joint in enumerate(joint_names):
        for field, label, unit, _ in JOINT_SIGNALS:
            out.append({"id": f"{field}/{index}", "label": f"{joint} {label}", "unit": unit, "group": joint})
    for field, label, unit, _ in ROBOT_SIGNALS:
        out.append({"id": field, "label": label, "unit": unit, "group": "robot"})
    return out


def _column(records: np.ndarray, signal: str, joints: int):
    field, _, index = signal.partition("/")
    table = JOINT_SIGNALS if index else ROBOT_SIGNALS
    factor = next((f for name, _, _, f in table if name == field), None)
    if factor is None or (index and not (index.isdigit() and int(index) < joints)):
        return None
    values = records[field][:, int(index)] if index else records[field]
    return values.astype(np.float64) * factor


def valid(signal: str, joints: int) -> bool:
    field, _, index = signal.partition("/")
    table = JOINT_SIGNALS if index else ROBOT_SIGNALS
    known = any(name == field for name, _, _, _ in table)
    return known and (not index or (index.isdigit() and int(index) < joints))


def batch(records: np.ndarray, wanted: list[str], stride: int, joints: int, t0_ns: int) -> dict | None:
    """The samples of `wanted` in every stride-th record, as the message for the page:
    {"t": [seconds], "y": {signal: [values]}}; a value that is not a number is null."""
    if not len(records) or not wanted:
        return None
    picked = records[records["cycle"] % stride == 0] if stride > 1 else records
    if not len(picked):
        return None
    out = {"type": "plot", "t": np.round((picked["t_mono_ns"] - t0_ns) * 1e-9, 4).tolist(), "y": {}}
    for signal in wanted:
        column = _column(picked, signal, joints)
        if column is None:
            continue
        values = np.round(column, 5)
        listed = values.tolist()
        if not np.isfinite(values).all():
            listed = [v if np.isfinite(v) else None for v in listed]
        out["y"][signal] = listed
    return out
