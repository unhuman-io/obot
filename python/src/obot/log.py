"""Record the daemon's state stream to an MCAP file.

Messages are protobuf (`obot.RobotState`), one per control cycle, with the schema embedded
in the file, so Foxglove, PlotJuggler and Rerun can open it without anything from this
repository. `read_states()` loads a file back into the same numpy records the client
returns.
"""

from __future__ import annotations

import signal
import time

import numpy as np

from . import layout
from .client import Robot

TOPIC = "/obot/state"

# Scalar and per-joint fields, in protobuf field-number order. Changing an existing entry
# breaks old files; append instead.
_SCALARS = [
    ("cycle", "uint64"), ("t_mono_ns", "int64"), ("t_real_ns", "int64"), ("mode", "uint32"),
    ("fault", "uint32"), ("warning", "uint32"), ("command_seq", "uint64"), ("command_source", "uint32"),
    ("command_age_us", "int32"), ("wake_latency_ns", "int32"), ("exec_ns", "int32"), ("rx_wait_ns", "int32"),
    ("overruns", "uint32"), ("odom_x", "double"), ("odom_y", "double"), ("odom_yaw", "double"),
    ("base_v", "float"), ("base_omega", "float"),
]  # fmt: skip
_ARRAYS = [
    ("joint_mode", "uint32"), ("joint_fault", "uint32"), ("drive_error", "uint32"), ("rx_miss", "uint32"),
    ("rx_age_us", "int32"), ("q", "float"), ("qd", "float"), ("tau", "float"), ("q_cmd", "float"),
    ("qd_cmd", "float"), ("tau_cmd", "float"), ("kp_cmd", "float"), ("kd_cmd", "float"),
    ("temperature", "float"), ("bus_voltage", "float"),
]  # fmt: skip


def _message_class():
    from google.protobuf import descriptor_pb2, descriptor_pool, message_factory

    types = descriptor_pb2.FieldDescriptorProto
    codes = {"uint64": types.TYPE_UINT64, "int64": types.TYPE_INT64, "uint32": types.TYPE_UINT32,
             "int32": types.TYPE_INT32, "float": types.TYPE_FLOAT, "double": types.TYPE_DOUBLE,
             "string": types.TYPE_STRING}
    file = descriptor_pb2.FileDescriptorProto(name="obot/robot_state.proto", package="obot", syntax="proto3")
    message = file.message_type.add(name="RobotState")
    number = 1
    for name, kind in _SCALARS:
        message.field.add(name=name, number=number, type=codes[kind], label=types.LABEL_OPTIONAL)
        number += 1
    for name, kind in _ARRAYS + [("joint_name", "string")]:
        message.field.add(name=name, number=number, type=codes[kind], label=types.LABEL_REPEATED)
        number += 1
    pool = descriptor_pool.DescriptorPool()
    pool.Add(file)
    cls = message_factory.GetMessageClass(pool.FindMessageTypeByName("obot.RobotState"))
    schema = descriptor_pb2.FileDescriptorSet(file=[file]).SerializeToString()
    return cls, schema


def record(robot: Robot, path: str, duration: float | None = None, decimate: int = 1) -> int:
    """Write states to `path` until `duration` elapses, the daemon exits, or SIGINT/SIGTERM."""
    from mcap.writer import CompressionType, Writer

    cls, schema_data = _message_class()
    names = list(robot.info.joint_names)
    n = robot.info.num_joints
    stop = False

    def on_signal(*_):
        nonlocal stop
        stop = True

    previous = [signal.signal(s, on_signal) for s in (signal.SIGINT, signal.SIGTERM)]
    written = dropped = 0
    last_cycle = None
    start = time.monotonic()
    try:
        with open(path, "wb") as f:
            writer = Writer(f, compression=CompressionType.ZSTD)
            writer.start(profile="", library="obot")
            schema_id = writer.register_schema(name="obot.RobotState", encoding="protobuf", data=schema_data)
            channel_id = writer.register_channel(topic=TOPIC, message_encoding="protobuf", schema_id=schema_id)
            while not stop and robot.alive():
                if duration is not None and time.monotonic() - start >= duration:
                    break
                records = robot.read()
                for record_ in records:
                    cycle = int(record_["cycle"])
                    if last_cycle is not None and cycle != last_cycle + 1:
                        dropped += cycle - last_cycle - 1
                    last_cycle = cycle
                    if cycle % decimate:
                        continue
                    message = cls()
                    for name, _ in _SCALARS:
                        setattr(message, name, record_[name].item())
                    for name, _ in _ARRAYS:
                        getattr(message, name).extend(record_[name][:n].tolist())
                    message.joint_name.extend(names)
                    stamp = int(record_["t_real_ns"])
                    writer.add_message(channel_id=channel_id, log_time=stamp, publish_time=stamp,
                                       data=message.SerializeToString(), sequence=cycle & 0xFFFFFFFF)
                    written += 1
                time.sleep(0.02)
            writer.finish()
    finally:
        for sig, handler in zip((signal.SIGINT, signal.SIGTERM), previous, strict=True):
            signal.signal(sig, handler)
    print(f"obot record: {written} records to {path}" + (f", {dropped} missed" if dropped else ""))
    return 0 if written else 1


def read_states(path: str) -> tuple[np.ndarray, list[str]]:
    """Load a recording: (structured array of layout.STATE records, joint names)."""
    from mcap.reader import make_reader

    cls, _ = _message_class()
    rows = []
    names: list[str] = []
    with open(path, "rb") as f:
        for _, _, message in make_reader(f).iter_messages(topics=[TOPIC]):
            decoded = cls()
            decoded.ParseFromString(message.data)
            row = np.zeros((), layout.STATE)
            for name, _ in _SCALARS:
                row[name] = getattr(decoded, name)
            for name, _ in _ARRAYS:
                values = getattr(decoded, name)
                row[name][: len(values)] = values
            row["num_joints"] = len(decoded.joint_name)
            names = list(decoded.joint_name)
            rows.append(row)
    return (np.array(rows, dtype=layout.STATE) if rows else np.empty(0, layout.STATE)), names
