"""Live plots of the robot in the Rerun viewer.

    obot viz                      open a viewer on this machine and stream to it
    obot viz --save run.rrd       write a recording to open later with `rerun run.rrd`
    obot viz --connect URL        stream to a viewer running elsewhere (`rerun --serve` there)

Every state record is logged (the full control rate) under:
    joint/<name>/{q, q_cmd, qd, tau, temperature}    loop/{wake_latency_us, exec_us}
    base/{v, omega}    robot/mode    world/base (the odometry pose)
"""

from __future__ import annotations

import time

import numpy as np

from .client import Robot

_JOINT_FIELDS = ("q", "q_cmd", "qd", "tau", "temperature")


def run(robot: Robot, save: str | None = None, connect: str | None = None, urdf: str | None = None,
        duration: float | None = None) -> int:
    import rerun as rr

    rr.init(f"obot/{robot.info.name}", spawn=not save and not connect)
    if save:
        rr.save(save)
    elif connect:
        rr.connect_grpc(connect)
    if urdf:
        rr.log_file_from_path(urdf, static=True)

    names = robot.info.joint_names
    start = time.monotonic()
    logged = 0
    try:
        while robot.alive():
            if duration is not None and time.monotonic() - start >= duration:
                break
            records = robot.read()
            if len(records) == 0:
                time.sleep(0.02)
                continue
            times = [rr.TimeColumn("time", timestamp=records["t_real_ns"].astype(np.float64) * 1e-9)]

            def column(path, values, times=times):
                rr.send_columns(path, indexes=times, columns=rr.Scalars.columns(scalars=values.astype(np.float64)))

            for i, name in enumerate(names):
                for field in _JOINT_FIELDS:
                    values = records[field][:, i]
                    if field == "temperature" and not np.isfinite(values).any():
                        continue
                    column(f"joint/{name}/{field}", values)
            column("loop/wake_latency_us", records["wake_latency_ns"] / 1000.0)
            column("loop/exec_us", records["exec_ns"] / 1000.0)
            column("robot/mode", records["mode"])
            if robot.info.has_base:
                column("base/v", records["base_v"])
                column("base/omega", records["base_omega"])
                last = records[-1]
                rr.set_time("time", timestamp=float(last["t_real_ns"]) * 1e-9)
                rr.log("world/base", rr.Transform3D(
                    translation=[float(last["odom_x"]), float(last["odom_y"]), 0.0],
                    rotation=rr.RotationAxisAngle(axis=[0, 0, 1], radians=float(last["odom_yaw"])),
                ))
            logged += len(records)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    print(f"obot viz: {logged} records")
    return 0 if logged else 1
