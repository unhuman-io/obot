"""The `obot` command.

    obot status                 one-line summary and the joints
    obot watch                  the same, continuously
    obot enable | disable | clear | stop
    obot ui                     operator web page: cameras, state, gamepad driving
    obot teleop                 drive with a controller plugged into the robot, no web page
    obot record out.mcap        log every state record to an MCAP file
    obot viz                    live plots in the Rerun viewer
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from . import config as config_module
from .client import NotRunning, Request, Robot, RobotMode, describe


def _robot(args) -> Robot:
    name = args.shm_name or config_module.load(args.config).shm_name
    return Robot(name)


def _print_status(robot: Robot, state) -> None:
    d = describe(state, robot.info)
    flags = ", ".join(d["faults"] + d["warnings"]) or "ok"
    print(f"{robot.info.name}: {d['mode']} [{flags}]  loop late {d['wake_latency_us']:.0f} us, "
          f"cycle {d['exec_us']:.0f} us, overruns {d['overruns']}")
    print(f"  {'joint':<24}{'mode':<10}{'q':>9}{'qd':>9}{'tau':>9}{'temp':>7}  faults")
    for i, joint in enumerate(d["joints"]):
        temp = d["temperature"][i]
        print(f"  {joint['name']:<24}{joint['mode']:<10}{d['q'][i]:>9.3f}{d['qd'][i]:>9.3f}{d['tau'][i]:>9.2f}"
              f"{'-' if temp is None else format(temp, '.0f'):>7}  {', '.join(joint['faults'])}")
    if robot.info.has_base:
        x, y, yaw = d["odom"]
        print(f"  base: {d['base_v']:.2f} m/s, {d['base_omega']:.2f} rad/s, odometry x {x:.2f} y {y:.2f} "
              f"yaw {np.degrees(yaw):.0f} deg")


def cmd_status(args) -> int:
    with _robot(args) as robot:
        state = robot.wait_for(lambda s: True, timeout=1.0)
        if state is None:
            print("no state yet", file=sys.stderr)
            return 1
        _print_status(robot, state)
        return 0 if state["mode"] in (RobotMode.IDLE, RobotMode.ACTIVE) else 3


def cmd_watch(args) -> int:
    with _robot(args) as robot:
        while robot.alive():
            state = robot.state()
            if state is not None:
                print("\033[2J\033[H", end="")
                _print_status(robot, state)
            time.sleep(0.2)
    return 0


def cmd_request(args) -> int:
    with _robot(args) as robot:
        if args.command == "enable":
            ok = robot.enable()
        elif args.command == "disable":
            ok = robot.disable()
        elif args.command == "clear":
            ok = robot.clear_faults()
        else:
            robot.estop()
            ok = robot.wait_mode(RobotMode.ESTOP, 1.0) is not None
        state = robot.state()
        mode = RobotMode(int(state["mode"])).name.lower() if state is not None else "unknown"
        print(f"{robot.info.name}: {mode}")
        return 0 if ok else 1


def cmd_ui(args) -> int:
    from .ui import server

    tools = config_module.load(args.config)
    if args.shm_name:
        tools.shm_name = args.shm_name
    if args.test_camera:
        tools.cameras = [config_module.CameraConfig(name="test", source="test")]
    if args.sim_cameras:
        tools.cameras = [
            config_module.CameraConfig(name=f"{where} {mode}", source=f"sim:{link}:{mode}")
            for where, link in (("base", "base_camera_link"), ("gripper", "wrist_camera_link"))
            for mode in ("color", "depth")
        ]
    where = f"http://{'<this machine>' if args.host == '0.0.0.0' else args.host}:{args.port}/"
    print(f"obot ui: {where}{'?token=...' if args.token else ''}", file=sys.stderr)
    if not args.token and args.host == "0.0.0.0":
        print("obot ui: anyone who can reach this port can drive the robot; see --token", file=sys.stderr)
    server.run(tools, host=args.host, port=args.port, token=args.token, joystick=args.joystick,
               cameras=not args.no_cameras)
    return 0


def cmd_teleop(args) -> int:
    from .teleop.joystick import Joystick
    from .teleop.mapper import InputWatch, TeleopMapper

    tools = config_module.load(args.config)
    joystick = Joystick(args.joystick)
    print(f"obot teleop: {joystick.path}", file=sys.stderr)
    if args.show:
        while joystick.connected:
            pad = joystick.poll()
            axes = " ".join(f"{a:+.2f}" for a in pad.axes)
            buttons = "".join("1" if b > 0.5 else "." for b in pad.buttons)
            print(f"\raxes {axes}  buttons {buttons}  LT {pad.buttons[6]:.2f} RT {pad.buttons[7]:.2f}", end="")
            time.sleep(0.05)
        return 1
    with Robot(args.shm_name or tools.shm_name) as robot:
        mapper = TeleopMapper(robot.info, tools.teleop, tools.base_v_max, tools.base_omega_max, tools.arm)
        dt = 0.02
        next_tick = time.monotonic()
        watch = InputWatch(tools.teleop.controller_timeout_s)
        while robot.alive() and joystick.connected:
            out = mapper.step(watch.filter(joystick.poll(), time.monotonic()), robot.state(), dt)
            robot.send(out.command, out.request)
            next_tick += dt
            time.sleep(max(0.0, next_tick - time.monotonic()))
        # A neutral command on the way out; the daemon's command timeout covers a crash.
        robot.send(None, Request.NONE)
    print("obot teleop: controller or daemon gone", file=sys.stderr)
    return 1


def cmd_record(args) -> int:
    from .log import record

    with _robot(args) as robot:
        return record(robot, args.output, duration=args.duration, decimate=args.decimate)


def cmd_viz(args) -> int:
    from .viz import run

    with _robot(args) as robot:
        return run(robot, save=args.save, connect=args.connect, urdf=args.urdf, duration=args.duration)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="obot", description=__doc__.split("\n")[0])
    parser.add_argument("--config", help="robot configuration (default: $OBOT_CONFIG or /etc/obot/robot.toml)")
    parser.add_argument("--shm-name", help="shared memory name of the daemon (default: from the configuration)")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("status", help="print the robot's state").set_defaults(func=cmd_status)
    sub.add_parser("watch", help="print the robot's state continuously").set_defaults(func=cmd_watch)
    for name, text in (("enable", "start following commands"), ("disable", "stop following commands"),
                       ("clear", "clear latched faults"), ("stop", "emergency stop")):
        sub.add_parser(name, help=text).set_defaults(func=cmd_request)

    ui = sub.add_parser("ui", help="operator web page")
    ui.add_argument("--host", default="0.0.0.0")
    ui.add_argument("--port", type=int, default=8080)
    ui.add_argument("--token", help="require this token (?token=...) to open the page")
    ui.add_argument("--joystick", nargs="?", const="auto",
                    help="also accept a controller connected to this machine (default device: first found)")
    ui.add_argument("--no-cameras", action="store_true")
    ui.add_argument("--test-camera", action="store_true", help="show a test pattern instead of real cameras")
    ui.add_argument("--sim-cameras", action="store_true",
                    help="simulated base and gripper cameras (colour and depth) rendered from the robot model")
    ui.set_defaults(func=cmd_ui)

    teleop = sub.add_parser("teleop", help="drive with a controller connected to this machine")
    teleop.add_argument("--joystick", help="device, e.g. /dev/input/js0 (default: first found)")
    teleop.add_argument("--show", action="store_true", help="print what the controller reports and do nothing else")
    teleop.set_defaults(func=cmd_teleop)

    rec = sub.add_parser("record", help="log states to an MCAP file")
    rec.add_argument("output")
    rec.add_argument("--duration", type=float, help="seconds (default: until interrupted)")
    rec.add_argument("--decimate", type=int, default=1, help="keep every Nth record")
    rec.set_defaults(func=cmd_record)

    viz = sub.add_parser("viz", help="live plots in the Rerun viewer")
    viz.add_argument("--save", help="write a .rrd recording instead of opening a viewer")
    viz.add_argument("--connect", help="stream to a viewer at this URL")
    viz.add_argument("--urdf", help="robot model to show (logged once)")
    viz.add_argument("--duration", type=float, help="seconds (default: until interrupted)")
    viz.set_defaults(func=cmd_viz)

    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except NotRunning as e:
        print(f"obot: {e}", file=sys.stderr)
        return 2
    except FileNotFoundError as e:
        print(f"obot: {e}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
