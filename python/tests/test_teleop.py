import time

import numpy as np
import pytest

from obot import JointMode, Request, RobotMode
from obot.client import RobotInfo
from obot.layout import STATE
from obot.teleop import mapper as m
from obot.teleop.mapper import GamepadState, TeleopMapper


def make_info(arm=4) -> RobotInfo:
    n = arm + 2
    return RobotInfo(
        name="t", num_joints=n, period_s=0.001,
        joint_names=tuple(f"j{i}" for i in range(arm)) + ("wl", "wr"),
        q_min=np.full(n, -1.0, np.float32), q_max=np.full(n, 1.0, np.float32),
        qd_max=np.full(n, 2.0, np.float32), tau_max=np.ones(n, np.float32),
        continuous=np.array([False] * arm + [True, True]),
        left_wheel=arm, right_wheel=arm + 1, wheel_radius=0.1, track_width=0.4,
    )


def active_state(q=None):
    state = np.zeros((), STATE)
    state["mode"] = RobotMode.ACTIVE
    if q is not None:
        state["q"][: len(q)] = q
    return state


def pad(axes=(0, 0, 0, 0), **buttons) -> GamepadState:
    p = GamepadState(list(map(float, axes)))
    for name, value in buttons.items():
        p.buttons[getattr(m, "BTN_" + name.upper())] = float(value)
    return p


def test_nothing_moves_without_the_deadman():
    mapper = TeleopMapper(make_info(), base_v_max=0.5, base_omega_max=1.5)
    out = mapper.step(pad(axes=(1, -1, 1, -1), up=1), active_state(), 0.02)
    assert not out.moving
    assert out.command["base_v"] == 0 and out.command["base_omega"] == 0
    assert np.all(out.command["q"] == 0)
    # ...and with it, forward on the stick is forward on the robot, left is a left turn.
    out = mapper.step(pad(axes=(-1, -1, 0, 0), lb=1), active_state(), 0.02)
    assert out.moving
    assert out.command["base_v"] == pytest.approx(0.6 * 0.5)
    assert out.command["base_omega"] == pytest.approx(0.6 * 1.5)
    out = mapper.step(pad(axes=(0, -1, 0, 0), lb=1, rt=1), active_state(), 0.02)
    assert out.command["base_v"] == pytest.approx(0.5)      # boost reaches the base limit


def test_small_stick_deflection_is_ignored():
    mapper = TeleopMapper(make_info())
    out = mapper.step(pad(axes=(0.1, -0.1, 0.1, 0.1), lb=1), active_state(), 0.02)
    assert out.command["base_v"] == 0
    assert np.all(out.command["q"] == 0)


def test_arm_jog_is_bounded_by_lead_and_limits():
    mapper = TeleopMapper(make_info())
    state = active_state()
    for _ in range(500):                       # the joint is stuck at 0: the target must not run away
        out = mapper.step(pad(axes=(0, 0, 0, -1), lb=1), state, 0.02)
    assert out.command["joint_mode"][0] == JointMode.POSITION
    assert out.command["q"][0] == pytest.approx(mapper.config.max_lead)
    assert out.command["q"][1] == 0             # only the stick's own joint moved
    # A joint that follows runs to its limit and stays there.
    q = np.zeros(4, np.float32)
    for _ in range(500):
        out = mapper.step(pad(axes=(0, 0, 0, -1), lb=1), active_state(q), 0.02)
        q[0] = out.command["q"][0]
    assert q[0] == pytest.approx(1.0)
    # Releasing the deadman leaves the target within reach of where the joint is.
    out = mapper.step(pad(), active_state(np.array([0.2, 0, 0, 0])), 0.02)
    assert abs(out.command["q"][0] - 0.2) <= mapper.config.max_lead + 1e-6


def test_dpad_and_groups():
    mapper = TeleopMapper(make_info(arm=6))
    assert mapper.num_groups == 2
    out = mapper.step(pad(lb=1, up=1, right=1), active_state(), 0.02)
    assert out.command["q"][2] > 0 and out.command["q"][3] > 0
    assert out.command["q"][4] == 0
    mapper.step(pad(lb=1, rb=1), active_state(), 0.02)       # next group
    out = mapper.step(pad(axes=(0, 0, 0, -1), lb=1, rb=1), active_state(), 0.02)   # still held: no change
    assert out.group == 1
    assert out.command["q"][4] > 0
    mapper.step(pad(lb=1), active_state(), 0.02)
    out = mapper.step(pad(lb=1, rb=1), active_state(), 0.02)  # pressed again: wraps round
    assert out.group == 0


def test_buttons_are_requests_once_per_press():
    mapper = TeleopMapper(make_info())
    idle = np.zeros((), STATE)
    idle["mode"] = RobotMode.IDLE
    assert mapper.step(pad(start=1), idle, 0.02).request == Request.ENABLE
    assert mapper.step(pad(start=1), idle, 0.02).request == Request.NONE
    assert mapper.step(pad(), idle, 0.02).request == Request.NONE
    assert mapper.step(pad(b=1), idle, 0.02).request == Request.DISABLE
    assert mapper.step(pad(back=1), idle, 0.02).request == Request.CLEAR_FAULTS
    assert mapper.step(pad(y=1, start=1), idle, 0.02).request == Request.ESTOP   # stop wins
    assert not mapper.step(pad(lb=1, axes=(0, -1, 0, 0)), idle, 0.02).moving


def test_targets_restart_from_the_joints_after_a_disable():
    mapper = TeleopMapper(make_info())
    mapper.step(pad(axes=(0, 0, 0, -1), lb=1), active_state(), 0.02)
    idle = np.zeros((), STATE)
    mapper.step(pad(), idle, 0.02)
    out = mapper.step(pad(), active_state(np.array([0.7, 0, 0, 0])), 0.02)
    assert out.command["q"][0] == pytest.approx(0.7)


def test_untrusted_gamepad_input_is_sanitised():
    p = GamepadState.from_lists([5, "x", float("nan"), -9, 1, 1, 1], [2, -1, None] + [1] * 40)
    assert p.axes == [1.0, 0.0, 0.0, -1.0]
    assert len(p.buttons) == m.NUM_BUTTONS
    assert p.buttons[:3] == [1.0, 0.0, 0.0]
    assert GamepadState.from_lists([], []).axes == [0.0] * 4


def test_driving_the_simulated_robot(robot):
    from obot.config import load

    tools = load(str(__import__("conftest").CONFIG))
    mapper = TeleopMapper(robot.info, tools.teleop, tools.base_v_max, tools.base_omega_max)
    presses = [pad(start=1)] + [pad(axes=(0, -1, 0, -1), lb=1)] * 100 + [pad()] * 60
    for p in presses:
        out = mapper.step(p, robot.state(), 0.02)
        robot.send(out.command, out.request)
        time.sleep(0.02)
        if p.pressed(m.BTN_LB):
            moving_state = robot.state()
    assert moving_state["mode"] == RobotMode.ACTIVE
    assert moving_state["base_v"] == pytest.approx(0.6 * tools.base_v_max, rel=0.1)
    assert moving_state["q"][0] > 0.2            # the shoulder was jogged
    state = robot.state()
    assert abs(state["base_v"]) < 0.02           # let go: stopped
    assert abs(state["qd"][0]) < 0.05
    assert state["odom_x"] > 0.3


def test_input_that_stops_changing_is_treated_as_released():
    from obot.teleop.mapper import InputWatch

    watch = InputWatch(5.0)
    held = pad(axes=(0, -1, 0, 0), lb=1)
    assert watch.filter(held, 100.0).pressed(m.BTN_LB)
    assert watch.filter(held, 104.9).pressed(m.BTN_LB)
    assert watch.filter(held, 105.1).neutral()               # frozen for over five seconds
    nudged = pad(axes=(0, -0.9, 0, 0), lb=1)
    assert watch.filter(nudged, 105.2).pressed(m.BTN_LB)     # any change brings it back
    assert watch.filter(pad(), 200.0).neutral()              # a resting controller is just neutral
    assert InputWatch(0).filter(held, 1e9).pressed(m.BTN_LB)  # 0 disables


def test_resync_starts_again_from_the_joints():
    mapper = TeleopMapper(make_info())
    mapper.step(pad(lb=1), active_state(), 0.02)             # targets taken at q = 0
    moved = active_state(np.array([0.9, 0, 0, 0]))           # something else moved the arm since
    out = mapper.step(pad(lb=1), moved, 0.02)
    assert out.command["q"][0] == pytest.approx(0.9 - mapper.config.max_lead)   # the old target trails
    mapper.resync()
    out = mapper.step(pad(lb=1), moved, 0.02)
    assert out.command["q"][0] == pytest.approx(0.9)
    # A joint the daemon is holding in position is taken over at its setpoint, not at the
    # (sagging) measured position.
    held = active_state(np.array([0.84, 0, 0, 0]))
    held["joint_mode"][0] = JointMode.POSITION
    held["q_cmd"][0] = 0.9
    mapper.resync()
    assert mapper.step(pad(lb=1), held, 0.02).command["q"][0] == pytest.approx(0.9)
