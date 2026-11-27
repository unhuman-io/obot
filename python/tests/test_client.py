import time

import numpy as np
import pytest

from obot import JointMode, NotRunning, Request, Robot, RobotMode


def test_attach_reports_what_is_wrong():
    with pytest.raises(NotRunning, match="obot-rtd running"):
        Robot("obot_pytest_no_such_daemon")


def test_info_and_idle_state(robot):
    info = robot.info
    assert info.name == "obot3"
    assert info.num_joints == 6
    assert info.joint_names[0] == "shoulder_pitch_joint"
    assert info.has_base
    assert info.arm_joints == (0, 1, 2, 3)
    assert info.period_s == pytest.approx(0.001)
    assert info.q_max[1] == pytest.approx(3.229)
    assert list(info.continuous) == [False] * 4 + [True] * 2
    state = robot.state()
    assert state["mode"] == RobotMode.IDLE
    assert state["fault"] == 0
    assert state["num_joints"] == 6


def test_read_returns_consecutive_records(robot):
    robot.read()
    time.sleep(0.2)
    records = robot.read()
    assert 20 <= len(records) <= 260           # 200 cycles at 1 kHz, fewer on a loaded test machine
    assert np.all(np.diff(records["cycle"].astype(np.int64)) == 1)
    assert np.all(np.diff(records["t_mono_ns"]) > 0)
    more = robot.read()                        # continues exactly where the last call stopped
    if len(more):
        assert more["cycle"][0] == records["cycle"][-1] + 1


def test_enable_move_disable(robot):
    assert robot.enable()
    target = 0.4
    command = robot.new_command()
    command["joint_mode"][:4] = JointMode.POSITION
    command["q"][0] = target
    command["q"][1] = 1.0                       # elbow, inside its range
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        robot.send(command)
        time.sleep(0.01)
    state = robot.state()
    assert state["mode"] == RobotMode.ACTIVE
    assert int(state["warning"]) & ~4 == 0            # nothing but (on a loaded machine) overruns
    assert state["q_cmd"][0] == pytest.approx(target, abs=1e-4)
    assert state["q"][1] == pytest.approx(1.0, abs=0.03)
    assert state["command_source"] == robot.source
    assert robot.disable()
    assert robot.state()["mode"] == RobotMode.IDLE


def test_a_client_that_stops_sending_makes_the_robot_hold(robot):
    assert robot.enable()
    command = robot.new_command()
    command["joint_mode"][:4] = JointMode.POSITION
    command["q"][1] = 2.0
    for _ in range(40):
        robot.send(command)
        time.sleep(0.01)
    time.sleep(0.8)                             # silence: longer than command_timeout_ms
    state = robot.state()
    assert state["mode"] == RobotMode.ACTIVE
    assert state["warning"] & 1                  # command_stale
    assert state["q_cmd"][1] < 1.5               # stopped on the way, did not run on to 2.0
    assert abs(state["qd"][1]) < 0.05


def test_requests_and_estop(robot):
    robot.send(None, Request.ENABLE)
    robot.send(None)                             # an ordinary command right behind it
    assert robot.wait_mode(RobotMode.ACTIVE, 1.0) is not None
    robot.estop()
    assert robot.wait_mode(RobotMode.ESTOP, 1.0) is not None
    assert not robot.enable(timeout=0.3)         # latched
    assert robot.clear_faults()
    assert robot.enable()


def test_invalid_commands_are_refused_by_the_daemon(robot):
    assert robot.enable()
    command = robot.new_command()
    command["joint_mode"][0] = JointMode.POSITION
    command["q"][0] = np.nan
    robot.send(command)
    state = robot.wait_for(lambda s: s["warning"] & 2, timeout=1.0)   # command_invalid
    assert state is not None
    assert state["mode"] == RobotMode.ACTIVE


def test_two_clients_share_the_mailbox(robot, daemon):
    with Robot(daemon, source=77) as other:
        robot.send(None)
        other.send(None)
        robot.send(None)
        state = robot.wait_for(lambda s: s["command_seq"] >= 3, timeout=1.0)
        assert state is not None
        assert state["command_source"] == robot.source


def test_daemon_exit_is_noticed(shm_name):
    from conftest import start_daemon, stop_daemon

    process = start_daemon(shm_name, "--sim")
    robot = Robot(shm_name)
    assert robot.alive()
    stop_daemon(process)
    assert not robot.alive()
    robot.close()
    with pytest.raises(NotRunning):
        Robot(shm_name)
