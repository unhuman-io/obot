"""The daemon over real SocketCAN sockets, against obot-sim-node on virtual CAN interfaces.

    sudo modprobe vcan
    for i in 0 1 2; do sudo ip link add vcan$i type vcan; sudo ip link set vcan$i mtu 72 up; done
"""

import subprocess
import time
from pathlib import Path

import pytest
from conftest import CONFIG, bin_dir, start_daemon, stop_daemon

from obot import JointMode, Robot, RobotMode

BUSES = ["--bus", "arm_a=vcan0", "--bus", "arm_b=vcan1", "--bus", "base=vcan2"]

pytestmark = pytest.mark.vcan


@pytest.fixture
def vcan_robot(shm_name, tmp_path):
    if not all(Path(f"/sys/class/net/vcan{i}").exists() for i in range(3)):
        pytest.skip("vcan0..vcan2 are not set up")
    # Three processes without realtime priority on a shared test machine: allow the
    # actuators 100 ms of silence instead of 5 ms before calling it a fault.
    config = tmp_path / "robot.toml"
    text = CONFIG.read_text()
    assert "comm_timeout_cycles = 5 " in text
    config.write_text(text.replace("comm_timeout_cycles = 5 ", "comm_timeout_cycles = 100 "))
    node = subprocess.Popen([str(bin_dir() / "obot-sim-node"), "--config", str(config), *BUSES],
                            stderr=subprocess.DEVNULL)
    time.sleep(0.2)
    daemon = start_daemon(shm_name, *BUSES, config=config)
    with Robot(shm_name) as robot:
        yield robot, node
    stop_daemon(daemon)
    node.terminate()
    node.wait(timeout=5)


def test_arm_and_base_over_socketcan(vcan_robot):
    robot, _ = vcan_robot
    assert robot.wait_mode(RobotMode.IDLE, 3) is not None
    assert robot.enable()
    command = robot.new_command()
    command["joint_mode"][:4] = JointMode.POSITION
    command["q"][2] = 0.5                          # wrist pitch
    command["base_mode"] = 1
    command["base_v"] = 0.3                        # wheels
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        robot.send(command)
        time.sleep(0.01)
    state = robot.state()
    assert state["mode"] == RobotMode.ACTIVE
    assert state["fault"] == 0
    assert state["q"][2] == pytest.approx(0.5, abs=0.03)
    assert state["base_v"] == pytest.approx(0.3, rel=0.15)
    assert state["odom_x"] > 0.4
    assert state["temperature"][4] == pytest.approx(30.0)
    assert state["rx_wait_ns"] > 0                 # replies were waited for on the bus


def test_losing_the_actuators_faults_the_robot(vcan_robot):
    robot, node = vcan_robot
    assert robot.wait_mode(RobotMode.IDLE, 3) is not None
    assert robot.enable()
    node.terminate()                               # every actuator goes silent
    state = robot.wait_mode(RobotMode.FAULT, 2)
    assert state is not None
    assert all(state["joint_fault"][:6] & 1)       # comm_timeout on each joint
