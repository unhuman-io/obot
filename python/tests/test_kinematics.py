import math
import random

import numpy as np
import pytest
from conftest import CONFIG
from test_teleop import active_state, pad

from obot.client import RobotInfo
from obot.config import load
from obot.kinematics import Planar3R
from obot.teleop.mapper import TeleopMapper

ARM = Planar3R((0.25, 0.22, 0.1308), (math.pi / 2, 0, 0), (-1.78, -0.087, -1.658), (0.733, 3.229, 1.431))


def test_zero_pose_matches_the_urdf():
    # All joints at zero: upper arm straight up, forearm and tool forward.
    x, z, pitch = ARM.fk((0, 0, 0))
    assert (x, z, pitch) == pytest.approx((0.22 + 0.1308, 0.25, 0.0))
    # Positive shoulder angle tips the arm back (the URDF axis is -y), positive elbow lifts the forearm.
    assert ARM.fk((0.3, 0, 0))[0] < x
    assert ARM.fk((0, 0.3, 0))[1] > z


def test_ik_inverts_fk_inside_the_limits():
    rng = random.Random(1)
    for _ in range(2000):
        q = tuple(rng.uniform(lo, hi) for lo, hi in zip(ARM.q_min, ARM.q_max, strict=True))
        pose = ARM.fk(q)
        solution = ARM.ik(*pose, near=q)
        assert solution is not None
        assert ARM.within_limits(solution)
        assert ARM.fk(solution) == pytest.approx(pose, abs=1e-9)
        assert solution == pytest.approx(q, abs=1e-6)        # the nearest branch is the one we came from


def test_ik_refuses_what_the_arm_cannot_do():
    assert ARM.ik(2.0, 0.0, 0.0) is None                     # out of reach
    assert ARM.ik(0.0, 0.0, 0.0) is None                     # folded onto the shoulder
    x, z, pitch = ARM.fk((0, 0, 0))
    assert ARM.ik(x, z, pitch + 3.0) is None                 # wrist limit


def make_mapper():
    tools = load(str(CONFIG))
    names = ("shoulder_pitch_joint", "elbow_pitch_joint", "wrist_pitch_joint", "wrist_roll_joint", "wl", "wr")
    info = RobotInfo(
        name="t", num_joints=6, period_s=0.001, joint_names=names,
        q_min=np.array([-1.78, -0.087, -1.658, -2.827, 0, 0], np.float32),
        q_max=np.array([0.733, 3.229, 1.431, 2.827, 0, 0], np.float32),
        qd_max=np.full(6, 2.0, np.float32), tau_max=np.ones(6, np.float32),
        continuous=np.array([False] * 4 + [True] * 2), left_wheel=4, right_wheel=5, wheel_radius=0.1, track_width=0.4,
    )
    return TeleopMapper(info, tools.teleop, arm=tools.arm)


def follow(mapper, p, seconds, q):
    """Run the mapper with joints that follow their targets perfectly."""
    for _ in range(int(seconds / 0.02)):
        out = mapper.step(p, active_state(q), 0.02)
        q = out.command["q"][:4].copy()
    return q, out


def test_cartesian_mode_moves_the_tool_in_straight_lines():
    mapper = make_mapper()
    q = np.array([0.5, 0.0, -0.5, 0.0], np.float32)
    start = mapper.kinematics.fk(q[:3])
    mapper.step(pad(x=1), active_state(q), 0.02)             # X: Cartesian mode
    q, out = follow(mapper, pad(axes=(0, 0, 0, -1), lb=1), 1.0, q)   # right stick up: forward
    assert out.cartesian
    x, z, pitch = mapper.kinematics.fk(q[:3])
    assert x - start[0] == pytest.approx(0.15, abs=0.01)     # cartesian_speed for one second
    assert z == pytest.approx(start[1], abs=1e-4)
    assert pitch == pytest.approx(start[2], abs=1e-4)

    q, _ = follow(mapper, pad(lb=1, down=1), 0.5, q)         # D-pad down: lower
    x2, z2, pitch2 = mapper.kinematics.fk(q[:3])
    assert z2 - z == pytest.approx(-0.075, abs=0.01)
    assert x2 == pytest.approx(x, abs=1e-4)

    q, _ = follow(mapper, pad(lb=1, left=1), 0.5, q)         # D-pad left: pitch down, tool point stays
    x3, z3, pitch3 = mapper.kinematics.fk(q[:3])
    assert pitch3 - pitch2 == pytest.approx(-0.4, abs=0.02)
    assert (x3, z3) == pytest.approx((x2, z2), abs=1e-4)

    q, _ = follow(mapper, pad(axes=(0, 0, 1, 0), lb=1), 0.5, q)      # right stick right: roll only
    assert q[3] > 0.2
    assert mapper.kinematics.fk(q[:3]) == pytest.approx((x3, z3, pitch3), abs=1e-4)


def test_cartesian_mode_stops_at_reach_and_joint_limits():
    mapper = make_mapper()
    q = np.array([0.5, 0.0, -0.5, 0.0], np.float32)
    mapper.step(pad(x=1), active_state(q), 0.02)
    q, _ = follow(mapper, pad(axes=(0, 0, 0, -1), lb=1), 20.0, q)    # forward for far longer than the arm is long
    assert np.all(q[:3] >= mapper.info.q_min[:3] - 1e-5) and np.all(q[:3] <= mapper.info.q_max[:3] + 1e-5)
    assert mapper.kinematics.fk(q[:3])[0] < 0.25 + 0.22 + 0.1308
    settled = q.copy()
    q, _ = follow(mapper, pad(axes=(0, 0, 0, -1), lb=1), 1.0, q)     # still pushing: nothing moves, nothing jumps
    assert q[:3] == pytest.approx(settled[:3], abs=1e-3)
    q, _ = follow(mapper, pad(axes=(0, 0, 0, 1), lb=1), 1.0, q)      # and it comes back
    assert mapper.kinematics.fk(q[:3])[0] < mapper.kinematics.fk(settled[:3])[0] - 0.1


def test_cartesian_mode_needs_arm_geometry_and_the_deadman():
    from test_teleop import make_info

    plain = TeleopMapper(make_info())
    assert not plain.step(pad(x=1), active_state(), 0.02).cartesian   # no [arm] section: stays in joint mode
    mapper = make_mapper()
    q = np.array([0.5, 0.0, -0.5, 0.0], np.float32)
    mapper.step(pad(x=1), active_state(q), 0.02)
    moved, _ = follow(mapper, pad(axes=(0, 0, 0, -1)), 1.0, q)        # no deadman
    assert moved == pytest.approx(q)
