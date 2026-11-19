"""Gamepad to robot command, shared by the local controller and the web UI.

The gamepad is described in the W3C "standard" layout, which is what a browser reports
for an Xbox controller and what obot.teleop.joystick converts Linux joystick events to.

    hold LB            deadman: nothing moves unless it is held
    left stick         drive: forward / back, turn
    RT                 boost: from the configured base speed up to the base's limit
    right stick        arm joints 1 and 2 of the selected group (up / down, left / right)
    D-pad              arm joints 3 and 4 of the selected group (up / down, left / right)
    RB                 next group of four arm joints
    X                  switch between joint mode (above) and Cartesian mode, where
                       right stick up / down moves the tool forward / back, D-pad up / down
                       raises / lowers it, D-pad left / right pitches it and right stick
                       left / right rolls the wrist. Moves that would leave the joint
                       limits or the arm's reach are not made.
    Start              enable          B   disable
    Back               clear faults    Y   emergency stop

Arm joints are jogged: the stick sets how fast the position target moves. The target never
runs further ahead of the measured joint than `max_lead`, so letting go stops the joint.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..client import JointMode, Request, RobotInfo, RobotMode
from ..config import ArmConfig, TeleopConfig
from ..kinematics import Planar3R
from ..layout import COMMAND, BaseMode

# Standard gamepad indices.
AXIS_LX, AXIS_LY, AXIS_RX, AXIS_RY = 0, 1, 2, 3
BTN_A, BTN_B, BTN_X, BTN_Y, BTN_LB, BTN_RB, BTN_LT, BTN_RT = range(8)
BTN_BACK, BTN_START, BTN_LS, BTN_RS, BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT = range(8, 16)
NUM_AXES, NUM_BUTTONS = 4, 17


@dataclass
class GamepadState:
    axes: list[float] = field(default_factory=lambda: [0.0] * NUM_AXES)
    buttons: list[float] = field(default_factory=lambda: [0.0] * NUM_BUTTONS)  # 0..1, triggers are analog

    @classmethod
    def from_lists(cls, axes, buttons) -> GamepadState:
        """Build from untrusted input (a web client): pads, truncates and clamps."""

        def clean(values, n, lo):
            out = []
            for v in list(values)[:n]:
                try:
                    x = float(v)
                except (TypeError, ValueError):
                    x = 0.0
                out.append(min(1.0, max(lo, x)) if x == x else 0.0)
            return out + [0.0] * (n - len(out))

        return cls(clean(axes, NUM_AXES, -1.0), clean(buttons, NUM_BUTTONS, 0.0))

    def pressed(self, button: int) -> bool:
        return self.buttons[button] > 0.5


    def neutral(self) -> bool:
        return not any(b > 0.5 for b in self.buttons) and not any(abs(a) > 0.25 for a in self.axes)


def button_request(previous: GamepadState, pad: GamepadState) -> Request:
    """The request a newly pressed button stands for; stopping takes precedence."""
    for button, request in ((BTN_Y, Request.ESTOP), (BTN_B, Request.DISABLE), (BTN_BACK, Request.CLEAR_FAULTS),
                            (BTN_START, Request.ENABLE)):
        if pad.pressed(button) and not previous.pressed(button):
            return request
    return Request.NONE


class InputWatch:
    """Treats input that has not changed for too long as released.

    Neither the Linux joystick API nor the browser's gamepad API says whether a controller
    is still there: one that dropped out simply stops changing. So does a stick held hard
    against its stop, which is why the timeout is seconds, not milliseconds.
    """

    def __init__(self, timeout_s: float):
        self.timeout_s = timeout_s
        self._last: GamepadState | None = None
        self._changed_at = 0.0

    def filter(self, pad: GamepadState, now: float) -> GamepadState:
        if self._last is None or pad.axes != self._last.axes or pad.buttons != self._last.buttons:
            self._last = GamepadState(list(pad.axes), list(pad.buttons))
            self._changed_at = now
        if self.timeout_s > 0 and now - self._changed_at > self.timeout_s and not pad.neutral():
            return GamepadState()
        return pad


@dataclass
class TeleopOutput:
    command: np.ndarray  # a COMMAND record
    request: Request
    moving: bool  # the deadman is held and the robot is active
    group: int  # selected arm joint group
    cartesian: bool = False  # Cartesian mode is selected


class TeleopMapper:
    def __init__(self, info: RobotInfo, config: TeleopConfig | None = None,
                 base_v_max: float = 0.5, base_omega_max: float = 1.5, arm: ArmConfig | None = None):
        self.cartesian = False
        self.kinematics = None
        self._pose = None  # Cartesian target (x, z, pitch)
        if arm is not None and all(name in info.joint_names for name in arm.joints):
            self._plane = [info.index(name) for name in arm.joints]
            self._roll = info.index(arm.roll) if arm.roll in info.joint_names else None
            self.kinematics = Planar3R(
                tuple(arm.lengths), tuple(arm.zero),
                tuple(float(info.q_min[i]) for i in self._plane), tuple(float(info.q_max[i]) for i in self._plane),
            )
        self.info = info
        self.config = config or TeleopConfig()
        self.base_v_max = base_v_max
        self.base_omega_max = base_omega_max
        self.arm = list(info.arm_joints)
        self.group = 0
        self._targets = np.zeros(info.num_joints, dtype=np.float32)
        self._have_targets = False
        self._previous = GamepadState()

    def resync(self) -> None:
        """Forget the jog targets: the next step starts from where the joints are.

        Call when this mapper takes over after something else (a policy, another
        operator) may have moved the arm.
        """
        self._have_targets = False

    @property
    def num_groups(self) -> int:
        return max(1, (len(self.arm) + 3) // 4)

    def _stick(self, value: float) -> float:
        d = self.config.deadband
        if abs(value) <= d:
            return 0.0
        scaled = (abs(value) - d) / (1.0 - d)
        return float(np.sign(value)) * scaled * scaled  # finer control near the centre

    def _edge(self, pad: GamepadState, button: int) -> bool:
        return pad.pressed(button) and not self._previous.pressed(button)

    def step(self, pad: GamepadState, state, dt: float) -> TeleopOutput:
        """One tick. `state` is the newest RobotState record (or None)."""
        info = self.info
        command = np.zeros((), COMMAND)
        request = button_request(self._previous, pad)
        if self._edge(pad, BTN_X) and self.kinematics is not None:
            self.cartesian = not self.cartesian
            self._pose = None
        if self._edge(pad, BTN_RB):
            self.group = (self.group + 1) % self.num_groups
        self._previous = GamepadState(list(pad.axes), list(pad.buttons))

        active = state is not None and int(state["mode"]) == RobotMode.ACTIVE
        if not active:
            # Start from where the joints are the next time the robot is enabled.
            self._have_targets = False
            return TeleopOutput(command, request, False, self.group)
        n = info.num_joints
        if not self._have_targets:
            # Take over what the daemon is already holding where a joint is under position
            # control (so the arm does not sag by its tracking error), else where the joint is.
            modes = state["joint_mode"][:n]
            held = (modes == JointMode.POSITION) | (modes == JointMode.IMPEDANCE)
            self._targets[:] = np.where(held, state["q_cmd"][:n], state["q"][:n])
            self._have_targets = True
            self._pose = None

        deadman = pad.pressed(BTN_LB)
        if info.has_base:
            command["base_mode"] = BaseMode.TWIST
            if deadman:
                c = self.config
                scale = c.base_speed + (1.0 - c.base_speed) * pad.buttons[BTN_RT]
                command["base_v"] = -self._stick(pad.axes[AXIS_LY]) * scale * self.base_v_max
                command["base_omega"] = -self._stick(pad.axes[AXIS_LX]) * scale * self.base_omega_max

        # Rates for the four joints of the selected group, each in -1..1.
        rates = (
            -self._stick(pad.axes[AXIS_RY]),
            self._stick(pad.axes[AXIS_RX]),
            pad.buttons[BTN_UP] - pad.buttons[BTN_DOWN],
            pad.buttons[BTN_RIGHT] - pad.buttons[BTN_LEFT],
        )
        selected = self.arm[self.group * 4 : self.group * 4 + 4]
        if self.cartesian:
            selected = []
            if deadman:
                self._step_cartesian(rates, dt)
                if self._roll is not None:
                    selected, rates = [self._roll], (rates[1],)
        for joint in self.arm:
            q = float(state["q"][joint])
            target = float(self._targets[joint])
            if deadman and joint in selected:
                rate = rates[selected.index(joint)]
                target += rate * self.config.arm_speed * float(info.qd_max[joint]) * dt
            # Bounded lead: releasing the stick (or the deadman) stops the joint within max_lead.
            target = min(q + self.config.max_lead, max(q - self.config.max_lead, target))
            if not info.continuous[joint]:
                target = min(float(info.q_max[joint]), max(float(info.q_min[joint]), target))
            if self.cartesian and target != float(self._targets[joint]) and joint in self._plane:
                self._pose = None  # a joint was held back: continue from where the targets really are
            self._targets[joint] = target
            command["joint_mode"][joint] = JointMode.POSITION
            command["q"][joint] = target
        return TeleopOutput(command, request, deadman, self.group, self.cartesian)

    def _step_cartesian(self, rates, dt: float) -> None:
        """Move the Cartesian target and, if the arm can go there, the joint targets."""
        forward, _, up, pitch_rate = rates
        q_now = [float(self._targets[i]) for i in self._plane]
        if self._pose is None:
            self._pose = self.kinematics.fk(q_now)
        c = self.config
        steps = (forward * c.cartesian_speed * dt, up * c.cartesian_speed * dt, pitch_rate * c.pitch_speed * dt)
        # Whole move first; if that is not possible, each direction on its own, so the tool
        # slides along a limit instead of sticking to it.
        candidates = [steps] + [tuple(s if i == k else 0.0 for i, s in enumerate(steps)) for k in range(3)]
        for dx, dz, dp in candidates:
            if dx == dz == dp == 0.0:
                continue
            pose = (self._pose[0] + dx, self._pose[1] + dz, self._pose[2] + dp)
            q = self.kinematics.ik(*pose, near=q_now)
            # No jumping to the other elbow configuration in one tick.
            if q is not None and max(abs(a - b) for a, b in zip(q, q_now, strict=True)) < 0.2:
                self._pose = pose
                for index, value in zip(self._plane, q, strict=True):
                    self._targets[index] = value
                return
