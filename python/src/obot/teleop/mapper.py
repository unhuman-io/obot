"""Gamepad to robot command, shared by the local controller and the web UI.

The gamepad is described in the W3C "standard" layout, which is what a browser reports
for an Xbox controller and what obot.teleop.joystick converts Linux joystick events to.

    hold LB            deadman: nothing moves unless it is held
    left stick         drive: forward / back, turn
    RT                 boost: from the configured base speed up to the base's limit
    right stick        arm joints 1 and 2 of the selected group (up / down, left / right)
    D-pad              arm joints 3 and 4 of the selected group (up / down, left / right)
    RB                 next group of four arm joints
    Start              enable          B   disable
    Back               clear faults    Y   emergency stop

Arm joints are jogged: the stick sets how fast the position target moves. The target never
runs further ahead of the measured joint than `max_lead`, so letting go stops the joint.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..client import JointMode, Request, RobotInfo, RobotMode
from ..config import TeleopConfig
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


@dataclass
class TeleopOutput:
    command: np.ndarray  # a COMMAND record
    request: Request
    moving: bool  # the deadman is held and the robot is active
    group: int  # selected arm joint group


class TeleopMapper:
    def __init__(self, info: RobotInfo, config: TeleopConfig | None = None,
                 base_v_max: float = 0.5, base_omega_max: float = 1.5):
        self.info = info
        self.config = config or TeleopConfig()
        self.base_v_max = base_v_max
        self.base_omega_max = base_omega_max
        self.arm = list(info.arm_joints)
        self.group = 0
        self._targets = np.zeros(info.num_joints, dtype=np.float32)
        self._have_targets = False
        self._previous = GamepadState()

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
        request = Request.NONE
        if self._edge(pad, BTN_Y):
            request = Request.ESTOP
        elif self._edge(pad, BTN_B):
            request = Request.DISABLE
        elif self._edge(pad, BTN_BACK):
            request = Request.CLEAR_FAULTS
        elif self._edge(pad, BTN_START):
            request = Request.ENABLE
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
            self._targets[:] = state["q"][:n]
            self._have_targets = True

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
            self._targets[joint] = target
            command["joint_mode"][joint] = JointMode.POSITION
            command["q"][joint] = target
        return TeleopOutput(command, request, deadman, self.group)
