"""A controller plugged into (or paired with) the robot, through the Linux joystick API.

Reads /dev/input/jsN with nothing but the standard library and converts to the standard
gamepad layout used by obot.teleop.mapper. The mapping below is the kernel's for Xbox
controllers (xpad over USB or the wireless dongle, and Bluetooth through hid-generic /
xpadneo); other controllers may number their axes differently, which `obot teleop --show`
makes visible.
"""

from __future__ import annotations

import glob
import os
import struct

from . import mapper
from .mapper import GamepadState

_EVENT = struct.Struct("<IhBB")  # time ms, value, type, number
_TYPE_BUTTON, _TYPE_AXIS, _TYPE_INIT = 0x01, 0x02, 0x80

# js axis number -> what it is.
_AXES = {0: "lx", 1: "ly", 2: "lt", 3: "rx", 4: "ry", 5: "rt", 6: "dpad_x", 7: "dpad_y"}
# js button number -> standard button index.
_BUTTONS = {
    0: mapper.BTN_A, 1: mapper.BTN_B, 2: mapper.BTN_X, 3: mapper.BTN_Y,
    4: mapper.BTN_LB, 5: mapper.BTN_RB, 6: mapper.BTN_BACK, 7: mapper.BTN_START,
    9: mapper.BTN_LS, 10: mapper.BTN_RS,
}  # fmt: skip


def find_joystick() -> str | None:
    devices = sorted(glob.glob("/dev/input/js*"))
    return devices[0] if devices else None


class Joystick:
    def __init__(self, path: str | None = None):
        self.path = path or find_joystick()
        if not self.path:
            raise FileNotFoundError("no joystick found under /dev/input/js*")
        self._fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)
        self._raw_axes: dict[int, float] = {}
        self._raw_buttons: dict[int, bool] = {}
        self.connected = True

    def close(self) -> None:
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1

    def poll(self) -> GamepadState:
        """Apply every pending event and return the current state.

        A controller that disappears (unplugged, out of range) reads as neutral and
        `connected` turns false; the caller's deadman logic then stops the robot.
        """
        while self.connected:
            try:
                data = os.read(self._fd, _EVENT.size * 64)
            except BlockingIOError:
                break
            except OSError:
                self.connected = False
                break
            if not data:
                self.connected = False
                break
            for offset in range(0, len(data) - _EVENT.size + 1, _EVENT.size):
                _, value, kind, number = _EVENT.unpack_from(data, offset)
                kind &= ~_TYPE_INIT
                if kind == _TYPE_AXIS:
                    self._raw_axes[number] = value / 32767.0
                elif kind == _TYPE_BUTTON:
                    self._raw_buttons[number] = bool(value)
        return self.state()

    def state(self) -> GamepadState:
        pad = GamepadState()
        if not self.connected:
            return pad
        named = {name: self._raw_axes.get(number, 0.0) for number, name in _AXES.items()}
        pad.axes = [named["lx"], named["ly"], named["rx"], named["ry"]]
        for number, index in _BUTTONS.items():
            pad.buttons[index] = 1.0 if self._raw_buttons.get(number) else 0.0
        # Triggers rest at -1 and reach +1 when pulled; a trigger that has not moved since
        # the controller was opened reports 0.
        for name, index in (("lt", mapper.BTN_LT), ("rt", mapper.BTN_RT)):
            raw = self._raw_axes.get({"lt": 2, "rt": 5}[name])
            pad.buttons[index] = 0.0 if raw is None else (raw + 1.0) / 2.0
        pad.buttons[mapper.BTN_LEFT] = 1.0 if named["dpad_x"] < -0.5 else 0.0
        pad.buttons[mapper.BTN_RIGHT] = 1.0 if named["dpad_x"] > 0.5 else 0.0
        pad.buttons[mapper.BTN_UP] = 1.0 if named["dpad_y"] < -0.5 else 0.0
        pad.buttons[mapper.BTN_DOWN] = 1.0 if named["dpad_y"] > 0.5 else 0.0
        return pad
