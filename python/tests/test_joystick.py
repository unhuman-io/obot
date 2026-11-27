import os
import struct

import pytest

from obot.teleop import mapper as m
from obot.teleop.joystick import Joystick

EVENT = struct.Struct("<IhBB")
BUTTON, AXIS, INIT = 0x01, 0x02, 0x80


@pytest.fixture
def device(tmp_path):
    """A FIFO standing in for /dev/input/js0: the test writes kernel joystick events to it."""
    path = tmp_path / "js0"
    os.mkfifo(path)
    keep_open = os.open(path, os.O_RDONLY | os.O_NONBLOCK)   # lets the writer open without blocking
    writer = os.open(path, os.O_WRONLY)
    joystick = Joystick(str(path))
    os.close(keep_open)
    yield joystick, writer
    joystick.close()
    if writer is not None:
        try:
            os.close(writer)
        except OSError:
            pass


def send(writer, kind, number, value):
    os.write(writer, EVENT.pack(0, value, kind, number))


def test_xbox_events_become_the_standard_layout(device):
    joystick, writer = device
    # What the kernel sends when the device is opened: the resting state of every control.
    for axis in range(8):
        send(writer, AXIS | INIT, axis, -32767 if axis in (2, 5) else 0)
    pad = joystick.poll()
    assert pad.axes == [0, 0, 0, 0]
    assert pad.buttons[m.BTN_LT] == 0 and pad.buttons[m.BTN_RT] == 0
    assert not any(pad.buttons)

    send(writer, AXIS, 1, -32767)       # left stick forward
    send(writer, AXIS, 3, 16384)        # right stick half right
    send(writer, AXIS, 5, 32767)        # right trigger fully pulled
    send(writer, AXIS, 7, -32767)       # D-pad up
    send(writer, AXIS, 6, 32767)        # D-pad right
    send(writer, BUTTON, 4, 1)          # LB
    send(writer, BUTTON, 7, 1)          # Start
    pad = joystick.poll()
    assert pad.axes[m.AXIS_LY] == pytest.approx(-1.0)
    assert pad.axes[m.AXIS_RX] == pytest.approx(0.5, abs=0.01)
    assert pad.buttons[m.BTN_RT] == pytest.approx(1.0)
    assert pad.pressed(m.BTN_UP) and pad.pressed(m.BTN_RIGHT)
    assert not pad.pressed(m.BTN_DOWN) and not pad.pressed(m.BTN_LEFT)
    assert pad.pressed(m.BTN_LB) and pad.pressed(m.BTN_START)
    assert not pad.pressed(m.BTN_A)

    send(writer, BUTTON, 4, 0)
    assert not joystick.poll().pressed(m.BTN_LB)
    assert joystick.connected


def test_a_controller_that_goes_away_reads_as_neutral(device):
    joystick, writer = device
    send(writer, BUTTON, 4, 1)
    send(writer, AXIS, 1, -32767)
    assert joystick.poll().pressed(m.BTN_LB)
    os.close(writer)                    # unplugged
    pad = joystick.poll()
    assert not joystick.connected
    assert pad.axes == [0, 0, 0, 0]
    assert not any(pad.buttons)


def test_missing_device():
    with pytest.raises(FileNotFoundError):
        Joystick("/dev/input/obot-no-such-js")
