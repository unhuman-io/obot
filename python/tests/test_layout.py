import json
import subprocess

import numpy as np
from conftest import bin_dir

from obot import layout


def test_python_layout_matches_the_daemon():
    out = subprocess.run([str(bin_dir() / "obot-rtd"), "--dump-layout"], check=True, capture_output=True, text=True)
    assert layout.differences(json.loads(out.stdout)) == []


def test_differences_notices_a_change():
    out = subprocess.run([str(bin_dir() / "obot-rtd"), "--dump-layout"], check=True, capture_output=True, text=True)
    cpp = json.loads(out.stdout)
    cpp["RobotState"]["fields"][5][1] += 4
    cpp["layout_version"] += 1
    problems = layout.differences(cpp)
    assert any("layout_version" in p for p in problems)
    assert any("RobotState.warning" in p for p in problems)


def test_records_have_no_padding():
    for dtype in (layout.HEADER, layout.INFO, layout.STATE, layout.COMMAND):
        assert sum(dtype.fields[name][0].itemsize for name in dtype.names) == dtype.itemsize
    assert layout.STATE_SLOT.itemsize == 8 + layout.STATE.itemsize
    assert np.dtype(layout.COMMAND_SLOT).itemsize == 8 + layout.COMMAND.itemsize
