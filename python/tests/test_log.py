import numpy as np
import pytest

pytest.importorskip("mcap")
pytest.importorskip("google.protobuf")

from obot import log  # noqa: E402


def test_record_and_read_back(robot, tmp_path):
    path = tmp_path / "run.mcap"
    assert log.record(robot, str(path), duration=0.5) == 0
    states, names = log.read_states(str(path))
    assert names == list(robot.info.joint_names)
    assert len(states) > 20
    assert np.all(np.diff(states["cycle"].astype(np.int64)) >= 1)
    assert np.all(states["mode"] == 1)
    assert np.all(states["num_joints"] == 6)
    assert states["temperature"][-1][4] == pytest.approx(30.0)     # the simulated wheel drive

    # Any MCAP tool can read it: the schema travels with the file.
    from mcap.reader import make_reader

    with open(path, "rb") as f:
        summary = make_reader(f).get_summary()
    schema = next(iter(summary.schemas.values()))
    assert schema.name == "obot.RobotState"
    assert schema.encoding == "protobuf"


def test_decimation(robot, tmp_path):
    path = tmp_path / "slow.mcap"
    log.record(robot, str(path), duration=0.4, decimate=10)
    states, _ = log.read_states(str(path))
    assert len(states) > 2
    assert np.all(states["cycle"] % 10 == 0)
