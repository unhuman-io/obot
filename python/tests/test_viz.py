import pytest

pytest.importorskip("rerun")

from obot import viz  # noqa: E402


def test_streaming_to_a_recording_file(robot, tmp_path):
    path = tmp_path / "run.rrd"
    assert viz.run(robot, save=str(path), duration=0.5) == 0
    assert path.stat().st_size > 10_000      # joint series for six joints, half a second
