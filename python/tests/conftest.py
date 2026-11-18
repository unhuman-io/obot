"""Fixtures: a real obot-rtd on simulated actuators for every test that wants a robot."""

from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path

import pytest

from obot import NotRunning, Robot

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / "config" / "obot3.toml"


def bin_dir() -> Path:
    candidates = [os.environ.get("OBOT_RT_BIN_DIR"), Path.home() / ".cache/obot-build/release", REPO / "rt/build"]
    for candidate in candidates:
        if candidate and (Path(candidate) / "obot-rtd").exists():
            return Path(candidate)
    pytest.skip("obot-rtd is not built (set OBOT_RT_BIN_DIR)")


def start_daemon(name: str, *extra: str, config: Path = CONFIG) -> subprocess.Popen:
    process = subprocess.Popen(
        [str(bin_dir() / "obot-rtd"), "--config", str(config), "--no-rt", "--shm-name", name, *extra],
        stderr=subprocess.PIPE, text=True,
    )
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"obot-rtd exited: {process.stderr.read()}")
        try:
            Robot(name).close()
            return process
        except NotRunning:
            time.sleep(0.02)
    process.kill()
    raise RuntimeError("obot-rtd did not publish its shared memory")


def stop_daemon(process: subprocess.Popen) -> None:
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()


@pytest.fixture
def shm_name(request) -> str:
    return f"obot_pytest_{os.getpid()}_{abs(hash(request.node.name)) % 100000}"


@pytest.fixture
def daemon(shm_name):
    process = start_daemon(shm_name, "--sim")
    yield shm_name
    stop_daemon(process)


@pytest.fixture
def robot(daemon):
    with Robot(daemon) as r:
        assert r.wait_mode(1, timeout=3) is not None, "robot did not reach idle"
        yield r
