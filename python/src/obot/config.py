"""The parts of the robot's TOML configuration that the operator tools use.

obot-rtd reads the same file for everything that concerns the actuators; the [[camera]]
and [teleop] sections are only read here.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field

try:
    import tomllib
except ModuleNotFoundError:  # Python 3.10
    import tomli as tomllib

DEFAULT_PATHS = ("/etc/obot/robot.toml",)


@dataclass
class CameraConfig:
    name: str
    source: str
    width: int = 640
    height: int = 480
    fps: int = 15
    quality: int = 70  # JPEG quality of the stream


@dataclass
class TeleopConfig:
    base_speed: float = 0.6  # fraction of the base's v_max / omega_max at full stick, before the boost trigger
    arm_speed: float = 0.5  # fraction of each joint's qd_max at full stick
    deadband: float = 0.12
    max_lead: float = 0.25  # how far a jogged target may run ahead of the joint, rad
    # A controller whose state has not changed at all for this long while something is held
    # is treated as released (a wireless controller that dropped out keeps reporting its last
    # state until the OS notices). Costs a re-press on long steady moves; 0 disables.
    controller_timeout_s: float = 5.0


@dataclass
class ToolsConfig:
    shm_name: str = "obot"
    base_v_max: float = 0.5
    base_omega_max: float = 1.5
    cameras: list[CameraConfig] = field(default_factory=list)
    teleop: TeleopConfig = field(default_factory=TeleopConfig)


def find_config(path: str | None = None) -> str | None:
    """The configuration file to use: the argument, $OBOT_CONFIG, or the installed one."""
    for candidate in (path, os.environ.get("OBOT_CONFIG"), *DEFAULT_PATHS):
        if candidate and os.path.exists(candidate):
            return candidate
    return None


def load(path: str | None = None) -> ToolsConfig:
    found = find_config(path)
    if path and not found:
        raise FileNotFoundError(path)
    config = ToolsConfig()
    if not found:
        return config
    with open(found, "rb") as f:
        doc = tomllib.load(f)
    robot = doc.get("robot", {})
    config.shm_name = robot.get("shm_name", "obot")
    base = doc.get("base", {})
    config.base_v_max = float(base.get("v_max", config.base_v_max))
    config.base_omega_max = float(base.get("omega_max", config.base_omega_max))
    for entry in doc.get("camera", []):
        known = {k: entry[k] for k in ("name", "source", "width", "height", "fps", "quality") if k in entry}
        config.cameras.append(CameraConfig(**known))
    teleop = doc.get("teleop", {})
    for key in ("base_speed", "arm_speed", "deadband", "max_lead", "controller_timeout_s"):
        if key in teleop:
            setattr(config.teleop, key, float(teleop[key]))
    return config
