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
    # Where the camera is, for the 3D world view: the pose of its body frame (x along the
    # view, y left, z up) in a link of the robot model. No link: the base, whose origin is
    # on the floor between the wheels.
    link: str | None = None
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Horizontal field of view in degrees. The defaults are a RealSense D435's 4:3 modes
    # (640x480): 79.6 for depth, 55 for colour. Its 16:9 modes: 87 and 69.
    hfov: float | None = None
    color: str | None = None  # depth cameras: the camera whose picture colours the 3D points
    # How old a picture is when it arrives, s: the robot's pose that long ago is used.
    latency: float | None = None

    @property
    def is_depth(self) -> bool:
        return self.source.startswith("v4l2depth:") or self.source.split(":")[-1] == "depth"

    @property
    def drawn_by_page(self) -> bool:
        return self.source.startswith("sim:")

    def intrinsics(self):
        from .world import Intrinsics

        return Intrinsics.from_hfov(self.width, self.height, self.hfov or (79.6 if self.is_depth else 55.0))


@dataclass
class WorldConfig:
    """The 3D map built from the depth cameras ([world] in the configuration)."""

    enabled: bool = True
    voxel: float = 0.03  # edge of a map cell, m
    range: float = 3.5  # readings farther than this are not used, m
    near: float = 0.2
    max_voxels: int = 250_000
    stride: int = 4  # every n-th pixel of a depth picture in each direction
    rate: float = 5.0  # map updates per second and camera
    max_turn_rate: float = 1.0  # rad/s; while the base turns faster, pictures are not used
    # Correct the wheel odometry with what the depth cameras on the base see. Experimental:
    # on the one robot it was tried on, it did worse than the wheels alone.
    correct_odometry: bool = False


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
    cartesian_speed: float = 0.15  # tool speed at full stick in Cartesian mode, m/s
    pitch_speed: float = 0.8  # tool pitch rate in Cartesian mode, rad/s


@dataclass
class ArmConfig:
    joints: list[str]
    lengths: list[float]
    zero: list[float]
    roll: str | None = None


@dataclass
class ToolsConfig:
    shm_name: str = "obot"
    base_v_max: float = 0.5
    base_omega_max: float = 1.5
    cameras: list[CameraConfig] = field(default_factory=list)
    teleop: TeleopConfig = field(default_factory=TeleopConfig)
    arm: ArmConfig | None = None
    urdf: str | None = None  # robot model for the operator page, [model] urdf = "..."
    world: WorldConfig = field(default_factory=WorldConfig)


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
        known = {k: entry[k] for k in ("name", "source", "width", "height", "fps", "quality", "link", "hfov",
                                       "color", "latency") if k in entry}
        for key in ("xyz", "rpy"):
            if key in entry:
                if len(entry[key]) != 3:
                    raise ValueError(f"{found}: camera {entry.get('name')}: {key} needs three numbers")
                known[key] = tuple(float(v) for v in entry[key])
        config.cameras.append(CameraConfig(**known))
    names = [c.name for c in config.cameras]
    for camera in config.cameras:
        if camera.color is not None and camera.color not in names:
            raise ValueError(f"{found}: camera {camera.name}: color = \"{camera.color}\" is not a camera")
    world = doc.get("world", {})
    for key, kind in (("enabled", bool), ("voxel", float), ("range", float), ("near", float), ("max_voxels", int),
                      ("stride", int), ("rate", float), ("max_turn_rate", float), ("correct_odometry", bool)):
        if key in world:
            setattr(config.world, key, kind(world[key]))
    arm = doc.get("arm")
    if arm and arm.get("type") == "planar3r":
        config.arm = ArmConfig(list(arm["joints"]), [float(v) for v in arm["lengths"]],
                               [float(v) for v in arm["zero"]], arm.get("roll"))
    model = doc.get("model", {})
    if "urdf" in model:
        config.urdf = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(found)), model["urdf"]))
    teleop = doc.get("teleop", {})
    for key in ("base_speed", "arm_speed", "deadband", "max_lead", "controller_timeout_s", "cartesian_speed",
                "pitch_speed"):
        if key in teleop:
            setattr(config.teleop, key, float(teleop[key]))
    return config
