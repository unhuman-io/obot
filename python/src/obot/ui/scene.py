"""A small made-up room, seen from wherever a camera is: the server-side counterpart of the
props the operator page draws for its simulated cameras. It gives the `scene:depth` and
`scene:color` camera sources real depth and colour pictures, so the 3D world view and what
feeds it can be tried, and tested, without a depth camera.

The room is a floor at z = 0 and a few boxes, in the frame the odometry is in.
"""

from __future__ import annotations

import numpy as np

from ..world import Intrinsics

# centre (x, y, z), size (x, y, z), colour (r, g, b)
BOXES = (
    ((1.6, 0.9, 0.225), (0.4, 0.6, 0.45), (138, 111, 82)),      # a box
    ((1.3, -0.8, 0.2), (0.32, 0.32, 0.4), (63, 127, 191)),      # a basket
    ((0.9, 0.25, 0.04), (0.34, 0.24, 0.08), (201, 79, 79)),     # a shirt on the floor
    ((1.1, -0.3, 0.03), (0.2, 0.3, 0.06), (79, 175, 106)),      # a sock
    ((4.5, 0.0, 0.75), (0.1, 6.0, 1.5), (216, 212, 204)),       # a wall
)
FLOOR = ((185, 178, 166), (170, 163, 151))                      # half-metre tiles
_rays: dict[Intrinsics, np.ndarray] = {}


def render(camera: np.ndarray, k: Intrinsics, max_depth: float = 10.0) -> tuple[np.ndarray, np.ndarray]:
    """(depth in millimetres as uint16, 0 = nothing within range; colour picture, BGR) for a
    camera whose optical frame has the pose `camera` (4 x 4) in the world."""
    if k not in _rays:
        v, u = np.mgrid[0:k.height, 0:k.width]
        _rays[k] = np.stack([(u - k.cx) / k.fx, (v - k.cy) / k.fy, np.ones_like(u, float)], axis=-1).reshape(-1, 3)
    # The rays are scaled to z = 1 in the optical frame, so the distance along a ray *is* depth.
    direction = _rays[k] @ camera[:3, :3].T
    origin = camera[:3, 3]
    depth = np.full(len(direction), np.inf)
    rgb = np.zeros((len(direction), 3), np.uint8)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = -origin[2] / direction[:, 2]
        hit = (direction[:, 2] < 0) & (t > 0)
        depth[hit] = t[hit]
        where = origin[:2] + direction[hit, :2] * t[hit, None]
        tile = (np.floor(where[:, 0] * 2) + np.floor(where[:, 1] * 2)).astype(np.int64) & 1
        rgb[hit] = np.asarray(FLOOR, np.uint8)[tile]
        for centre, size, colour in BOXES:
            low = (np.subtract(centre, np.divide(size, 2)) - origin) / direction
            high = (np.add(centre, np.divide(size, 2)) - origin) / direction
            enter = np.nanmax(np.minimum(low, high), axis=1)
            leave = np.nanmin(np.maximum(low, high), axis=1)
            hit = (enter < leave) & (enter > 0) & (enter < depth)
            depth[hit] = enter[hit]
            rgb[hit] = colour
    seen = depth < max_depth
    depth_mm = np.where(seen, depth * 1000, 0).astype(np.uint16).reshape(k.height, k.width)
    rgb[~seen] = (207, 216, 227)  # sky
    return depth_mm, rgb[:, ::-1].reshape(k.height, k.width, 3).copy()
