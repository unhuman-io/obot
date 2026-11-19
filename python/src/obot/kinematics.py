"""Analytic kinematics of an arm whose first three joints pitch in one plane.

The tool pose in that plane is (x, z, pitch): x forward and z up from the first joint,
pitch the tool's direction measured from forward towards up. Link i points along
`zero[i] + q[0] + ... + q[i]`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class Planar3R:
    lengths: tuple[float, float, float]
    zero: tuple[float, float, float]
    q_min: tuple[float, float, float]
    q_max: tuple[float, float, float]

    def fk(self, q) -> tuple[float, float, float]:
        x = z = 0.0
        total = 0.0
        angle = 0.0
        for length, zero, qi in zip(self.lengths, self.zero, q, strict=True):
            total += qi
            angle = zero + total
            x += length * math.cos(angle)
            z += length * math.sin(angle)
        return x, z, angle

    def within_limits(self, q, margin: float = 1e-6) -> bool:
        return all(lo - margin <= qi <= hi + margin for qi, lo, hi in zip(q, self.q_min, self.q_max, strict=True))

    def ik(self, x: float, z: float, pitch: float, near=None) -> tuple[float, float, float] | None:
        """Joint angles for a tool pose, or None if it is out of reach or outside the joint
        limits. Of the two elbow configurations, the one inside the limits and closest to
        `near` is returned."""
        l1, l2, l3 = self.lengths
        wx = x - l3 * math.cos(pitch)
        wz = z - l3 * math.sin(pitch)
        c = (wx * wx + wz * wz - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        if abs(c) > 1.0:
            return None
        best = None
        for bend in (math.acos(c), -math.acos(c)):
            a1 = math.atan2(wz, wx) - math.atan2(l2 * math.sin(bend), l1 + l2 * math.cos(bend))
            angles = (a1, a1 + bend, pitch)
            q = []
            total = 0.0
            for angle, zero, lo, hi in zip(angles, self.zero, self.q_min, self.q_max, strict=True):
                qi = _wrap(angle - zero - total)
                # The same direction a turn away may be the one inside the limits.
                for candidate in (qi, qi + 2 * math.pi, qi - 2 * math.pi):
                    if lo <= candidate <= hi:
                        qi = candidate
                        break
                q.append(qi)
                total += qi
            if not self.within_limits(q):
                continue
            cost = 0.0 if near is None else sum((a - b) ** 2 for a, b in zip(q, near, strict=True))
            if best is None or cost < best[0]:
                best = (cost, tuple(q))
        return best[1] if best else None


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi
