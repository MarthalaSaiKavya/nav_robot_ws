"""
Pure pursuit path following - the geometric idea behind Nav2's Regulated Pure Pursuit (RPP).

Imagine holding a carrot on a stick: pick the point on the path that is `lookahead`
metres ahead of the robot, and drive along the circle arc that passes through it.

   In the robot frame, the lookahead point is (lx, ly).  The arc through it has
       curvature  kappa = 2 * ly / L^2         (L = distance to the point)
   so for forward speed v the turn rate is  w = v * kappa.

"Regulated" (what RPP adds, partially reproduced here):
   * slow down near the goal (approach_dist)
   * slow down in sharp turns (high curvature)
   * rotate in place first when the path points behind the robot
"""
import math

import numpy as np


def densify(path, step=0.05):
    """Resample a polyline so consecutive points are at most `step` metres apart.

    Pure pursuit looks for "the closest path POINT" and "the first point >= lookahead away".
    With a sparse path (e.g. 3 corners after shortcut smoothing) the closest point can be
    the START vertex behind the robot - the robot then turns around and chases it.
    Sampling the segments densely makes "closest point" mean "closest spot on the path"."""
    pts = np.asarray(path, dtype=float)
    if len(pts) < 2:
        return pts
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        n = max(1, int(math.ceil(math.hypot(*(b - a)) / step)))
        for k in range(1, n + 1):
            out.append(a + (b - a) * k / n)
    return np.asarray(out)


class PurePursuit:
    def __init__(self, lookahead=0.6, max_v=0.3, max_w=1.5, goal_tolerance=0.15,
                 approach_dist=0.8, rotate_threshold=1.0, min_v=0.05):
        self.lookahead = lookahead
        self.max_v, self.max_w = max_v, max_w
        self.goal_tolerance = goal_tolerance
        self.approach_dist = approach_dist
        self.rotate_threshold = rotate_threshold
        self.min_v = min_v

    def compute(self, pose, path):
        """pose = (x, y, yaw); path = list of (x, y). Returns (v, w, done)."""
        if not path:
            return 0.0, 0.0, True
        x, y, yaw = pose
        pts = densify(path)
        dist_goal = math.hypot(pts[-1, 0] - x, pts[-1, 1] - y)
        if dist_goal < self.goal_tolerance:
            return 0.0, 0.0, True
        # closest path point, then the first point at least `lookahead` away beyond it
        d = np.hypot(pts[:, 0] - x, pts[:, 1] - y)
        i0 = int(np.argmin(d))
        ahead = np.nonzero(d[i0:] >= self.lookahead)[0]
        target = pts[i0 + ahead[0]] if len(ahead) else pts[-1]
        # transform the target into the robot frame
        dx, dy = target[0] - x, target[1] - y
        lx = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        ly = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        angle = math.atan2(ly, lx)
        if abs(angle) > self.rotate_threshold:          # target is behind/side: turn on the spot
            return 0.0, math.copysign(min(self.max_w, 1.2), angle), False
        L2 = lx * lx + ly * ly
        kappa = 2.0 * ly / max(L2, 1e-6)
        v = self.max_v
        v *= min(1.0, dist_goal / self.approach_dist) if dist_goal < self.approach_dist else 1.0
        v /= (1.0 + 0.5 * abs(kappa))                   # regulate speed by curvature
        v = max(v, self.min_v)
        w = max(-self.max_w, min(self.max_w, v * kappa))
        return v, w, False
