"""
Occupancy-grid mapping WITH KNOWN POSES, using log-odds (Thrun, Probabilistic Robotics ch. 9).

This is the "mapping" half of SLAM. SLAM = this + estimating the poses at the same time.

LOG-ODDS IN 30 SECONDS
  Each cell stores l = log( p(occupied) / p(free) ).   l = 0  <->  p = 0.5 (unknown)
  Every time a laser beam passes THROUGH a cell, the cell is probably free:   l += l_free  (< 0)
  Every time a beam ENDS in a cell, the cell is probably occupied:            l += l_occ   (> 0)
  Adding is the Bayes update in log space - cheap, and order does not matter.
  Clamping l to [l_min, l_max] keeps the map able to change its mind (moving obstacles).
  Convert back with p = 1 - 1 / (1 + exp(l)).

Experiment (learning_mapper node, pose_source:=odom vs ground_truth): with drifting
odometry as the pose, walls get smeared/doubled. That picture is WHY SLAM exists.
"""
import math

import numpy as np


def bresenham(r0, c0, r1, c1):
    """Integer cells on the straight line from (r0, c0) to (r1, c1) - classic Bresenham.
    (Kept for teaching/tests; LogOddsMap.update_scan uses a vectorised equivalent.)"""
    cells = []
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr, sc = (1 if r1 > r0 else -1), (1 if c1 > c0 else -1)
    err = dc - dr
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr
    return cells


class LogOddsMap:
    def __init__(self, size_x=12.0, size_y=12.0, resolution=0.05, origin=(-6.0, -6.0),
                 l_occ=0.85, l_free=-0.4, l_min=-4.0, l_max=4.0):
        self.res = resolution
        self.origin = (origin[0], origin[1], 0.0)
        self.cols = int(round(size_x / resolution))
        self.rows = int(round(size_y / resolution))
        self.l = np.zeros((self.rows, self.cols), dtype=np.float32)
        self.l_occ, self.l_free, self.l_min, self.l_max = l_occ, l_free, l_min, l_max

    def _to_cells(self, x, y):
        c = np.floor((x - self.origin[0]) / self.res).astype(np.int64)
        r = np.floor((y - self.origin[1]) / self.res).astype(np.int64)
        return r, c

    def update_scan(self, sensor_pose, ranges, angles, max_range, hit_max_as_free=True):
        """Integrate one scan taken from sensor_pose = (x, y, yaw) in the map frame."""
        sx, sy, syaw = sensor_pose
        ranges = np.asarray(ranges, dtype=float)
        angles = np.asarray(angles, dtype=float) + syaw
        valid = np.isfinite(ranges) & (ranges > 0)
        hit = valid & (ranges < max_range)
        # Beams with no return ("inf") still tell us the space up to max_range is free.
        free_len = np.where(valid, np.minimum(ranges, max_range),
                            max_range if hit_max_as_free else 0.0)
        # --- free cells: sample every beam every half-cell up to (but excluding) its endpoint
        step = self.res * 0.5
        n_steps = int(math.ceil(max_range / step))
        t = (np.arange(n_steps) * step)[None, :]                 # (1, S)
        inside = t < (free_len[:, None] - self.res)               # (N, S) stop one cell before the hit
        fx = sx + t * np.cos(angles)[:, None]
        fy = sy + t * np.sin(angles)[:, None]
        fr, fc = self._to_cells(fx[inside], fy[inside])
        ok = (fr >= 0) & (fr < self.rows) & (fc >= 0) & (fc < self.cols)
        free_idx = np.unique(fr[ok] * self.cols + fc[ok])       # each cell updated once per scan
        flat = self.l.reshape(-1)                                 # a VIEW: editing it edits self.l
        flat[free_idx] += self.l_free
        # --- occupied cells: the beam endpoints
        ex = sx + ranges[hit] * np.cos(angles[hit])
        ey = sy + ranges[hit] * np.sin(angles[hit])
        er, ec = self._to_cells(ex, ey)
        ok = (er >= 0) & (er < self.rows) & (ec >= 0) & (ec < self.cols)
        occ_idx = np.unique(er[ok] * self.cols + ec[ok])
        flat[occ_idx] += self.l_occ
        np.clip(self.l, self.l_min, self.l_max, out=self.l)

    def probability(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.l))

    def to_occupancy(self, unknown_band=0.1):
        """-1 / 0..100 grid for nav_msgs/OccupancyGrid. Cells never touched stay -1."""
        p = self.probability()
        grid = np.round(p * 100).astype(np.int8)
        grid[np.abs(self.l) < unknown_band] = -1
        return grid
