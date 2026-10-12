"""
Monte-Carlo Localization (MCL) - a particle filter, i.e. the algorithm inside AMCL.

THE IDEA
  Keep N guesses ("particles") of the robot pose. Repeat:
    1. PREDICT  - when odometry says "I moved 0.25 m and turned 10 deg", move EVERY particle
                  by that amount PLUS random noise (odometry is not trusted blindly).
    2. WEIGHT   - for every particle, ask: "if I were standing HERE, would the current laser
                  scan line up with the walls of the map?"  Good match -> high weight.
    3. RESAMPLE - draw a new set of N particles, picking heavy ones more often.
                  Bad guesses die, good guesses multiply.
  The particle cloud shrinks around the true pose. Its mean is the estimate.

SENSOR MODEL: "likelihood field" (same as AMCL's default laser_model_type)
  Precompute, for every map cell, the distance to the nearest wall (distance_transform).
  A beam endpoint at distance d from a wall gets probability
        z_hit * N(d; 0, sigma_hit) + z_rand / max_range
  Fast (one table lookup per beam) and smooth, unlike ray-casting every beam.

MOTION MODEL: "sample_motion_model_odometry" (Thrun et al., Table 5.6)
  A motion is decomposed into rot1 -> trans -> rot2; noise grows with each part via the
  alpha1..alpha4 parameters (AMCL uses the same names!).

AUGMENTED MCL (recovery from kidnapping): track short- and long-term average weights;
  if the short-term average drops (sudden bad match = robot was moved), inject random particles.
"""
import math

import numpy as np

from .distance_transform import distance_transform


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class ParticleFilter:
    def __init__(self, grid, resolution, origin, num_particles=500,
                 alphas=(0.2, 0.2, 0.2, 0.2), sigma_hit=0.2, z_hit=0.9, z_rand=0.1,
                 max_range=12.0, max_beams=60, alpha_slow=0.001, alpha_fast=0.1, seed=None):
        self.rng = np.random.default_rng(seed)
        self.grid = np.asarray(grid)
        self.res = resolution
        self.origin = origin
        self.n = num_particles
        self.alphas = alphas
        self.z_hit, self.z_rand, self.max_range = z_hit, z_rand, max_range
        self.max_beams = max_beams
        self.alpha_slow, self.alpha_fast = alpha_slow, alpha_fast
        self.w_slow = self.w_fast = 0.0
        # ---- likelihood field: p_hit for every cell ----
        dist = distance_transform(self.grid >= 65, resolution)
        dist = np.minimum(dist, 3 * sigma_hit + 1.0)
        self.field = np.exp(-0.5 * (dist / sigma_hit) ** 2)
        self.free_cells = np.argwhere((self.grid >= 0) & (self.grid < 25))
        self.particles = np.zeros((self.n, 3))
        self.weights = np.full(self.n, 1.0 / self.n)

    # ----------------------------------------------------------------- init
    def init_gaussian(self, x, y, yaw, std_xy=0.5, std_yaw=0.35):
        self.particles[:, 0] = self.rng.normal(x, std_xy, self.n)
        self.particles[:, 1] = self.rng.normal(y, std_xy, self.n)
        self.particles[:, 2] = _wrap(self.rng.normal(yaw, std_yaw, self.n))
        self.weights[:] = 1.0 / self.n

    def init_uniform(self):
        """Global localization: particles spread over ALL free space."""
        self.particles = self._random_particles(self.n)
        self.weights[:] = 1.0 / self.n

    def _random_particles(self, k):
        idx = self.rng.integers(0, len(self.free_cells), k)
        cells = self.free_cells[idx]
        p = np.empty((k, 3))
        p[:, 0] = self.origin[0] + (cells[:, 1] + self.rng.random(k)) * self.res
        p[:, 1] = self.origin[1] + (cells[:, 0] + self.rng.random(k)) * self.res
        p[:, 2] = self.rng.uniform(-np.pi, np.pi, k)
        return p

    # ----------------------------------------------------------------- predict
    def predict(self, odom_prev, odom_now):
        """Move all particles by the motion measured by odometry between two poses."""
        dx, dy = odom_now[0] - odom_prev[0], odom_now[1] - odom_prev[1]
        trans = math.hypot(dx, dy)
        rot1 = _wrap(math.atan2(dy, dx) - odom_prev[2]) if trans > 0.01 else 0.0
        rot2 = _wrap(odom_now[2] - odom_prev[2] - rot1)
        a1, a2, a3, a4 = self.alphas
        n = self.n
        # noise standard deviations grow with the size of each motion component
        r1 = rot1 - self.rng.normal(0, np.sqrt(a1 * rot1 ** 2 + a2 * trans ** 2) + 1e-9, n)
        tr = trans - self.rng.normal(0, np.sqrt(a3 * trans ** 2 + a4 * (rot1 ** 2 + rot2 ** 2)) + 1e-9, n)
        r2 = rot2 - self.rng.normal(0, np.sqrt(a1 * rot2 ** 2 + a2 * trans ** 2) + 1e-9, n)
        th = self.particles[:, 2]
        self.particles[:, 0] += tr * np.cos(th + r1)
        self.particles[:, 1] += tr * np.sin(th + r1)
        self.particles[:, 2] = _wrap(th + r1 + r2)

    # ----------------------------------------------------------------- weight
    def update(self, ranges, angles, lidar_offset=(0.0, 0.0)):
        ranges = np.asarray(ranges, dtype=float)
        angles = np.asarray(angles, dtype=float)
        ok = np.isfinite(ranges) & (ranges < self.max_range)
        idx = np.nonzero(ok)[0]
        if len(idx) < 5:
            return False
        if len(idx) > self.max_beams:                    # subsample beams for speed (like AMCL max_beams)
            idx = idx[np.linspace(0, len(idx) - 1, self.max_beams).astype(int)]
        r, a = ranges[idx], angles[idx]
        px, py, pth = self.particles[:, 0:1], self.particles[:, 1:2], self.particles[:, 2:3]
        # LiDAR position for each particle, then beam endpoints (N particles x B beams)
        lx = px + lidar_offset[0] * np.cos(pth) - lidar_offset[1] * np.sin(pth)
        ly = py + lidar_offset[0] * np.sin(pth) + lidar_offset[1] * np.cos(pth)
        ex = lx + r[None, :] * np.cos(pth + a[None, :])
        ey = ly + r[None, :] * np.sin(pth + a[None, :])
        cols = np.floor((ex - self.origin[0]) / self.res).astype(np.int64)
        rows = np.floor((ey - self.origin[1]) / self.res).astype(np.int64)
        inside = (rows >= 0) & (rows < self.field.shape[0]) & (cols >= 0) & (cols < self.field.shape[1])
        p_hit = np.zeros(ex.shape)
        p_hit[inside] = self.field[rows[inside], cols[inside]]
        p = self.z_hit * p_hit + self.z_rand / self.max_range
        log_w = np.log(p).sum(axis=1)
        log_w -= log_w.max()                             # numerical stability
        w = np.exp(log_w) * self.weights
        total = w.sum()
        if not np.isfinite(total) or total <= 0:
            self.weights[:] = 1.0 / self.n
            return False
        # augmented MCL bookkeeping (average per-beam likelihood)
        w_avg = float(np.exp(np.log(p).mean(axis=1)).mean())
        self.w_slow += self.alpha_slow * (w_avg - self.w_slow)
        self.w_fast += self.alpha_fast * (w_avg - self.w_fast)
        self.weights = w / total
        return True

    # ----------------------------------------------------------------- resample
    def n_eff(self):
        return 1.0 / np.sum(self.weights ** 2)

    def resample(self, force=False):
        """Low-variance (systematic) resampling - one random number, evenly spaced picks."""
        if not force and self.n_eff() > self.n / 2:
            return False
        positions = (self.rng.random() + np.arange(self.n)) / self.n
        idx = np.searchsorted(np.cumsum(self.weights), positions)
        idx = np.minimum(idx, self.n - 1)
        self.particles = self.particles[idx].copy()
        # kidnapping recovery: replace a fraction with random particles
        if self.w_slow > 0:
            p_random = max(0.0, 1.0 - self.w_fast / self.w_slow)
            k = int(self.rng.binomial(self.n, min(p_random, 0.3)))
            if k > 0:
                self.particles[:k] = self._random_particles(k)
        self.weights[:] = 1.0 / self.n
        return True

    # ----------------------------------------------------------------- output
    def estimate(self):
        """Weighted mean pose (circular mean for the angle) and covariance diag."""
        w = self.weights
        x = float(np.sum(w * self.particles[:, 0]))
        y = float(np.sum(w * self.particles[:, 1]))
        yaw = float(math.atan2(np.sum(w * np.sin(self.particles[:, 2])), np.sum(w * np.cos(self.particles[:, 2]))))
        var_x = float(np.sum(w * (self.particles[:, 0] - x) ** 2))
        var_y = float(np.sum(w * (self.particles[:, 1] - y) ** 2))
        var_yaw = float(np.sum(w * _wrap(self.particles[:, 2] - yaw) ** 2))
        return (x, y, yaw), (var_x, var_y, var_yaw)
