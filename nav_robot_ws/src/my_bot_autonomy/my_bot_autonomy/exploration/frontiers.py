"""
Frontier detection, clustering and selection - pure numpy, no ROS.

THE IDEA (Yamauchi, "A Frontier-Based Approach for Autonomous Exploration", 1997):
  A FRONTIER is the border between space you KNOW is free and space you have
  never seen. Driving to a frontier necessarily reveals new space. Repeat until
  no frontiers are left -> the whole reachable area is mapped.

      ######################          # = wall    . = free (seen)
      #.....  ? ? ? ? ? ? ?#          ? = unknown
      #.....F ? ? ? ? ? ? ?#          F = frontier cell (free, touches unknown)
      #.....F ? ? ? ? ? ? ?#
      #..R..F ? ? ? ? ? ? ?#          R = robot

PIPELINE (each step is a function below):
  1. classify cells          free / occupied / unknown          (classify)
  2. find frontier cells     free AND has an unknown neighbour  (frontier_mask)
  3. group them              8-connected components             (cluster)
     + cut very long ones into pieces                           (split_cluster)
  4. pick a SAFE goal per cluster (cell with enough clearance from walls)
  5. score every cluster with a STRATEGY and pick the best      (score_frontiers)

STRATEGIES (try them with strategy:=...):
  nearest           go to the closest frontier. Simple, few long drives, but can
                    leave small pockets behind and come back later.
  largest           go to the biggest frontier first. Opens the map quickly, more driving.
  utility           explore_lite's cost:  potential_scale*distance - gain_scale*size
                    (a weighted trade-off; the tuning knobs are the two scales).
  information_gain  unknown area visible around the goal (~sensor footprint) per metre
                    travelled. A simple "next-best-view" approach.
  random            a random frontier - a baseline to compare the others against!
"""
import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np


FREE, OCCUPIED, UNKNOWN = 0, 1, 2
STRATEGIES = ('nearest', 'largest', 'utility', 'information_gain', 'random')


@dataclass
class Frontier:
    cells: np.ndarray                 # (N, 2) array of (row, col)
    size_m: float                     # length of the frontier in metres (~cells * resolution)
    centroid: tuple                   # world (x, y)
    goal: tuple                       # world (x, y) - a safe point to drive to
    clearance: float                  # distance of the goal from the nearest obstacle [m]
    distance: float = 0.0             # from the robot (euclidean or path length)
    gain: float = 0.0                 # unknown area [m^2] around the goal
    score: float = 0.0                # higher = better
    extras: dict = field(default_factory=dict)


def classify(grid, free_threshold=25, occupied_threshold=65):
    """OccupancyGrid values (-1 / 0..100) -> FREE / OCCUPIED / UNKNOWN labels.
    slam_toolbox only writes -1/0/100, but cartographer writes probabilities, so we threshold."""
    grid = np.asarray(grid)
    labels = np.full(grid.shape, UNKNOWN, dtype=np.uint8)
    labels[(grid >= 0) & (grid < free_threshold)] = FREE
    labels[grid >= occupied_threshold] = OCCUPIED
    return labels


def frontier_mask(labels):
    """A free cell with at least one UNKNOWN 4-neighbour."""
    unknown = labels == UNKNOWN
    nb = np.zeros_like(unknown)
    nb[1:, :] |= unknown[:-1, :]
    nb[:-1, :] |= unknown[1:, :]
    nb[:, 1:] |= unknown[:, :-1]
    nb[:, :-1] |= unknown[:, 1:]
    return (labels == FREE) & nb


def cluster(mask):
    """8-connected components of a boolean mask. Returns a list of (N, 2) arrays (row, col)."""
    rows, cols = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    clusters = []
    for r0, c0 in zip(*np.nonzero(mask)):
        if seen[r0, c0]:
            continue
        q = deque([(r0, c0)])
        seen[r0, c0] = True
        members = []
        while q:
            r, c = q.popleft()
            members.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols and mask[nr, nc] and not seen[nr, nc]:
                        seen[nr, nc] = True
                        q.append((nr, nc))
        clusters.append(np.array(members, dtype=int))
    return clusters


def _integral(img):
    """Summed-area table: lets us count cells in any rectangle in O(1)."""
    s = np.zeros((img.shape[0] + 1, img.shape[1] + 1), dtype=np.int64)
    s[1:, 1:] = np.cumsum(np.cumsum(img.astype(np.int64), axis=0), axis=1)
    return s


def _box_count(sat, r, c, half):
    r0, r1 = max(r - half, 0), min(r + half + 1, sat.shape[0] - 1)
    c0, c1 = max(c - half, 0), min(c + half + 1, sat.shape[1] - 1)
    return int(sat[r1, c1] - sat[r0, c1] - sat[r1, c0] + sat[r0, c0])


def clearance_at(cells, occupied_cells, resolution, chunk=2048):
    """Distance [m] from each cell in `cells` (N,2) to the nearest cell in `occupied_cells` (M,2).
    Brute force in numpy chunks: only the few hundred frontier cells are queried, which is
    much cheaper than a full distance transform of the whole map every second."""
    if len(occupied_cells) == 0:
        return np.full(len(cells), np.inf)
    occ = occupied_cells.astype(np.float32)
    out = np.empty(len(cells))
    for i in range(0, len(cells), chunk):
        c = cells[i:i + chunk].astype(np.float32)
        d2 = ((c[:, None, :] - occ[None, :, :]) ** 2).sum(axis=2)
        out[i:i + chunk] = np.sqrt(d2.min(axis=1))
    return out * resolution


def split_cluster(cells, max_cells, iterations=8):
    """Cut one long frontier into roughly equal, spatially compact pieces (k-means).

    WHY: the goal of a frontier is placed near its CENTROID. For a short frontier that is
    fine, but a long curved one has its centroid somewhere useless. Classic example: at the
    very start the whole ring of free space around the robot is ONE frontier, and its
    centroid is ... the robot itself. Splitting gives several pieces, each with a centroid
    that really lies on the border of the known world.
    Init = farthest-point sampling (deterministic), then a few k-means iterations."""
    k = int(math.ceil(len(cells) / float(max_cells)))
    if k <= 1:
        return [cells]
    pts = cells.astype(np.float64)
    centres = [pts[0]]
    for _ in range(k - 1):                       # farthest-point initialisation
        d = np.min([((pts - c) ** 2).sum(axis=1) for c in centres], axis=0)
        centres.append(pts[int(np.argmax(d))])
    centres = np.array(centres)
    for _ in range(iterations):
        assign = np.argmin(((pts[:, None, :] - centres[None, :, :]) ** 2).sum(axis=2), axis=1)
        for j in range(k):
            if np.any(assign == j):
                centres[j] = pts[assign == j].mean(axis=0)
    return [cells[assign == j] for j in range(k) if np.any(assign == j)]


def find_frontiers(grid, resolution, origin, min_size_m=0.4, min_clearance_m=0.30,
                   gain_radius_m=2.0, free_threshold=25, occupied_threshold=65,
                   max_frontier_length_m=3.0):
    """Steps 1-4. Returns a list of Frontier objects (not yet scored).
    max_frontier_length_m: longer frontiers are split into pieces (see split_cluster);
    0 or None = never split (the classic Yamauchi / explore_lite behaviour)."""
    labels = classify(grid, free_threshold, occupied_threshold)
    fmask = frontier_mask(labels)
    if not fmask.any():
        return []
    occupied_cells = np.argwhere(labels == OCCUPIED)
    unknown_sat = _integral(labels == UNKNOWN)
    half = max(1, int(gain_radius_m / resolution))

    pieces = []
    for cells in cluster(fmask):
        if len(cells) * resolution < min_size_m:
            continue                                         # tiny frontiers = noise
        if max_frontier_length_m:
            pieces += split_cluster(cells, max(2, int(max_frontier_length_m / resolution)))
        else:
            pieces.append(cells)

    out = []
    for cells in pieces:
        size_m = len(cells) * resolution
        cr, cc = cells.mean(axis=0)
        centroid = (origin[0] + (cc + 0.5) * resolution, origin[1] + (cr + 0.5) * resolution)
        # Safe goal: among cells with enough clearance, take the one nearest the centroid.
        clear = clearance_at(cells, occupied_cells, resolution)
        ok = clear >= min_clearance_m
        if not ok.any():
            continue                                         # frontier squeezed against a wall
        cand = cells[ok]
        d2 = (cand[:, 0] - cr) ** 2 + (cand[:, 1] - cc) ** 2
        best = int(np.argmin(d2))
        gr, gc = cand[best]
        goal = (origin[0] + (gc + 0.5) * resolution, origin[1] + (gr + 0.5) * resolution)
        gain = _box_count(unknown_sat, gr, gc, half) * resolution * resolution
        out.append(Frontier(cells=cells, size_m=size_m, centroid=centroid, goal=goal,
                            clearance=float(clear[ok][best]), gain=gain))
    return out


def filter_blacklist(frontiers, blacklist, radius=0.6):
    """Drop frontiers whose goal is near a point where navigation already failed."""
    if not blacklist:
        return frontiers
    return [f for f in frontiers
            if all(math.hypot(f.goal[0] - bx, f.goal[1] - by) > radius for bx, by in blacklist)]


def score_frontiers(frontiers, robot_xy, strategy='utility', potential_scale=3.0, gain_scale=1.0,
                    rng=None, distances=None):
    """Step 5: fill f.distance and f.score (higher = better) and return the list sorted best-first.

    distances: optional list of path lengths (e.g. from Nav2's planner) to use instead of
               straight-line distance. Path length is more honest: a frontier behind a wall
               is "close" in a straight line but far to drive to.
    """
    if strategy not in STRATEGIES:
        raise ValueError(f'unknown strategy {strategy!r}, choose from {STRATEGIES}')
    rng = rng or np.random.default_rng()
    for i, f in enumerate(frontiers):
        f.distance = (distances[i] if distances is not None
                      else math.hypot(f.goal[0] - robot_xy[0], f.goal[1] - robot_xy[1]))
        if strategy == 'nearest':
            f.score = -f.distance
        elif strategy == 'largest':
            f.score = f.size_m
        elif strategy == 'utility':
            # explore_lite (m-explore) frontier cost, negated so that higher = better
            f.score = -(potential_scale * f.distance - gain_scale * f.size_m)
        elif strategy == 'information_gain':
            f.score = f.gain / (f.distance + 1.0)
        else:
            f.score = float(rng.random())
    return sorted(frontiers, key=lambda f: f.score, reverse=True)
