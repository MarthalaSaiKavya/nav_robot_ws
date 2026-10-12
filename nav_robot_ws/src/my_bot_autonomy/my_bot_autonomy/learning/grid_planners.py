"""
Grid path planners: Dijkstra, A*, greedy best-first - the family Nav2's NavFn and Smac2D belong to.

ALL THREE are the same loop with a different PRIORITY for which cell to expand next:
    Dijkstra : g(n)                 cost so far              -> optimal, explores in all directions
    A*       : g(n) + h(n)          cost so far + estimate   -> optimal (h admissible), far fewer cells
    greedy   : h(n)                 estimate only            -> very fast, NOT optimal (hugs walls)
  h(n) = straight-line (Euclidean) distance to the goal.

COSTMAP-STYLE INFLATION (the key idea of Nav2 costmaps):
  * cells closer than robot_radius to an obstacle are LETHAL (the robot centre cannot go there)
  * cells a bit further away get an extra cost that decays with distance, so paths prefer
    the middle of corridors instead of scraping walls:
        cost = 1 + inflation_weight * exp(-decay * (d - robot_radius))
  This is the same exponential shape as nav2_costmap_2d's InflationLayer (cost_scaling_factor).
"""
import heapq
import math

import numpy as np

from .distance_transform import distance_transform

ALGORITHMS = ('astar', 'dijkstra', 'greedy')
SQRT2 = math.sqrt(2.0)


def build_cost_grid(grid, resolution, robot_radius=0.24, inflation_weight=5.0, decay=3.0,
                    allow_unknown=True):
    occ = np.asarray(grid) >= 65
    unknown = np.asarray(grid) < 0
    d = distance_transform(occ, resolution)
    cost = 1.0 + inflation_weight * np.exp(-decay * np.maximum(d - robot_radius, 0.0))
    cost[d < robot_radius] = np.inf
    if not allow_unknown:
        cost[unknown] = np.inf
    return cost


def plan(cost, start, goal, algorithm='astar'):
    """start/goal: (row, col). Returns (path [(r, c), ...] or None, number of expanded cells, expanded mask)."""
    if algorithm not in ALGORITHMS:
        raise ValueError(f'algorithm must be one of {ALGORITHMS}')
    rows, cols = cost.shape
    if not (0 <= goal[0] < rows and 0 <= goal[1] < cols) or not np.isfinite(cost[goal]):
        return None, 0, None
    g = np.full(cost.shape, np.inf)
    parent = {}
    expanded = np.zeros(cost.shape, dtype=bool)
    g[start] = 0.0

    def h(r, c):
        return math.hypot(r - goal[0], c - goal[1])

    def priority(gval, r, c):
        if algorithm == 'dijkstra':
            return gval
        if algorithm == 'astar':
            return gval + h(r, c)
        return h(r, c)              # greedy

    heap = [(priority(0.0, *start), start)]
    steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]
    n_expanded = 0
    while heap:
        _, (r, c) = heapq.heappop(heap)
        if expanded[r, c]:
            continue
        expanded[r, c] = True
        n_expanded += 1
        if (r, c) == goal:
            path = [(r, c)]
            while path[-1] in parent:
                path.append(parent[path[-1]])
            return path[::-1], n_expanded, expanded
        for dr, dc, step in steps:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols) or expanded[nr, nc]:
                continue
            cval = cost[nr, nc]
            if not np.isfinite(cval):
                continue
            new_g = g[r, c] + step * cval
            if new_g < g[nr, nc]:
                g[nr, nc] = new_g
                parent[(nr, nc)] = (r, c)
                heapq.heappush(heap, (priority(new_g, nr, nc), (nr, nc)))
    return None, n_expanded, expanded


def nearest_free(cost, cell, max_radius=20):
    """If the start/goal is inside an inflated obstacle, move it to the nearest valid cell."""
    if np.isfinite(cost[cell]):
        return cell
    r0, c0 = cell
    for rad in range(1, max_radius + 1):
        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                r, c = r0 + dr, c0 + dc
                if 0 <= r < cost.shape[0] and 0 <= c < cost.shape[1] and np.isfinite(cost[r, c]):
                    return (r, c)
    return None


def path_length(path, resolution):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path[:-1], path[1:])) * resolution


def shortcut_smooth(path, cost, iterations=1, safe_cost=4.0, end_cells=6):
    """Line-of-sight smoothing: drop waypoints you can see past (like Theta* / Nav2 smoothers).

    A shortcut is accepted only if the cells on it are "comfortable": cost <= safe_cost
    (end_cells cells at both ends are exempt, so a start/goal next to a wall still works).
    safe_cost=4.0 ~ 0.15 m more clearance than the robot radius with build_cost_grid's defaults
    (1 + 5 * exp(-3 * 0.15) = 4.2). Where no comfortable shortcut exists (a narrow door) the
    original A* cells are kept.
    Checking only "no LETHAL cell on the line" makes shortcuts scrape corners at exactly the
    robot radius - the robot then stops in front of the corner because its bumper check fires."""
    def line_cells(a, b):
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
        rs = np.linspace(a[0], b[0], n).round().astype(int)
        cs = np.linspace(a[1], b[1], n).round().astype(int)
        return rs, cs

    for _ in range(iterations):
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                rs, cs = line_cells(path[i], path[j])
                line = cost[rs, cs]
                k = min(end_cells, len(line) // 3)
                inner = line[k:len(line) - k]
                if np.all(np.isfinite(line)) and np.all(inner <= safe_cost):
                    break
                j -= 1
            out.append(path[j])
            i = j
        path = out
    return path
