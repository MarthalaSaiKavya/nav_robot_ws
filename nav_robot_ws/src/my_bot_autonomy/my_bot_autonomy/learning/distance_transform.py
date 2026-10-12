"""
Exact Euclidean Distance Transform (EDT) - "how far is every cell from the nearest wall?"

Used for:
  * the particle filter's LIKELIHOOD FIELD: a laser endpoint that lands 2 cm from a
    wall in the map is very likely, one that lands 50 cm away is unlikely.
    (AMCL's default laser_model_type "likelihood_field" is exactly this.)
  * costmap INFLATION: cost decreases with distance from obstacles.
  * map comparison: how far are SLAM walls from the true walls?

Algorithm: Felzenszwalb & Huttenlocher (2012), "Distance Transforms of Sampled Functions".
The 2D squared EDT is SEPARABLE: do a 1D transform on every column, then on every row.
The 1D transform computes, for every position q,  min_p ( (q - p)^2 + f(p) )
= the lower envelope of parabolas rooted at each p. O(n) per line.
"""
import numpy as np

INF = 1e20


def _edt_1d(f):
    """Squared distance transform of a 1D sampled function f (0 at features, INF elsewhere)."""
    n = len(f)
    d = np.empty(n)
    v = np.zeros(n, dtype=np.int64)      # locations of parabolas in the lower envelope
    z = np.empty(n + 1)                  # boundaries between parabolas
    k = 0
    v[0] = 0
    z[0] = -INF
    z[1] = INF
    for q in range(1, n):
        # intersection of the parabola from q with the rightmost envelope parabola
        s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
        while s <= z[k]:
            k -= 1
            s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
        k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = INF
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        d[q] = (q - v[k]) ** 2 + f[v[k]]
    return d


def distance_transform(obstacle_mask, resolution=1.0):
    """obstacle_mask: bool array (True = obstacle). Returns distance [same units as resolution]
    from each cell to the nearest obstacle cell. If there are no obstacles, returns INF everywhere."""
    mask = np.asarray(obstacle_mask, dtype=bool)
    if not mask.any():
        return np.full(mask.shape, np.inf)
    f = np.where(mask, 0.0, INF)
    # pass 1: columns
    tmp = np.empty_like(f)
    for c in range(f.shape[1]):
        tmp[:, c] = _edt_1d(f[:, c])
    # pass 2: rows
    out = np.empty_like(f)
    for r in range(f.shape[0]):
        out[r, :] = _edt_1d(tmp[r, :])
    return np.sqrt(out) * resolution


def brushfire_distance(obstacle_mask, resolution=1.0, max_cells=None):
    """The simpler, approximate alternative: BFS "wavefront" from all obstacles (8-connected,
    diagonal steps cost sqrt(2)). Shown for comparison - this is how many textbooks
    introduce inflation. Slower in Python and slightly inexact, but very intuitive."""
    import heapq
    mask = np.asarray(obstacle_mask, dtype=bool)
    dist = np.full(mask.shape, np.inf)
    heap = []
    for r, c in zip(*np.nonzero(mask)):
        dist[r, c] = 0.0
        heap.append((0.0, int(r), int(c)))
    heapq.heapify(heap)
    rows, cols = mask.shape
    steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, 2 ** 0.5), (-1, 1, 2 ** 0.5), (1, -1, 2 ** 0.5), (1, 1, 2 ** 0.5)]
    while heap:
        d, r, c = heapq.heappop(heap)
        if d > dist[r, c] or (max_cells is not None and d > max_cells):
            continue
        for dr, dc, w in steps:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and d + w < dist[nr, nc]:
                dist[nr, nc] = d + w
                heapq.heappush(heap, (d + w, nr, nc))
    return dist * resolution
