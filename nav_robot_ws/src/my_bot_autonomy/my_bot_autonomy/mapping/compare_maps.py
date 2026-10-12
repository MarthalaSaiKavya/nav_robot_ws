"""
compare_maps - put a NUMBER on map quality (Phase 3 verification).

    ros2 run my_bot_autonomy compare_maps --map ~/maps/room.yaml --world nav_room.sdf
    ros2 run my_bot_autonomy compare_maps --map a.yaml --reference b.yaml

Metrics (both maps are in the same frame because map == world in this project):
  wall_error_mean / p95   For every OCCUPIED cell of your map, the distance to the nearest
                          TRUE wall. A crisp SLAM map: mean < 3 cm, p95 < 10 cm.
                          A smeared/ghosted map (bad odometry, no loop closure): much larger.
  ghost_wall_ratio        Fraction of your occupied cells > 15 cm from any true wall
                          (doubled walls, phantom obstacles). Should be < 2 %.
  wall_recall             Fraction of TRUE wall cells that your map marked occupied
                          within 10 cm. Low = parts of the room were never seen.
  coverage                Fraction of TRUE free space your map marked free.
  -> PASS if wall_error_p95 <= 0.10 and ghost_wall_ratio <= 0.02 and coverage >= 0.90
"""
import argparse
import json

import numpy as np

from ..common.map_io import FREE, OCCUPIED, read_map
from ..learning.distance_transform import distance_transform
from ..mapping.reference_map import build_reference_grid, resolve_world_path
from ..common.world_model import World2D


def _resample(grid, res, origin, xs, ys):
    """Look up grid values at world points (nearest cell). Outside = unknown (-1)."""
    cols = np.floor((xs - origin[0]) / res).astype(int)
    rows = np.floor((ys - origin[1]) / res).astype(int)
    inside = (rows >= 0) & (rows < grid.shape[0]) & (cols >= 0) & (cols < grid.shape[1])
    out = np.full(xs.shape, -1, dtype=np.int16)
    out[inside] = grid[rows[inside], cols[inside]]
    return out


def compare(map_grid, map_res, map_origin, ref_grid, ref_res, ref_origin, tol=0.10, ghost=0.15):
    # distance from every reference cell to the nearest TRUE wall
    ref_dist = distance_transform(ref_grid == OCCUPIED, ref_res)
    # world coordinates of every cell of the evaluated map
    rows, cols = np.indices(map_grid.shape)
    mx = map_origin[0] + (cols + 0.5) * map_res
    my = map_origin[1] + (rows + 0.5) * map_res
    occ = map_grid == OCCUPIED
    # distance of each evaluated occupied cell to a true wall (looked up in the ref EDT)
    rr = np.floor((my[occ] - ref_origin[1]) / ref_res).astype(int)
    rc = np.floor((mx[occ] - ref_origin[0]) / ref_res).astype(int)
    valid = (rr >= 0) & (rr < ref_grid.shape[0]) & (rc >= 0) & (rc < ref_grid.shape[1])
    d = np.full(rr.shape, 1.0)
    d[valid] = ref_dist[rr[valid], rc[valid]]
    metrics = {
        'occupied_cells': int(occ.sum()),
        'wall_error_mean': float(d.mean()) if d.size else float('nan'),
        'wall_error_p95': float(np.percentile(d, 95)) if d.size else float('nan'),
        'ghost_wall_ratio': float((d > ghost).mean()) if d.size else float('nan'),
    }
    # recall + coverage: sample the evaluated map at reference cell centres
    rrows, rcols = np.indices(ref_grid.shape)
    rx = ref_origin[0] + (rcols + 0.5) * ref_res
    ry = ref_origin[1] + (rrows + 0.5) * ref_res
    eval_at_ref = _resample(map_grid, map_res, map_origin, rx, ry)
    map_dist = distance_transform(map_grid == OCCUPIED, map_res)
    near_map_wall = _resample(np.round(map_dist / map_res).astype(np.int32), map_res, map_origin, rx, ry)
    # Only wall SURFACES (occupied cells touching free space) can be seen by a LiDAR,
    # so recall is measured on those, not on the inside of thick walls.
    free_ref = ref_grid == FREE
    touch_free = np.zeros_like(free_ref)
    touch_free[1:, :] |= free_ref[:-1, :]
    touch_free[:-1, :] |= free_ref[1:, :]
    touch_free[:, 1:] |= free_ref[:, :-1]
    touch_free[:, :-1] |= free_ref[:, 1:]
    true_walls = (ref_grid == OCCUPIED) & touch_free
    hit = (near_map_wall >= 0) & (near_map_wall * map_res <= tol)
    metrics['wall_recall'] = float((hit & true_walls).sum() / max(true_walls.sum(), 1))
    true_free = ref_grid == FREE
    metrics['coverage'] = float(((eval_at_ref == FREE) & true_free).sum() / max(true_free.sum(), 1))
    metrics['pass'] = bool(metrics['wall_error_p95'] <= tol and metrics['ghost_wall_ratio'] <= 0.02
                           and metrics['coverage'] >= 0.90)
    return metrics


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', required=True, help='map yaml to evaluate (e.g. from SLAM)')
    g = ap.add_mutually_exclusive_group()
    g.add_argument('--reference', help='reference map yaml')
    g.add_argument('--world', default='nav_room.sdf', help='world SDF to build the reference from')
    ap.add_argument('--json', help='also write the metrics to this json file')
    args, _ = ap.parse_known_args(argv)

    m_grid, m_res, m_origin = read_map(args.map)
    if args.reference:
        r_grid, r_res, r_origin = read_map(args.reference)
    else:
        r_grid, r_res, r_origin = build_reference_grid(World2D.from_sdf_file(resolve_world_path(args.world)))
    metrics = compare(m_grid, m_res, m_origin, r_grid, r_res, r_origin)
    print('\nMap quality report for', args.map)
    for k, v in metrics.items():
        print(f'  {k:20s} {v:.4f}' if isinstance(v, float) else f'  {k:20s} {v}')
    print('\n  RESULT:', 'PASS' if metrics['pass'] else 'needs work (see docstring for thresholds)')
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(metrics, f, indent=2)
    return None if metrics else 1


if __name__ == '__main__':
    main()
