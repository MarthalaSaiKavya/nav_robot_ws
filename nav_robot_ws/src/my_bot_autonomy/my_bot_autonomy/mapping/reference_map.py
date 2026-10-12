"""
make_reference_map - build the PERFECT map of a world straight from its SDF file.

    ros2 run my_bot_autonomy make_reference_map --world nav_room.sdf --out ~/maps/reference
    (--world accepts a file name inside my_bot_gazebo/worlds or a full path)

Why is this useful?
  1. Ground truth to MEASURE your SLAM map against (see compare_maps).
  2. Lets you practise Phases 4-7 (AMCL, Nav2, missions) before your SLAM works:
        ros2 launch my_bot_bringup phase5_navigation.launch.py map:=<path>/reference.yaml
     (a reference map for both shipped worlds is already in my_bot_navigation/maps/)

How: every grid cell whose centre lies inside an obstacle footprint (at LiDAR height)
is OCCUPIED. Then a flood fill from the robot start marks every reachable cell FREE.
Everything else (outside the outer walls) stays UNKNOWN - just like a real SLAM map.
"""
import argparse
import os
from collections import deque

import numpy as np

from ..common.map_io import FREE, OCCUPIED, UNKNOWN, write_map
from ..common.world_model import World2D


def resolve_world_path(world):
    if os.path.isfile(world):
        return world
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = os.path.join(get_package_share_directory('my_bot_gazebo'), 'worlds', world)
        if os.path.isfile(cand):
            return cand
    except Exception:
        pass
    raise FileNotFoundError(f'world file not found: {world}')


def build_reference_grid(world, resolution=0.05, margin=0.3, start=(0.0, 0.0), scan_height=0.155):
    xmin, ymin, xmax, ymax = world.bounds()
    xmin, ymin = xmin - margin, ymin - margin
    xmax, ymax = xmax + margin, ymax + margin
    cols = int(np.ceil((xmax - xmin) / resolution))
    rows = int(np.ceil((ymax - ymin) / resolution))
    xs = xmin + (np.arange(cols) + 0.5) * resolution
    ys = ymin + (np.arange(rows) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys)
    occ = world.occupancy(gx, gy, inflate=0.0, scan_height=scan_height)

    grid = np.full((rows, cols), UNKNOWN, dtype=np.int8)
    grid[occ] = OCCUPIED
    # flood fill free space from the start position (4-connected BFS)
    r0 = int((start[1] - ymin) / resolution)
    c0 = int((start[0] - xmin) / resolution)
    if occ[r0, c0]:
        raise ValueError('start position is inside an obstacle')
    seen = np.zeros_like(occ)
    q = deque([(r0, c0)])
    seen[r0, c0] = True
    while q:
        r, c = q.popleft()
        grid[r, c] = FREE
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not seen[nr, nc] and not occ[nr, nc]:
                seen[nr, nc] = True
                q.append((nr, nc))
    return grid, resolution, (xmin, ymin, 0.0)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default='nav_room.sdf')
    ap.add_argument('--out', default=os.path.expanduser('~/maps/reference'))
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--start', type=float, nargs=2, default=[0.0, 0.0])
    args, _ = ap.parse_known_args(argv)
    world = World2D.from_sdf_file(resolve_world_path(args.world))
    grid, res, origin = build_reference_grid(world, args.resolution, start=tuple(args.start))
    path = write_map(os.path.expanduser(args.out), grid, res, origin)
    free = int((grid == FREE).sum())
    occ = int((grid == OCCUPIED).sum())
    print(f'wrote {path}: {grid.shape[1]}x{grid.shape[0]} cells, free {free * res * res:.1f} m^2, '
          f'occupied {occ} cells')


if __name__ == '__main__':
    main()
