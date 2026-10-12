"""
Read / write occupancy-grid maps in the standard ROS format (pure Python + numpy).

A saved map is TWO files (exactly what `ros2 run nav2_map_server map_saver_cli` writes):

  room.pgm   - a greyscale image. Each pixel is one grid cell.
                 black (0)   = occupied (wall)
                 white (254) = free
                 grey (205)  = unknown (never seen)
  room.yaml  - the metadata:
                 image: room.pgm
                 mode: trinary
                 resolution: 0.05            # metres per pixel
                 origin: [-5.5, -5.5, 0.0]    # world pose of the BOTTOM-LEFT pixel
                 negate: 0
                 occupied_thresh: 0.65       # pixel "occupancy" above this -> occupied
                 free_thresh: 0.196          # below this -> free (205 grey stays unknown)

In memory we use the same convention as nav_msgs/OccupancyGrid:
  numpy int8 array grid[row, col], row 0 = BOTTOM of the map (lowest y),
  values -1 = unknown, 0 = free, 100 = occupied.
Images store row 0 at the TOP, so we flip vertically when reading/writing.
"""
import os

import numpy as np
import yaml

UNKNOWN, FREE, OCCUPIED = -1, 0, 100


def write_map(base_path, grid, resolution, origin, free_thresh=0.25, occupied_thresh=0.65):
    """Writes base_path.pgm and base_path.yaml. Returns the yaml path.

    grid   : int8 array (rows = y, cols = x), -1/0/100 (values 1..99 are thresholded)
    origin : (x, y, yaw) of cell (row 0, col 0)
    """
    grid = np.asarray(grid)
    img = np.full(grid.shape, 205, dtype=np.uint8)            # unknown = grey
    known = grid >= 0
    occ = known & (grid >= occupied_thresh * 100)
    free = known & (grid <= free_thresh * 100)
    img[free] = 254
    img[occ] = 0
    img = np.flipud(img)                                       # image row 0 = top

    base_path = os.path.splitext(base_path)[0]
    os.makedirs(os.path.dirname(os.path.abspath(base_path)), exist_ok=True)
    pgm_path = base_path + '.pgm'
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n# CREATOR: my_bot_autonomy map_io {resolution:.3f} m/pix\n'
                f'{img.shape[1]} {img.shape[0]}\n255\n'.encode('ascii'))
        f.write(img.tobytes())

    meta = {
        'image': os.path.basename(pgm_path),
        'mode': 'trinary',
        'resolution': float(resolution),
        'origin': [float(origin[0]), float(origin[1]), float(origin[2]) if len(origin) > 2 else 0.0],
        'negate': 0,
        # Same trick as nav2's map_saver in trinary mode: the thresholds describe the PIXEL
        # values written above (0 / 205 / 254), so they are fixed. 205 -> 0.196... > 0.196 -> unknown.
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    yaml_path = base_path + '.yaml'
    with open(yaml_path, 'w') as f:
        yaml.safe_dump(meta, f, sort_keys=False, default_flow_style=None)
    return yaml_path


def _read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    # Parse the (possibly commented) ASCII header: magic, width, height, maxval
    tokens, idx = [], 0
    while len(tokens) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':
            while data[idx:idx + 1] not in (b'\n', b''):
                idx += 1
            continue
        start = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        tokens.append(data[start:idx].decode('ascii'))
    idx += 1  # single whitespace after maxval
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == 'P5':
        img = np.frombuffer(data[idx:idx + w * h], dtype=np.uint8).reshape(h, w)
    elif magic == 'P2':
        img = np.array(data[idx:].split(), dtype=np.int64).reshape(h, w).astype(np.uint8)
    else:
        raise ValueError(f'{path}: only PGM (P5/P2) images are supported, got {magic}')
    if maxval != 255:
        img = (img.astype(float) * 255.0 / maxval).astype(np.uint8)
    return img


def read_map(yaml_path):
    """Reads a ROS map yaml (+ its PGM image). Returns (grid, resolution, origin)."""
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
    if not img_path.lower().endswith(('.pgm',)):
        raise ValueError('map_io only reads .pgm images (that is what map_saver writes)')
    img = _read_pgm(img_path).astype(float)
    negate = int(meta.get('negate', 0))
    occ_prob = img / 255.0 if negate else (255.0 - img) / 255.0
    grid = np.full(img.shape, UNKNOWN, dtype=np.int8)
    # identical rules to nav2_map_server (map_io.cpp): >= occupied_thresh, <= free_thresh
    grid[occ_prob >= float(meta.get('occupied_thresh', 0.65))] = OCCUPIED
    grid[occ_prob <= float(meta.get('free_thresh', 0.196))] = FREE
    grid = np.flipud(grid).copy()                              # back to row 0 = bottom
    origin = list(meta.get('origin', [0.0, 0.0, 0.0]))
    return grid, float(meta['resolution']), origin


# ----------------------------------------------------------------- ROS conversions
def grid_from_msg(msg):
    """nav_msgs/OccupancyGrid -> (grid int8 [rows, cols], resolution, origin(x, y, yaw))."""
    from .geometry import quaternion_to_yaw
    info = msg.info
    grid = np.array(msg.data, dtype=np.int8).reshape(info.height, info.width)
    q = info.origin.orientation
    origin = (info.origin.position.x, info.origin.position.y, quaternion_to_yaw(q.x, q.y, q.z, q.w))
    return grid, info.resolution, origin


def msg_from_grid(grid, resolution, origin, frame_id, stamp):
    """(grid, resolution, origin) -> nav_msgs/OccupancyGrid."""
    from nav_msgs.msg import OccupancyGrid
    from .geometry import yaw_to_quaternion
    msg = OccupancyGrid()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.info.resolution = float(resolution)
    msg.info.width = int(grid.shape[1])
    msg.info.height = int(grid.shape[0])
    msg.info.origin.position.x = float(origin[0])
    msg.info.origin.position.y = float(origin[1])
    qx, qy, qz, qw = yaw_to_quaternion(origin[2] if len(origin) > 2 else 0.0)
    msg.info.origin.orientation.x, msg.info.origin.orientation.y = qx, qy
    msg.info.origin.orientation.z, msg.info.origin.orientation.w = qz, qw
    msg.data = np.asarray(grid, dtype=np.int8).ravel().tolist()
    return msg


def world_to_cell(x, y, resolution, origin):
    """World (x, y) -> integer (row, col). (Assumes an un-rotated map, which ROS maps always are.)"""
    col = int(np.floor((x - origin[0]) / resolution))
    row = int(np.floor((y - origin[1]) / resolution))
    return row, col


def cell_to_world(row, col, resolution, origin):
    """Integer (row, col) -> world coordinates of the cell CENTRE."""
    return (origin[0] + (col + 0.5) * resolution, origin[1] + (row + 0.5) * resolution)
