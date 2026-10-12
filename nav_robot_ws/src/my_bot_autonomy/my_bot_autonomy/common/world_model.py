"""
A tiny 2D model of the simulated room, built by READING THE GAZEBO SDF FILE.

Used by:
  * lite_sim            - ray-casts the LiDAR and checks collisions against it
  * make_reference_map  - rasterises it into a "perfect" occupancy-grid map
  * mission validation  - checks that mission goals are not inside obstacles
  * scenario_director   - builds SDF strings for obstacles it spawns

Only what a 2D robot needs is kept: every model with a <box> or <cylinder>
collision becomes a Box or Cylinder footprint on the floor. Planes (the ground)
are ignored. Heights are kept only so we can ignore things the LiDAR passes over.

This file is pure Python + numpy (no ROS), so it is easy to test:
    python3 -c "from my_bot_autonomy.common.world_model import World2D; \
                w = World2D.from_sdf_file('nav_room.sdf'); print(w.summary())"
"""
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

from .geometry import compose


@dataclass
class Box:
    name: str
    cx: float
    cy: float
    sx: float          # size along the box's own x axis
    sy: float          # size along the box's own y axis
    yaw: float = 0.0
    height: float = 1.0
    z_min: float = 0.0  # bottom of the box above the floor

    def corners(self):
        """4 corners (x, y) counter-clockwise in the world frame."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hx, hy = self.sx / 2.0, self.sy / 2.0
        pts = []
        for lx, ly in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)):
            pts.append((self.cx + c * lx - s * ly, self.cy + s * lx + c * ly))
        return pts


@dataclass
class Cylinder:
    name: str
    cx: float
    cy: float
    radius: float
    height: float = 1.0
    z_min: float = 0.0


@dataclass
class World2D:
    world_name: str = 'nav_room'
    shapes: list = field(default_factory=list)

    # ------------------------------------------------------------------ parsing
    @staticmethod
    def from_sdf_file(path):
        with open(path, 'r') as f:
            return World2D.from_sdf_string(f.read())

    @staticmethod
    def from_sdf_string(text):
        root = ET.fromstring(text)
        world_el = root.find('world')
        world = World2D(world_name=world_el.get('name', 'default') if world_el is not None else 'default')
        container = world_el if world_el is not None else root
        for model in container.findall('model'):
            world.shapes.extend(parse_model_element(model))
        return world

    # ------------------------------------------------------------------ editing
    def add_shapes(self, shapes):
        self.shapes.extend(shapes)

    def remove_model(self, model_name):
        """Removes every shape that belongs to `model_name` (shape names are 'model/link/collision')."""
        before = len(self.shapes)
        self.shapes = [s for s in self.shapes if s.name.split('/')[0] != model_name]
        return before - len(self.shapes)

    def move_model(self, model_name, x, y, yaw):
        """Moves a model so that its origin is at (x, y, yaw). Shapes keep their relative offsets."""
        moved = False
        for s in self.shapes:
            if s.name.split('/')[0] == model_name:
                ox, oy, oyaw = getattr(s, '_model_pose', (s.cx, s.cy, 0.0))
                rel = _relative(s, (ox, oy, oyaw))
                nx, ny, nyaw = compose((x, y, yaw), rel)
                s.cx, s.cy = nx, ny
                if isinstance(s, Box):
                    s.yaw = nyaw
                s._model_pose = (x, y, yaw)
                moved = True
        return moved

    def model_names(self):
        return sorted({s.name.split('/')[0] for s in self.shapes})

    def summary(self):
        lines = [f"world '{self.world_name}' with {len(self.shapes)} shapes:"]
        for s in self.shapes:
            if isinstance(s, Box):
                lines.append(f'  box  {s.name:40s} centre=({s.cx:6.2f},{s.cy:6.2f}) '
                             f'size=({s.sx:.2f}x{s.sy:.2f}) yaw={s.yaw:.2f}')
            else:
                lines.append(f'  cyl  {s.name:40s} centre=({s.cx:6.2f},{s.cy:6.2f}) r={s.radius:.2f}')
        return '\n'.join(lines)

    # ------------------------------------------------------------- geometry
    def _visible(self, scan_height):
        """Shapes that intersect the horizontal LiDAR plane at `scan_height`."""
        if scan_height is None:
            return self.shapes
        return [s for s in self.shapes if s.z_min <= scan_height <= s.z_min + s.height]

    def segments(self, scan_height=None):
        """All box edges as an (M, 4) array [x1, y1, x2, y2]."""
        segs = []
        for s in self._visible(scan_height):
            if isinstance(s, Box):
                c = s.corners()
                for i in range(4):
                    (x1, y1), (x2, y2) = c[i], c[(i + 1) % 4]
                    segs.append((x1, y1, x2, y2))
        return np.array(segs, dtype=float).reshape(-1, 4)

    def circles(self, scan_height=None):
        """All cylinders as an (K, 3) array [cx, cy, r]."""
        circ = [(s.cx, s.cy, s.radius) for s in self._visible(scan_height) if isinstance(s, Cylinder)]
        return np.array(circ, dtype=float).reshape(-1, 3)

    def raycast(self, ox, oy, angles, max_range, scan_height=None):
        """
        Vectorised ray casting = what a LiDAR physically does.

        For every ray direction d (unit vector) we solve, against every wall segment
        P + u*(Q-P) (0<=u<=1), the equation  O + t*d = P + u*(Q-P)  for the distance t,
        and against every circle |O + t*d - C| = r. The smallest positive t is the range.

        Returns an array of ranges (np.inf where nothing is hit within max_range).
        """
        angles = np.asarray(angles, dtype=float)
        dx, dy = np.cos(angles), np.sin(angles)                     # (N,)
        best = np.full(angles.shape, np.inf)

        segs = self.segments(scan_height)
        if len(segs):
            px, py = segs[:, 0], segs[:, 1]                          # (M,)
            ex, ey = segs[:, 2] - px, segs[:, 3] - py
            # 2D cross products, broadcast to (N, M)
            denom = dx[:, None] * ey[None, :] - dy[:, None] * ex[None, :]
            wx, wy = px - ox, py - oy
            with np.errstate(divide='ignore', invalid='ignore'):
                t = (wx[None, :] * ey[None, :] - wy[None, :] * ex[None, :]) / denom
                u = (wx[None, :] * dy[:, None] - wy[None, :] * dx[:, None]) / denom
            hit = (np.abs(denom) > 1e-12) & (t > 1e-9) & (u >= 0.0) & (u <= 1.0)
            t = np.where(hit, t, np.inf)
            best = np.minimum(best, t.min(axis=1))

        circ = self.circles(scan_height)
        if len(circ):
            cx, cy, r = circ[:, 0], circ[:, 1], circ[:, 2]
            fx, fy = ox - cx, oy - cy                                 # (K,)
            b = dx[:, None] * fx[None, :] + dy[:, None] * fy[None, :]   # d . (O - C)
            c = fx * fx + fy * fy - r * r
            disc = b * b - c[None, :]
            with np.errstate(invalid='ignore'):
                sq = np.sqrt(np.where(disc >= 0, disc, 0.0))
            t1 = -b - sq
            t2 = -b + sq
            t = np.where(t1 > 1e-9, t1, np.where(t2 > 1e-9, t2, np.inf))
            t = np.where(disc >= 0, t, np.inf)
            best = np.minimum(best, t.min(axis=1))

        best[best > max_range] = np.inf
        return best

    def distance_to_obstacles(self, x, y, scan_height=None):
        """Shortest distance from point (x, y) to any obstacle surface (0 if inside)."""
        d = np.inf
        for s in self._visible(scan_height):
            if isinstance(s, Box):
                c, sn = math.cos(-s.yaw), math.sin(-s.yaw)
                lx = c * (x - s.cx) - sn * (y - s.cy)
                ly = sn * (x - s.cx) + c * (y - s.cy)
                ddx = max(abs(lx) - s.sx / 2.0, 0.0)
                ddy = max(abs(ly) - s.sy / 2.0, 0.0)
                d = min(d, math.hypot(ddx, ddy))
            else:
                d = min(d, max(math.hypot(x - s.cx, y - s.cy) - s.radius, 0.0))
        return d

    def occupancy(self, xs, ys, inflate=0.0, scan_height=None):
        """Boolean mask: which of the points (xs, ys) (same-shape arrays) lie inside an obstacle
        (grown by `inflate` metres)."""
        xs = np.asarray(xs, dtype=float)
        ys = np.asarray(ys, dtype=float)
        occ = np.zeros(xs.shape, dtype=bool)
        for s in self._visible(scan_height):
            if isinstance(s, Box):
                c, sn = math.cos(-s.yaw), math.sin(-s.yaw)
                lx = c * (xs - s.cx) - sn * (ys - s.cy)
                ly = sn * (xs - s.cx) + c * (ys - s.cy)
                ddx = np.maximum(np.abs(lx) - s.sx / 2.0, 0.0)
                ddy = np.maximum(np.abs(ly) - s.sy / 2.0, 0.0)
                occ |= np.hypot(ddx, ddy) <= inflate + 1e-9
            else:
                occ |= np.hypot(xs - s.cx, ys - s.cy) <= s.radius + inflate
        return occ

    def bounds(self):
        """(xmin, ymin, xmax, ymax) of all shapes."""
        xs, ys = [], []
        for s in self.shapes:
            if isinstance(s, Box):
                for (x, y) in s.corners():
                    xs.append(x)
                    ys.append(y)
            else:
                xs += [s.cx - s.radius, s.cx + s.radius]
                ys += [s.cy - s.radius, s.cy + s.radius]
        if not xs:
            return (-1.0, -1.0, 1.0, 1.0)
        return (min(xs), min(ys), max(xs), max(ys))


# ---------------------------------------------------------------------- SDF helpers
def _floats(text, n):
    vals = [float(v) for v in (text or '').split()]
    return (vals + [0.0] * n)[:n]


def _pose_of(el):
    """SDF <pose>x y z roll pitch yaw</pose> -> (x, y, z, yaw). Missing pose = zeros."""
    p = el.find('pose') if el is not None else None
    x, y, z, _r, _p, yaw = _floats(p.text if p is not None else '', 6)
    return x, y, z, yaw


def _relative(shape, model_pose):
    """Offset of a shape relative to its model origin (used when moving a model)."""
    mx, my, myaw = model_pose
    c, s = math.cos(-myaw), math.sin(-myaw)
    dx, dy = shape.cx - mx, shape.cy - my
    yaw = (shape.yaw - myaw) if isinstance(shape, Box) else 0.0
    return (c * dx - s * dy, s * dx + c * dy, yaw)


def parse_model_element(model, pose_override=None):
    """Turns one <model> element into a list of Box / Cylinder shapes in the world frame.

    pose_override=(x, y, z, yaw) replaces the model's own <pose> (used when spawning
    an entity with an explicit pose, like Gazebo's EntityFactory does).
    """
    name = model.get('name', 'model')
    mx, my, mz, myaw = pose_override if pose_override is not None else _pose_of(model)
    shapes = []
    for link in model.findall('link'):
        lx, ly, lz, lyaw = _pose_of(link)
        for col in link.findall('collision'):
            cx_, cy_, cz_, cyaw = _pose_of(col)
            geom = col.find('geometry')
            if geom is None:
                continue
            # world pose of the collision = model (+) link (+) collision
            wx, wy, wyaw = compose(compose((mx, my, myaw), (lx, ly, lyaw)), (cx_, cy_, cyaw))
            wz = mz + lz + cz_
            sname = f"{name}/{link.get('name', 'link')}/{col.get('name', 'collision')}"
            box = geom.find('box')
            cyl = geom.find('cylinder')
            if box is not None:
                sx, sy, sz = _floats(box.findtext('size'), 3)
                shp = Box(sname, wx, wy, sx, sy, wyaw, sz, wz - sz / 2.0)
            elif cyl is not None:
                r = float(cyl.findtext('radius', '0.1'))
                h = float(cyl.findtext('length', '1.0'))
                shp = Cylinder(sname, wx, wy, r, h, wz - h / 2.0)
            else:
                continue   # planes, meshes ... are ignored in 2D
            shp._model_pose = (mx, my, myaw)
            shapes.append(shp)
    return shapes


def shapes_from_model_sdf(sdf_text, x=None, y=None, z=None, yaw=None, name_override=None):
    """Parses an SDF string that contains ONE <model> (as sent to /world/<w>/create)."""
    root = ET.fromstring(sdf_text)
    model = root if root.tag == 'model' else root.find('model')
    if model is None:
        raise ValueError('SDF string does not contain a <model>')
    if name_override:
        model.set('name', name_override)
    override = None
    if x is not None:
        override = (x, y, z or 0.0, yaw or 0.0)
    return model.get('name'), parse_model_element(model, override)


def box_model_sdf(name, sx, sy, sz, color=(0.9, 0.45, 0.1)):
    """SDF for a static box whose origin is at its bottom-centre (so z=0 puts it on the floor).

    Single quotes are used for attributes so the string can be embedded in other
    formats (e.g. the `gz service --req` text) without escaping."""
    r, g, b = color
    return (
        "<?xml version='1.0'?><sdf version='1.9'>"
        f"<model name='{name}'><static>true</static>"
        f"<link name='link'><pose>0 0 {sz / 2.0} 0 0 0</pose>"
        f"<collision name='collision'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>"
        f"<visual name='visual'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
        f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>"
        "</visual></link></model></sdf>"
    )
