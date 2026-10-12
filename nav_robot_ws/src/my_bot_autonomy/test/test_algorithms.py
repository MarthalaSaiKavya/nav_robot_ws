"""
Unit tests for the pure-Python algorithms (no ROS needed, runs in seconds).

    cd ~/nav_robot_ws && colcon test --packages-select my_bot_autonomy && colcon test-result --verbose
    # or directly:
    cd ~/nav_robot_ws/src/my_bot_autonomy && python3 -m pytest test -q

These tests also double as EXAMPLES of how to use each module.
"""
import math
import os

import numpy as np
import pytest

from my_bot_autonomy.common.geometry import compose, inverse, normalize_angle, quaternion_to_yaw, yaw_to_quaternion
from my_bot_autonomy.common.map_io import read_map, write_map
from my_bot_autonomy.common.world_model import World2D, box_model_sdf, shapes_from_model_sdf
from my_bot_autonomy.exploration.frontiers import STRATEGIES, cluster, find_frontiers, frontier_mask, classify, score_frontiers
from my_bot_autonomy.learning.distance_transform import distance_transform
from my_bot_autonomy.learning.grid_planners import build_cost_grid, nearest_free, path_length, plan
from my_bot_autonomy.learning.occupancy_mapping import LogOddsMap, bresenham
from my_bot_autonomy.learning.particle_filter import ParticleFilter
from my_bot_autonomy.learning.pure_pursuit import PurePursuit
from my_bot_autonomy.mapping.compare_maps import compare
from my_bot_autonomy.mapping.reference_map import build_reference_grid

HERE = os.path.dirname(os.path.abspath(__file__))
WORLDS = os.path.join(HERE, '..', '..', 'my_bot_gazebo', 'worlds')
MISSIONS = os.path.join(HERE, '..', '..', 'my_bot_bringup', 'config', 'missions')
ANGLES = -math.pi + np.arange(360) * (2 * math.pi / 360)


@pytest.fixture(scope='module')
def world():
    return World2D.from_sdf_file(os.path.join(WORLDS, 'nav_room.sdf'))


@pytest.fixture(scope='module')
def ref(world):
    return build_reference_grid(world)


# ----------------------------------------------------------------- geometry
def test_quaternion_roundtrip():
    for yaw in np.linspace(-3.1, 3.1, 13):
        assert abs(quaternion_to_yaw(*yaw_to_quaternion(yaw)) - yaw) < 1e-9


def test_compose_inverse():
    a = (1.0, 2.0, 0.7)
    b = (0.3, -0.4, -1.2)
    c = compose(a, b)
    back = compose(inverse(a), c)
    assert np.allclose(back[:2], b[:2]) and abs(normalize_angle(back[2] - b[2])) < 1e-9


# ----------------------------------------------------------------- world model
def test_world_parsing(world):
    assert world.world_name == 'nav_room'
    assert len(world.shapes) == 14
    # the room interior is exactly 10 x 10 m: inner faces at +-5
    r = world.raycast(0.0, -1.0, np.array([0.0, math.pi]), 20.0)
    assert r[0] == pytest.approx(5.0, abs=1e-6)   # east wall
    assert r[1] == pytest.approx(5.0, abs=1e-6)   # west wall


def test_raycast_cylinder(world):
    # pillar_b: centre (0.6, 3.2), r = 0.3; look at it from (0.6, 2.0) towards +y
    r = world.raycast(0.6, 2.0, np.array([math.pi / 2]), 20.0)
    assert r[0] == pytest.approx(3.2 - 0.3 - 2.0, abs=1e-6)


def test_spawn_and_remove(world):
    w = World2D(world.world_name, list(world.shapes))
    name, shapes = shapes_from_model_sdf(box_model_sdf('scenario_test', 0.5, 0.5, 0.6), 0.0, -1.0, 0.0, 0.0)
    w.add_shapes(shapes)
    assert w.distance_to_obstacles(0.0, -1.0) == 0.0
    assert w.remove_model(name) == 1
    assert w.distance_to_obstacles(0.0, -1.0) > 0.5


# ----------------------------------------------------------------- maps
def test_map_roundtrip(tmp_path, ref):
    grid, res, origin = ref
    path = write_map(str(tmp_path / 'm'), grid, res, origin)
    g2, r2, o2 = read_map(path)
    assert np.array_equal(grid, g2) and r2 == res and np.allclose(o2, origin)


def test_compare_identical(ref):
    g, r, o = ref
    m = compare(g, r, o, g, r, o)
    assert m['pass'] and m['wall_error_mean'] == 0.0 and m['coverage'] == 1.0


def test_distance_transform_exact():
    rng = np.random.default_rng(0)
    mask = rng.random((30, 40)) > 0.97
    d = distance_transform(mask)
    pts = np.argwhere(mask)
    for r, c in [(0, 0), (15, 20), (29, 39), (7, 33)]:
        brute = np.min(np.hypot(pts[:, 0] - r, pts[:, 1] - c))
        assert d[r, c] == pytest.approx(brute, abs=1e-6)


# ----------------------------------------------------------------- frontiers
def test_frontier_detection_simple():
    grid = np.full((20, 20), -1, dtype=np.int8)
    grid[5:15, 2:10] = 0            # a known free rectangle; its right edge borders unknown
    labels = classify(grid)
    fm = frontier_mask(labels)
    assert fm[:, 9].sum() == 10      # whole right column is frontier
    assert len(cluster(fm)) == 1


def test_ring_frontier_is_split():
    # At start-up the known area is a disc around the robot -> ONE ring-shaped frontier whose
    # centroid is the robot itself. Splitting must give goals ON the ring, away from the centre.
    res, origin = 0.05, (-5.0, -5.0)
    rows, cols = np.indices((200, 200))
    x, y = origin[0] + (cols + 0.5) * res, origin[1] + (rows + 0.5) * res
    grid = np.full((200, 200), -1, dtype=np.int8)
    grid[x ** 2 + y ** 2 < 1.5 ** 2] = 0
    whole = find_frontiers(grid, res, origin, max_frontier_length_m=0)
    assert len(whole) == 1
    pieces = find_frontiers(grid, res, origin, max_frontier_length_m=3.0)
    assert len(pieces) >= 3
    assert all(math.hypot(*f.goal) > 1.0 for f in pieces)


def test_frontier_strategies(ref):
    grid, res, origin = ref
    rows, cols = np.indices(grid.shape)
    y = origin[1] + (rows + 0.5) * res
    part = grid.copy()
    part[y > 0.5] = -1                                # pretend we only saw the south part
    fr = find_frontiers(part, res, origin)
    assert fr, 'expected frontiers'
    for s in STRATEGIES:
        best = score_frontiers(list(fr), (0.0, 0.0), s, rng=np.random.default_rng(1))[0]
        assert best.clearance >= 0.30


# ----------------------------------------------------------------- mapping
def test_bresenham_endpoints():
    cells = bresenham(0, 0, 3, 7)
    assert cells[0] == (0, 0) and cells[-1] == (3, 7) and len(cells) == 8


def test_log_odds_mapping_known_poses(world, ref):
    m = LogOddsMap(size_x=11.0, size_y=11.0, resolution=0.05, origin=(-5.5, -5.5))
    for pose in [(0, 0, 0), (-2.5, -1.0, 1.0), (2.5, -1.0, 2.0), (0.0, -4.2, 0.0), (-3.5, 3.0, 0.0), (3.0, 2.5, 3.0)]:
        lx, ly, lyaw = compose(pose, (0.05, 0.0, 0.0))
        for _ in range(3):
            ranges = world.raycast(lx, ly, ANGLES + lyaw, 12.0, 0.155)
            m.update_scan((lx, ly, lyaw), ranges, ANGLES, 12.0)
    grid = m.to_occupancy()
    grid = np.where(grid >= 65, 100, np.where((grid >= 0) & (grid < 25), 0, -1)).astype(np.int8)
    g, r, o = ref
    metrics = compare(grid, 0.05, m.origin, g, r, o)
    assert metrics['wall_error_p95'] <= 0.10
    assert metrics['coverage'] > 0.6


# ----------------------------------------------------------------- particle filter
def test_particle_filter_converges(world, ref):
    grid, res, origin = ref
    pf = ParticleFilter(grid, res, origin, num_particles=400, seed=3)
    truth = (1.0, -4.2, math.pi)
    pf.init_gaussian(truth[0] + 0.35, truth[1] - 0.25, truth[2] + 0.25, 0.5, 0.35)
    rng = np.random.default_rng(4)
    odom_prev = (0.0, 0.0, 0.0)
    pose = truth
    for k in range(25):
        # rotate in place 0.25 rad per step (like the pre-mission spin)
        new_pose = (pose[0], pose[1], normalize_angle(pose[2] + 0.25))
        odom_now = (0.0, 0.0, odom_prev[2] + 0.25)
        pf.predict(odom_prev, odom_now)
        lx, ly, lyaw = compose(new_pose, (0.05, 0.0, 0.0))
        ranges = world.raycast(lx, ly, ANGLES + lyaw, 12.0, 0.155) + rng.normal(0, 0.01, 360)
        pf.update(ranges, ANGLES, lidar_offset=(0.05, 0.0))
        pf.resample()
        odom_prev, pose = odom_now, new_pose
    (x, y, yaw), _ = pf.estimate()
    assert math.hypot(x - pose[0], y - pose[1]) < 0.15
    assert abs(normalize_angle(yaw - pose[2])) < 0.1


# ----------------------------------------------------------------- planning + control
def test_planners(ref):
    grid, res, origin = ref
    cost = build_cost_grid(grid, res)
    to_cell = lambda x, y: (int((y - origin[1]) / res), int((x - origin[0]) / res))  # noqa: E731
    start = nearest_free(cost, to_cell(-4.2, -4.2))
    goal = nearest_free(cost, to_cell(4.3, 3.2))      # other zone -> must go through a doorway
    results = {}
    for alg in ('astar', 'dijkstra', 'greedy'):
        path, n, _ = plan(cost, start, goal, alg)
        assert path is not None
        results[alg] = (path_length(path, res), n)
    assert results['astar'][1] < results['dijkstra'][1]                 # A* expands fewer cells
    assert results['astar'][0] == pytest.approx(results['dijkstra'][0], rel=0.05)  # both ~optimal


def test_pure_pursuit_reaches_goal():
    pp = PurePursuit(lookahead=0.5, max_v=0.4)
    path = [(x, 0.5 * math.sin(x)) for x in np.linspace(0, 4, 80)]
    pose = (0.0, 0.0, 0.0)
    for _ in range(2000):
        v, w, done = pp.compute(pose, path)
        if done:
            break
        dt = 0.05
        pose = (pose[0] + v * math.cos(pose[2]) * dt, pose[1] + v * math.sin(pose[2]) * dt,
                normalize_angle(pose[2] + w * dt))
    assert done


def test_pure_pursuit_sparse_path():
    # A shortcut-smoothed plan has only a few corners; the robot must not turn back to the start.
    pp = PurePursuit(lookahead=0.6, max_v=0.3)
    path = [(0.0, 0.0), (-1.8, 1.6), (-3.5, 3.0)]
    pose = (0.0, 0.0, 2.4)
    for _ in range(4000):
        v, w, done = pp.compute(pose, path)
        if done:
            break
        dt = 0.05
        pose = (pose[0] + v * math.cos(pose[2]) * dt, pose[1] + v * math.sin(pose[2]) * dt,
                normalize_angle(pose[2] + w * dt))
    assert done


# ----------------------------------------------------------------- mission files
def test_mission_goals_are_safe(world):
    from my_bot_autonomy.mission.mission_io import load_mission, validate_goals
    for f in os.listdir(MISSIONS):
        if f.endswith('.yaml'):
            m = load_mission(os.path.join(MISSIONS, f))
            assert m['goals'], f
            problems = validate_goals(m, world)
            assert not problems, f'{f}: {problems}'
