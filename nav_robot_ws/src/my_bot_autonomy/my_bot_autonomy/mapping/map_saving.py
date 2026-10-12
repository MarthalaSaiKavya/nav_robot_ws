"""
Three ways to SAVE a map (same result, different technique) + saving the SLAM pose graph.

  method 'nav2'          -> service /map_saver/save_map  (nav2_map_server's map_saver_server)
                            This is what `ros2 run nav2_map_server map_saver_cli -f <path>` does.
  method 'slam_toolbox'  -> service /slam_toolbox/save_map (slam_toolbox calls map_saver for you)
  method 'python'        -> grab one /map message and write PGM + YAML ourselves (map_io.py)
                            -> shows there is no magic in the file format.

  serialize_pose_graph() -> service /slam_toolbox/serialize_map
        Saves slam_toolbox's internal pose graph (<path>.posegraph + <path>.data).
        Needed for "localization with slam_toolbox" (localization_method:=slam_toolbox),
        and to CONTINUE mapping later. A .pgm image alone cannot do that, because it
        has thrown away all the laser scans.

`save_map_with_fallbacks` tries the methods in order until one works.
All functions block -> call them from a worker thread (see common/ros_utils.py).
"""
import os
import threading

from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ..common.map_io import grid_from_msg, write_map
from ..common.ros_utils import call_service


def save_with_nav2(node, path, map_topic='/map', timeout=10.0):
    from nav2_msgs.srv import SaveMap
    cli = node.create_client(SaveMap, '/map_saver/save_map')
    req = SaveMap.Request()
    req.map_topic = map_topic
    req.map_url = path
    req.image_format = 'pgm'
    req.map_mode = 'trinary'
    req.free_thresh = 0.25
    req.occupied_thresh = 0.65
    res = call_service(cli, req, timeout_sec=timeout, wait_service_sec=3.0)
    node.destroy_client(cli)
    return bool(res is not None and res.result)


def save_with_slam_toolbox(node, path, timeout=15.0):
    from slam_toolbox.srv import SaveMap
    from std_msgs.msg import String
    cli = node.create_client(SaveMap, '/slam_toolbox/save_map')
    req = SaveMap.Request()
    req.name = String(data=path)
    res = call_service(cli, req, timeout_sec=timeout, wait_service_sec=3.0)
    node.destroy_client(cli)
    return bool(res is not None and res.result == 0)


def save_with_python(node, path, map_topic='/map', timeout=10.0):
    """Subscribe to the latched (transient-local) /map topic and write it ourselves."""
    got = {}
    event = threading.Event()
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

    def cb(msg):
        got['msg'] = msg
        event.set()
    sub = node.create_subscription(OccupancyGrid, map_topic, cb, qos)
    ok = event.wait(timeout)
    node.destroy_subscription(sub)
    if not ok:
        return False
    grid, res, origin = grid_from_msg(got['msg'])
    write_map(path, grid, res, origin)
    return True


def serialize_pose_graph(node, path, timeout=20.0):
    from slam_toolbox.srv import SerializePoseGraph
    cli = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
    req = SerializePoseGraph.Request()
    req.filename = path
    res = call_service(cli, req, timeout_sec=timeout, wait_service_sec=2.0)
    node.destroy_client(cli)
    return bool(res is not None and res.result == 0)


METHODS = {'nav2': save_with_nav2, 'slam_toolbox': save_with_slam_toolbox, 'python': save_with_python}


def save_map_with_fallbacks(node, path, methods=('nav2', 'slam_toolbox', 'python'), map_topic='/map'):
    """Tries each method until <path>.yaml exists. Returns the method that worked or None."""
    path = os.path.splitext(os.path.expanduser(path))[0]
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    log = node.get_logger()
    for m in methods:
        fn = METHODS[m]
        try:
            ok = fn(node, path) if m == 'slam_toolbox' else fn(node, path, map_topic=map_topic)
        except ImportError as e:
            log.warn(f'map save via {m} unavailable ({e})')
            ok = False
        except Exception as e:  # never let a failing saver kill the mission
            log.warn(f'map save via {m} raised {e!r}')
            ok = False
        if ok and os.path.exists(path + '.yaml'):
            log.info(f'map saved with method "{m}" -> {path}.yaml')
            return m
        log.warn(f'map save via "{m}" did not produce {path}.yaml, trying next method')
    log.error('all map saving methods failed')
    return None
