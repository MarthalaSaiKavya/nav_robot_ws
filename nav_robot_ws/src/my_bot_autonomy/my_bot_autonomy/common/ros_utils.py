"""
Small ROS 2 helpers shared by the nodes.

THREADING PATTERN USED IN THIS PROJECT (read this once, it explains every node):
--------------------------------------------------------------------------------
A ROS 2 node only receives messages / service replies while it is being "spun"
by an executor. Long sequential logic ("go to goal 1, then goal 2, then ...")
written inside a callback would BLOCK the executor, so the replies it is waiting
for would never arrive -> deadlock.

So our long-running nodes do this:
    * main thread  : rclpy.spin(node, MultiThreadedExecutor)   -> handles callbacks
    * worker thread: plain sequential Python ("the mission")    -> calls helpers
                     below, which WAIT on a threading.Event that a callback sets.
"""
import datetime
import json
import math
import os
import threading

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time

from .geometry import quaternion_to_yaw, yaw_to_quaternion


# --------------------------------------------------------------------- messages
def quaternion_msg(yaw):
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def yaw_of(q):
    return quaternion_to_yaw(q.x, q.y, q.z, q.w)


def make_pose_stamped(x, y, yaw, frame_id='map', stamp=None):
    p = PoseStamped()
    p.header.frame_id = frame_id
    if stamp is not None:
        p.header.stamp = stamp
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation = quaternion_msg(float(yaw))
    return p


def make_initial_pose(x, y, yaw, stamp, std_xy=0.5, std_yaw=0.35, frame_id='map'):
    """PoseWithCovarianceStamped for /initialpose. The covariance tells AMCL how UNSURE
    we are: bigger std -> particles spread wider -> slower but more robust convergence."""
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation = quaternion_msg(float(yaw))
    cov = [0.0] * 36
    cov[0] = std_xy ** 2       # var(x)
    cov[7] = std_xy ** 2       # var(y)
    cov[35] = std_yaw ** 2     # var(yaw)
    msg.pose.covariance = cov
    return msg


# --------------------------------------------------------------------- TF
def lookup_pose_2d(tf_buffer, target_frame, source_frame, time=None, timeout_sec=0.2):
    """Pose of `source_frame` expressed in `target_frame` as (x, y, yaw), or None.

    Example: lookup_pose_2d(buf, 'map', 'base_footprint') -> where the robot thinks it is.
    """
    try:
        t = tf_buffer.lookup_transform(target_frame, source_frame, time or Time(),
                                       timeout=Duration(seconds=timeout_sec))
    except Exception:   # tf2 raises several exception types; all mean "not available yet"
        return None
    tr = t.transform.translation
    return (tr.x, tr.y, yaw_of(t.transform.rotation))


# --------------------------------------------------------------------- waiting on futures
def wait_for_future(future, timeout_sec=None):
    """Blocks the CALLING (worker) thread until an rclpy Future completes.
    The node must be spun by an executor in another thread. Returns result or None."""
    event = threading.Event()
    future.add_done_callback(lambda _f: event.set())
    if future.done():
        event.set()
    if not event.wait(timeout=timeout_sec):
        return None
    return future.result()


def call_service(client, request, timeout_sec=5.0, wait_service_sec=5.0):
    """Synchronous service call from a worker thread (see module docstring)."""
    if not client.wait_for_service(timeout_sec=wait_service_sec):
        return None
    return wait_for_future(client.call_async(request), timeout_sec)


def ignore_sigint():
    """Called when shutting down: Ctrl+C reaches every node TWICE (from the terminal and
    from `ros2 launch`). Ignoring the second one lets the node finish saving its results."""
    import signal
    try:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    except ValueError:      # not in the main thread
        pass


def run_node(node_class, args=None, multithreaded=False):
    """Standard main(): init ROS, create the node, spin until Ctrl+C, clean up."""
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor
    rclpy.init(args=args)
    node = None
    try:
        node = node_class()
        executor = MultiThreadedExecutor() if multithreaded else SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ignore_sigint()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def spin_in_background(node, num_threads=4):
    """Starts a MultiThreadedExecutor for `node` in a daemon thread and returns it.
    Call executor.shutdown() BEFORE node.destroy_node(): our shutdown() also waits for the
    spin thread to leave, otherwise it may touch the half-destroyed node ("InvalidHandle:
    cannot use Destroyable because destruction was requested")."""
    from rclpy.executors import MultiThreadedExecutor

    class _BackgroundExecutor(MultiThreadedExecutor):
        def shutdown(self, timeout_sec=None):
            ok = super().shutdown(timeout_sec)
            thread.join(timeout=3.0)
            return ok

    executor = _BackgroundExecutor(num_threads=num_threads)
    executor.add_node(node)

    def _spin():
        try:
            executor.spin()
        except Exception:
            # Ctrl+C makes rclpy shut the context down while this thread is waiting for
            # messages -> harmless "context is not valid" error. Only re-raise real errors.
            if node.context.ok():
                raise
    thread = threading.Thread(target=_spin, daemon=True)
    thread.start()
    return executor


def sleep_sim(node, seconds):
    """Sleep for `seconds` of ROS time (sim time if use_sim_time is true)."""
    end = node.get_clock().now() + Duration(seconds=seconds)
    while rclpy.ok() and node.get_clock().now() < end:
        threading.Event().wait(0.02)


# --------------------------------------------------------------------- run directories
def default_run_root():
    return os.path.expanduser(os.environ.get('MY_BOT_RUNS', '~/nav_robot_runs'))


def make_run_dir(run_dir=''):
    """Folder where a run stores maps, CSV logs and reports. '' -> ~/nav_robot_runs/<timestamp>."""
    if not run_dir:
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        run_dir = os.path.join(default_run_root(), stamp)
    run_dir = os.path.expanduser(run_dir)
    os.makedirs(run_dir, exist_ok=True)
    return run_dir


def write_json(path, data):
    """Atomic write: write a temp file, then rename. A crash never leaves a half-written file."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=2, default=_json_default)
    os.replace(tmp, path)


def _json_default(o):
    if isinstance(o, float) and (math.isnan(o) or math.isinf(o)):
        return None
    try:
        return float(o)
    except Exception:
        return str(o)
