"""
2D geometry helpers (pure Python, no ROS).

A 2D robot pose is (x, y, yaw):
    x, y : metres in some frame (map, odom, world ...)
    yaw  : heading in radians, 0 = facing +x, counter-clockwise positive

ROS stores orientation as a quaternion (x, y, z, w). For a robot that only rotates
around the vertical axis the quaternion is simply:
    (0, 0, sin(yaw/2), cos(yaw/2))
"""
import math


def normalize_angle(a):
    """Wrap an angle to [-pi, pi). Always do this before comparing headings!
    e.g. 350 deg and -10 deg are the same heading; their raw difference is 360 deg."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quaternion(yaw):
    """Returns (x, y, z, w) for a rotation of `yaw` around z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(x, y, z, w):
    """Extracts the heading (rotation around z) from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def compose(a, b):
    """Pose composition a (+) b: "b expressed in a's frame" -> pose in a's parent frame.

    Example: a = robot pose in the map, b = LiDAR pose on the robot
             compose(a, b) = LiDAR pose in the map.
    """
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (ax + c * bx - s * by, ay + s * bx + c * by, normalize_angle(ath + bth))


def inverse(a):
    """Inverse pose: if a = T_parent_child then inverse(a) = T_child_parent."""
    x, y, th = a
    c, s = math.cos(th), math.sin(th)
    return (-c * x - s * y, s * x - c * y, normalize_angle(-th))


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def pose_error(estimate, truth):
    """Returns (position error [m], absolute heading error [rad])."""
    return distance(estimate, truth), abs(normalize_angle(estimate[2] - truth[2]))
