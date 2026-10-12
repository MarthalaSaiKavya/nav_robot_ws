"""
twist_stamper - converts geometry_msgs/Twist into geometry_msgs/TwistStamped.

Needed for drive:=ros2_control because, in ROS 2 Jazzy, diff_drive_controller only
accepts TwistStamped, while Nav2 (default) and teleop_twist_keyboard publish Twist.

(An equivalent community package exists: `ros-jazzy-twist-stamper`. Writing it
yourself is 20 lines and shows how simple a "relay" node is.)

    /cmd_vel_in  (Twist)  --->  [twist_stamper]  --->  /cmd_vel_out (TwistStamped)
Remap cmd_vel_in / cmd_vel_out in the launch file.
"""
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from ..common.ros_utils import run_node


class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        self.frame_id = self.declare_parameter('frame_id', 'base_footprint').value
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel_out', 10)
        self.create_subscription(Twist, 'cmd_vel_in', self._on_twist, 10)

    def _on_twist(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)


def main(args=None):
    run_node(TwistStamper, args)


if __name__ == '__main__':
    main()
