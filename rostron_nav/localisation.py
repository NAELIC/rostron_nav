import rclpy
from rclpy.node import Node

from rostron_interfaces.msg import Robot
from rclpy.qos import QoSProfile

from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import Quaternion
from math import sin, cos, pi


class MinimalLocalisation(Node):
    def __init__(self):
        super().__init__('localisation')
        qos_profile = QoSProfile(depth=10)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.declare_parameter('robot_id', 0)
        self.id_ = self.get_parameter(
            'robot_id').get_parameter_value().integer_value

        self.declare_parameter('team', 'yellow')
        self.team_ = self.get_parameter(
            'team').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Robot,
            '/%s/allies/r_%d' % (self.team_, self.id_),
            self.vision_callback,
            10)

    def vision_callback(self, msg: Robot):
        now = self.get_clock().now()
        odom_trans = TransformStamped()
        odom_trans.header.stamp = now.to_msg()
        odom_trans.header.frame_id = 'map'
        odom_trans.child_frame_id = 'odom'
        odom_trans.header.stamp = now.to_msg()
        odom_trans.transform.translation.x = msg.pose.position.x
        odom_trans.transform.translation.y = msg.pose.position.y
        odom_trans.transform.translation.z = 0.0
        # TODO : Verify ?
        odom_trans.transform.rotation = self.euler_to_quaternion(
            0, 0, msg.pose.orientation.z)  # roll,pitch,yaw
        self.broadcaster.sendTransform(odom_trans)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - \
            cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + \
            sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - \
            sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + \
            sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)

    localisation = MinimalLocalisation()
    rclpy.spin(localisation)

    localisation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
