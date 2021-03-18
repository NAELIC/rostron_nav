import rclpy
from rclpy.node import Node

from rostron_interfaces.msg import Robot

from rostron_interfaces.msg import Order
from geometry_msgs.msg import Twist


class Convertisseur(Node):
    def __init__(self):
        super().__init__('conv_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.vision_callback,
            10)
            
        self.declare_parameter('robot_id', 0)
        self.id_ = self.get_parameter(
            'robot_id').get_parameter_value().integer_value

        self.declare_parameter('team', 'yellow')
        self.team_ = self.get_parameter(
            'team').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(
            Order, '/yellow/order', 10)

    def vision_callback(self, msg: Twist):
        self.get_logger().info("%s" % self.team_)
        self.get_logger().info("%d" % self.id_)
        order = Order()
        order.id = self.id_
        order.velocity = msg
        self.publisher_.publish(order)


def main(args=None):
    rclpy.init(args=args)

    conv = Convertisseur()
    rclpy.spin(conv)

    conv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
