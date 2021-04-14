import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from rostron_interfaces.msg import Robot

from rostron_interfaces.msg import Order, Hardware
from geometry_msgs.msg import Twist


class Nav2Order(Node):
    kick_type_ = 0
    kick_power_ = 0.0
    spin_power_ = 0.0
    halt = False

    def __init__(self):
        super().__init__('nav2order')

        # parameters

        self.declare_parameter('robot_id', 0)
        self.declare_parameter('team', 'yellow')
        
        self.id_ = self.get_parameter(
            'robot_id').get_parameter_value().integer_value

        self.team_ = self.get_parameter(
            'team').get_parameter_value().string_value


        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.vel_callback,
            10)

        self.h_sub = self.create_subscription(
            Hardware,
            'hardware_order',
            self.hardware_callback,
            10)

        self.create_subscription(
            Bool,
            '/%s/halt' % self.team_,
            self.halt_callback,
            10)


        self.publisher_ = self.create_publisher(
            Order, '/%s/order' % self.team_, 10)

    def vel_callback(self, msg: Twist):
        if self.halt:
            self.publish_order(Twist())
        else:
            self.publish_order(msg)

    def halt_callback(self, msg: Bool):
        self.halt = msg.data
        self.publish_order(Twist())

    def publish_order(self, twist: Twist):
        order = Order()
        order.id = self.id_
        order.velocity = twist
        order.hardware.spin_power = self.spin_power_
        order.hardware.kick_type = self.kick_type_
        order.hardware.kick_power = self.kick_power_
        self.publisher_.publish(order)

    def hardware_callback(self, msg: Hardware):
        self.kick_type_ = msg.kick_type
        self.kick_power_ = msg.kick_power
        self.spin_power_ = msg.spin_power


def main(args=None):
    rclpy.init(args=args)

    node = Nav2Order()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
