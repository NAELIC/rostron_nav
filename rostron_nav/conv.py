import rclpy
from rclpy.node import Node

from rostron_interfaces.msg import Robot

from rostron_interfaces.msg import Order
from geometry_msgs.msg import Twist

class Convertisseur(Node):
    def __init__(self):
        super().__init__('minimal_filter')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.vision_callback,
            10)
        self.publisher_ = self.create_publisher(Order, '/yellow/order', 10)

    def vision_callback(self, msg: Twist):
        self.get_logger().info("good !")
       
        order = Order()
        order.id = 0
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
