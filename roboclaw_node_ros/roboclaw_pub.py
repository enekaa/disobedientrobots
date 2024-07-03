#!/usr/bin/env python
import diagnostic_updater
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from tcr_roboclaw import Roboclaw
from .electrical_wrapper import ElectricalWrapper
from .encoder_wrapper import EncoderWrapper
from . import utils as u

class TeleopNode(Node):

    def __init__(self):
        super().__init__('my_node_name')

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.subscription = self.create_subscription(
        #     String,
        #     'input_topic',
        #     self.callback,
        #     10)

    def timer_callback(self):
        msg = Twist()
        msg.angular.x = 5.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        msg.linear.x = 10.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Publishing Linear x: "%s"' % msg.linear.x)
        self.get_logger().info('Publishing Angular x: "%s"' % msg.angular.x)



# if __name__ == "__main__":
#     rclpy.init()
#     my_node = TeleopNode()
#     rclpy.spin(my_node)
#     my_node.destroy_node()  # cleans up pub-subs, etc
#     rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    roboclaw_pub_node = TeleopNode()
    rclpy.spin(roboclaw_pub_node) 
    roboclaw_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()