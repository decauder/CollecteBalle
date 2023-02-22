import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

import numpy as np
import math


class Point_Publisher(Node):

    def __init__(self):
        super().__init__('point_publisher')
        self.publisher_ = self.create_publisher(Pose2D, 'path/point2go', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Pose2D()
        msg.x = 1.
        msg.y = 2.
        msg.theta = 3.
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%f","%f","%f"' % (msg.x, msg.y, msg.theta))


def main(args=None):
    rclpy.init(args=args)

    point_publisher = Point_Publisher()

    rclpy.spin(point_publisher)

    point_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
