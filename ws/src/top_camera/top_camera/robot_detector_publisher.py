import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import numpy as np
from robot_detector import RobotDetector


class RobotDetectorPublisher(Node):

    def __init__(self):
        super().__init__('robot_detector_publisher')
        self._publisher = self.create_publisher(Pose2D, 'robot/position', 10)
        self._robot_detector = RobotDetector(self._new_position_received_callback, debug=False, display_camera=False)

    def _new_position_received_callback(self, position, heading, timestamp):
        # print("new position received", position, heading, timestamp)

        msg = Pose2D()
        msg.x = position[0]
        msg.y = position[1]
        msg.theta = heading

        self._publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    robot_detector_publisher = RobotDetectorPublisher()

    rclpy.spin(robot_detector_publisher)

    robot_detector_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
