import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D


class DummyTargetPositionPublisher(Node):
    """
        Dummy target position publisher - publishes a target position every 0.5 seconds
    """

    def __init__(self):
        super().__init__('dummy_target_position_publisher')
        self._publisher = self.create_publisher(Pose2D, 'robot/target_position', 10)

        self._timer = self.create_timer(0.5, self._timer_callback)

    def _timer_callback(self):
        msg = Pose2D()
        msg.x = 850.  # set the target position here [in px]
        msg.y = 300.  # and here [in px]
        msg.theta = 0.

        self._publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    dummy_target_position_publisher = DummyTargetPositionPublisher()

    rclpy.spin(dummy_target_position_publisher)

    dummy_target_position_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
