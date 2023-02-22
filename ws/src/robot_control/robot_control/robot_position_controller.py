import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
import numpy as np


class RobotPositionController(Node):
    """
    Class handling the control of the robot's position

    Subscribes to:
        - robot/target_position: target position for the robot
        - robot/position: current position of the robot

    Publishes to:
        - demo/cmd_vel: control command for the robot
    """

    def __init__(self):
        super().__init__('robot_position_controller')
        self._publisher = self.create_publisher(Twist, 'demo/cmd_vel', 10)

        self._robot_pos_subscription = self.create_subscription(
            Pose2D, 'robot/position', self._new_position_received_callback, 10)
        self._robot_pos_subscription  # prevent unused variable warning

        self._target_pos_subscription = self.create_subscription(
            Pose2D, 'robot/target_position', self._new_target_position_received_callback, 10)
        self._target_pos_subscription  # prevent unused variable warning

        self._target_position = (None, None)  # [x, y]

    def _new_target_position_received_callback(self, pose):
        """
            Callback for the target position - updates the target position for the controller

            :param pose: Pose2D message containing the target position
        """
        self._target_position = (pose.x, pose.y)
        self.get_logger().info('New target position set: [%s, %s]' %
                               (self._target_position[0], self._target_position[1]))

    def _new_position_received_callback(self, pose):
        """
            Callback for the robot position updated - updates the robot position for the controller

            :param pose: Pose2D message containing the current robot position
        """
        position = (pose.x, pose.y)
        heading = pose.theta

        #  In case we don't have a target position, we use the current position as target = stops the robot
        if self._target_position[0] is None:
            self._target_position = position

        #  Compute the desired heading
        desired_heading = np.arctan2(self._target_position[1] - position[1], self._target_position[0] - position[0])

        #  Compute the distance to the target
        distance_to_target = np.sqrt(
            (self._target_position[1] - position[1]) ** 2 + (self._target_position[0] - position[0]) ** 2)

        self._heading_controller(desired_heading, heading, distance_to_target)

    def __sawtooth(self, x):
        return 2 * np.arctan(np.tan(x / 2))

    def _heading_controller(self, desired_heading, heading, distance_to_target):
        """
            Heading controller - computes the angular velocity to reach the desired heading and publishes control command
            Publishes a control command to "demo/cmd_vel"

            :param desired_heading: desired heading in radians
            :param heading: current heading in radians
            :param distance_to_target: distance to the target in pixels

            :return: None
        """

        MAX_SPEED = 3.0
        MAX_HEAD = 0.75

        u_x = distance_to_target / 70.0
        if abs(u_x) > MAX_SPEED:
            u_x = MAX_SPEED * np.sign(u_x)

        msg = Twist()
        msg.linear.x = float(u_x)
        msg.linear.y = 0.
        msg.linear.z = 0.

        msg.angular.x = 0.
        msg.angular.y = 0.
        u_z = -1 * self.__sawtooth(desired_heading - heading)
        if abs(u_z) > MAX_HEAD:
            u_z = MAX_HEAD * np.sign(u_z)
        msg.angular.z = u_z
        #  [-] .. clockwise rotation; [+] .. counter-clockwise rotation

        # Stop the robot - if he is close enough to the target
        if distance_to_target < 5:
            msg.linear.x = 0.
            msg.angular.z = 0.

        self._publisher.publish(msg)
        self.get_logger().info('heading/desired: "%s"/"%s"; err: "%s" => u_z = %s; u_x = %s' %
                               (heading, desired_heading, (desired_heading - heading), u_z, u_x))


def main(args=None):
    rclpy.init(args=args)

    robot_detector_publisher = RobotPositionController()

    rclpy.spin(robot_detector_publisher)

    robot_detector_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
