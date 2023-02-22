import cv2 as cv
import numpy as np
from rclpy.node import Node
from zenith_camera_subscriber import *
from tools.hsv_range_finder import HSVRangeFinder


class RobotDetector(Node):
    """
        Class for detecting the robot position and heading using the top camera

        Public methods:
            - get_position: returns the robot position (x, y) in pixels
            - get_heading: returns the robot heading in radians

        Private methods:
            - _camera_data_updated_callback: callback function for the camera data

    """

    def __init__(self, data_updated_callback, debug=False, display_camera=False):
        """
            Constructor

            :param data_updated_callback: called when the robot's position and heading is updated
            :param debug: if True, prints debugging information
            :param display_camera: if True, displays the camera image
        """
        self._position = [0, 0]
        self._heading = 0
        self._timestamp = 0
        self._terrain = None
        self._hsvFinder = None
        self._Camera = ZenithCameraSubscriber(
            self._camera_data_updated_callback)
        self.data_updated_callback = data_updated_callback
        self.__debug = debug
        self.__display_camera = display_camera
        rclpy.spin(self._Camera)

    def _camera_data_updated_callback(self, img, timestamp):
        self._terrain = img

        # #  GUI for finding the correct HSV range
        # if (self._hsvFinder is None):
        #     # Create an instance of the ColorThreshold class with the image
        #     self._hsvFinder = HSVRangeFinder(self._terrain)
        #     self._hsvFinder.run()

        #  Detect the markers on the robot
        blue_marker = [
            (116, 200, 88),
            (137, 255, 175)
        ]
        # green_marker = [
        #     (39, 0, 77),
        #     (63, 255, 149)
        # ]
        # red_marker = [
        #     (0, 35, 67),
        #     (10, 255, 202)
        # ]
        # purple_marker = [
        #     (130, 27, 70),
        #     (168, 255, 199)
        # ]
        black_marker = [
            (0, 0, 0),
            (180, 255, 40)
        ]

        front_m_pos = self._detect_marker(black_marker[0], black_marker[1])
        rear_m_pos = self._detect_marker(blue_marker[0], blue_marker[1])

        if self.__debug:
            print("Front marker: ", front_m_pos)
            print("Rear marker: ", rear_m_pos)

        #  If both markers are detected, calculate the position and heading of the robot
        if (front_m_pos[0] is not None and rear_m_pos[0] is not None):
            self._heading = np.arctan2(
                front_m_pos[1] - rear_m_pos[1], front_m_pos[0] - rear_m_pos[0])
            self._position = [rear_m_pos[0], rear_m_pos[1]]
            self._timestamp = timestamp

            self.data_updated_callback(
                self._position, self._heading, self._timestamp)
            if self.__debug:
                print('angle', np.rad2deg(self._heading))

        if self.__display_camera:
            cv.imshow('Terrain', self._terrain)
            cv.waitKey(1)

    def _detect_marker(self, hsv_min=(24, 1, 1), hsv_max=(51, 255, 255)):
        """
            Detects a single marker on the robot

            :param hsv_min: minimum HSV value for the marker's color
            :param hsv_max: maximum HSV value for the marker's color

            :return:
                (x, y) position of the marker in pixels;
                (None, None) if no or multiple markers are detected
        """
        frame_HSV = cv.cvtColor(self._terrain, cv.COLOR_BGR2HSV)
        frame_threshold = cv.inRange(frame_HSV, hsv_min, hsv_max)
        ret, thresh = cv.threshold(frame_threshold, 127, 255, cv.THRESH_BINARY)
        contours, h = cv.findContours(thresh, cv.RETR_CCOMP,
                                      cv.CHAIN_APPROX_NONE)

        n_contours = len(contours)

        if n_contours == 1:
            cnt = contours[0]
            # find circle
            (cx, cy), radius = cv.minEnclosingCircle(cnt)
            cv.circle(self._terrain, (int(cx), int(cy)), int(
                radius), (0, 255, 0), 1)  # draw circle

            return (cx, cy)
        else:
            return (None, None)  # No marker / multiple markers found

    def get_heading(self):
        """
            Returns the robot heading in radians (0 = East, -pi/2 = North, +/-pi = West, pi/2 = North)
        """
        return self._heading

    def get_position(self):
        """
            Returns the robot position (x, y) in pixels
        """
        return self._position


def dummy_cb(position, heading, timestamp):
    pass


def main(args=None):
    rclpy.init(args=args)

    #  For testing / debugging purposes:
    rd = RobotDetector(dummy_cb, debug=True, display_camera=True)
    rd.Camera.destroy_node()


if __name__ == '__main__':
    main()
