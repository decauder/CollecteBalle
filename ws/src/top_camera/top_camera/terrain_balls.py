import cv2 as cv
import numpy as np
from zenith_camera_subscriber import *
from path_planner import PathPlanner
from ball import Ball


class TerrainBalls:
    def __init__(self, timeStamp):
        self.time = timeStamp
        self.score = 0
        self.terrain = None
        self.balls = []
        self.nb_balls = 0
        rclpy.init()
        self.Camera = ZenithCameraSubscriber(self._update)
        self.path_ = PathPlanner(self.Camera)
        rclpy.spin(self.Camera)

    def _detect_balls(self, terrain):
        """
            Detects the balls on the terrain and calculates a score for each ball
            All detected balls with their score are stored in the self.balls list

            :param terrain: the terrain image
        """

        # -- Balls detection --
        frame_HSV = cv.cvtColor(terrain, cv.COLOR_BGR2HSV)
        frame_threshold = cv.inRange(frame_HSV, (16, 65, 90), (45, 255, 156))
        ret, thresh = cv.threshold(frame_threshold, 127, 255, cv.THRESH_BINARY)
        contours, h = cv.findContours(
            thresh, cv.RETR_CCOMP, cv.CHAIN_APPROX_NONE)
        ball_centers = []
        for cnt in contours:
            (cx, cy), radius = cv.minEnclosingCircle(cnt)
            if radius > 2.5:
                ball_centers.append([int(cx), int(cy)])
                cv.drawContours(terrain, [cnt], 0, (255, 0, 0), -1)

        # -- Filter the balls --
        n = len(ball_centers)
        for i in range(self.nb_balls):
            k = self._find_match(self.balls[i], ball_centers)
            if k != -1:
                self.balls[i].set_position(ball_centers[k])
                ball_centers.pop(k)

        #  -- Add balls to the final list --
        if self.nb_balls < n:
            for j in range(len(ball_centers)):
                self.nb_balls = self.nb_balls + 1
                self.balls.append(
                    Ball(self.nb_balls, self.time, ball_centers[j]))

        # -- Path planning --
        for b in self.balls:
            b.set_time(self.time)
            # Line bug
            X = b.position
            X_ = self.path_.pixel_to_xy(X[0], X[1])
            self.path_.path_planner(X_[0], X_[1])
            for i in range(len(self.path_.path) - 1):
                cv2.line(self.terrain, self.path_.xy_to_pixel(
                    *self.path_.path[i]), self.path_.xy_to_pixel(*self.path_.path[i + 1]), (255, 0, 0), 2)

            cv.putText(terrain, str(b.id), tuple(b.position), cv.FONT_HERSHEY_SIMPLEX,
                       1, (255, 255, 0), 1, cv.LINE_AA)

    def _find_match(self, ball, ball_centers):
        """
            Matches balls

            :param ball: the ball to match
            :param ball_centers: the list of ball centers

            :return: the index of the closest ball center
        """

        if len(ball_centers) == 0:
            return -1
        else:
            min = 5000

            for i in range(len(ball_centers)):
                n = np.sqrt((ball_centers[i][0] - ball.position[0]) **
                            2 + (ball_centers[i][1] - ball.position[1]) ** 2)

                if n < min:
                    min = n
                    closest_ball = i
        return closest_ball

    def _show_balls(self, terrain):
        """
            Shows the terrain image with the balls

            :param terrain: the terrain image
        """
        cv.imshow('Terrain', terrain)
        cv.waitKey(1)

    def __del__(self):
        self.Camera.destroy_node()
        cv.destroyAllWindows()
        rclpy.shutdown()

    def _update(self, img, timeStamp):
        """
            Called when a new image is received from the camera

            Does the following:
                - updates the terrain image
                - updates the path planner
                - detects the balls
                - shows the balls

            :param img: the new image
            :param timeStamp: the time stamp of the image
        """
        self.terrain = img
        self.path_ = PathPlanner(self.terrain)
        self.time = timeStamp
        self._detect_balls(self.terrain)
        self._show_balls(self.terrain)


def main(args=None):

    terrain_balls = TerrainBalls(0)
    terrain_balls.Camera.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
