import cv2 as cv
import numpy as np
from zenith_camera_subscriber import *

class Ball:
    def __init__(self,id,timeStamp,X):
        self.time0=timeStamp
        self.elapsedTime=0
        self.score=0
        self.position=X
        self.id=id

    def get_new_position(self,X):
        self.position=X

    def get_new_time(self,timeStamp):
        self.elapsedTime=timeStamp-self.time0

class TerrainBalls:
    def __init__(self,timeStamp):
        self.time=timeStamp
        self.score=0
        self.terrain=None
        self.balls=[]
        self.nb_balls=0
        rclpy.init()
        self.Camera=ZenithCameraSubscriber(self.update)
        rclpy.spin(self.Camera)

    
    def detect_balls(self):
        frame_HSV = cv.cvtColor(self.terrain, cv.COLOR_BGR2HSV)
        frame_threshold = cv.inRange(frame_HSV, (24, 1, 1), (51, 255, 255))
        ret,thresh = cv.threshold(frame_threshold,127,255,cv.THRESH_BINARY)
        kernel=np.ones((3,3))
        opening = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
        cv2.imshow('Test',opening)
        cv2.waitKey(1)
        contours,h = cv.findContours(opening,cv.RETR_CCOMP,cv.CHAIN_APPROX_NONE)
        ball_centers=[]
        for cnt in contours:
            (cx, cy), radius = cv.minEnclosingCircle(cnt)
            # if radius>2.5:
            ball_centers.append([int(cx),int(cy)])
            cv.drawContours(self.terrain, [cnt], 0, (0, 0, 255), -1)
        n=len(ball_centers)
        if self.nb_balls==n:
            for i in range(self.nb_balls):
                print('boucle')
                k=self.findMatch(self.balls[i],ball_centers)
                if k!=-1:
                    self.balls[i].get_new_position(ball_centers[k])
                    ball_centers.pop(k)
        elif self.nb_balls<n:
            for j in range(len(ball_centers)):
                print('new_ball')
                self.nb_balls=self.nb_balls+1
                self.balls.append(Ball(self.nb_balls,self.time,ball_centers[j]))
        for b in self.balls:
            b.get_new_time(self.time)
            cv2.putText(self.terrain, str(b.id), tuple(b.position), cv2.FONT_HERSHEY_SIMPLEX,
            1, (255,255,0), 1, cv2.LINE_AA)

    def findMatch(self,ball,ball_centers):
        # match=False
        # ball_positions=np.zeros((2,len(ball_centers)))
        # for i in range(len(self.balls)):
        #     ball_positions[:,i]=np.array([ball_centers[i]])
        # ball_distances=np.linalg.norm(ball_positions.T-ball.position,axis=1)
        # closest_ball=np.argmin(ball_distances)
        if len(ball_centers)==0:
            return -1
        else: 
            min=5000
            print(len(ball_centers))
            for i in range(len(ball_centers)):
                n=np.sqrt((ball_centers[i][0]-ball.position[0])**2+(ball_centers[i][1]-ball.position[1])**2)
                if n<min:
                    min=n
                    closest_ball=i
            return closest_ball

    def show_balls(self):
        cv.imshow('Terrain',self.terrain)
        cv.waitKey(1)
    
    def __del__(self):
        self.Camera.destroy_node()
        cv.destroyAllWindows()
        rclpy.shutdown()

    def update(self,img,timeStamp):
        self.terrain=img
        self.time=timeStamp
        self.detect_balls()
        self.show_balls()

t=TerrainBalls(0)
t.Camera.destroy_node()
