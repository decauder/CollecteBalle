import numpy as np
import cv2



#################Parameters#################

click1=0
click2=0
i=-1
scale=20/1216
passage1=np.array([0,4.5])
passage2=np.array([0,-4.5])
colect1=np.array([-9.19407895,-4.55592105])
colect2=np.array([9.14473684,4.6875])
path=[]

#################Parameters#################

def pixel_to_xy(lin,col,img_shape):
    X=np.array([lin,col])
    n,p,_=img_shape
    return (X-np.array([p,n])/2)*scale

def xy_to_pixel(x,y,img_shape):
    X=x,y
    n,p,_=img_shape
    X=np.array([int(X[0]/scale+p/2),int(X[1]/scale+n/2)])
    return X

def path_planner(x,y,img,click2,i):
    X=(x,y)
    n,p,_=img.shape
    if x>0:
        x,y=xy_to_pixel(x,y,img.shape)
        if (i==0) :
            path.append(colect2.tolist())
            x0,y0=xy_to_pixel(*colect2,img.shape)
        # if (sign(pixel_to_xy(click2[0],click2[1],img.shape)[0])!=sign(pixel_to_xy(click1[0],click1[1],img.shape)[0])) :

        else :
            x0,y0=click2[0],click2[1]
        cv2.line(img, (x0,y0),(x,y), (255,0,0), 2)
        # return colect2
    else:
        x,y=xy_to_pixel(x,y,img.shape)
        if (i==0) :
            path.append(colect1.tolist())
            x0,y0=xy_to_pixel(*colect1,img.shape)
        else :
            x0,y0 = click2[0],click2[1]
        # x0,y0=int(colect1[0]/scale+p/2),int(colect1[1]/scale+n/2)
        cv2.line(img, (x0,y0),(x,y), (255,0,0), 2)
        # return colect1
    path.append(pixel_to_xy(x,y,img.shape).tolist())
    print(path)
    # print(pixel_to_xy(x,y,img.shape))

def click_event(event, x, y, flags, params):
    global i,click1,click2
    if event == cv2.EVENT_LBUTTONDOWN:
        i+=1
        click2=click1
        # print(click1)
        click1=np.array([x,y])
        n,p,_=img.shape
        # print((click1-np.array([p,n])/2)*scale)
        if i%2==1:
            cv2.circle(img, (x,y), 10, (0,0,255), 5)
            cv2.putText(img, str(i//2), (x,y), cv2.FONT_HERSHEY_SIMPLEX,
            1, (255,255,0), 1, cv2.LINE_AA)
        if i%2==0:
            cv2.circle(img, (x,y), 10, (255,0,0), 5)
            # print(np.linalg.norm(click2-click1)*scale)
            cv2.putText(img, str(i//2-1), (x,y), cv2.FONT_HERSHEY_SIMPLEX,
            1, (255,255,0), 1, cv2.LINE_AA)
        x,y=pixel_to_xy(x,y,img.shape)
        path_planner(x,y,img,click2,i)
        cv2.imshow('image', img)

# def detect_balls(terrain,nb_balls,balls,time):
#     frame_HSV = cv.cvtColor(terrain, cv.COLOR_BGR2HSV)
#     frame_threshold = cv.inRange(frame_HSV, (24, 1, 1), (51, 255, 255))
#     ret, thresh = cv.threshold(frame_threshold, 127, 255, cv.THRESH_BINARY)
#     contours, h = cv.findContours(
#         thresh, cv.RETR_CCOMP, cv.CHAIN_APPROX_NONE)
#     ball_centers = []
#     for cnt in contours:
#         (cx, cy), radius = cv.minEnclosingCircle(cnt)
#         ball_centers.append([int(cx), int(cy)])
#         cv.drawContours(terrain, [cnt], 0, (0, 0, 255), -1)
#     n = len(ball_centers)
#     for i in range(nb_balls):
#         # print('boucle')
#         k = find_match(balls[i], ball_centers)
#         if k != -1:
#             balls[i].set_position(ball_centers[k])
#             ball_centers.pop(k)
#     if nb_balls < n:
#         for j in range(len(ball_centers)):
#             # print('new_ball')
#             nb_balls = nb_balls+1
#             balls.append(
#                 Ball(nb_balls, time, ball_centers[j]))
#     for b in balls:
#         b.set_time(time)
#         cv2.putText(terrain, str(b.id), tuple(b.position), cv2.FONT_HERSHEY_SIMPLEX,
#                     1, (255, 255, 0), 1, cv2.LINE_AA)

# def find_match(ball, ball_centers):
#         if len(ball_centers) == 0:
#             return -1
#         else:
#             min = 5000
#             # print(len(ball_centers))
#             for i in range(len(ball_centers)):
#                 n = np.sqrt((ball_centers[i][0]-ball.position[0])
#                             ** 2+(ball_centers[i][1]-ball.position[1])**2)
#                 if n < min:
#                     min = n
#                     closest_ball = i
#             return closest_ball

# driver function
if __name__ == "__main__":
    img = cv2.imread('terrain_1.png', 1)
    cv2.imshow('image', img)
    cv2.setMouseCallback('image', click_event)
    cv2.waitKey(0)
    cv2.destroyAllWindows()