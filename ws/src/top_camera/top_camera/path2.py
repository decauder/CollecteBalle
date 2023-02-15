import numpy as np
import cv2



#################Parameters#################
class Path_Planner :
    def __init__(self,click1=0,click2=0,i=-1,scale=20/1216,passage1=np.array([0,4.5]),passage2=np.array([0,-4.5]),collect1=np.array([-9.19407895,-4.55592105]),collect2=np.array([9.14473684,4.6875]),path=[]) :
        self.click1=click1
        self.click2=click2
        self.i=i
        self.scale=scale
        self.passage1=passage1
        self.passage2=passage2
        self.collect1=collect1
        self.collect2=collect2
        self.path=path
        self.img=cv2.imread('terrain_1.png', 1)

#################Parameters#################

    # def distance(self.)

    def pixel_to_xy(self,lin,col):
        X=np.array([lin,col])
        n,p,_=self.img.shape
        return (X-np.array([p,n])/2)*self.scale

    def xy_to_pixel(self,x,y):
        X=x,y
        n,p,_=self.img.shape
        X=np.array([int(X[0]/self.scale+p/2),int(X[1]/self.scale+n/2)])
        return X

    def path_planner(self,x,y):
        X=(x,y)
        n,p,_=self.img.shape
        if x>0:
            x,y=self.xy_to_pixel(x,y)
            if (self.i==0) :
                self.path.append(self.collect2.tolist())
                x0,y0=self.xy_to_pixel(*self.collect2)

            else :
                x0,y0=self.click2[0],self.click2[1]
            # cv2.line(self.img, (x0,y0),(x,y), (255,0,0), 2)
            # return self.collect2
        else:
            x,y=self.xy_to_pixel(x,y)
            if (self.i==0) :
                self.path.append(self.collect1.tolist())
                x0,y0=self.xy_to_pixel(*self.collect1)
            else :
                x0,y0 = self.click2[0],self.click2[1]
            # x0,y0=int(collect1[0]/self.scale+p/2),int(collect1[1]/self.scale+n/2)
            # cv2.line(self.img, (x0,y0),(x,y), (255,0,0), 2)
            # return collect1

        #utilisation i=indice auquel on va ajouter un nouveau chemin
        if np.sign(self.pixel_to_xy(x,y)[0])==np.sign(self.pixel_to_xy(x0,y0)[0]) :
            self.path.append(self.pixel_to_xy(x,y).tolist())
        else :
            self.path=np.concatenate((np.array([self.collect1,self.pixel_to_xy(x,y),self.passage1]),np.array(self.path)),axis=0).tolist()
        for i in range(len(self.path)-1) :
            cv2.line(self.img,self.xy_to_pixel(self.path[i][0],self.path[i][1]),self.xy_to_pixel(self.path[i+1][0],self.path[i+1][1]),(255,0,0),2)
        print(self.path)

    def click_event(self,event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click2=self.click1
            # print(self.click2)
            self.click1=np.array([x,y])
            self.i+=1
            n,p,_=self.img.shape
            # print((click1-np.array([p,n])/2)*self.scale)
            # if self.i%2==1:
            cv2.circle(self.img, (x,y), 10, (0,0,255), 5)
            cv2.putText(self.img, str(len(self.path)-self.i+1), (x,y), cv2.FONT_HERSHEY_SIMPLEX,1, (255,255,0), 1, cv2.LINE_AA)
            # if self.i%2==0:
                # cv2.circle(self.img, (x,y), 10, (255,0,0), 5)
                # print(np.linalg.norm(self.click2-click1)*self.scale)
                # cv2.putText(self.img, str(self.i//2-1), (x,y), cv2.FONT_HERSHEY_SIMPLEX,
                # 1, (255,255,0), 1, cv2.LINE_AA)
            x,y=self.pixel_to_xy(x,y)
            # print(x,y)
            self.path_planner(x,y)
            cv2.imshow('image', self.img)

# driver function
if __name__ == "__main__":
    path_=Path_Planner()
    cv2.imshow('image', path_.img)
    cv2.setMouseCallback('image', path_.click_event)
    cv2.waitKey(0)
    cv2.destroyAllWindows()