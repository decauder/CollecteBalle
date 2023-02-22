import cv2 as cv
import numpy as np


class PathPlanner:
    def __init__(self, img, pos=[0, 0], prev_pos=[0, 0], i=0,
                 scale=20 / 1216, passage_n=np.array([0, 4.5]),
                 passage_s=np.array([0, -4.5]), collect_no=np.array([-9.19407895, -4.55592105]),
                 collect_se=np.array([9.14473684, 4.6875]), path=[]):
        self.pos = pos  # position de l'ancienne balle
        self.prev_pos = prev_pos  # position de la nouvelle balle
        self.i = i  # incrementeur
        # echelle pour le changement de coordonnées (xy et pixels)
        self.scale = scale
        self.passage_n = passage_n  # passage entre le filet et le mur au nord
        self.passage_s = passage_s  # passage entre le filet et le mur au sud
        self.collect_no = collect_no  # point de collecte au nord ouest
        self.collect_se = collect_se  # point de collecte au sud est
        self.path = []  # liste de la trajectoire à suivre pour ramener les balles
        self.ind = 0  # indice dans le path de la balle ciblée actuellement
        self.img = img

    def pixel_to_xy(self, lin, col):
        X = np.array([lin, col])
        n, p, _ = self.img.shape
        return (X - np.array([p, n]) / 2) * self.scale

    def xy_to_pixel(self, x, y):
        X = x, y
        n, p, _ = self.img.shape
        X = np.array([int(X[0] / self.scale + p / 2),
                     int(X[1] / self.scale + n / 2)])
        return X

    def distance(x0, x1):
        return np.sqrt((x0[0] + x1[0])**2 + (x0[1] + x1[1])**2)

    def path_planner(self, x, y):
        self.prev_pos = self.pos
        self.pos = np.array([x, y])  # en coordonnées xy
        X = (x, y)
        n, p, _ = self.img.shape
        self.path = np.array(self.path)
        first_ball_detected = (len(self.path) != 0 and ((self.path[0] == np.array(self.collect_se))[
                               0] or (self.path[0] == np.array(self.collect_no))[0]))
        if first_ball_detected:
            self.i += 1  # i>0 pour ne pas ajouter le point de collecte

        if x > 0:  # balle sur le terrain de droite
            if (self.i == 0):  # on va s'occuper d'ajouter le point de collecte pour que le robot termine par déposer les balles dans
                self.path = np.array([self.collect_se])
                x0, y0 = self.collect_se.tolist()
            else:
                x0, y0 = self.prev_pos[0], self.prev_pos[1]

        else:  # balle sur le terrain de gauche
            if (self.i == 0):
                self.path = np.array([self.collect_no])
                x0, y0 = self.collect_no.tolist()
            else:
                x0, y0 = self.prev_pos[0], self.prev_pos[1]

        if np.sign(self.pos[0]) == np.sign(self.path[self.ind][0]):
            self.ind += 1
            self.path = np.insert(self.path, self.ind,
                                  self.pos.tolist(), axis=0)
        else:
            self.ind = 2
            k = np.sign(self.pos[0])
            passage = (1 + k) / 2 * self.passage_s + \
                (1 - k) / 2 * self.passage_n
            collect = (1 + k) / 2 * self.collect_se + \
                (1 - k) / 2 * self.collect_no
            self.path = np.concatenate(
                (np.array([collect, self.pos.tolist(), passage]), self.path), axis=0).tolist()

    # TO DELETE

    def click_event(self, event, x, y, flags, params):
        if event == cv.EVENT_LBUTTONDOWN:
            x_, y_ = self.pixel_to_xy(x, y)
            print(self.i)
            if x_ > 0:
                n, p, _ = self.img.shape
                cv.circle(self.img, (x, y), 10, (0, 0, 255), 5)
                cv.putText(self.img, str(len(self.path) - self.i + 1), (x, y),
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 1, cv2.LINE_AA)
                for i in range(len(self.path) - 1):
                    cv.line(self.img, self.xy_to_pixel(self.path[i][0], self.path[i][1]), self.xy_to_pixel(
                        self.path[i + 1][0], self.path[i + 1][1]), (255, 0, 0), 2)
                self.path_planner(x_, y_)
                cv.imshow('Terrain', self.img)
