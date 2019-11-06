import cv2 as cv
import numpy as np
from base_proc import BaseProc
from numpy.linalg import norm
from pickle import dump

class ColorSeg(BaseProc):

    def __init__(self, scale=0.7):
        super().__init__(scale)

        self.next_session = False
        self.paused = False

    def showPixelValue(self, event, x, y, flags, param):
        if event == cv.EVENT_MOUSEMOVE and not self.paused:
            bgr = self.frame[y, x]
            self.min_ycb = cv.cvtColor(np.uint8([[bgr]]), cv.COLOR_BGR2LAB)[0][0]

    def pixel_session(self):
        cv.namedWindow('Pixel', cv.WINDOW_AUTOSIZE)
        cv.setMouseCallback('Pixel', self.showPixelValue)
        while True:
            if not self.paused:
                _, self.frame = self.cap.read()
                self.img_resize()

            cv.imshow('Pixel', self.frame)

            k = cv.waitKey(5) & 0xFF
            if k == ord('q'):
                cv.destroyAllWindows()
                break
            elif k == ord(' '):
                self.paused = not self.paused
            elif k == ord('n'):
                self.next_session = not self.next_session
                cv.destroyWindow('Pixel')
                break

        self.select_session()

    def onTrackbarActivity(self, x):
        pass

    def select_session(self):
        self.next_session = not self.next_session

        cv.namedWindow('SelectYCB',cv.WINDOW_AUTOSIZE)
        cv.createTrackbar('y_min','SelectYCB',self.min_ycb[0],255,self.onTrackbarActivity)
        cv.createTrackbar('y_max','SelectYCB',self.max_ycb[0],255,self.onTrackbarActivity)
        cv.createTrackbar('cr_min','SelectYCB',self.min_ycb[1],255,self.onTrackbarActivity)
        cv.createTrackbar('cr_max','SelectYCB',self.max_ycb[1],255,self.onTrackbarActivity)
        cv.createTrackbar('cb_min','SelectYCB',self.min_ycb[2],255,self.onTrackbarActivity)
        cv.createTrackbar('cb_max','SelectYCB',self.max_ycb[2],255,self.onTrackbarActivity)

        while True:
            if not self.paused:
                _, self.frame = self.cap.read()
                self.img_resize()

            y_min = cv.getTrackbarPos('y_min','SelectYCB')
            cr_min = cv.getTrackbarPos('cr_min','SelectYCB')
            cb_min = cv.getTrackbarPos('cb_min','SelectYCB')
            y_max = cv.getTrackbarPos('y_max','SelectYCB')
            cr_max = cv.getTrackbarPos('cr_max','SelectYCB')
            cb_max = cv.getTrackbarPos('cb_max','SelectYCB')

            self.min_ycb = np.array([y_min, cr_min, cb_min])
            self.max_ycb = np.array([y_max, cr_max, cb_max])

            result_ycb = self.cvt_ycb(self.frame)
            cv.imshow('SelectYCB', result_ycb)

            k = cv.waitKey(5) & 0xFF
            if k == ord('q'):
                cv.destroyAllWindows()
                break
            elif k == ord(' '):
                self.paused = not self.paused

            elif k == ord('n'):
                self.next_session = not self.next_session
                cv.destroyWindow('SelectYCB')
                break

        self.morph_session()

    def morph_update(self, dummy=None):
        self.k_size = cv.getTrackbarPos('k_size', 'morphology')*2+1
        self.iters = cv.getTrackbarPos('iters', 'morphology')
        self.morph_elem = cv.getStructuringElement(cv.MORPH_ELLIPSE, (self.k_size, self.k_size))

    def morph_session(self):
        cv.namedWindow('morphology',cv.WINDOW_AUTOSIZE)
        cv.createTrackbar('k_size', 'morphology', self.k_size, 21, self.morph_update)
        cv.createTrackbar('iters', 'morphology', self.iters, 10, self.morph_update)
        self.morph_update()

        while True:
            if not self.paused:
                _, self.frame = self.cap.read()
                self.img_resize()

            self.morph_transform(self.cvt_ycb(self.frame))
            cv.imshow('morphology', self.morph_img)

            k = cv.waitKey(5) & 0xFF
            if k == ord('q'):
                cv.destroyAllWindows()
                break
            elif k == ord(' '):
                self.paused = not self.paused
                break
            elif k == ord('d'):
                color = input(''.join(["{}:{}\n".format(i,c) for i,c in enumerate(self.colors)])+'select:')

                self.dump_color(self.colors[int(color)])
            elif k == ord('s'):
                self.save_config()

            self.morph_update()

        self.contour_session()

    def contour_session(self):
        cv.namedWindow('contour',cv.WINDOW_AUTOSIZE)
        found = False
        ball = np.array([0,0,0,0])
        while True:
            if not self.paused:
                _, self.frame = self.cap.read()
                self.img_resize()

            self.morph_transform(self.cvt_ycb(self.frame))
            coordinates = self.select_area()

            if coordinates is not None:
                if not found:
                    found = True
                    ball = coordinates[-1]

                ball = coordinates[np.argmin([norm(ball[:2] - c[:2]) for c in coordinates])]

                for c in coordinates:
                    self.draw_ctr(c)

                self.draw_ctr(ball, (255,0,0))

            cv.imshow('contour', self.frame)
            k = cv.waitKey(5) & 0xFF
            if k == ord('q'):
                cv.destroyAllWindows()
                break
            elif k == ord(' '):
                self.paused = not self.paused

    def save_config(self):
        print("dump pickle!")
        dump(self.store, open("../config.pkl", 'wb'))

    def dump_color(self, color):
        self.store[color] = (self.min_ycb, self.max_ycb,self.k_size, self.iters)

    def run(self):
        if self.restore_config():
            self.select_session()
        else:
            self.pixel_session()

if __name__ == '__main__':
    ColorSeg().run()
