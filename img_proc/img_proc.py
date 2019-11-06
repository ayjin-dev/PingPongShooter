from threading import Thread
from queue import Empty
from .base_proc import BaseProc
from cv2 import WINDOW_AUTOSIZE, namedWindow, imshow, waitKey, destroyAllWindows,line
from numpy import array,argmin
from numpy.linalg import norm

CLIPPER = (181, 188), (204, 189)# left clipper, right clipper

class Ball:
    def __init__(self):
        self.__found = False
        self.ball = None
    def run(self, coordinates):
        if coordinates is not None:
            if self.__found:
                self.ball = coordinates[argmin([norm(self.ball[:2] - c[:2]) for c in coordinates])]
            else:
                self.ball = coordinates[-1]
                self.__found = True

            # targ_p = self.ball[:2] + self.ball[-2:]//2
            return self.ball
        else:
            return None


class ImgProc(BaseProc):
    def __init__(self, scale, img_q, mde_q, debug=True):
        super().__init__(scale)
        self.debug = debug
        self.img_q = img_q
        self.mde_q = mde_q
        self.exit = False

        self.mode = None
        self.draw_mode = None
        self.cur_mode = None

    def draw_ball(self, ball):
        if ball is not None:
            self.draw_ctr(ball)
        line(self.frame, CLIPPER[0], CLIPPER[1], (0,0,255), 2)

    def select_mode(self, mode):
        if mode == 'ball':
            self.change_color('red')
            self.mode = Ball()
            if self.debug:
                self.draw_mode = self.draw_ball
        elif mode == 'green_zone':
            pass

    def processing(self):
        mode = self.mde_q.get()
        self.select_mode(mode)
        if self.debug:
            namedWindow('main', WINDOW_AUTOSIZE)

        while True:
            _, self.frame = self.cap.read()
            self.img_resize()

            try:
                mode = self.mde_q.get_nowait()
                self.select_mode(mode)
            except Empty:
                pass

            self.morph_transform()
            coordinates = self.select_area()

            coordinate = self.mode.run(coordinates)
            self.img_q.put(item=coordinate, block=False)

            if self.debug:
                # [self.draw_ctr(c) for c in coordinates]
                self.draw_mode(coordinate)
                print('coordinate:', coordinate)
                imshow('main', self.frame)
                k = waitKey(1) & 0xFF
                if k == ord('q'):
                    destroyAllWindows()
                    break

    def start(self):
        t = Thread(target=self.processing)
        self.thread = t
        self.deamon = True
        t.start()

    def stop(self):
        self.exit = True
        self.thread.join()


class ImageProcessingThread(Thread):
    def __init__(self, img_p,  exit_condition):
        super(ImageProcessingThread, self).__init__()
        self.deamon = True
        self.exit_condition = exit_condition
        self.img_p = img_p

    def run(self):
        img_p = self.img_p
        img_p.start()

        with self.exit_condition:
            self.exit_condition.wait()

        img_p.stop()
