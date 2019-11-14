from threading import Thread
from queue import Empty
from .base_proc import BaseProc, SCALE,RESOLUTIONS
from cv2 import WINDOW_AUTOSIZE, namedWindow, imshow, waitKey, destroyAllWindows,line
from numpy import array,argmin,around
from numpy.linalg import norm

def scale_factor(x):
    return around(array(x)*SCALE).astype(int).tolist()

SCALE_BOTTOM_CENTER_PREFER_DISTANCE = 20
SCALE_BOTTOM_CENTER =  tuple((RESOLUTIONS * SCALE).astype(int) // (2,1) - (0, SCALE_BOTTOM_CENTER_PREFER_DISTANCE))
READY_CLIP = (684, 575)
CLIPPER = (684, 658)

SCALE_CLIPPER = scale_factor(CLIPPER)
SCALE_READY_CLIP = scale_factor(READY_CLIP)


class Ball:
    def __init__(self):
        self.__found = False
        self.__ball = None
    def run(self, coordinates):
        if coordinates is not None:
            if self.__found:
                self.__ball = coordinates[argmin([norm(self.__ball[:2] - c[:2]) for c in coordinates])]
            else:
                self.__ball = coordinates[-1]
                self.__found = True
            return self.__ball
        else:
            return None

class GreenZone:
    def __init__(self):
        self.__found = False

    def run(self, coordinate):
        if coordinate is not None:
            return coordinate[-1]
        else:
            return None

class ImgProc(BaseProc):
    def __init__(self, img_q, mde_q, debug=True):
        super().__init__()
        self.debug = debug
        self.img_q = img_q
        self.mde_q = mde_q
        self.exit = False

        self.restore_config()

        self.mode = None
        self.draw_mode = None


    def draw_ball(self, ball):
        if ball is not None:
            self.draw_ctr(ball, (255,0,0))

    def draw_green(self, green):
        if green is not None:
            self.draw_ctr(green)
            x,y,w,h = green
            line(self.frame, (x,y), (x+w,y), (255,0,0), 2)
            line(self.frame, (SCALE_BOTTOM_CENTER[0]-30, SCALE_BOTTOM_CENTER[1]), (SCALE_BOTTOM_CENTER[0]+30, SCALE_BOTTOM_CENTER[1]), (0,0,255), 2)


    def select_mode(self, mode):
        if mode == 'ball':
            self.change_color('red')
            self.mode = Ball()
            if self.debug:
                self.draw_mode = self.draw_ball
        elif mode == 'green_zone':
            self.change_color('green')
            self.mode = GreenZone()
            if self.debug:
                self.draw_mode = self.draw_green
        elif mode == 'barrel':
            self.change_color('pink')
            self.mode = GreenZone()
            if self.debug:
                self.draw_mode = self.draw_green


    def processing(self):
        mode = self.mde_q.get()
        self.select_mode(mode)
        if self.debug:
            namedWindow('main', WINDOW_AUTOSIZE)

        while not self.exit:
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
                self.draw_mode(coordinate)
                if coordinates is not None:
                    [self.draw_ctr(c,lw=1) for c in coordinates]

                # print('coordinate:', coordinate)
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
