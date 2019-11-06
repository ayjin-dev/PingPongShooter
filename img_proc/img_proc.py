from threading import Thread
from queue import Empty
from .base_proc import BaseProc
from cv2 import WINDOW_AUTOSIZE, namedWindow, imshow, waitKey, destroyAllWindows

class ImgProc(BaseProc):

    def __init__(self, scale, img_q, mde_q, debug=True):
        super().__init__(scale)
        self.debug = debug
        self.img_q = img_q
        self.mde_q = mde_q

        self.exit = False
        self.restore_config()

    def processing(self):
        if self.debug:
            namedWindow('main', WINDOW_AUTOSIZE)

        self.change_color(list(self.store.keys())[0])

        while True:
            _, self.frame = self.cap.read()
            self.img_resize()

            try:
                mode = self.mde_q.get_nowait()
                if mode is not None and self.debug:
                    self.draw_ctr(mode, (255,0,0))
                # elif mode[0] == 'c':
                #     self.change_color(mode[1])
            except Empty:
                pass

            self.morph_transform()
            coordinates = self.select_area()
            self.img_q.put(item=coordinates, block=False)


            if self.debug and coordinates is not None:
                [self.draw_ctr(c) for c in coordinates]
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
