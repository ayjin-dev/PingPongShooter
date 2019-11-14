from queue import Queue, Empty, Full
from threading import Lock, Condition, Event
from time import sleep
from glob import glob
import sys

from car_control.commander import CommandThread,ARM_DOWN,ARM_UP,CLIPPER_CLOSE,CLIPPER_CLIP,CLIPPER_OPEN,CAM_VIEW,CAM_FULL_VIEW, ARM_REAL_UP
from img_proc.img_proc import ImageProcessingThread, ImgProc
from car_control.rst_serial import CustomQueue
from functions import PickBall, GreenZone, Barrel

class mainControl:
    def __init__(self, cmd_q, img_q, mde_q):
        self.cmd_clear = cmd_q.clear
        self.cmd_put = cmd_q.put_nowait
        self.img_get = img_q.get
        self.mde_q = mde_q
        self.exit = False
        self.n_full, self.n_total = 0, 0

        self.init_mode = [False, False, False]
        self.cur_mode = 0

        self.mde_q.put('ball')

    def run(self):
        while not self.exit:
            try:
                coordinate = self.img_get()
                # NOTE: test run...
                # NOTE: pick_ball, green_zone, shoot_barrel
                # if self.green_zone(coordinate):
                #     return True

                # NOTE: auto mode..
                if self.cur_mode == 0:
                    should_next = self.pick_ball(coordinate)
                elif self.cur_mode == 1:
                    should_next = self.green_zone(coordinate)
                elif self.cur_mode == 2:
                    should_next = self.shoot_barrel(coordinate)
                elif self.cur_mode == 3:
                    should_next = self.reset_mode()
                if should_next:
                    self.cur_mode += 1

            except KeyboardInterrupt:
                break
        self.stop()

    def send(self, o, p):
        try:
            self.cmd_put((o,p))
            sleep(0.01)
        except Full:
            self.n_full += 1
            print('got full:{}'.format(o,p))
        self.n_total += 1

    def stop(self):
        self.cmd_clear()
        self.send('spst', 0)
        print('full:{}, total:{}'.format(self.n_full, self.n_total))

    def reset_mode(self):
        self.init_mode = [False, False, False]
        self.cur_mode = 0
        self.stop()
        sys.exit()
        return False

    def pick_ball(self, coordinate):
        if not self.init_mode[0]:
            self.mde_q.put('ball')
            self.send('spst', 0)
            self.init_mode[0] = True
            self.send('clip', CLIPPER_CLOSE)
            self.send('arm', ARM_DOWN)
            self.send('cam', CAM_VIEW)
            sleep(0.5)
            self.PickBall = PickBall()

        state, param = self.PickBall.run(coordinate)
        if state == 0 and param is not None:
            self.send('spds', param)
        elif state == 1:
            self.send('spst', 0)
            sleep(0.01)
            self.send('clip', CLIPPER_CLIP)
            sleep(0.1)
            self.send('spds', (-150,-150))
            sleep(0.1)
            self.send('spst', 0)
            sleep(0.01)
            self.send('arm', ARM_UP)
            sleep(.8)
            self.send('clip', CLIPPER_OPEN)
            sleep(0.1)
            self.stop()
            return True
        elif state == 2:
            self.send('spst', 0)
            sleep(0.1)
            self.send('clip', CLIPPER_OPEN)
        elif state == 3:
            self.send('clip', CLIPPER_CLOSE)
        return False

    def green_zone(self,coordinate):
        if not self.init_mode[1]:
            self.send('spst', 0)
            self.mde_q.put('green_zone')
            self.send('cam', CAM_FULL_VIEW)
            self.send('arm', ARM_UP)
            self.send('clip', CLIPPER_OPEN)
            sleep(0.5)
            self.init_mode[1] = True
            self.GreenZone = GreenZone()

        state, param = self.GreenZone.run(coordinate)
        if param is not None:
            if state == 0:
                self.send('spds', param)
            elif state == 2:
                self.send('spst', param)
        elif state == 1:
            self.send('spst', 0)
            self.mde_q.put('barrel')
            return True
        elif state == 3:
            self.send('spst', 0)
            self.send('arm', ARM_REAL_UP)
            self.send('cam', CAM_VIEW)
            sleep(0.2)
        return False

    def shoot_barrel(self, coordinate):
        if not self.init_mode[2]:
            self.send('spst', 0)
            self.mde_q.put('barrel')
            self.send('cam', CAM_FULL_VIEW)
            sleep(0.5)
            self.init_mode[2] = True
            self.ShootBarrel = Barrel()

        state, param = self.ShootBarrel.run(coordinate)
        if param is not None:
            if state == 0:
                self.send('spds', param)
            elif state == 1:
                self.send('spst', param)
        elif state == 2:
            self.send('spst', 0)
            self.send('shoot', 55)
            sleep(0.4)
            self.send('shoot', 27)
            sleep(0.7)
            self.send('shoot', 10)
            sleep(0.4)
            return True
        return False

if __name__ == '__main__':
    cmd_q = CustomQueue(20)
    img_q = Queue()
    mde_q = Queue()

    condition_lock = Lock()
    exit_condition = Condition(condition_lock)
    exit_event = Event()

    debug = True
    while not (len(glob('/dev/video2'))>0 and len(glob('/dev/ttyUSB[0-9]*'))>0):
        print('try again')
        sleep(1)

    print('connected!')
    image_thread = ImageProcessingThread(ImgProc(img_q, mde_q, debug), exit_condition)

    threads = [CommandThread(cmd_q, exit_event), image_thread]
    for thread in threads:
        thread.start()
    sleep(1.7)
    main_control = mainControl(cmd_q, img_q, mde_q)
    main_control.run()

    exit_event.set()
    with exit_condition:
        exit_condition.notify_all()

    for thread in threads:
        thread.join()
