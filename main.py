from queue import Queue, Empty, Full
from threading import Lock, Condition, Event
from time import sleep
from numpy import array,around

from car_control.commander import CommandThread
from img_proc.img_proc import ImageProcessingThread, ImgProc
from car_control.rst_serial import CustomQueue
from functions import PickBall

class mainControl:
    def __init__(self, cmd_q, img_q, mde_q,scale):
        self.cmd_clear = cmd_q.clear
        self.cmd_put = cmd_q.put_nowait
        self.img_get = img_q.get
        # self.img_clear = img_q.clear
        self.mde_q = mde_q
        self.exit = False
        self.win_center = around(scale*array([1920,1080])*0.5).astype(int)

        self.n_full, self.n_total = 0,0

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
        print('stop!')
        print('full:{}, total:{}'.format(self.n_full, self.n_total))

    def run(self):
        self.send('cam', 150)
        self.send('clip', 27)
        self.send('arm', 9)
        # self.send('cam_swtch', False)
        self.mde_q.put('ball')
        self.PickBall = PickBall()

        while not self.exit:
            try:
                coordinate = self.img_get()

                state, param = self.PickBall.run(coordinate)

                if state == 0 and param is not None:
                    # self.send('spds', param)
                    pass
                elif state == 1:
                    self.send('clip', 5)
                    sleep(0.2)
                    self.send('arm', 180)
                    sleep(0.3)
                    self.send('clip', 30)
                    break

            except KeyboardInterrupt:
                break
        self.stop()

if __name__ == '__main__':
    cmd_q = CustomQueue(20)
    img_q = Queue()
    mde_q = Queue()

    condition_lock = Lock()
    exit_condition = Condition(condition_lock)
    exit_event = Event()

    scale = 0.20
    debug = True# True

    threads = [CommandThread(cmd_q, exit_event), ImageProcessingThread(ImgProc(scale, img_q, mde_q, debug), exit_condition)]
    for thread in threads:
        thread.start()

    main_control = mainControl(cmd_q, img_q, mde_q, scale)
    main_control.run()

    exit_event.set()
    with exit_condition:
        exit_condition.notify_all()

    for thread in threads:
        thread.join()
