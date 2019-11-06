from numpy import array,argmin,around,sum,cross
from numpy.linalg import norm
from time import time

from img_proc.img_proc import CLIPPER

class PickBall:
    def __init__(self):
        self.searching = 130

        self.Kp = [.3, .3]
        self.Ki = array([0, 0])
        self.Kd = array([0, 0])

        self.last_time = 0
        self.last_err = array([0.,0.])
        self.err_sum = array([0.,0.])

        self.__found = False
        self.should_run = False
        self.last_spd = array([0.,0.])
        self.clipper = array(CLIPPER)
        self.expect = sum(self.clipper)//2
        self.clipper[1] -= self.clipper[0]

    def run(self, ball):
        if ball is not None:
            if not self.__found:
                self.last_err = array([0,0])
                self.last_time = time()
                self.err_sum = array([0.,0.])
                self.__found = True

            targ_p = ball[:2] + ball[-2:]//2
            if cross(targ_p - self.clipper[0], self.clipper[1]) <= 0:
                print('targ_p:', targ_p)
                return 1, None
            # xerr, yerr
            err = self.expect - targ_p
            err_d = err - self.last_err
            time_d = time() - self.last_time
            self.err_sum += err
            self.last_err = err
            self.last_time = time()
            # lwspd, rwspd
            spd = self.Kp*err + self.Ki*self.err_sum + self.Kd*(err_d/time_d)
            spd = (spd[1], sum(spd * (-1,1))) if spd[0] > 0 else (sum(spd), spd[1])
        else:
            if self.last_err[0] > 0:
                spd = (-self.searching,self.searching)
            else:
                spd = (self.searching,-self.searching)

        spd = around(spd).astype(int)
        should_run = sum(self.last_spd - spd) == 0
        self.last_spd = spd

        if should_run == 0:
            return 0,None
        else:
            return 0,spd
