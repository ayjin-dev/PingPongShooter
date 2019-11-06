from numpy import array,around,sum,argmin,cross
from numpy.linalg import norm
from time import time

class PickBall:
    def __init__(self, expect):
        self.searching = 130

        self.Kp = [.3, .3]
        self.Ki = array([0, 0])
        self.Kd = array([0, 0])

        self.last_time = 0
        self.last_err = array([0.,0.])
        self.err_sum = array([0.,0.])

        self.found = False
        self.expect = expect
        self.ball = None
        self.should_run = False
        self.last_spd = array([0.,0.])
        # left clipper, right clipper
        self.clipper = array([[198, 193],[218, 197]])
        self.clipper[1] -= self.clipper[0]

    def run(self, coordinates):
        if coordinates is not None:
            if self.found:
                self.ball = coordinates[argmin([norm(self.ball[:2] - c[:2]) for c in coordinates])]
            # first found
            else:
                self.ball = coordinates[-1]
                self.last_err = array([0,0])
                self.last_time = time()
                self.err_sum = array([0.,0.])
                self.found = True

            targ_p = self.ball[:2] + self.ball[-2:]//2
            if cross(targ_p-self.clipper[0], self.clipper[1]) <= 0:
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
