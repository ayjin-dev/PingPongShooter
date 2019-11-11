from numpy import array,argmin,around,cross,size,where,clip,abs,prod,sum,searchsorted
from numpy.linalg import norm
from time import time

from img_proc.img_proc import SCALE_CLIPPER, SCALE_READY_CLIP,SCALE_BOTTOM_CENTER

SEARCHING = 10

class PickBall:
    def __init__(self):
        self.__err_rng = array([[5, 15, 30,45,70,100], [5, 15, 30,45,70,100]])
        self.__Kp = array([[1.7, 1.2, 1, 0.9, 0.7,0.6, 0.5], [1.5, 1.3, 1, 0.8, 0.9, 0.7, 0.6]])
        self.__last_spd = array([0.,0.])
        self.__reset_pid()
        self.__expect = array(SCALE_READY_CLIP)
        self.states = {'found': False, 'ready': False}
        self.err_tolerance = array([2,2])

    def run(self, ball, balls):
        if ball is not None:
            if not self.states['found']:
                self.__reset_pid()
                self.states['found'] = True

            targ_p = (ball[:2] + (ball[2]//2, 1))
            err = self.__expect - targ_p
            if prod(abs(err) - self.err_tolerance < 0) or sum(prod(abs(self.__expect - array([(b[:2] + (b[2]//2, 1)) for b in balls[-3:]])) - self.err_tolerance < 0)) > 0:
                # size(where((self.clipper[0] - targ_p)>0)[0]) == 0 and targ_p[0] < self.clipper[1][0]
                if not self.states['ready']:
                    self.err_tolerance = array([15,10])
                    self.__expect = array(SCALE_CLIPPER)
                    self.states = {'found': False, 'ready': True}
                    return 2, None
                return 1,None


            self.update_Kp(err)
            err_d = (err - self.__last_err)/(time() - self.__last_time)
            self.__err_sum += err
            self.__last_err,self.__last_time = err, time()
            spd = sum((err, self.__err_sum, err_d) * self.__K, axis=0)
            spd = (sum(spd), spd[1]) if spd[0]>0 else (spd[1], spd[1] - spd[0])
        else:
            self.states['found'] = False
            spd = SEARCHING * array((-1,1) if self.__last_err[0]>0 else (1,-1))

        spd = around(spd).astype(int)
        should_run = sum(self.__last_spd - spd) == 0
        self.__last_spd = spd
        return 0, spd.tolist() if should_run else None

    def __reset_pid(self):
        # Kp, Ki, Kd
        self.__K  = array([[0.0, 0.0], [.00, .00], [.02, .02]])
        self.__last_time = time()
        # x,y
        self.__last_err = array([0.,0.])
        self.__err_sum = array([0.,0.])

    def update_Kp(self, err):
        err = abs(err)
        self.__K[0] = array([self.__Kp[0][searchsorted(self.__err_rng[0], err[0])], self.__Kp[1][searchsorted(self.__err_rng[1], err[1])]])

class GreenZone:
    def __init__(self):
        self.__last_spd = array([0.,0.])
        self.__reset_pid()
        self.__expect = array(SCALE_BOTTOM_CENTER)
        self.states = {'found': False, 'front': False, 'aimed': False}
        self.err_tolerance = array([10,30])

    def run(self, green):
        if green is not None:
            if not self.states['found']:
                self.green_aim = GreenAim()
                self.states['found'],self.states['aimed'] = True, False

            if not self.states['aimed']:
                aim_state = self.green_aim.run(green)
                if aim_state[0] == 0:
                    return 2, aim_state[1]
                elif aim_state[0] == 1:
                    self.__reset_pid()
                    self.states['aimed'] = True
                    return 0, None
            else:
                targ_p = (green[:2] + (green[2]//2, 0)) if self.states['front'] else (green[:2] + green[-2:]//(2,1))
                err = self.__expect - targ_p
                if prod(abs(err) - self.err_tolerance < 0):
                    if self.states['front']:
                        return 1, None
                    self.states['front'] = True
                    self.__reset_pid(.7)
                    return 2, 0

                err_d = (err - self.__last_err)/(time() - self.__last_time)
                self.__last_err,self.__last_time = err, time()
                self.__err_sum += err
                spd = sum((err, self.__err_sum, err_d) * self.__K, axis=0)
                spd = (sum(spd), spd[1]) if spd[0]>0 else (spd[1], spd[1] - spd[0])
        else:
            self.states['aimed'], self.states['found'] = False, False
            spd = SEARCHING * array((-1,1) if self.__last_err[0]>0 else (1,-1))

        spd = around(spd).astype(int)
        should_run = sum(self.__last_spd - spd) == 0
        self.__last_spd = spd
        return 0, spd.tolist() if should_run else None

    def __reset_pid(self, scale_pid=1.0):
        # Kp, Ki, Kd
        self.__K  = array([[1.1*scale_pid, 1.3*scale_pid], [.001, .001], [.03, .03]])
        self.__last_time = time()
        # x,y
        self.__last_err = array([0.,0.])
        self.__err_sum = array([0.,0.])

class GreenAim:
    def __init__(self):
        self.__K  = array([.75, 0, 0.001])
        self.__last_spd,self.__last_err,self.__err_sum = 0,0,0
        self.__found,self.__last_time = False, time()
        self.__expect = SCALE_BOTTOM_CENTER[0]

    def run(self, coordinate):
        if not self.__found:
            self.__last_err,self.__err_sum = 0,0
            self.__last_time = time()
            self.__found = True

        err = self.__expect - sum(coordinate[[0,2]] // (1,2))
        if abs(err) < 5:
            return 1,None
        err_d = (err - self.__last_err)/(time() - self.__last_time)
        self.__err_sum += err
        self.__last_err,self.__last_time = err, time()
        spd = around(clip(sum((err, self.__err_sum, err_d) * self.__K), -100, 100)).astype(int)
        should_run = self.__last_spd != spd
        self.__last_spd = spd
        return 0, int(spd) if should_run else None
