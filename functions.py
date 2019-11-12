from numpy import array,argmin,around,cross,size,where,clip,abs,prod,sum,searchsorted
from numpy.linalg import norm
from time import time, sleep

from img_proc.img_proc import SCALE_CLIPPER, SCALE_READY_CLIP,SCALE_BOTTOM_CENTER

SEARCHING = 150

class PickBall:
    def __reset_pid(self):
        self.__K  = array([[0.0, 0.0], [.2, .2]])
        self.__last_err = array([0.,0.])

    def clipper_mode(self, is_ready=False):
        if is_ready:
            self.__expect = array(SCALE_CLIPPER)
            self.err_tolerance = array([1,1])
            self.states = {'found': False, 'ready': True}
        else:
            self.__expect = array(SCALE_READY_CLIP)
            self.err_tolerance = array([1,1])
            self.states['ready'] = False

    def __init__(self):
        self.Kp_step = array([1, 1]) * .05
        self.Kp_base = array([1, 1]) * 2.0
        self.Kp_min = array([.81, .8])

        self.err_difference = 5
        self.escape = array([20,50])
        self.__last_spd = array([0.,0.])
        self.__reset_pid()
        self.states = {'found': False, 'ready': False}
        self.clipper_mode(is_ready=False)

    def run(self, ball):
        if ball is not None:
            if not self.states['found']:
                self.__reset_pid()
                self.states['found'] = True

            targ_p = ball[:2] + (ball[2]//2, ball[3])
            err = self.__expect - targ_p
            if prod(abs(err) - self.err_tolerance < 0) and norm(self.__last_err - err) < self.err_difference:
                if not self.states['ready']:
                    self.clipper_mode(is_ready=True)
                    return 2, None
                return 1,None
            elif self.states['ready'] and sum(abs(err) - self.escape > 0):
                self.clipper_mode(is_ready=False)
                return 3,None
            self.update_Kp(err)
            err_d = err - self.__last_err
            self.__last_err = err
            spd = sum((err, err_d) * self.__K, axis=0)
            spd = (sum(spd), spd[1]) if spd[0]>0 else (spd[1], spd[1] - spd[0])
        else:
            self.states['found'] = False
            spd = SEARCHING * array((-1,1) if self.__last_err[0]>0 else (1,-1))

        spd = around(spd).astype(int)
        should_run = sum(self.__last_spd - spd) != 0
        self.__last_spd = spd
        return 0, spd.tolist() if should_run else None

    def update_Kp(self, err):
        self.__K[0] = clip(self.Kp_base + self.Kp_step*((0, 1)- abs(err)), self.Kp_min, self.Kp_base)

class GreenZone:
    def __reset_pid(self,):
        self.__last_err = array([0.,0.])
        self.__last_time = time()
        self.__last_spd = array([0.1,0])
        self.__last_targ = array([0.,0.])

        self.__K  = array([[0.,0.], self.Kd])

    def __init__(self):
        xerr_rng = [2, 5, 10, 25, 50, 95, 120]
        yerr_rng = [2, 5, 10, 25, 50, 95, 120]
        Kp_x = [1, 1.5, 1.7, 1.5, 2.7, 4.1, 2.7, 2.3]
        Kp_y = [1, 1.2, 1.5, 1.3, 2.5, 3.9, 2.6, 2.0]

        self.__err_rng = array([xerr_rng, yerr_rng])
        self.Kp = array([Kp_x, Kp_y])
        self.Kd = [0., 0.]

        self.err_tolerance = array([2,2])

        self.__expect = array(SCALE_BOTTOM_CENTER)

        self.__reset_pid()
        self.states = {'found': False, 'aimed': False}

    def run(self, green):
        if green is not None:
            if not self.states['found']:
                self.green_aim = GreenAim()
                self.states['found'],self.states['aimed'] = True, False

            if not self.states['aimed']:
                aim_state = self.green_aim.run(green)
                if not aim_state[0]:
                    return 2, aim_state[1]
                self.__reset_pid()
                self.states['aimed'] = True
                return 2, 0
            else:
                targ_p = (green[:2] + (green[2]//2, 0))
                err = self.__expect - targ_p
                if prod(abs(err) - self.err_tolerance < 0):
                    return 1, None

                self.update_K(err)
                err_d =  (self.__last_targ - targ_p)/(self.__last_time - time())
                self.__last_err = err
                self.__last_time = time()
                spd = sum((err, err_d) * self.__K, axis=0)
                spd = (sum(spd), spd[1]) if spd[0]>0 else (spd[1], spd[1] - spd[0])
                # DEBUG
                # spd = around(spd).astype(int)
                # print('xerr:{}, yerr:{}, lspd:{}, rspd:{}'.format(err[0],err[1], spd[0], spd[1]))
        else:
            self.states['aimed'], self.states['found'] = False, False
            spd = SEARCHING * array((-1,1) if self.__last_err[0]>0 else (1,-1))

        spd = around(spd).astype(int)

        should_run = sum(self.__last_spd - spd) != 0
        self.__last_spd = spd
        return 0, spd.tolist() if should_run else None

    def update_K(self, err):
        err = abs(err)
        self.__K[0] = array([self.Kp[0][searchsorted(self.__err_rng[0], err[0])], self.Kp[1][searchsorted(self.__err_rng[1], err[1])]])

class GreenAim:
    def update_Kp(self, err):
        self.__K[0] = clip(self.Kp_base - self.Kp_step*abs(err), self.Kp_min, self.Kp_base)

    def __init__(self):
        self.Kp_base = 1.0
        self.Kp_step = .01
        self.Kp_min = .8

        self.__K  = array([.75, 0.001])
        self.__last_spd,self.__last_err = 0,0
        self.__found = False
        self.__expect = SCALE_BOTTOM_CENTER[0]

    def run(self, coordinate):
        if not self.__found:
            self.__last_err = 0
            self.__found = True

        err = self.__expect - sum(coordinate[[0,2]] // (1,2))
        if abs(err) < 5:
            return 1,None

        self.update_Kp(err)
        err_d = err - self.__last_err
        self.__last_err = err
        spd = around(sum((err, err_d) * self.__K)).astype(int)
        should_run = self.__last_spd != spd
        self.__last_spd = spd
        return 0, int(spd) if should_run else None
