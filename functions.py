from numpy import array,argmin,around,cross,size,where
from numpy.linalg import norm
from time import time

from img_proc.img_proc import SCALE_CLIPPER, SCALE_READY_CLIP

class PickBall:
    def __init__(self):
        self.__searching = 60
        self.__Kp = [.6, .6]
        self.__Ki = array([.001, .001])
        self.__Kd = array([.01, .01])

        self.__last_time = 0
        self.__last_err = array([0.,0.])
        self.__err_sum = array([0.,0.])

        self.__found = False
        self.__should_run = False
        self.__last_spd = array([0.,0.])

        self.clipper = array(SCALE_READY_CLIP)
        self.__expect = sum(self.clipper)//2
        self.__expect[1] += 5
        self.__ready_for_clip = False

    def run(self, ball):
        if ball is not None:
            if not self.__found:
                self.__last_err = array([0,0])
                self.__last_time = time()
                self.__err_sum = array([0.,0.])
                self.__found = True

            targ_p = ball[:2] + ball[-2:]//2
            if size(where((self.clipper[0] - targ_p)>0)[0]) == 0 and targ_p[0] < self.clipper[1][0]:
                if not self.__ready_for_clip:
                    self.clipper = array(SCALE_CLIPPER)
                    self.__expect = sum(self.clipper)//2
                    self.__expect[1] += 20
                    self.__ready_for_clip = True
                    self.__found = False
                    return 2, None
                else:
                    return 1,None

            # xerr, yerr
            err = self.__expect - targ_p
            err_d = err - self.__last_err
            time_d = time() - self.__last_time
            self.__err_sum += err
            self.__last_err = err
            self.__last_time = time()

            spd = self.__Kp*err + self.__Ki*self.__err_sum + self.__Kd*(err_d / time_d)

            # lwspd, rwspd
            # target is in the left screen
            if spd[0] > 0:
                spd = spd[1]+spd[0], spd[1]
            else:
                if spd[1] > 0:
                    spd = spd[1], spd[1] - spd[0]
                else:
                    spd = spd[1], spd[1]+spd[0]
            # lwspd = rwspd = spd[1]
            # if spd[0] > 0:
            #     lwspd += spd[0]
            # else:
            #     rwspd -= spd[0]
            # spd = lwspd, rwspd

        else:
            self.__found = False
            if self.__last_err[0] > 0:
                spd = (-self.__searching,self.__searching)
            else:
                spd = (self.__searching,-self.__searching)

        spd = around(spd).astype(int)
        # print('spd:', spd)
        __should_run = sum(self.__last_spd - spd) == 0
        self.__last_spd = spd
        if __should_run:
            return 0,spd
        else:
            return 0,None

class GreenZone:
    def __init__(self):
        self.__searching = 130

        self.__Kp = [.3, .3]
        self.__Ki = array([0, 0])
        self.__Kd = array([0, 0])

        self.__last_time = 0
        self.__last_err = array([0.,0.])
        self.__err_sum = array([0.,0.])

        self.__found = False
        self.__aimed = False
        self.__should_run = False
        self.__last_spd = array([0.,0.])

        self.__expect = sum(array(SCALE_CLIPPER))//2

    def run(self, green):
        if green is not None:
            if not self.__found:
                self.__found = True
                self.__aimed = False
                self.green_aim = GreenAim()

            if not self.__aimed:
                state = self.green_aim.run(green)
                if state[0] == 0:
                    return 2, state[1]
                elif state[0] == 1:
                    # NOTE end of aiming session
                    self.__last_err = array([0,0])
                    self.__last_time = time()
                    self.__err_sum = array([0.,0.])
                    self.__aimed = True

            else:
                targ_p = green[:2] + (green[2]//2, 0)
                err = self.__expect - targ_p
                if size(where((self.clipper[0] - targ_p)>0)[0]) == 0 and targ_p[0] < self.clipper[1][0]:
                    return 1, None

                err_d = err - self.__last_err
                time_d = time() - self.__last_time
                self.__err_sum += err
                self.__last_err = err
                self.__last_time = time()
                spd = self.__Kp*err + self.__Ki*self.__err_sum + self.__Kd*(err_d/time_d)
                spd = (spd[1], sum(spd * (-1,1))) if spd[0] > 0 else (sum(spd), spd[1])
        else:
            self.__aimed = False
            self.__found = False
            if self.__last_err[0] > 0:
                spd = (-self.__searching,self.__searching)
            else:
                spd = (self.__searching,-self.__searching)

        spd = around(spd).astype(int)
        __should_run = sum(self.__last_spd - spd) == 0
        self.__last_spd = spd
        if __should_run:
            return 0,spd
        else:
            return 0,None


class GreenAim:
    def __init__(self):
        self.__Kp = .3
        self.__Ki = 0
        self.__Kd = 0

        self.__last_time = 0
        self.__last_err = 0
        self.__err_sum = 0

        self.__found = False
        self.__should_run = False
        self.__last_spd = 0
        self.__expect = (sum(array(SCALE_CLIPPER))//2)[0]

    def run(self, coordinate):
        if coordinate is not True:
            if not self.__found:
                self.__last_err = 0
                self.__last_time = time()
                self.__err_sum = 0
                self.__found = True

            x,_,w,_ = coordinate
            targ_p = x + w//2
            err = self.__expect - targ_p
            if abs(err) < 5:
                return 1,None

            err_d = err - self.__last_err
            time_d = time() - self.__last_time
            self.__err_sum += err
            self.__last_err = err
            self.__last_time = time()
            spd = self.__Kp*err + self.__Ki*self.__err_sum + self.__Kd*(err_d/time_d)

        spd = around(spd).astype(int)
        self.__should_run = self.__last_spd != spd
        self.__last_spd = spd

        if __should_run:
            return 0,spd
        else:
            return 0,None
