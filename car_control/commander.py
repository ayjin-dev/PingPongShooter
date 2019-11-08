from .rst_serial import write_order,write_i16,write_i32,write_i8
from .rst_serial import Order, open_serial_port

from threading import Thread
from queue import Empty

from time import sleep

class RoboCar():
    def __init__(self):
        self._serial_file = open_serial_port()
        sleep(1)
        write_order(self._serial_file, Order.CAN_INIT)
        # sleep(0.1)
        print("can init!")

    def __set_motors_spd(self,m_id,vel):
        write_order(self._serial_file, m_id)
        write_i16(self._serial_file, vel)
        sleep(0.01)

    def __set_servos_pos(self, s_id, pos):
        write_order(self._serial_file, s_id)
        write_i16(self._serial_file, pos)

    def __cam_switch(self, should_on=False):
        write_order(self._serial_file, Order.CAM_SWITCH)
        if should_on:
            write_i8(self._serial_file, 88)
            print('cam switch on!')
        else:
            write_i8(self._serial_file, 66)
            print('cam switch off!')

    def reset(self,t_vel=20):
        write_order(self._serial_file, Order.RESET)
        sleep(0.31)
        print("reset done!")

    def set_motor_1_spd(self, vel):
        self.__set_motors_spd(Order.MOTOR_1, vel)

    def set_motor_2_spd(self, vel):
        self.__set_motors_spd(Order.MOTOR_2, vel)

    def two_motors_spd(self, lvel, rvel):
        write_order(self._serial_file, Order.MOTORS)
        write_i16(self._serial_file, lvel)
        write_i16(self._serial_file, rvel)

    def speed_broadcast(self, vel=40):
        write_order(self._serial_file, Order.MOTOR_BROADCAST)
        write_i16(self._serial_file, vel)

    def set_arm(self, pos):
        self.__set_servos_pos(Order.ARM, pos)

    def set_cam(self, pos):
        self.__cam_switch(True)
        sleep(0.05)
        self.__set_servos_pos(Order.CAM, pos)
        sleep(0.05)
        self.__cam_switch(False)

    def set_clip(self, pos):
        self.__set_servos_pos(Order.CLIP, pos)

    def set_shoot(self, pos):
        self.__set_servos_pos(Order.SHOOT, pos)

    def rotate(self, vel=40, wait_time=2):
        self.speed_broadcast(vel)
        sleep(wait_time)
        self.speed_broadcast(0)

    def forward(self, vel=40, wait_time=2):
        # self.set_motor_1_spd(vel)
        # self.set_motor_2_spd(-vel)
        self.two_motors_spd(vel, -vel)
        sleep(wait_time)
        self.two_motors_spd(0,0)

class CommandThread(Thread):
    def __init__(self, cmd_q, exit_event):
        Thread.__init__(self)
        self.deamon = True
        self.cmd_q = cmd_q
        self.exit_event = exit_event
        self.car = RoboCar()
        # sleep(.001)
        self.car.reset()

    def run(self):
        car = self.car

        while not self.exit_event.is_set():
            if self.exit_event.is_set():
                break
            try:
                order, param = self.cmd_q.get_nowait()
            except Empty:
                sleep(0.01)
                continue

            if order == 'clip':
                car.set_clip(param)
            elif order == 'arm':
                car.set_arm(param)
            elif order == 'cam':
                car.set_cam(param)
            elif order == 'shoot':
                pass
            elif order == 'spst':
                car.speed_broadcast(param)
            elif order == 'spds':
                # now, positive for forward
                car.two_motors_spd(-param[0], param[1])
