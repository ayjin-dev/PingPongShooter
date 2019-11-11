from .rst_serial import write_order,write_i16,write_i32,write_i8
from .rst_serial import Order, open_serial_port

from threading import Thread
from queue import Empty

from time import sleep

ARM_DOWN = 30
ARM_UP = 178
CLIPPER_CLOSE = 0
CLIPPER_CLIP = 2
CLIPPER_OPEN = 30
CAM_VIEW = 145
CAM_FULL_VIEW = 149

class RoboCar():
    def __init__(self):
        self._serial_file = open_serial_port()
        sleep(1)
        if self.__send_comm(Order.CAN_INIT):
            print("can init ok!")
        else:
            print("can init fail!")
            exit()

    def __send_comm(self, order, wait_time=0):
        write_order(self._serial_file, order)
        sleep(wait_time)
        is_done = False
        while not is_done:
            print("Waiting for reply...")
            bytes_array = bytearray(self._serial_file.read(1))
            if not bytes_array:
                time.sleep(0.2)
                print("byte not found!")
                continue
            byte = bytes_array[0]
            if byte == order.value:
                is_done = True
        if(is_done):
            return True
        else:
            return False

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

    def reset(self):
        if self.__send_comm(Order.RESET, wait_time=1.5):
            print("reset ok!")
        else:
            print("reset fail!")
        sleep(0.31)

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
        self.car.set_cam(CAM_VIEW)

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
                car.two_motors_spd(param[0], -param[1])
