from struct import pack, unpack
from glob import glob
from serial import Serial
from enum import Enum
from queue import Queue

class Order(Enum):
    CAN_INIT = 0
    RESET = 1
    MOTOR_1 = 2
    MOTOR_2 = 3
    MOTOR_BROADCAST = 4
    ARM = 5
    CAM = 6
    CLIP  = 7
    CAM_SWITCH = 8
    SHOOT = 9
    MOTORS = 10

def write_order(f, order):
    write_i8(f, order.value)

def write_i8(f, value):
    if -128 <= value <= 127:
        f.write(pack('<b', value))
    else:
        print("Value error:{}".format(value))

def write_i16(f, value):
    f.write(pack('<h', value))

def write_i32(f, value):
    f.write(pack('<l', value))


def open_serial_port():
    ports = glob('/dev/ttyUSB[0-9]*')
    if len(ports) < 1:
        print('ports not found!')
        return None
    return Serial(port=ports[0], baudrate=115200)


def read_order(f):
    return Order(read_i8(f))

def read_i8(f):
    return unpack('<b', bytearray(f.read(1)))[0]

def read_i16(f):
    return unpack('<h', bytearray(f.read(2)))[0]

class CustomQueue(Queue):
    def clear(self):
        with self.mutex:
            unfinished = self.unfinished_tasks - len(self.queue)
            if unfinished <= 0:
                if unfinished < 0:
                    raise ValueError('task_done() called too many times')
                self.all_tasks_done.notify_all()
            self.unfinished_tasks = unfinished
            self.queue.clear()
            self.not_full.notify_all()
