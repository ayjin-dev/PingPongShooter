#include <Arduino.h>
#ifndef ORDER_H
#define ORDER_H
enum Order {
    CAN_INIT = 10,
    MOTORS = 11,
    RESET = 1,
    MOTOR_1 = 2,
    MOTOR_2 = 3,
    MOTOR_BROADCAST = 4,
    ARM = 5,
    CAM = 6,
    CLIP  = 7,
    CAM_SWITCH = 8,
    SHOOT = 9,
};
typedef enum Order Order;
#endif


int16_t read_i16();
int8_t read_i8();
int32_t read_i32();
Order read_order();
void write_order(enum Order myOrder);
void write_i8(int8_t num);
void write_i16(int16_t num);
