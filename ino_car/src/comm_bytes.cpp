#include <SPI.h>
#include <Servo.h>

#include "mcp_can.h"
#include "bytes_trans.h"
#include "comm_bytes.h"

#define WAIT_RESET_TIME 150

#define SPI_CS_PIN 10
MCP_CAN CAN (SPI_CS_PIN);

Servo arm_joint;
Servo cam_pan;
Servo clip;
Servo shoot;

#define CAM_PAN_PIN A0
#define ARM_JOINT_PIN A3
#define CLIP_PIN A4
#define SHOOT_PIN A5

int16_t t_pwm = 5000;
bool cam_switch = true;

unsigned char reset_comm[8] = {0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};
unsigned char spd_mode[8] = {0x03,0x55,0x55,0x55,0x55,0x55,0x55,0x55};


bool can_init() {
    int count = 50;
    arm_joint.attach(ARM_JOINT_PIN);
    cam_pan.attach(CAM_PAN_PIN);
    clip.attach(CLIP_PIN);
    shoot.attach(SHOOT_PIN);

    do {
        if(CAN_OK == CAN.begin(CAN_1000KBPS))
        {
            return true;
            break;
        }
        else
        {
            delay(100);
        }
    } while(count--);
    return false;
}

unsigned char id_modifier(unsigned char motor_id, unsigned char mode_id) {
    return motor_id * 0x10 + mode_id;
}

void run_motors_reset(unsigned char motor_id) {
    motor_id = id_modifier(motor_id, 0x0);
    CAN.sendMsgBuf(motor_id,0, 8, reset_comm);
}

void run_mode(unsigned char motor_id) {
    motor_id = id_modifier(motor_id, 0x1);
    CAN.sendMsgBuf(motor_id, 0, 8, spd_mode);
}

void reset_all(unsigned char motor_id) {
    run_motors_reset(motor_id);
    delay(WAIT_RESET_TIME);
    run_mode(motor_id);
    delay(WAIT_RESET_TIME);
}

/* NOTE
    t_pwm: 0~+5000,
    t_vel: 0~+32757(int16_t, rpm),
    t_pos: -2147483648~+2147483647(int32_t, qc)
*/
void move(unsigned char motor_id) {
    unsigned char input_value[8] = {};
    unsigned char mode = 0x4;

    int16_t t_vel = read_i16();

    input_value[0] = (unsigned char)((t_pwm>>8)&0xff);
    input_value[1] = (unsigned char)((t_pwm)&0xff);
    input_value[2] = (unsigned char)((t_vel>>8)&0xff);
    input_value[3] = (unsigned char)(t_vel&0xff);
    input_value[4] = 0x55;
    input_value[5] = 0x55;
    input_value[6] = 0x55;
    input_value[7] = 0x55;

    motor_id = id_modifier(motor_id, mode);
    CAN.sendMsgBuf(motor_id, 0, 8, input_value);
}

void twoMove() {
    unsigned char input_value[8] = {};
    unsigned char mode = 0x4;

    int16_t t_vel_1 = read_i16();
    int16_t t_vel_2 = read_i16();

    input_value[0] = (unsigned char)((t_pwm>>8)&0xff);
    input_value[1] = (unsigned char)((t_pwm)&0xff);
    input_value[2] = (unsigned char)((t_vel_1>>8)&0xff);
    input_value[3] = (unsigned char)(t_vel_1&0xff);
    input_value[4] = 0x55;
    input_value[5] = 0x55;
    input_value[6] = 0x55;
    input_value[7] = 0x55;

    unsigned char motor_id = id_modifier(MOTOR_1_ID, mode);
    CAN.sendMsgBuf(motor_id, 0, 8, input_value);

    delay(10);

    input_value[2] = (unsigned char)((t_vel_2>>8)&0xff);
    input_value[3] = (unsigned char)(t_vel_2&0xff);

    motor_id = id_modifier(MOTOR_2_ID, mode);
    CAN.sendMsgBuf(motor_id, 0, 8, input_value);
}

void set_arm_pos() {
    int16_t pos = read_i16();
    arm_joint.write(pos);
}

void set_cam_pan_pos() {
    int16_t pos = read_i16();
    cam_pan.write(pos);
}

void set_clip_pos() {
    int16_t pos = read_i16();
    clip.write(pos);
}

void set_shoot_pos() {
    int16_t pos = read_i16();
    shoot.write(pos);
}

void get_msg_from_bytes() {
    if(Serial.available() > 0) {
        Order order_received = read_order();
        switch(order_received) {
            case CAN_INIT:
            {
                can_init();
                // if(ok) {
                //     write_order(CAN_INIT);
                // }
                break;
            }
            case RESET:
            {
                reset_all(MOTOR_1_ID);
                reset_all(MOTOR_2_ID);

                // write_order(RESET);
                break;
            }
            case MOTOR_1:
            {
                move(MOTOR_1_ID);
                break;
            }
            case MOTOR_2:
            {
                move(MOTOR_2_ID);
                break;
            }
            case MOTOR_BROADCAST:
            {
                move(MOTOR_BROADCAST_ID);
                break;
            }
            case ARM:
            {
                set_arm_pos();
                break;
            }
            case CAM:
            {
                if(cam_switch) {
                    set_cam_pan_pos();
                }
                break;
            }
            case CLIP:
            {
                set_clip_pos();
                break;
            }
            case CAM_SWITCH:
            {
                int8_t mode = read_i8();
                if(mode == 88) {
                    cam_pan.attach(CAM_PAN_PIN);
                    cam_switch = true;
                }
                else if(mode == 66) {
                    cam_pan.detach();
                    cam_switch = false;
                }
                break;
            }
            case MOTORS:
            {
                twoMove();
                break;
            }
            case SHOOT:
            {
                set_shoot_pos();
                break;
            }
        }
    }
}
