#include <Servo.h>

#define MOTOR_1_ID 0x01
#define MOTOR_2_ID 0x04
#define MOTOR_BROADCAST_ID 0x00

#define WAIT_RESET_TIME 150

#define CAM_PAN_PIN 3
#define ARM_JOINT_PIN 6
#define CLIP_PIN 9
#define SHOOT_PIN 10

#define CLIP_RESET_POS 25
#define SHOT_RESET_POS 30
#define CAM_RESET_POS  150
#define ARM_RESET_POS  178

#define BAUDRATE 115200

bool can_init();
void reset_all(unsigned char motor_id);
void move(unsigned char motor_id);
void get_msg_from_bytes();
void setup();
