#define MOTOR_1_ID 0x01
#define MOTOR_2_ID 0x04
#define MOTOR_BROADCAST_ID 0x00

#define BAUDRATE 115200

bool can_init();
void reset_all(unsigned char motor_id);
void move(unsigned char motor_id);
void get_msg_from_bytes();
