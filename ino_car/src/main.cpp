#include <Arduino.h>
// #include "bytes_trans.h"

#include "comm_bytes.h"

void setup()
{
    Serial.begin(BAUDRATE);
    while (!Serial) {;}
}

void loop()
{
    get_msg_from_bytes();
}
