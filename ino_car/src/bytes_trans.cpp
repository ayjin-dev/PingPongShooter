#include <Arduino.h>
#include "bytes_trans.h"

void wait_for_bytes(int num_bytes, unsigned long timeout)
{
    unsigned long startTime = millis();
    while ((Serial.available() < num_bytes) && (millis() - startTime < timeout)) {}
}

void read_signed_bytes(int8_t* buffer, size_t n)
{
    size_t i = 0;
    int c;
    while (i < n)
    {
        c = Serial.read();
        if (c < 0) break;
        *buffer++ = (int8_t) c; // buffer[i] = (int8_t)c;
        i++;
    }
}

int16_t read_i16()
{
    int8_t buffer[2];
    wait_for_bytes(2, 100); // Wait for 1 byte with a timeout of 100 ms
    read_signed_bytes(buffer, 2);
    return (((int16_t) buffer[0]) & 0xff) | (((int16_t) buffer[1]) << 8 & 0xff00);
}

int8_t read_i8()
{
    wait_for_bytes(1, 100);
    return (int8_t) Serial.read();
}

int32_t read_i32()
{
  int8_t buffer[4];
        wait_for_bytes(4, 200); // Wait for 4 bytes with a timeout of 200 ms
        read_signed_bytes(buffer, 4);
  return (((int32_t) buffer[0]) & 0xff) | (((int32_t) buffer[1]) << 8 & 0xff00) | (((int32_t) buffer[2]) << 16 & 0xff0000) | (((int32_t) buffer[3]) << 24 & 0xff000000);
}

Order read_order()
{
    return (Order) Serial.read();
}

void write_order(enum Order myOrder)
{
    uint8_t* Order = (uint8_t*) &myOrder;
    Serial.write(Order, sizeof(uint8_t));
}

void write_i8(int8_t num)
{
  Serial.write(num);
}

void write_i16(int16_t num)
{
    int8_t buffer[2] = {(int8_t) (num & 0xff), (int8_t) (num >> 8)};
    Serial.write((uint8_t*)&buffer, 2*sizeof(int8_t));
}
