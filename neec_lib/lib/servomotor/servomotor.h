#pragma once

#if defined(ARDUINO_ARCH_ESP32)
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif

class ServoMotor {
public:
    ServoMotor(uint8_t pinId);
    uint8_t getAngle();  // 0~180
    void setAngle(uint8_t angle);  // 0~180
private:
    Servo servo;
};
