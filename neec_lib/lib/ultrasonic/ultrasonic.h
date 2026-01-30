#pragma once

#include <stdint.h>
#include <gpio.h>

class Ultrasonic {
public:
    Ultrasonic(uint8_t pinId);
    ~Ultrasonic();
    bool measureDistanceMm(uint16_t* distance);
private:
    void requestPulse();
    bool measurePulseWidthUs(uint32_t* width);
    Gpio* gpio;  // init in Ultrasonic()
};
