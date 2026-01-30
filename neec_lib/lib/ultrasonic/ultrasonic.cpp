#include <Arduino.h>
#include <ultrasonic.h>

Ultrasonic::Ultrasonic(uint8_t pinId) {
    this->gpio = new Gpio(pinId);
}

Ultrasonic::~Ultrasonic() {
    delete this->gpio;
}

bool Ultrasonic::measureDistanceMm(uint16_t* distance) {
    this->requestPulse();
    uint32_t width;
    if (!this->measurePulseWidthUs(&width)) return false;
    constexpr float sensitivity = 5.8;
    *distance = width / sensitivity;
    return true;
}

void Ultrasonic::requestPulse() {
    gpio->output(false);
    delayMicroseconds(2);
    gpio->output(true);
    delayMicroseconds(5);
    gpio->output(false);
}

bool Ultrasonic::measurePulseWidthUs(uint32_t* width) {
    constexpr uint32_t timeoutUs = 100ul * 1000;
    const uint32_t startedUs = micros();
    while (gpio->input()) {
        if (micros() - startedUs > timeoutUs) return false;
    }
    while (!gpio->input()) {
        if (micros() - startedUs > timeoutUs) return false;
    }
    const uint32_t pulseStartedUs = micros();
    while (gpio->input()) {
        if (micros() - startedUs > timeoutUs) return false;
    }
    *width = micros() - pulseStartedUs;
    return true;
}
