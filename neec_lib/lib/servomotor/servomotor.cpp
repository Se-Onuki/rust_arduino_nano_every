#include <Arduino.h>
#include <servomotor.h>

ServoMotor::ServoMotor(uint8_t pinId) {
    const uint16_t minPulseWidth = max(MIN_PULSE_WIDTH, 600);
    const uint16_t maxPulseWidth = min(MAX_PULSE_WIDTH, 2350);
    this->servo.attach(pinId, minPulseWidth, maxPulseWidth);
}

uint8_t ServoMotor::getAngle() {
    return this->servo.read();
}

void ServoMotor::setAngle(uint8_t angle) {
    this->servo.write(angle);
}
