#include <Arduino.h>
#include <speaker.h>

Speaker::Speaker(uint8_t pinId) {
    this->pinId = pinId;
}

void Speaker::playTone(uint16_t frequency, uint32_t durationMs) {
    tone(this->pinId, frequency, durationMs);
}

void Speaker::stopTone() {
    noTone(this->pinId);
}
