#pragma once

#include <stdint.h>

class Speaker {
public:
    Speaker(uint8_t pinId);
    void playTone(uint16_t frequency, uint32_t durationMs = 0);
    void stopTone();
private:
    uint8_t pinId;  // init in Speaker()
};
