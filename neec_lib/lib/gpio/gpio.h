#pragma once

#include <stdint.h>

class Gpio {
public:
    enum class Mode : uint8_t { IN, OUT, PWM, DAC };
    enum class Edge : uint8_t { BOTH, FALL, RISE };
    Gpio(uint8_t pinId);
    ~Gpio();
    Mode getMode();
    void setMode(Mode mode);
    bool input();
    uint16_t inputAnalog();  // Arduino: 0~1023, ESP32: 0~4095
    void enablePullup(bool enabled);
    void setInterrupt(void (*isr)(void) = nullptr, Edge edge = Edge::BOTH);
    void output(bool value);
    void outputPwm(uint16_t value);  // Arduino: 0~255, ESP32: 0~256
    void outputAnalog(uint8_t value);
private:
    void detachPin();
    uint8_t pinId;  // init in Gpio()
    Mode mode = Mode::IN;
    bool isPullupEnabled = false;
    bool isInterruptSet = false;
#ifdef ARDUINO_ARCH_ESP32
    static constexpr uint8_t PWM_CHANNEL_COUNT = 16;
    static bool isPwmChannelUsed[PWM_CHANNEL_COUNT];  // init in .cpp
    uint8_t pwmChannel;  // available if mode == Mode::PWM
#else
    uint8_t timerId;  // available if mode == Mode::DAC
#endif
};
