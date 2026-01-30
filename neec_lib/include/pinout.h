#pragma once

#include <stdint.h>

class Pinout {
public:
#if defined(ARDUINO_ARCH_ESP32)
    static constexpr uint8_t CN1_D1_DAC = 25;
    static constexpr uint8_t CN1_D2 = 14;
    static constexpr uint8_t CN2_D3_DAC = 26;
    static constexpr uint8_t CN2_D4 = 15;
    static constexpr uint8_t CN3_A0 = 4;
    // static constexpr uint8_t CN456_I2C_SDA = 19;
    // static constexpr uint8_t CN456_I2C_SCL = 21;
#else
    static constexpr uint8_t D0_UART_RX = 0;
    static constexpr uint8_t D1_UART_TX = 1;
    static constexpr uint8_t D2 = 2;
    static constexpr uint8_t D3_PWM = 3;
    static constexpr uint8_t D4 = 4;
    static constexpr uint8_t D5_PWM = 5;
    static constexpr uint8_t D6_PWM = 6;
    static constexpr uint8_t D7 = 7;
    // static constexpr uint8_t D8 = 8;
    // static constexpr uint8_t D9_PWM = 9;
    // static constexpr uint8_t D10_PWM = 10;
    #if defined(ARDUINO_ARCH_MEGAAVR)
    // static constexpr uint8_t D11 = 11;
    #else
    // static constexpr uint8_t D11_PWM = 11;
    #endif
    // static constexpr uint8_t D12 = 12;
    static constexpr uint8_t D13_LED = 13;
    static constexpr uint8_t A0 = 14;
    static constexpr uint8_t A1 = 15;
    static constexpr uint8_t A2 = 16;
    static constexpr uint8_t A3 = 17;
    // static constexpr uint8_t A4_I2C_SDA = 18;
    // static constexpr uint8_t A5_I2C_SCL = 19;
    #if defined(ARDUINO_ARCH_MEGAAVR)
    static constexpr uint8_t A6 = 20;
    static constexpr uint8_t A7 = 21;
    #else
    static constexpr uint8_t A6_AIN = 20;
    static constexpr uint8_t A7_AIN = 21;
    #endif
#endif
};
