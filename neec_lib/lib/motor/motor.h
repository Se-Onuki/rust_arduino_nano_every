#pragma once

#include <i2c.h>

class Motor {
public:
    enum class Address : uint8_t { CH1 = 0x65, CH2 = 0x60 };
    Motor(I2c* i2c, Address address = Address::CH1);
    bool drive(float voltage);
    bool brake();
    float maxVoltage = 3;
private:
    static constexpr uint8_t REG_CONTROL = 0x00;
    static constexpr uint8_t VAL_CONTROL__VSET_MIN = 0x06;
    static constexpr uint8_t VAL_CONTROL__VSET_MAX = 0x3f;
    static constexpr uint8_t VAL_CONTROL__BRIDGE_COAST = 0x00;
    static constexpr uint8_t VAL_CONTROL__BRIDGE_FORWARD = 0x01;
    static constexpr uint8_t VAL_CONTROL__BRIDGE_REVERSE = 0x02;
    static constexpr uint8_t VAL_CONTROL__BRIDGE_BRAKE = 0x03;
    I2c* i2c;  // init in Motor()
    uint8_t address;  // init in Motor()
};
