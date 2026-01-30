#include <Arduino.h>
#include <motor.h>

Motor::Motor(I2c* i2c, Address address) {
    this->i2c = i2c;
    this->address = (uint8_t)address;
}

bool Motor::drive(float voltage) {
    const float absoluteVoltage = min(fabs(voltage), this->maxVoltage);
    constexpr float voltagePerLsb = 4 * 1.285 / 64;
    uint8_t vset = min(absoluteVoltage / voltagePerLsb, (float)this->VAL_CONTROL__VSET_MAX);
    uint8_t bridge = this->VAL_CONTROL__BRIDGE_COAST;
    if (vset < this->VAL_CONTROL__VSET_MIN) {
        vset = 0;
    } else {
        bridge = voltage > 0 ? this->VAL_CONTROL__BRIDGE_FORWARD : this->VAL_CONTROL__BRIDGE_REVERSE;
    }
    return this->i2c->write(this->address, this->REG_CONTROL, (vset << 2) + bridge);
}

bool Motor::brake() {
    return this->i2c->write(this->address, this->REG_CONTROL, this->VAL_CONTROL__BRIDGE_BRAKE);
}
