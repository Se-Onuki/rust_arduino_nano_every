#include <Arduino.h>
#include <imu.h>

Imu::Imu(I2c* i2c, Address address) {
    this->i2c = i2c;
    this->address = (uint8_t)address;
}

bool Imu::init() {
    if (!(this->i2c->write(this->address, this->REG_CONFIG, this->VAL_CONFIG__GYRO_176HZ) &&
        this->i2c->write(this->address, this->REG_GYRO_CONFIG, this->VAL_GYRO_CONFIG__2KDPS) &&
        this->i2c->write(this->address, this->REG_ACCEL_CONFIG, this->VAL_ACCEL_CONFIG__16G) &&
        this->i2c->write(this->address, this->REG_ACCEL_CONFIG2, this->VAL_ACCEL_CONFIG2__420HZ) &&
        this->i2c->write(this->address, this->REG_PWR_MGMT_1, this->VAL_PWR_MGMT_1__LOW_NOISE))) return false;
    constexpr uint8_t accelStartupMs = 20;  // データシート p.10
    delay(accelStartupMs);
    return true;
}

bool Imu::getAccelG(Vector3* vector) {
    if (!this->i2c->readVector(this->address, this->REG_ACCEL_XOUT_H, vector, false)) return false;
    constexpr float sensitivity_16g = 2048;
    *vector /= sensitivity_16g;
    return true;
}

bool Imu::getGyroDps(Vector3* vector) {
    if (!this->i2c->readVector(this->address, this->REG_GYRO_XOUT_H, vector, false)) return false;
    constexpr float sensitivity_2kdps = 16.4;
    *vector /= sensitivity_2kdps;
    return true;
}

bool Imu::getTemperature(float* value) {
    if (!this->i2c->readFloat(this->address, this->REG_TEMP_OUT_H, value, false)) return false;
    constexpr float sensitivity = 326.8;
    constexpr float offset = 25;
    *value = *value / sensitivity + offset;
    return true;
}

bool Imu::getAccelRaw(int16_t* values) {
    return this->i2c->readInt16(this->address, this->REG_ACCEL_XOUT_H, values, 3, false);
}

bool Imu::getAccelRawX(int16_t* value) {
    return this->i2c->readInt16(this->address, this->REG_ACCEL_XOUT_H, value, 1, false);
}

bool Imu::getAccelRawY(int16_t* value) {
    return this->i2c->readInt16(this->address, this->REG_ACCEL_YOUT_H, value, 1, false);
}

bool Imu::getAccelRawZ(int16_t* value) {
    return this->i2c->readInt16(this->address, this->REG_ACCEL_ZOUT_H, value, 1, false);
}
