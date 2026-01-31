#pragma once

#include <i2c.h>
#include <vector3.h>

class Imu {
public:
    enum class Address : uint8_t { OPTION1 = 0x69, OPTION2 = 0x68 };
    Imu(I2c* i2c, Address address = Address::OPTION1);
    bool init();
    bool getAccelG(Vector3* vector);
    bool getGyroDps(Vector3* vector);
    bool getTemperature(float* value);
    bool getAccelRaw(int16_t* value);
    bool getAccelRawX(int16_t* value);
    bool getAccelRawY(int16_t* value);
    bool getAccelRawZ(int16_t* value);
    bool getMagUT(Vector3* vector);
private:
    static constexpr uint8_t REG_CONFIG = 0x1a;
    static constexpr uint8_t REG_GYRO_CONFIG = 0x1b;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1c;
    static constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1d;
    static constexpr uint8_t REG_INT_PIN_CFG = 0x37;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3b;
    static constexpr uint8_t REG_ACCEL_YOUT_H = 0x3d;
    static constexpr uint8_t REG_ACCEL_ZOUT_H = 0x3f;
    static constexpr uint8_t REG_TEMP_OUT_H = 0x41;
    static constexpr uint8_t REG_GYRO_XOUT_H = 0x43;
    static constexpr uint8_t REG_PWR_MGMT_1 = 0x6b;
    static constexpr uint8_t VAL_CONFIG__GYRO_176HZ = 0x01;
    static constexpr uint8_t VAL_GYRO_CONFIG__2KDPS = 0x18;
    static constexpr uint8_t VAL_ACCEL_CONFIG__16G = 0x18;
    static constexpr uint8_t VAL_ACCEL_CONFIG2__420HZ = 0x07;
    static constexpr uint8_t VAL_PWR_MGMT_1__LOW_NOISE = 0x01;
    static constexpr uint8_t VAL_INT_PIN_CFG__BYPASS_EN = 0x02;

    // AK09918
    static constexpr uint8_t ADDR_AK09918 = 0x0C;
    static constexpr uint8_t AK09918_REG_HXL = 0x11;
    static constexpr uint8_t AK09918_REG_ST2 = 0x18;
    static constexpr uint8_t AK09918_REG_CNTL2 = 0x31;
    static constexpr uint8_t AK09918_VAL_CNTL2__CONT_100HZ = 0x08;

    I2c* i2c;  // init in Imu()
    uint8_t address;  // init in Imu()
};
