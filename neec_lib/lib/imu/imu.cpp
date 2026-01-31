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
        this->i2c->write(this->address, this->REG_PWR_MGMT_1, this->VAL_PWR_MGMT_1__LOW_NOISE) &&
        // Enable I2C Bypass to access AK09918
        this->i2c->write(this->address, this->REG_INT_PIN_CFG, this->VAL_INT_PIN_CFG__BYPASS_EN))) return false;
    
    constexpr uint8_t accelStartupMs = 20;  // データシート p.10
    delay(accelStartupMs);

    // Initialize AK09918 (Magnetometer)
    // Continuous measurement mode 4 (100Hz)
    if (!this->i2c->write(this->ADDR_AK09918, this->AK09918_REG_CNTL2, this->AK09918_VAL_CNTL2__CONT_100HZ)) {
        return false;
    }

    return true;
}

bool Imu::getMagUT(Vector3* vector) {
    int16_t raw[3];
    // AK09918 data is Little Endian (HXL, HXH, HYL, HYH, HZL, HZH)
    if (!this->i2c->readInt16(this->ADDR_AK09918, this->AK09918_REG_HXL, raw, 3, true)) return false;
    
    // データ読み出し後にST2レジスタを読み取る必要がある (データ更新のため)
    uint8_t st2;
    if (!this->i2c->read(this->ADDR_AK09918, this->AK09918_REG_ST2, &st2, 1)) return false;
    
    // Sensitivity: 0.15 uT/LSB
    constexpr float sensitivity = 0.15f;
    
    // 座標系: 右手 (X:右, Y:奥, Z:上)
    // AK09918の軸定義と上記要件に合わせてマッピング
    // 通常のICM20600+AK09918ボードでは軸が揃っていると仮定してそのまま代入
    // 必要に応じて符号や入れ替えを行う
    vector->x = raw[0] * sensitivity;
    vector->y = raw[1] * sensitivity;
    vector->z = raw[2] * sensitivity;
    
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
