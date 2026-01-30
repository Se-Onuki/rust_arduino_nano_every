#include <Arduino.h>
#include <Wire.h>
#include <i2c.h>

I2c::I2c() {
    Wire.begin();
}

I2c::~I2c() {
    Wire.end();
}

void I2c::setFastMode(bool enabled) {
    Wire.setClock(enabled ? 400000 : 100000);
}

bool I2c::read(uint8_t address, uint8_t reg, uint8_t* data, uint8_t length) {
    if (!this->write(address, reg)) return false;
    const uint8_t receivedLength = Wire.requestFrom(address, (size_t)length);  // size_t for MEGAAVR
    for (uint8_t* p = data; p < data + min(receivedLength, length); p++) {
        *p = Wire.read();
    }
    while (Wire.available()) {
        Wire.read();
    }
    return receivedLength >= length;
}

bool I2c::readInt16(uint8_t address, uint8_t reg, int16_t* values, uint8_t length, bool isLittleEndian) {
    uint8_t data[2 * length];
    if (!this->read(address, reg, data, sizeof(data))) return false;
    for (uint8_t i = 0; i < length; i++) {
        if (isLittleEndian) {
            values[i] = data[i * 2] + ((uint16_t)data[i * 2 + 1] << 8);
        } else {
            values[i] = ((uint16_t)data[i * 2] << 8) + data[i * 2 + 1];
        }
    }
    return true;
}

bool I2c::readFloat(uint8_t address, uint8_t reg, float* value, bool isLittleEndian) {
    uint8_t data[2];
    if (!this->read(address, reg, data, sizeof(data))) return false;
    *value = (int16_t)(isLittleEndian ? data[0] + ((uint16_t)data[1] << 8) : ((uint16_t)data[0] << 8) + data[1]);
    return true;
}

bool I2c::readVector(uint8_t address, uint8_t reg, Vector3* vector, bool isLittleEndian) {
    uint8_t data[2 * 3];
    if (!this->read(address, reg, data, sizeof(data))) return false;
    if (isLittleEndian) {
        vector->x = (int16_t)(data[0] + ((uint16_t)data[1] << 8));
        vector->y = (int16_t)(data[2] + ((uint16_t)data[3] << 8));
        vector->z = (int16_t)(data[4] + ((uint16_t)data[5] << 8));
    } else {
        vector->x = (int16_t)(((uint16_t)data[0] << 8) + data[1]);
        vector->y = (int16_t)(((uint16_t)data[2] << 8) + data[3]);
        vector->z = (int16_t)(((uint16_t)data[4] << 8) + data[5]);
    }
    return true;
}

bool I2c::write(uint8_t address, uint8_t reg) {
    Wire.beginTransmission(address);
    bool isOk = Wire.write(reg) == 1;
    isOk &= Wire.endTransmission() == 0;
    return isOk;
}

bool I2c::write(uint8_t address, uint8_t reg, uint8_t data) {
    Wire.beginTransmission(address);
    bool isOk = Wire.write(reg) == 1 && Wire.write(data) == 1;
    isOk &= Wire.endTransmission() == 0;
    return isOk;
}

bool I2c::setChannel(uint8_t channel) {
    if (channel >= I2c::CHANNEL_COUNT) return false;
    return this->write(MULTIPLEXER, 1 << channel);
}

bool I2c::setChannelMap(uint8_t channelMap) {
    return this->write(MULTIPLEXER, channelMap);
}
