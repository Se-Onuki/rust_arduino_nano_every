#pragma once

#include <stdint.h>
#include <vector3.h>

class I2c {
public:
    I2c();
    ~I2c();
    void setFastMode(bool enabled);
    bool read(uint8_t address, uint8_t reg, uint8_t* data, uint8_t length = 1);
    bool readInt16(uint8_t address, uint8_t reg, int16_t* values, uint8_t length = 1, bool isLittleEndian = true);
    bool readFloat(uint8_t address, uint8_t reg, float* value, bool isLittleEndian = true);
    bool readVector(uint8_t address, uint8_t reg, Vector3* vector, bool isLittleEndian = true);
    bool write(uint8_t address, uint8_t reg);
    bool write(uint8_t address, uint8_t reg, uint8_t data);
    bool setChannel(uint8_t channel);
    bool setChannelMap(uint8_t channelMap);
    static constexpr uint8_t CHANNEL_COUNT = 8;
private:
    static constexpr uint8_t MULTIPLEXER = 0x70;
};
