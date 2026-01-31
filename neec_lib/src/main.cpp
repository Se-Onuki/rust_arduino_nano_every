#include <Arduino.h>
#include <pinout.h>
#include <imu.h>
#include <i2c.h>

I2c i2c;
Imu imu(&i2c);

void setup() {
    Serial.begin(115200);
    
    // I2C initialization
    // No explicit init needed for this custom I2c class based on constructor
    
    // IMU initialization
    if (!imu.init()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }
}

void loop() {
    Vector3 accel;
    Vector3 gyro;

    if (imu.getAccelG(&accel) && imu.getGyroDps(&gyro)) {
        // Send as binary (Little Endian floating point)
        // 6 floats * 4 bytes = 24 bytes
        float data[6];
        data[0] = accel.x;
        data[1] = accel.y;
        data[2] = accel.z;
        data[3] = gyro.x;
        data[4] = gyro.y;
        data[5] = gyro.z;
        
        Serial.write((uint8_t*)data, sizeof(data));
    }

    delay(20); // Approx 50Hz
}
