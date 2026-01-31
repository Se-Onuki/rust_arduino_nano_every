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
        Serial.print(accel.x, 4); Serial.print(",");
        Serial.print(accel.y, 4); Serial.print(",");
        Serial.print(accel.z, 4); Serial.print(",");
        Serial.print(gyro.x, 4); Serial.print(",");
        Serial.print(gyro.y, 4); Serial.println(gyro.z, 4);
    }

    delay(20); // Approx 50Hz
}
