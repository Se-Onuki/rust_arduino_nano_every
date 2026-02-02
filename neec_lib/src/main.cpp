#include <Arduino.h>
#include <pinout.h>
#include <imu.h>
#include <i2c.h>

I2c i2c;
Imu imu(&i2c);

void setup() {
    Serial.begin(115200);
    
    // I2Cの初期化
    // このカスタムI2cクラスはコンストラクタで設定されるため、明示的な初期化は不要です
    
    // IMUの初期化
    if (!imu.init()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }
}

void loop() {
    Vector3 accel;
    Vector3 gyro;
    Vector3 mag;

    bool result = imu.getAccelG(&accel) 
                && imu.getGyroDps(&gyro) 
                && imu.getMagUT(&mag);

    if (result) {
        // バイナリとして送信 (リトルエンディアン浮動小数点数)
        // float(4バイト) * 9 = 36バイト
        float data[9];
        data[0] = accel.x;
        data[1] = accel.y;
        data[2] = accel.z;
        data[3] = gyro.x;
        data[4] = gyro.y;
        data[5] = gyro.z;
        data[6] = mag.x;
        data[7] = mag.y;
        data[8] = mag.z;
        
        Serial.write((uint8_t*)data, sizeof(data));
    }


    // delay(20); // 速度最適化のため削除
}
