#include "BMI270Driver.h"
#include <Wire.h>

// I2C Pin definitions from RX_PCB schematic
#define IMU_SDA_PIN 8
#define IMU_SCL_PIN 18
#define IMU_SDO_PIN 7

bool BMI270Driver::Init() {

    Serial.println("DEBUG IMU: Before Wire.begin");

    // RX PCB pins. BMI270 SDO HIGH selects I2C address 0x69.
    pinMode(IMU_SDO_PIN, OUTPUT);
    digitalWrite(IMU_SDO_PIN, HIGH);
    delay(10);

    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);

    Serial.println("DEBUG IMU: After Wire.begin");
    Serial.println("DEBUG IMU: Before BMI270 beginI2C");

    // BMI270 address 0x69 with SDO HIGH
    if (imu.beginI2C(0x69, Wire) != 0) {
        isConnected = false;
        return false;
    }
    Serial.print("DEBUG IMU: beginI2C result = ");
    isConnected = true;
    Serial.println("DEBUG IMU: BMI270 connected");

    return true;
}

void BMI270Driver::Update() {
    imu.getSensorData();

    rawMotion.accel[0] = imu.data.accelX;
    rawMotion.accel[1] = imu.data.accelY;
    rawMotion.accel[2] = imu.data.accelZ;

    rawMotion.gyro[0] = imu.data.gyroX;
    rawMotion.gyro[1] = imu.data.gyroY;
    rawMotion.gyro[2] = imu.data.gyroZ;
}

const RawMotion& BMI270Driver::GetRawMotion() {
    return rawMotion;
}

bool BMI270Driver::IsHealthy() {
    return isConnected;
}