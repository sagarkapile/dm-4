#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include "IMUConfig.h"

// I2C Pin definitions from RX_PCB schematic
#define IMU_SDA_PIN 8
#define IMU_SCL_PIN 18

struct VehicleMotion {
    float yawRate;
    float forwardAccel;
    float lateralAccel;
    float verticalAccel;
};

bool IMU_Init();
void IMU_Update();
void IMU_SetConfig(const IMUConfig& config);
const VehicleMotion& IMU_GetMotion();
void PrintIMUConfig();

#endif // IMU_H