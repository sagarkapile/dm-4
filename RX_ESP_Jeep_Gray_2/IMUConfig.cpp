#include "IMUConfig.h"

// Instantiate the global object with default values matching IMUConfig struct
IMUConfig imuConfig = {
    IMU_BMI270,
    AXIS_X,
    AXIS_Y,
    AXIS_Z,
    false,
    false,
    false,
    1, // forwardSign
    1, // rightSign
    1  // upSign
};

void IMU_SetConfig(const IMUConfig& config) {
    imuConfig = config;
}

void PrintIMUConfig() {
    Serial.print("Forward : ");
    Serial.println((int)imuConfig.forwardAxis);
}