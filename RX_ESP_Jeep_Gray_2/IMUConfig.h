#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

#include <Arduino.h>

enum IMUType {
    IMU_BMI270,
    IMU_BNO055
};

enum Axis {
    AXIS_X,
    AXIS_Y,
    AXIS_Z
};

struct IMUConfig {
    IMUType type;
    Axis forwardAxis;
    Axis rightAxis;
    Axis upAxis;
    bool invertForward;
    bool invertRight;
    bool invertUp;
    int8_t forwardSign; // Added
    int8_t rightSign;   // Added
    int8_t upSign;      // Added
};

extern IMUConfig imuConfig;

void IMU_SetConfig(const IMUConfig& config);
void PrintIMUConfig();

#endif // IMU_CONFIG_H