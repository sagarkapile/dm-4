#ifndef BMI270_DRIVER_H
#define BMI270_DRIVER_H

#include "IMUDriver.h"
#include <SparkFun_BMI270_Arduino_Library.h>

class BMI270Driver : public IMUDriver {
private:
    BMI270 imu;
    RawMotion rawMotion;
    bool isConnected = false;

public:
    bool Init() override;
    void Update() override;
    const RawMotion& GetRawMotion() override;
    bool IsHealthy() override; // Declare IsHealthy
};

#endif