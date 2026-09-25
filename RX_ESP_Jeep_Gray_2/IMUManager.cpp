#include "IMUManager.h"
#include "IMUDriver.h"
#include "BMI270Driver.h" // Updated Driver Header
#include "VehicleMapper.h"
#include "IMUConfig.h"

static IMUDriver* driver = nullptr;
static BMI270Driver bmi270Driver; // Instantiate BMI270 Driver Instance

static VehicleMotion motion;
static VehicleMotion emptyMotion;

bool IMU_Init()
{
    switch (imuConfig.type)
    {
        case IMU_BMI270:
            driver = &bmi270Driver;
            break;

        case IMU_BNO055:
            // Optional fallback if BNO055 is still needed, otherwise target BMI270
            driver = &bmi270Driver;
            break;

        default:
            driver = &bmi270Driver;
            break;
    }

    return driver->Init();
}

void IMU_Update()
{
    if (driver)
    {
        driver->Update();
    }
}

const VehicleMotion& IMU_GetMotion()
{
    if (!driver)
    {
        return emptyMotion;
    }

    const RawMotion& raw = driver->GetRawMotion();

    motion = MapVehicleMotion(raw);

    return motion;
}