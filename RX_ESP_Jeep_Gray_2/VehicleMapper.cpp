#include "VehicleMapper.h"
#include "IMUConfig.h"

inline float AxisValue(const float axis[3], Axis a)
{
    int index = (int)a;
    if (index < 0 || index > 2)
    {
        return 0.0f;
    }
    return axis[index];
}

VehicleMotion MapVehicleMotion(const RawMotion& raw)
{
    VehicleMotion motion;

    motion.forwardAccel =
        AxisValue(raw.accel, imuConfig.forwardAxis) * imuConfig.forwardSign;


    motion.lateralAccel =
        AxisValue(raw.accel, imuConfig.rightAxis) * imuConfig.rightSign;

    // Subtract 1.0g gravity baseline so static vertical acceleration reads 0.0
    float rawVert = AxisValue(raw.accel, imuConfig.upAxis) * imuConfig.upSign;
    motion.verticalAccel = rawVert - 1.0f;

    motion.rollRate =
        AxisValue(raw.gyro, imuConfig.forwardAxis) * imuConfig.forwardSign;

    motion.pitchRate =
        AxisValue(raw.gyro, imuConfig.rightAxis) * imuConfig.rightSign;

    motion.yawRate =
        AxisValue(raw.gyro, imuConfig.upAxis) * imuConfig.upSign;

    return motion;
}