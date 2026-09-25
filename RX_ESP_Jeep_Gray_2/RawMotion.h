#pragma once

struct RawMotion
{
    float accel[3];

    float gyro[3];

    bool valid = false;
};