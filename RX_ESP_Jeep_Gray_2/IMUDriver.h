#pragma once

#include "RawMotion.h"

class IMUDriver
{
public:

    virtual ~IMUDriver() {}

    virtual bool Init() = 0;

    virtual void Update() = 0;

    virtual const RawMotion& GetRawMotion() = 0;

    virtual bool IsHealthy() = 0;
};