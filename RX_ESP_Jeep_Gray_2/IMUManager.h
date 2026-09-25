#pragma once

#include "VehicleMotion.h"

bool IMU_Init();

void IMU_Update();

const VehicleMotion& IMU_GetMotion();