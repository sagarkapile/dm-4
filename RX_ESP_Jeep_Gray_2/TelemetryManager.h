#ifndef TELEMETRY_MANAGER_H
#define TELEMETRY_MANAGER_H

#include <Arduino.h>
#include "DriveMatrixProtocol.h"

bool Telemetry_Init();
void Telemetry_Update();
const TelemetryPacket& Telemetry_GetPacket();

#endif // TELEMETRY_MANAGER_H



// #pragma once

// #include "TelemetryPacket.h"

// bool Telemetry_Init();

// void Telemetry_Update();

// const TelemetryPacket&
// Telemetry_GetPacket();