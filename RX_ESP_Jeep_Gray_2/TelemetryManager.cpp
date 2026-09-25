#include "TelemetryManager.h"
#include <Arduino.h>
#include <WiFi.h>
#include "IMU.h"

static TelemetryPacket packet;

bool Telemetry_Init()
{
    memset(&packet, 0, sizeof(packet));
    packet.type = PACKET_TELEMETRY;
    return true;
}

void Telemetry_Update()
{
    const VehicleMotion& motion = IMU_GetMotion();

    packet.type = PACKET_TELEMETRY;

    // Include the permanent ESP32 factory MAC so the Pi can
    // unambiguously associate telemetry with the correct RX/car.
    uint64_t chipId = ESP.getEfuseMac();
    packet.receiverMAC[0] = (uint8_t)(chipId >> 40);
    packet.receiverMAC[1] = (uint8_t)(chipId >> 32);
    packet.receiverMAC[2] = (uint8_t)(chipId >> 24);
    packet.receiverMAC[3] = (uint8_t)(chipId >> 16);
    packet.receiverMAC[4] = (uint8_t)(chipId >> 8);
    packet.receiverMAC[5] = (uint8_t)chipId;

    packet.timestamp = millis();

    packet.accelX = motion.forwardAccel;
    packet.accelY = motion.lateralAccel;
    packet.accelZ = motion.verticalAccel;

    // VehicleMotion currently provides only yaw rate.
    packet.gyroX = 0.0f;
    packet.gyroY = 0.0f;
    packet.gyroZ = motion.yawRate;
}

const TelemetryPacket& Telemetry_GetPacket()
{
    return packet;
}