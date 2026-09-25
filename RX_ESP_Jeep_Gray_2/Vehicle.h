#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdint.h>
#include <ESP32Servo.h>

// Updated according to RX_PCB schematic
#define STEERING_SERVO_PIN 4  // IO4 -> SERVO_PWM
#define ESC_PIN            5  // IO5 -> ESC_PWM

void Vehicle_Init();
void Vehicle_SetSteering(int16_t steering);
void Vehicle_SetThrottle(int16_t throttle);
void Vehicle_SetSteeringReverse(bool reverse);
void Vehicle_SetSteeringCenter(int center);
void Vehicle_SetSteeringLeft(int left);
void Vehicle_SetSteeringRight(int right);
void Vehicle_Failsafe();

#endif