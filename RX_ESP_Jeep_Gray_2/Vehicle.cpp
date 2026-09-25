#include "Vehicle.h"
#include <Arduino.h>
#include <Preferences.h>

extern Preferences prefs;
Servo steeringServo;
Servo escServo;

static int steeringMin = 0;
static int steeringCenter = 90;
static int steeringMax = 180;
static bool steeringReverse = false;
static bool reversePending = false;
static unsigned long reverseStartTime = 200;

static const int ESC_NEUTRAL_US = 1500;
static const int ESC_FORWARD_MAX_US = 2000;
static const int ESC_REVERSE_MAX_US = 1000;

enum ESCState
{
    ESC_NEUTRAL,
    ESC_FORWARD,
    ESC_REVERSE_READY,
    ESC_REVERSE
};

static ESCState escState = ESC_NEUTRAL;

bool vehicleArmed = false;

void Vehicle_Init()
{
    prefs.begin("vehicle", false);

    // Read stored calibration bounds (with safe fallbacks)
    steeringReverse = prefs.getBool("steer_rev", false);
    steeringCenter  = prefs.getInt("steer_ctr", 90);
    steeringMin     = prefs.getInt("steer_min", 0);
    steeringMax     = prefs.getInt("steer_max", 180);

    Serial.print("Steering Center  = "); Serial.println(steeringCenter);
    Serial.print("Steering Left    = "); Serial.println(steeringMin);
    Serial.print("Steering Right   = "); Serial.println(steeringMax);
    Serial.print("Steering Reverse = "); Serial.println(steeringReverse);

    // Allocate ESP32 PWM timers for servo control
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);

    steeringServo.setPeriodHertz(50);
    
    // Standard 1000us - 2000us pulse width matching RC standard
    steeringServo.attach(STEERING_SERVO_PIN, 1000, 2000);

    // Force explicit initial centering write
    steeringServo.write(steeringCenter);

    Serial.println("Steering Servo Initialized & Centered");

    escServo.setPeriodHertz(50);
    escServo.attach(ESC_PIN, 1000, 2000);
    escServo.writeMicroseconds(ESC_NEUTRAL_US);

    delay(2000);

    Serial.println("ESC Initialized");
}

void Vehicle_SetSteeringCenter(int center)
{
    center = constrain(center, 0, 180);

    if (steeringCenter == center)
    {
        return;
    }

    steeringCenter = center;
    prefs.putInt("steer_ctr", steeringCenter);

    Serial.print("Steering Center Saved = ");
    Serial.println(steeringCenter);

    steeringServo.write(steeringCenter);
}

void Vehicle_SetSteeringReverse(bool reverse)
{
    if (steeringReverse == reverse)
    {
        return;
    }

    steeringReverse = reverse;
    prefs.putBool("steer_rev", steeringReverse);

    Serial.print("Steering Reverse Saved = ");
    Serial.println(steeringReverse);
}

void Vehicle_SetSteering(int16_t steering)
{
    steering = constrain(steering, -1000, 1000);

    if (steeringReverse)
    {
        steering = -steering;
    }

    int angle;

    if (steering < 0)
    {
        angle = map(
            steering,
            -1000,
            0,
            steeringMin,
            steeringCenter
        );
    }
    else
    {
        angle = map(
            steering,
            0,
            1000,
            steeringCenter,
            steeringMax
        );
    }

    steeringServo.write(angle);
}

void Vehicle_SetSteeringLeft(int left)
{
    left = constrain(left, 0, 180);

    if (steeringMin == left)
    {
        return;
    }

    steeringMin = left;
    prefs.putInt("steer_min", steeringMin);

    Serial.print("Steering Left Saved = ");
    Serial.println(steeringMin);
}

void Vehicle_SetSteeringRight(int right)
{
    right = constrain(right, 0, 180);

    if (steeringMax == right)
    {
        return;
    }

    steeringMax = right;
    prefs.putInt("steer_max", steeringMax);

    Serial.print("Steering Right Saved = ");
    Serial.println(steeringMax);
}

void Vehicle_SetThrottle(int16_t throttle)
{
    throttle = constrain(throttle, -1000, 1000);

    if (throttle > 0)
    {
        reversePending = false;

        int pulse = map(
            throttle,
            0,
            1000,
            ESC_NEUTRAL_US,
            2000
        );

        escServo.writeMicroseconds(pulse);

        return;
    }

    if (throttle == 0)
    {
        reversePending = false;

        escServo.writeMicroseconds(ESC_NEUTRAL_US);

        return;
    }

    // Reverse requested
    if (!reversePending)
    {
        reversePending = true;

        escServo.writeMicroseconds(ESC_NEUTRAL_US);

        Serial.println("Reverse Armed");

        return;
    }

    int pulse = map(
        throttle,
        -1000,
        0,
        1000,
        ESC_NEUTRAL_US
    );

    escServo.writeMicroseconds(pulse);
}

void Vehicle_Failsafe()
{
    Serial.println("FAILSAFE");
    Vehicle_SetThrottle(0);
    Vehicle_SetSteering(0);
}