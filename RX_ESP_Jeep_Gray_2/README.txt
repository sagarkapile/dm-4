DriveMatrix RX Functional build using the supplied modular vehicle/IMU/telemetry files.

RX PCB pins:
  Steering servo: GPIO 4
  ESC:            GPIO 5
  IMU SDA:        GPIO 8
  IMU SCL:        GPIO 18
  BMI270 SDO:     GPIO 7

The supplied IMUConfig defaults to BMI270, and IMUManager selects BMI270.
The supplied BMI270 driver uses I2C address 0x69. GPIO 7 is driven HIGH
before I2C initialization.

BNO055 is not used in this build.

The nRF24 control path remains independent of Wi-Fi. IMU initialization
failure is non-fatal to RF control. UDP telemetry is best-effort.

Supporting source is the supplied version, with only BMI270 pin/SDO
selection changed for the new PCB.
