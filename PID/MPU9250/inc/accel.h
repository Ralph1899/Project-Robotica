#ifndef ACCEL_H
#define ACCEL_H

#include "../inc/i2c.h"

//defines address for register responsible for configuring accelerometer
#define ACCEL_CONFIG_ADDR 0X1C

//defines scales for accelerometer
#define ACCEL_FSCALE_2G 0x00
#define ACCEL_FSCALE_4G 0x01
#define ACCEL_FSCALE_8G 0x02
#define ACCEL_FSCALE_16G 0x03

//defines gravitational acceleration at the equator in m/s^2
#define GRAVITATIONAL_CONST 9.7803

enum accel_sensitivity
{
    ACCEL_SENSITIVITY_2G = 16384,
    ACCEL_SENSITIVITY_4G = 8192,
    ACCEL_SENSITIVITY_8G = 4096,
    ACCEL_SENSITIVITY_16G = 2048
};

enum accel_axis
{
    ACCEL_X = 0x3B,
    ACCEL_Y = 0x3D,
    ACCEL_Z = 0x3F
};

class Accelerometer
{
private:
    double scaleFactor;
    double scaleAccel(int raw);
public:
    Accelerometer(accel_sensitivity sensitivity);
    ~Accelerometer() {};

    double getAcceleration(int sensor, accel_axis axis);
};

#endif // !ACCEL_H