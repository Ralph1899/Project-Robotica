#include "../inc/accel.h"

Accelerometer::Accelerometer(accel_sensitivity sensitivity)
{
    scaleFactor = sensitivity;
}

double Accelerometer::getAcceleration(int sensor, accel_axis axis)
{
   int rawAcceleration = i2c::readDevice(sensor, axis);
   return scaleAccel(rawAcceleration);
}

double Accelerometer::scaleAccel(int raw)
{
    return ((raw / scaleFactor) * GRAVITATIONAL_CONST);
}