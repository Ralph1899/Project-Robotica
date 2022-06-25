#include "../inc/gyro.h"

Gyroscope::Gyroscope(gyros_sensitivity sensitivity)
{
    scaleFactor = (sensitivity / 10);
}

double Gyroscope::getGyroscope(int sensor, gyros_axis axis)
{
    int rawGyro = i2c::readDevice(sensor, axis);
    return scaleGyro(rawGyro);
}

double Gyroscope::scaleGyro(int raw)
{
    return (raw / scaleFactor);
}