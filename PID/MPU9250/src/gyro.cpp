#include "../inc/gyro.h"

Gyroscope::Gyroscope(gyro_sensitivity sensitivity)
{
    scaleFactor = (sensitivity / 10);
}

double Gyroscope::getGyroscope(int sensor, gyro_axis axis)
{
    int rawGyro = i2c::readDevice(sensor, axis);
    return scaleGyro(rawGyro);
}

double Gyroscope::scaleGyro(int raw)
{
    return (raw / scaleFactor);
}