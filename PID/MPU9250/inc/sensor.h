#ifndef SENSOR_H
#define SENSOR_H

#include <iostream>

#include "../inc/accel.h"
#include "../inc/clock.h"
#include "../inc/gyro.h"
#include "../inc/i2c.h"
#include "../inc/mag.h"

class MPU_9250
{
private:
    int mAccGyroHandler, mAccGyroWakeHandler, mMagHandler, mMagWakeHandler;
    Accelerometer *pAccel;
    Gyroscope *pGyros;
    double *pMemoryBuffer = nullptr;
public:
    MPU_9250();
    ~MPU_9250() { };

    void setAccelSensitivity(accel_sensitivity sensitivity);
    void setGyrosSensitivity(gyros_sensitivity sensitivity);
    void setMemoryBuffer(double *pMemoryLocation);

    void getSensorReading();
};

#endif // !SENSOR_H