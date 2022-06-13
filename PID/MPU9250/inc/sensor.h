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
    int accel_gyro_handler, mag_handler, wake_handler, mag_wake_handler;
    Accelerometer *accel;
    Gyroscope *gyro;
public:
    MPU_9250();
    ~MPU_9250() { };

    void getAccelReading(double *readings);
    void getGyrosReading(double *readings);
};

#endif // !SENSOR_H