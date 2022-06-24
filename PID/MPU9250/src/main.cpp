/*
    @author: Ralph Stortenbeker
    @date: 13.06.22
*/

#include <iostream>

#include "../inc/sensor.h"
#include "../inc/clock.h"
#include "../inc/Kalman.h"

#define X 0
#define Y 1
#define Z 2

int main()
{
    Kalman *pKalmanX, *pKalmanY, *pKalmanZ;
    MPU_9250 *sensor = new MPU_9250();
    double *readings = new double[3];

    float accX;
    float accY;
    float accZ;

    double roll = (180 / 1) * atan(accX / sqrt(sq(accY) + sq(accZ)));

    while (true)
    {
        sensor->getAccelReading(readings);


        std::cout << "ACCELERATION[X,Y,Z] [" << readings[X];
        std::cout << ", " << readings[Y] << ", " << readings[Z] << "]\n";

        sensor->getGyrosReading(readings);
        std::cout << "GYROSCOPE [X,Y,Z] [" << readings[X];
        std::cout << ", " << readings[Y] << ", " << readings[Z] << "]\n\n";

        clock::sleep_milliseconds(500);
    }

    return 0;
}
