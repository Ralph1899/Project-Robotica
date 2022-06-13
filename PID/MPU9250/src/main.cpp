/*
    @author: Ralph Stortenbeker
    @date: 13.06.22
*/

#include <iostream>

#include "../inc/sensor.h"
#include "../inc/clock.h"

#define X 0
#define Y 1
#define Z 2

int main()
{
    MPU_9250 *sensor = new MPU_9250();
    double *readings = new double[3];

    while (true)
    {
        sensor->getAccelReading(readings);


        std::cout << "ACCELERATION[X,Y,Z] [" << readings[X];
        std::cout << ", " << readings[Y] << ", " << readings[Z] << "]\n";

        sensor->getGyrosReading(readings);
        std::cout << "GYROSCOPE [X,Y,Z] [" << readings[X];
        std::cout << ", " << readings[Y] << ", " << readings[Z] << "]\n\n";

        clock::sleep_milliseconds(100);
    }
    
    return 0;
}