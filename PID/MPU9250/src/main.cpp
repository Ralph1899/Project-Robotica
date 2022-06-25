/*
    @author: Ralph Stortenbeker
    @date: 13.06.22
*/

#include <iostream>
#include <math.h>
#include <sys/time.h>

#include "../inc/sensor.h"
#include "../inc/clock.h"
#include "../inc/Kalman.h"

#define M_PI 3.14159265358979323846  /* pi */
#define RAD_TO_DEG 57.29577951308233

double micros()
{
   struct timeval tv;
   double t;

   gettimeofday(&tv, 0);

   t = (double)tv.tv_sec + (double)tv.tv_usec;

   return t;
}

int main()
{
    Kalman *pKalmanX = new Kalman(), *pKalmanY = new Kalman();
    
    MPU_9250 *sensor = new MPU_9250();
    sensor->setAccelSensitivity(ACCEL_SENSITIVITY_2G);
    sensor->setGyrosSensitivity(GYRO_SENSITIVITY_250);

    double *pSensorBuffer = new double[6];

    sensor->setMemoryBuffer(pSensorBuffer);

    double *pAccelX = &pSensorBuffer[0];
    double *pAccelY = &pSensorBuffer[1];
    double *pAccelZ = &pSensorBuffer[2];
    double *pGyrosX = &pSensorBuffer[3];
    double *pGyrosY = &pSensorBuffer[4];
    double *pGyrosZ = &pSensorBuffer[5];

    double roll = (180 / M_PI) * atan2(pAccelX, sqrt(pow(pAccelY, 2) + pow(pAccelZ, 2)));
    double pitch = (180 / M_PI) * atan2(pAccelY, sqrt(pow(pAccelX, 2) + pow(pAccelZ, 2)));

    pKalmanX->setAngle(roll);
    pKalmanY->setAngle(pitch);

    double compRoll, compPitch, gyroYaw;
    double gyroXrate = 0;
    double gyroYrate = 0;
    double timer = micros();

    while (true)
    {
        sensor->getSensorReading();
        
        double dT = (double)(micros() - timer) / 1000000; 
        timer = micros();
        
        //Angle from accelorometer
        double roll = (180 / M_PI) * atan2(pAccelX, sqrt(pow(pAccelY, 2) + pow(pAccelZ, 2)));
        double pitch = (180 / M_PI) * atan2(pAccelY, sqrt(pow(pAccelX, 2) + pow(pAccelZ, 2)));

        // Angle from gyro
        gyroXrate += (pGyrosX * RAD_TO_DEG) * dT;
        gyroYrate += (pGyrosY * RAD_TO_DEG) * dT;

        // Angle from Kalman
        double kalRoll = pKalmanX->getAngle(roll, gyroXrate, dT);
        double kalPitch = pKalmanY->getAngle(pitch, gyroYrate, dT);

        //Angle from comp.
        //compRoll = (double)0.96 * (compRoll + pGyrosY * dT) + 0.04 * roll;
        //compPitch = (double)0.96 * (compPitch + pGyrosX * dT) + 0.04 * pitch;
        //gyroYaw = (double)(gyroYaw + (pGyrosZ * dT));

        std::cout << "Original Roll: " << roll << "\n";
        std::cout << "Filtered Roll: " << kalRoll << "\n\n";

        //std::cout << "Original Pitch: " << pitch << "\n";
        //std::cout << "Filtered Pitch: " << kalPitch << "\n\n";


        clock::sleep_milliseconds(1000);
    }

    return 0;
}
