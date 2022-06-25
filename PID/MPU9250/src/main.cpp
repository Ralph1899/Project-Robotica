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

#define X 0
#define Y 1
#define Z 2

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
    double *readings = new double[3];

    sensor->getAccelReading(readings);
    float accX = readings[X];
    float accY = readings[Y];
    float accZ = readings[Z];

    sensor->getGyrosReading(readings);
    float gyroX = readings[X];
    float gyroY = readings[Y];
    float gyroZ = readings[Z];

    double roll = (180 / M_PI) * atan2(accX / sqrt(pow(accY, 2) + pow(accZ, 2)));
    double pitch = (180 / M_PI) * atan2(accY / sqrt(pow(accX, 2) + pow(accZ, 2)));

    pKalmanX->setAngle(roll);
    pKalmanY->setAngle(pitch);

    //double gyroXangle = roll;
    //double gyroYangle = pitch;
    //double compAngleX = roll;
    //double compAngleY = pitch;

    double timer = micros(), compRoll, compPitch, gyroYaw;
    double gyroXrate = 0;
    double gyroYrate = 0;


    while (true)
    {
        sensor->getAccelReading(readings);
        accX = readings[X];
        accY = readings[Y];
        accZ = readings[Z];

        sensor->getGyrosReading(readings);
        gyroX = readings[X];
        gyroY = readings[Y];
        gyroZ = readings[Z];

        double dT = (double)(micros() - timer) / 1000000;
        timer = micros();

        //Angle from accelorometer
        double roll = (180 / M_PI) * atan2(accX / sqrt(pow(accY, 2) + pow(accZ, 2)));
        double pitch = (180 / M_PI) * atan2(accY / sqrt(pow(accX, 2) + pow(accZ, 2)));

        // Angle from gyro
        gyroXrate += (gyroX * RAD_TO_DEG) * dT;
        gyroYrate += (gyroY * RAD_TO_DEG) * dT;

        // Angle from Kalman
        double kalRoll = pKalmanX->getAngle(roll, gyroXrate, dT);
        double kalPitch = pKalmanY->getAngle(pitch, gyroYrate, dT);

        //Angle from comp.
        compRoll = (double)0.96 * (compRoll + gyroY * dT) + 0.04 * roll;
        compPitch = (double)0.96 * (compPitch + gyroX * dT) + 0.04 * pitch;
        gyroYaw = (double)(gyroYaw + (gyroZ * dT));

        std::cout << "Original Roll: " << roll << "\n";
        std::cout << "Filtered Roll: " << kalRoll << "\n\n";

        //std::cout << "Original Pitch: " << pitch << "\n";
        //std::cout << "Filtered Pitch: " << kalPitch << "\n\n";

        //runPIDX(kalRoll, 0, dT);
        //runPIDY(kalPitch, 0, dT);

        //std::cout << "micros    : " <<micros() << "\n";
        //std::cout << "lastXdelay: " <<lastMotorXDelayTime << "\n";
        //std::cout << "subtracted: " <<micros() - lastMotorXDelayTime << "\n\n";
        //std::cout << "set delay : " <<motorXDelayActual << "\n";

        
        //if ((micros() - lastMotorXDelayTime) > motorXDelayActual) 
        //{ 
        //    runMotorX(); 
        //    lastMotorXDelayTime = micros();
        //}

        //if ((micros() - lastMotorYDelayTime) > motorYDelayActual) 
        //{ 
        //    runMotorY(); 
        //    lastMotorYDelayTime = micros(); 
        //}

        clock::sleep_milliseconds(1000);
    }

    return 0;
}
